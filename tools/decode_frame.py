#!/usr/bin/env python3
# Copyright 2026 jcfurey
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Split HP60C 640x642 composite MJPG buffers into colour, depth and calibration.

Written from docs/FRAME_FORMAT.md. Input is a raw stream as written by
`v4l2-ctl --stream-to` (buffers back to back):

    v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=642,pixelformat=MJPG
    v4l2-ctl -d /dev/video0 --stream-mmap=4 --stream-count=10 --stream-to=cap.mjpg
    tools/decode_frame.py cap.mjpg --out frames/
"""
import argparse
import pathlib

import numpy as np

PREFIX_BYTES = 2560
DEPTH_W, DEPTH_H = 480, 640            # stored rotated
DEPTH_BYTES = DEPTH_W * DEPTH_H * 2
TRAILER_BYTES = 512


def jpeg_end(buf, start):
    """Return the index just past the EOI of the JPEG beginning at `start`."""
    sos = buf.find(b'\xff\xda', start)
    if sos < 0:
        raise ValueError('no SOS marker')
    i = sos + 2 + int.from_bytes(buf[sos + 2:sos + 4], 'big')
    while True:
        i = buf.find(b'\xff', i)
        if i < 0 or i + 1 >= len(buf):
            raise ValueError('JPEG not terminated')
        nxt = buf[i + 1]
        if nxt == 0x00 or 0xD0 <= nxt <= 0xD7:   # stuffed byte / restart marker
            i += 2
            continue
        if nxt != 0xD9:
            raise ValueError(f'unexpected marker FF{nxt:02X} in scan data')
        return i + 2


def split_stream(data):
    """Yield (prefix, jpeg_bytes, depth_raw, trailer) for each buffer."""
    off = 0
    while off + PREFIX_BYTES < len(data):
        soi = off + PREFIX_BYTES
        if data[soi:soi + 2] != b'\xff\xd8':
            raise ValueError(f'no JPEG SOI at offset {soi}')
        end = jpeg_end(data, soi)
        declared = frame_meta(data[off:soi])['jpeg_len']
        if end - soi != declared:
            raise ValueError(f'JPEG is {end - soi} B but header says {declared}')
        tail = data[end:end + DEPTH_BYTES + TRAILER_BYTES]
        if len(tail) < DEPTH_BYTES + TRAILER_BYTES:
            break                                   # truncated last buffer
        yield (data[off:soi], data[soi:end], tail[:DEPTH_BYTES],
               tail[DEPTH_BYTES:])
        off = end + DEPTH_BYTES + TRAILER_BYTES


def depth_mm(depth_raw):
    """Return 640x480 uint16 depth in millimetres (rounded), 0 = no measurement."""
    stored = np.frombuffer(depth_raw, '<u2').reshape(DEPTH_H, DEPTH_W).astype(np.uint32)
    return np.ascontiguousarray((np.rot90(stored, 3) + 8) >> 4).astype(np.uint16)


def calibration(prefix):
    """
    Read the static calibration from the prefix (docs/FRAME_FORMAT.md).

    Returns a dict with depth/colour intrinsics as (fx, fy, cx, cy) and the
    extrinsics R (3x3), t (mm) mapping P_colour = R @ P_depth + t.
    """
    f = np.frombuffer(prefix[:128], '<f4').astype(np.float64)
    return {
        'depth_K': tuple(f[2:6]),
        'colour_K': tuple(f[11:15]),
        'R': f[20:29].reshape(3, 3),
        't': f[29:32].copy(),
    }


def frame_meta(prefix):
    """Return per-frame metadata: device timestamp (us) and JPEG length (bytes)."""
    w = np.frombuffer(prefix, '<u4')
    return {
        'stamp_us': int(w[256]) | (int(w[257]) << 32),
        'jpeg_len': int(w[268]),
    }


def register_to_colour(depth, calib):
    """
    Reproject depth (mm, depth camera) into the colour camera's pixels.

    Nearest-pixel forward projection with a z-buffer (nearest surface wins).
    No distortion model: the calibration's distortion slots are all zero.
    """
    fxd, fyd, cxd, cyd = calib['depth_K']
    fxc, fyc, cxc, cyc = calib['colour_K']
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    valid = depth > 0
    z = depth[valid].astype(np.float64)
    pts = np.stack([(u[valid] - cxd) / fxd * z, (v[valid] - cyd) / fyd * z, z])
    pc = calib['R'] @ pts + calib['t'][:, None]
    uc = np.round(pc[0] / pc[2] * fxc + cxc).astype(int)
    vc = np.round(pc[1] / pc[2] * fyc + cyc).astype(int)
    ok = (pc[2] > 0) & (uc >= 0) & (uc < w) & (vc >= 0) & (vc < h)
    out = np.full((h, w), np.inf)
    np.minimum.at(out, (vc[ok], uc[ok]), pc[2][ok])
    out[np.isinf(out)] = 0
    return np.round(out).astype(np.uint16)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('stream', type=pathlib.Path)
    ap.add_argument('--out', type=pathlib.Path)
    ap.add_argument('--register', action='store_true',
                    help='also save depth registered to the colour camera')
    args = ap.parse_args()
    data = args.stream.read_bytes()
    prev = None
    for n, (prefix, jpeg, raw, _trailer) in enumerate(split_stream(data)):
        d = depth_mm(raw)
        valid = d[d > 0]
        cal, meta = calibration(prefix), frame_meta(prefix)
        dt = '' if prev is None else f', dt {(meta["stamp_us"] - prev) / 1000:.1f} ms'
        prev = meta['stamp_us']
        print(f'frame {n}: jpeg {len(jpeg)} B, depth valid '
              f'{valid.size / d.size:.0%}, median {int(np.median(valid))} mm, '
              f'baseline {np.linalg.norm(cal["t"]):.2f} mm{dt}')
        if args.out:
            args.out.mkdir(parents=True, exist_ok=True)
            (args.out / f'{n:04d}.jpg').write_bytes(jpeg)
            np.save(args.out / f'{n:04d}_depth_mm.npy', d)
            if args.register:
                np.save(args.out / f'{n:04d}_depth_mm_registered.npy',
                        register_to_colour(d, cal))


if __name__ == '__main__':
    main()
