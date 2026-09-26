#!/usr/bin/env python3
"""Split HP60C 640x642 composite MJPG buffers into colour, depth and calibration.

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
        tail = data[end:end + DEPTH_BYTES + TRAILER_BYTES]
        if len(tail) < DEPTH_BYTES + TRAILER_BYTES:
            break                                   # truncated last buffer
        yield (data[off:soi], data[soi:end], tail[:DEPTH_BYTES],
               tail[DEPTH_BYTES:])
        off = end + DEPTH_BYTES + TRAILER_BYTES


def depth_mm(depth_raw):
    """640x480 uint16 depth in millimetres, 0 = no measurement."""
    stored = np.frombuffer(depth_raw, '<u2').reshape(DEPTH_H, DEPTH_W)
    return np.ascontiguousarray(np.rot90(stored, 3) >> 4)


def intrinsics(prefix):
    """(depth, colour) intrinsics as (fx, fy, cx, cy) tuples."""
    f = np.frombuffer(prefix[:64], '<f4')
    return tuple(map(float, f[2:6])), tuple(map(float, f[11:15]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('stream', type=pathlib.Path)
    ap.add_argument('--out', type=pathlib.Path)
    args = ap.parse_args()
    data = args.stream.read_bytes()
    for n, (prefix, jpeg, raw, _trailer) in enumerate(split_stream(data)):
        d = depth_mm(raw)
        valid = d[d > 0]
        (fxd, fyd, _, _), (fxc, fyc, _, _) = intrinsics(prefix)
        print(f'frame {n}: jpeg {len(jpeg)} B, depth valid '
              f'{valid.size / d.size:.0%}, median {int(np.median(valid))} mm, '
              f'fx depth/colour {fxd:.1f}/{fxc:.1f}')
        if args.out:
            args.out.mkdir(parents=True, exist_ok=True)
            (args.out / f'{n:04d}.jpg').write_bytes(jpeg)
            np.save(args.out / f'{n:04d}_depth_mm.npy', d)


if __name__ == '__main__':
    main()
