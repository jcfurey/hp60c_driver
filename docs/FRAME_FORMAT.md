# HP60C composite frame format (UVC, 640x642 MJPEG mode)

Established 2026-09-26 on a Jetson Orin Nano (L4T R39.2.1, kernel `uvcvideo`)
purely from captured frames and the vendor driver's *published* ROS output — no
vendor code was read. Every claim below says how it was observed.

## Transport

The camera enumerates as a standard UVC device (`3482:6723`, "ASJ ZNX_NVT") and
streams through the kernel `uvcvideo` driver. No libusb and no vendor
library are needed. It advertises:

| Format | Size | Rate |
|---|---|---|
| MJPG | 640x642, 320x564, 160x768, 1280x720 | 30 fps |
| YUYV | 1280x1040 | 8 fps |

The "640x642" MJPG mode is **not** a 640x642 JPEG. Each V4L2 buffer is a
composite of three parts. (Other composite sizes are not yet examined.)

## Buffer layout (640x642 MJPG)

```
offset 0          2560 B   prefix      calibration + per-frame metadata (see below)
offset 2560       ~17 KB   JPEG        colour, baseline JFIF-less JPEG, 640x480, YCbCr 4:2:2
after JPEG EOI    614912 B tail        depth: 614400 B image + 512 B trailer
```

- The JPEG's length varies per frame. Find its end by walking the
  entropy-coded segment after SOS: the first `FF xx` with `xx` not `00` and not
  `D0..D7` is the EOI (`FF D9`). Do **not** use the last `FF D9` in the buffer;
  the depth payload contains that byte pair by chance.
- Observed buffer sizes: 631–634 KB. The prefix and tail sizes were constant
  across all 15 frames examined.

### Depth (first 614400 bytes of the tail)

- 480 wide x 640 tall, `uint16` little-endian. The image is stored **rotated**:
  `np.rot90(depth, 3)` gives the 640x480 landscape image in the colour
  camera's orientation.
- **The value is depth in 1/16 mm** (max 4095.94 mm). Integer millimetres are
  `(value + 8) >> 4`, rounded. Plain `>> 4` truncates, which biases everything
  by −0.47 mm on average.

  Evidence for the fraction (100 frames, static scene): the distinct raw
  values in a narrow range sit on a ladder whose spacing grows as Z² (0.63 mm
  at 1.02 m, 1.4 mm at 1.52 m, 3.8 mm at 2.53 m, 9.3 mm at 3.95 m, matching
  (Z/1.02 m)² to within 3%). That is the signature of depth computed from
  quantised disparity. It also explains why the low nibble is far from uniform
  (χ²/dof ≈ 189,000) and sticky per pixel: values snap to rungs. It is not a
  flag field.
- `0` = no measurement (~39% of pixels in the test scene).

Evidence: against the vendor node's published `depth0/image_raw` (16UC1, mm,
registered to the colour camera) for the same static scene, warping our decode
with the two intrinsic sets below (no extrinsic rotation, x-shift searched)
gave correlation 0.978 over 172,485 pixels, median ratio 1.0007 and median
absolute error 18 mm.

### Trailer (last 512 bytes of the tail)

Mostly zero. Not yet decoded.

### Prefix (2560 bytes)

Two regions: static calibration (bytes 0–1023) and per-frame metadata (bytes
1024–2559). Verified across 15 frames from two capture sessions.

#### Calibration: bytes 0–1023, constant

Little-endian 32-bit words:

| word | type | value (this unit) | meaning |
|---|---|---|---|
| 0–1 | ? | `0x6dac62ce 0x6acc4ce8` | unknown, constant (not plausible floats; maybe an ID) |
| 2–5 | f32 | 443.635, 443.320, 321.799, 238.200 | **depth** fx, fy, cx, cy (px) |
| 6–10 | f32 | 0 | probably depth distortion (all zero) |
| 11–14 | f32 | 590.229, 589.842, 332.799, 232.275 | **colour** fx, fy, cx, cy (px) |
| 15–19 | f32 | 0 | probably colour distortion (all zero) |
| 20–28 | f32 | ≈ identity, 2.31° roll | **R**, row-major 3x3 rotation |
| 29–31 | f32 | −12.036, −0.254, −0.043 | **t**, millimetres (12.04 mm baseline) |
| 32–33 | u32 | 90, 100 | unknown integers |
| 34–255 | — | 0 | unused |

R is a proper rotation (‖RRᵀ−I‖ < 6e-8, det = 1). Convention, established by
testing all four candidates against the vendor's registered output:

```
P_colour = R · P_depth + t        (P in mm, camera frames, z forward)
```

Registering depth into the colour camera with this convention (no distortion,
nearest pixel, z-buffer min) matches the vendor's `depth0/image_raw` with
correlation 0.9978, median |error| 7.6 mm and 95.4% of pixels within 2%
(99,422 pixels). The alternatives scored 13–32 mm; no registration scored 106 mm.
The remaining error is consistent with vendor-side filtering: only 14% of
pixels match to ≤1 mm. The frames also came from different moments.

Intrinsics are per-unit calibration, so read them from each frame rather than
hard-coding them.

#### Per-frame metadata: bytes 1024–2559

Only bytes 1024–1135 change between frames. Among them, as little-endian u32
words:

| word | meaning | evidence |
|---|---|---|
| 256–257 | u64 timestamp, µs | steps of ~40,320 per frame |
| 258–259 | u64 timestamp, µs | a second clock, ~6.9 ms later than 256 |
| 268, 274 | **JPEG length in bytes** | equals the parsed JPEG size exactly, every frame |
| 269, 275 | timestamp, ms | steps of 40 |
| 270, 276 | copies of words 256 and 258 | |
| 272 | timestamp, whole seconds | |
| 273 | ? | changes rarely |
| 278, 279, 283 | ? | vary per frame; 279 and 283 move together |

The rest of words 280–639 (all except 283) is non-zero but constant, and doesn't
look like floats. It's unknown.

**Frame rate:** timestamps step by 40.3 ms, so the camera delivers **~24.8 fps**,
not the 30 fps the UVC descriptor advertises. One frame in five was missing
from a v4l2 capture (an 80.6 ms step).

## Open questions (next milestones)

1. ~~Meaning of the low 4 depth bits~~: resolved. It's the 1/16 mm fraction
   (see Depth).
2. The 512 B trailer, prefix words 0–1 and 32–33, and the constant block at
   words 280–639.
3. Whether the vendor applies filtering we want to match (it reports fewer
   near-range points: min 341 mm vs our 5th percentile ~540 mm raw).
4. ~~Frame rate~~: resolved. The sensor cycle is 40.3 ms (24.8 Hz), but the
   camera sends 5 of every 6 cycles: the device timestamps show one 80.7 ms gap
   exactly every 5 frames. The host loses nothing: `uvcvideo` stats over 100
   frames show 0 errors, 0 invalid, 0 empty. Effective rate ~20.7 fps. Every
   delivered frame is fresh (no repeated depth or JPEG payload in 99
   consecutive pairs). On a static scene, per-pixel depth changes ~21 mm
   between frames: that is raw sensor noise, relevant to filtering.
5. The other composite sizes (320x564, 160x768).
