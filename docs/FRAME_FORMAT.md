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
offset 0          2560 B   prefix      calibration block (see below)
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
- **`depth_mm = value >> 4`**. The upper 12 bits are millimetres (max 4095). The
  meaning of the low 4 bits (sub-mm fraction or flags) is **not yet known**.
- `0` = no measurement (~39% of pixels in the test scene).

Evidence: against the vendor node's published `depth0/image_raw` (16UC1, mm,
registered to the colour camera) for the same static scene, warping our decode
with the two intrinsic sets below (no extrinsic rotation, x-shift searched)
gave correlation 0.978 over 172,485 pixels, median ratio 1.0007 and median
absolute error 18 mm.

### Trailer (last 512 bytes of the tail)

Mostly zero. Not yet decoded.

### Prefix (2560 bytes): calibration

Read as little-endian `float32` from offset 0:

| float index | value (this unit) | interpretation |
|---|---|---|
| 0–1 | garbage-looking | unknown (maybe a header/ID) |
| 2–5 | 443.635, 443.320, 321.799, 238.200 | **depth** fx, fy, cx, cy |
| 6–10 | 0 | (distortion?) |
| 11–14 | 590.228, 589.842, 332.799, 232.275 | **colour** fx, fy, cx, cy |

Intrinsics are per-unit calibration, so read them from each frame rather than
hard-coding them. The remaining ~2.5 KB is undecoded. It likely holds the
depth→colour extrinsics needed for proper registration.

## Open questions (next milestones)

1. Meaning of the low 4 depth bits, the 512 B trailer and the rest of the prefix
   (extrinsics, distortion).
2. Proper depth→colour registration using those extrinsics, then compare
   per-pixel against the vendor output again. The target is well under the
   current 18 mm median error.
3. Whether the vendor applies filtering we want to match (it reports fewer
   near-range points: min 341 mm vs our 5th percentile ~540 mm raw).
4. Frame rate: the vendor publishes depth at ~12 Hz. Check whether composites
   really arrive at 30 fps and whether depth is fresh in every one.
5. The other composite sizes (320x564, 160x768).
