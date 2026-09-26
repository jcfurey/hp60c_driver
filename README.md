# hp60c_driver

Open, clean-room ROS 2 driver for the **Angstrong Nuwa HP60C** RGB-D camera
(USB `3482:6723`, sold by Yahboom), targeting ROS 2 Jazzy, Kilted and Lyrical.

It exists to replace the vendor `ascamera` package, which wraps closed prebuilt
`.so` libraries that cannot be modified, fixed or moved to the GPU.

## Clean-room rule

This repository must contain **no vendor source and no vendor binaries**, and no
code derived from disassembling them. Implementations are written only from
observed device behaviour (USB descriptors, captured frames, published outputs)
and the written specification in `docs/`. Interoperability research notes live in
`jcfurey/jcfurey-jetson-ws` → `docs/HP60C_REVERSE_ENGINEERING.md`.

## Usage

```bash
# in a colcon workspace
git clone https://github.com/jcfurey/hp60c_driver src/hp60c_driver
rosdep install --from-paths src --ignore-src -y     # pulls libturbojpeg
colcon build --packages-select hp60c_driver
ros2 launch hp60c_driver hp60c.launch.py            # namespace /hp60c
```

Topics, each computed only while someone subscribes:

| Topic | Type | Notes |
|---|---|---|
| `color/image_raw` | Image `rgb8` 640x480 | JPEG decoded with libturbojpeg |
| `color/image_raw/compressed` | CompressedImage `jpeg` | the camera's own JPEG, passed through with no decode |
| `depth/image_raw` | Image `16UC1`, mm | depth camera frame |
| `aligned_depth_to_color/image_raw` | Image `16UC1`, mm | registered to the colour camera |
| `*/camera_info` | CameraInfo | from the per-unit calibration in the stream |
| `/tf_static` | | `hp60c_color_optical_frame` → `hp60c_depth_optical_frame` |

Parameters: `device` (empty = find the camera by USB id), `color_frame_id`,
`depth_frame_id`, `publish_tf`, and `best_effort` (default `false`: images are
reliable, which serves both reliable and best-effort subscribers).

The node needs the camera bound to the kernel's `uvcvideo` driver. The vendor
SDK detaches that. If `/dev/video*` is missing after the vendor driver ran,
re-plug the camera or re-authorize its USB device.

## Measured (Jetson Orin Nano, Jazzy, 2026-09-26)

| | this driver | vendor `ascamera` |
|---|---|---|
| Frame rate delivered | ~20.7 Hz on all streams | RGB 19 Hz, depth 12 Hz |
| CPU, nothing subscribed | 0.3% of a core | 0% |
| CPU, colour + depth + aligned + JPEG | 32% of a core | ~50% (RGB + depth only) |
| Aligned depth vs vendor | correlation 0.999, median 5 mm, 99% within 2%, 58% vs 56% coverage | — |

The camera itself emits ~24.8 fps. About 1 frame in 5 is lost below the driver
(plain `v4l2-ctl` shows the same drops).

## Status

- **Milestone 1: frame format** decoded (`docs/FRAME_FORMAT.md`).
- **Milestone 2: calibration** decoded: intrinsics, depth→colour extrinsics,
  per-frame device timestamps.
- **Milestone 3: ROS 2 node** built and tested on hardware as above. It builds and
  passes its tests on Jazzy, Kilted and Lyrical.

Next: move JPEG decode and registration to the GPU; resolve the low 4 depth
bits; decide whether to match the vendor's depth filtering.
