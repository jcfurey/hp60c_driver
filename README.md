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
ros2 launch hp60c_driver hp60c.launch.py params_file:=my_hp60c.yaml
ros2 param set /hp60c/hp60c color.exposure_mode software   # driver AE + auto gain
ros2 param describe /hp60c/hp60c color.gain                 # the camera's range
```

Topics, each computed only while someone subscribes:

| Topic | Type | Notes |
|---|---|---|
| `color/image_raw` | Image `rgb8` 640x480 | JPEG decoded with libturbojpeg |
| `color/image_raw/compressed` | CompressedImage `jpeg` | the camera's own JPEG, passed through with no decode |
| `depth/image_raw` | Image `16UC1`, mm | depth camera frame |
| `aligned_depth_to_color/image_raw` | Image `16UC1`, mm | registered to the colour camera |
| `depth/image_filtered` | Image `16UC1`, mm | `depth/image_raw`, noise-filtered |
| `aligned_depth_to_color/image_filtered` | Image `16UC1`, mm | filtered, then registered |
| `*/camera_info` | CameraInfo | from the per-unit calibration in the stream |
| `/tf_static` | | `hp60c_color_optical_frame` → `hp60c_depth_optical_frame` (and optionally `base_frame_id` → colour) |
| `/diagnostics` | DiagnosticArray | connection, frame rate, lost frames, exposure/gain |

Settings live in `config/hp60c.yaml` (pass your own with `params_file:=...`).
Every parameter carries a description and its valid range (`ros2 param describe`).

| Parameter | Default | |
|---|---|---|
| `device` | `""` | `/dev/videoN`; empty = find the camera by USB id |
| `color_frame_id`, `depth_frame_id` | `hp60c_{color,depth}_optical_frame` | |
| `base_frame_id` | `""` | if set, also publish this body frame (x forward, z up) at the colour camera; leave empty when a URDF already places the optical frame |
| `publish_tf` | `true` | |
| `best_effort` | `false` | see QoS below |
| `use_cuda` | `true` | |
| `filter.*` | | depth filter, below; changeable at runtime |
| `color.exposure_mode` | `auto` | `auto`, `manual` or `software`; below |
| `color.software_ae.*` | | the driver's auto exposure + auto gain; below |
| `color.<control>` | the camera's | its UVC controls, declared when it first connects; below |

The first six are read-only (fixed at startup); `ros2 param set` on them is refused
rather than silently ignored. Everything else applies at runtime.

**Depth filtering (on by default; raw always published too).** Raw depth jitters
~19 mm frame to frame on a static scene. The `*/image_filtered` topics apply a
per-pixel temporal average, which resets instantly when a pixel changes by more
than `filter.temporal_reset_fraction` (3%) so motion doesn't smear, followed by
a 3x3 median over valid neighbours. They never fill holes or invent depth.
Parameters: `filter.enabled`, `filter.temporal`, `filter.temporal_alpha` (0.4),
`filter.temporal_reset_fraction` (0.03), `filter.spatial`. The `*/image_raw`
topics are untouched, for consumers that do their own filtering.

**Exposure, gain and white balance.** The camera's UVC image controls become
parameters under `color.` when it first connects: their ranges and the camera's
current values come from the camera, and the node logs every control it offers.
Names are fixed per control (they don't follow the kernel's labels, which changed
between kernel versions): `exposure_us` (UVC steps of 100 µs), `gain`,
`auto_white_balance`, `white_balance_temperature`, `exposure_dynamic_framerate`,
`power_line_frequency`, `brightness`, `contrast`, `saturation`, `hue`, `gamma`,
`sharpness`, `backlight_compensation`, whichever the camera has. Values given in
the parameter file are applied on every (re)connect. `color.exposure_mode`
chooses who sets exposure:

- `auto`: the camera's own auto exposure (its default). `gain` still applies.
- `manual`: `color.exposure_us` and `color.gain` as set.
- `software`: the driver's auto exposure and auto gain. UVC has no auto-gain or
  ISO control, and the camera's own AE can't be bounded, so the driver closes
  the loop itself: it measures each frame's mean brightness (from a 1/8-scale
  decode of the JPEG) and drives exposure and gain toward
  `color.software_ae.target_brightness` (110 of 255). It lengthens exposure first, up
  to `max_exposure_us` (30 ms, which bounds motion blur and stays under the ~40 ms
  frame period), and only then raises gain, up to `max_gain`. When darkening, it
  drops gain first. Setting `min_gain` = `max_gain` turns auto gain off. It moves
  only when brightness leaves a ±10% band (`tolerance`), waits 3 frames for each
  change to show, and halves its step if it overshoots, so it settles even
  though cameras' gain scales differ.

The node writes switches before the values they gate: the camera refuses
`exposure_us` while its auto exposure is on, and `white_balance_temperature`
while auto white balance is on. A refused write is logged once and the stream
goes on. `/diagnostics` shows the exposure mode, the exposure and gain in use (read
back once a second, so the camera's own AE can be watched too) and, in software
mode, the measured brightness. *Which controls the HP60C actually offers over
UVC has not been checked on hardware yet: the startup log lists them.* Exposure
of the depth (IR) sensor is not a UVC control and is out of reach for now.

**CUDA backend (optional).** Where a CUDA compiler is found at build time (e.g.
JetPack's `/usr/local/cuda`), depth unpacking and registration can run on the
GPU. If no GPU is usable at runtime, the node logs it and uses the CPU path, and
it falls back the same way if CUDA fails mid-run. The CUDA results are tested
against the CPU path (`test/test_cuda_registrar.cpp`: unpacking identical,
registration within 1 mm at <0.1% of pixels). It passes on the Orin Nano's GPU
and on an RTX 5090. Measured on the Orin (percent of one core):

| subscribed | CPU path | CUDA |
|---|---|---|
| filtered aligned depth | 34.5% | **15.2%** |
| raw aligned depth | 23.6% | **3.5%** |
| colour + depth + both aligned | 56.2% | **31.8%** |

If the GPU is unavailable (the dev unit's once failed to power on with nvgpu
`ACR bootstrap failed`), the node says so and runs everything on the CPU.

**ROS 2 conventions.** Frames follow REP 103/105: `*_optical_frame` is z forward,
x right, y down, and `base_frame_id` adds the usual body frame (x forward, z up).
Depth is REP 118 (`16UC1` millimetres, 0 = no measurement). Image stamps are the
acquisition time, as `sensor_msgs/Image` defines it: uvcvideo's timestamp for
the frame moved onto the ROS clock, which takes the USB transfer time of each
~630 KB frame out of the stamp. Publishers follow REP 2003: reliable, volatile
(the system default), which serves reliable and `SensorDataQoS` subscribers
alike. The keep-last depth is 2, because intra-process comms needs an explicit
depth. Per topic, the
standard `qos_overrides.<topic>.publisher.{reliability,history,depth}` parameters
change it at startup. The compressed topic is the image_transport `compressed`
transport, so `image_transport` subscribers and `republish` work with it. The
node is an `rclcpp_components` component, reports to `/diagnostics` through
`diagnostic_updater`, and validates every parameter against its descriptor.

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
| Depth noise, static scene (per-pixel temporal std, median) | raw 18.5 mm → filtered 8.7 mm, same coverage | — |
| CPU, filtered aligned depth only (the default consumer) | 35% of a core (filter itself: 2.2 ms/frame) | — |

The camera itself emits ~24.8 fps. About 1 frame in 5 is lost below the driver
(plain `v4l2-ctl` shows the same drops).

## Status

- **Milestone 1: frame format** decoded (`docs/FRAME_FORMAT.md`).
- **Milestone 2: calibration** decoded: intrinsics, depth→colour extrinsics,
  per-frame device timestamps.
- **Milestone 3: ROS 2 node** built and tested on hardware as above. It builds and
  passes its tests on Jazzy, Kilted and Lyrical.

- **CUDA backend** for depth unpacking + registration: verified on the Orin Nano
  and an RTX 5090; roughly halves the node's CPU on the Orin.
- **Camera controls and software auto exposure / auto gain**: unit-tested against
  simulated cameras, and run end to end against an emulated UVC device. Not yet
  run on the HP60C itself, so its control set is still unknown.

Next: move the depth filter onto the GPU too. JPEG decode stays on the CPU: the Orin Nano
has no hardware JPEG engine, and nvJPEG would still Huffman-decode on the CPU.
