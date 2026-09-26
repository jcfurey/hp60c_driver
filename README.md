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

## Status

**Milestone 1 done: frame format decoded** (`docs/FRAME_FORMAT.md`). The camera
is plain UVC. Each 640x642 MJPG buffer carries a calibration prefix, a 640x480
colour JPEG and 640x480 depth (`uint16 >> 4` = mm, stored rotated). This
matches the vendor driver's published depth (correlation 0.978, median ratio
1.0007). `tools/decode_frame.py` splits captured buffers.

**Milestone 2 done: calibration decoded.** The prefix carries depth and colour
intrinsics, the depth→colour extrinsics (`P_colour = R·P_depth + t`, 12 mm
baseline) and per-frame device timestamps (the camera runs at ~24.8 fps).
Registering with them matches the vendor's aligned depth to a median 7.6 mm,
with 95% of pixels within 2%. `tools/decode_frame.py --register` does this.

Next: the ROS 2 node (v4l2 capture → colour, depth, registered depth,
CameraInfo, TF), portable across Jazzy, Kilted and Lyrical.
