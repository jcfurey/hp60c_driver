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

Milestone 1: determine the composite frame layout (where depth sits in the
640x642 stream and how it is encoded) from raw v4l2 captures.
