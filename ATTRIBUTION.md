# Attribution

Ava builds on two open-source projects. Their license texts are in [`LICENSES/`](LICENSES/).

## OmArm Zero — Omar Draidrya (OmArTronics)
- Source: https://github.com/ODraidrya/OmArm-Zero · https://omartronics.com
- License: **MIT** © 2026 Omar Draidrya — [`LICENSES/MIT-OmArm-Zero.txt`](LICENSES/MIT-OmArm-Zero.txt)
- Used: the whole `ros2_ws/` started as a fork of the OmArm Zero ROS 2 workspace
  (`omarm_zero_*` packages, renamed to `ava_*`), including the launch files, MoveIt
  config, `ava_control` nodes and `ava_vision` (ArUco markers included).
- Not included: the OmArm PDF guides (all rights reserved, not redistributable).

## SO-ARM100 / SO-101 — TheRobotStudio
- Source: https://github.com/TheRobotStudio/SO-ARM100
- License: **Apache-2.0** — [`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt)
- Used: the STL meshes in `ros2_ws/src/ava_description/meshes/`.

## so_arm_ros2 — Aditya Kamath
- Source: https://github.com/adityakamath/so_arm_ros2
- License: **Apache-2.0** — [`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt)
- Used, **modified**: `ava.urdf.xacro`, `ava.common.xacro`, `ava.control.xacro` in
  `ava_description/urdf/` (from `so101.*.xacro`). Changes: renamed to `ava`, package and
  mesh paths, bare joint names, `<ros2_control>` moved to `ava.ros2control.xacro` with a
  Gazebo option, Gazebo anchoring in `ava.gazebo.xacro`.

## Reference only (compared against, not copied)
- https://github.com/MuammerBay/SO-ARM_ROS2_URDF
- https://github.com/brukg/SO-100-arm
