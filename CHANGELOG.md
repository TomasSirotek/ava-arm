# Changelog

All notable changes to this repo. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### In progress — arm redesign
- **Why:** the SO-101 model failed the load test in Gazebo. Most of the weight and the payload
  sit at the end of the arm, so the shoulder and elbow need bigger, stronger, more expensive
  servos than planned. The torque math (`tools/torque_monitor.py`) confirmed it: shortening the
  links barely helps.
- **Decision:** switch to a different open-source / free 3D model built for **2× MG996R + 4× SG90**.
- **Now:** redrawing it in Fusion 360. The MG995 drawing had no usable dimensions — the
  documentation and the available models all disagreed — so the servo was re-measured from the
  manufacturer's details and redrawn.

### TODO
- Import the new model (URDF + meshes) into `ava_description` and re-run the Gazebo load test.
- Servo layout for the new model: 2× MG996R, 4× SG90.

### Added
- `tools/torque_monitor.py`: live torque per joint vs its servo (OK / HOT / FAIL); Gazebo `payload_kg:=` option to test with a weight in the gripper.
- SO-101 robot model in `ava_description` (from [so_arm_ros2](https://github.com/adityakamath/so_arm_ros2), meshes from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)), with a single entry point `ava.urdf.xacro`.
- `ava.ros2control.xacro`: one switch for `real` / `gazebo` / `mock_components` / `mujoco`.
- `scripts/generate_urdf.sh`: builds the plain `urdf/ava.urdf` (for the dashboard and other tools) and validates it with `check_urdf`.
- Gazebo: robot anchored to the ground, mesh search path set, `headless:=true` launch option.
- MoveIt on Gazebo: `gazebo_moveit.launch.py` publishes the `world` frame; SRDF and joint limits rewritten for SO-101.
- `ava_bringup` package (empty stub).
- `LICENSES/` with the MIT (OmArm Zero) and Apache-2.0 (SO-ARM100, so_arm_ros2) texts.

### Changed
- Joint names follow SO-101: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` (were `Revolute 1–7`).
- Packages renamed from `omarm_zero_*` to `ava_*`; gamepad and real-hardware control merged into `ava_control`; `computer_vision_aruco` is now `ava_vision`.
- `ATTRIBUTION.md` rewritten: correct licenses and what was taken from where.

### Removed
- The old OmArm Zero model (`ava.xacro`, `ava.gazebo`, `ava.ros2control`, `materials.xacro`) and its meshes.
- Legacy OmArm ESP32 firmware and electronics files.

### Known issues
- `arm_ik.py` and `cup_pick_and_place.py` still describe the old 5-DOF arm (`TODO(so101-migration)`).
- The `real` hardware option drives Feetech STS servos, not the planned MG996R/SG90.

## 2026-09-24 — Initial import
- Repo created from the OmArm Zero ROS 2 workspace.
