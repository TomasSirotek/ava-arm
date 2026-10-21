# Changelog

All notable changes to this repo. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- CI: config tests, dashboard tests, firmware tests + build. Status badges in the README.
- `tests/`: cross-package config checks.
- Dependabot updates for the dashboard submodule, actions and PlatformIO.
- `hardware/bom.md`: parts list with key specs and prices in CZK and EUR (701 CZK / 28.82 EUR).
- ESP32-C3 servo firmware (serial protocol v1 → PCA9685), unit-tested on the PC.
- `tools/torque_monitor.py`: live torque per joint vs its servo (OK / HOT / FAIL); Gazebo `payload_kg:=` option to test with a weight in the gripper.
- `ava.ros2control.xacro`: one switch for `real` / `gazebo` / `mock_components` / `mujoco`.
- `scripts/generate_urdf.sh`: builds the plain `urdf/ava.urdf` (for the dashboard and other tools) and validates it with `check_urdf`.
- Gazebo: robot anchored to the ground, mesh search path set, `headless:=true` launch option.
- MoveIt on Gazebo: `gazebo_moveit.launch.py` publishes the `world` frame.
- `ava_bringup` package (empty stub).
- Top-level `LICENSE` (Apache-2.0) for this repo's own code.
- `LICENSES/` with the MIT (OmArm Zero) and Apache-2.0 (SO-ARM100, so_arm_ros2) texts.

### Changed
- In-house arm model (Fusion 360) replaces the SO-101 model; generator script `generate_kinematics_xacro.py`; SRDF rebuilt.
- Servo layout 3× MG996R + 3× SG90 (elbow now MG996R); servo ratings at 6.0 V.
- Joint names: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` (were `Revolute 1–7`).
- Packages renamed from `omarm_zero_*` to `ava_*`; gamepad and real-hardware control merged into `ava_control`; `computer_vision_aruco` is now `ava_vision`.
- `ATTRIBUTION.md` rewritten.

### Removed
- The old OmArm Zero model (`ava.xacro`, `ava.gazebo`, `ava.ros2control`, `materials.xacro`) and its meshes.
- Legacy OmArm ESP32 firmware and electronics files.
- Legacy `ava_control` hardware bridges and their configs.
- STS3215 serial-bus parameters (motor IDs, EEPROM PID, `/dev/ttySERVO`) from the xacro.

### Known issues
- Pick-and-place scripts (`arm_ik.py`, `cup_pick_and_place.py`) not yet ported to the current arm (`TODO(cad-redesign)`).
- `joint_limits.yaml` velocities not tuned.
- Overhead-camera extrinsics need recalibration.
- Hardware layer (`ava_hardware`, firmware) not yet tested on hardware.

## 2026-09-24 — Initial import
- Repo created from the OmArm Zero ROS 2 workspace.
