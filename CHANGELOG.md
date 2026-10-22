# Changelog

All notable changes to this repo. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- CI: config tests, dashboard tests, firmware tests + build. Status badges in the README.
- `tests/`: cross-package config checks.
- Dependabot updates for the dashboard submodule, actions and PlatformIO.
- `hardware/bom.md`: parts list with key specs and prices in CZK and EUR (701 CZK / 28.82 EUR).
- ESP32-C3 servo firmware (serial protocol v1 → PCA9685), unit-tested on the PC.
- `ava_hardware` ros2_control plugin is built (no longer parked); `real_robot.launch.py` drives the PCA9685 through the firmware.
- `tools/torque_monitor.py`: live torque per joint vs its servo (OK / HOT / FAIL); Gazebo `payload_kg:=` option to test with a weight in the gripper.
- `ava.ros2control.xacro`: one switch for `real` / `gazebo` / `mock_components` / `mujoco`.
- `scripts/generate_urdf.sh`: builds the plain `urdf/ava.urdf` (for the dashboard and other tools) and validates it with `check_urdf`.
- Gazebo: robot anchored to the ground, mesh search path set, `headless:=true` launch option.
- MoveIt on Gazebo: `gazebo_moveit.launch.py` publishes the `world` frame.
- `ava_bringup` package (empty stub).
- Top-level `LICENSE` (Apache-2.0) for this repo's own code.
- `LICENSES/` with the MIT (OmArm Zero) and Apache-2.0 (SO-ARM100, so_arm_ros2) texts.
- Real arm: per-joint rest pose (`ava.control.xacro`); the arm starts there and parks there smoothly on Ctrl+C (`servo_park_duration_s`).
- SRDF test poses: `wrist_test_a`, `wrist_test_b`, `reach_left`, `reach_right` (arm) and `half_open` (gripper).
- README: wiring diagram (`hardware/wiring.png`), KiCad schematic (`hardware/schematic.png`) and pin table.
- README: step-by-step guide to run the real arm.

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

### Fixed
- `real_moveit.launch.py` now starts RViz (the included `real_robot.launch.py` switched it off).
- `ava_hardware`: no clamp warning for pulses that only round onto a servo limit.

### Known issues
- Pick-and-place scripts (`arm_ik.py`, `cup_pick_and_place.py`) not yet ported to the current arm (`TODO(cad-redesign)`).
- `joint_limits.yaml` velocities not tuned.
- Overhead-camera extrinsics need recalibration.
- `ava_hardware` tested against a simulated board only; real-arm calibration pending.

## 2026-09-24 — Initial import
- Repo created from the OmArm Zero ROS 2 workspace.
