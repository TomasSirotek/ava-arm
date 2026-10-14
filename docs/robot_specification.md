# AVA Robot Specification

Every value below is copied from a file in this repository; the source is cited next to it.
`TBD` = not recorded anywhere in the repo. Do not fill a `TBD` from memory or a shop listing —
measure it or take it from the datasheet of the part actually bought, then cite the source.

Last verified against the repo: 2026-10-12

## 1. Overview

| Item | Value | Source |
|---|---|---|
| Robot name | `ava` | `ros2_ws/src/ava_description/urdf/ava.urdf.xacro` (`<robot name="ava">`) |
| Type | 6-DOF desktop robotic arm with gripper | `README.md`, `ava_description/README.md` |
| Design origin | Custom Fusion 360 design built around a fixed servo budget (replaced the SO-101 model). **Under construction**: the elbow MG996R mount is being redrawn and further changes are expected, so the CAD-derived values in this file are provisional | `CHANGELOG.md` §1–2 |
| Middleware | ROS 2 Jazzy | `README.md`, `AGENTS.md` |
| Simulator | Gazebo Harmonic (gz sim), `empty.sdf` world | `ava_description/README.md`, `launch/gazebo.launch.py` |
| Motion planning | MoveIt 2 | `ros2_ws/src/ava_moveit_config/` |
| Current stage | Testing the servo layout under load in simulation; no hardware connected | `CHANGELOG.md` §3, `README.md` |

## 2. Physical properties

| Item | Value | Notes |
|---|---|---|
| Overall dimensions (L × W × H) | TBD | `docs/02-link-dimensions.md` exists but is empty |
| Link lengths | TBD | Geometry is in the git-ignored CAD export; no link lengths recorded in the docs |
| Reach | TBD | |
| Total mass | TBD | |
| Rated payload | 20 g for now (simulation only) | Gazebo sweep: all test poses held at 20 g, `wrist_flex` fails at 50 g (`simulation_validation.md`, `decisions.md` O-3). Not verified on hardware |
| Repeatability / accuracy | TBD | |
| Materials / print settings | TBD | `hardware/bom.md` is empty |

### Measured frame offsets (from CAD meshes, not hardware)

| Item | Value | Source |
|---|---|---|
| `base_mount_z` (base_link underside above `base_footprint`) | 0.0337 m | `urdf/ava.common.xacro` |
| `world_to_base` rotation | π about X (corrects Z-down CAD export) | `urdf/ava.urdf.xacro` |
| Tool frame `end_effector_xyz` (in `gripper_base_link`) | `-0.065 0.003 -0.025` m | `urdf/ava.common.xacro` |
| Tool frame `end_effector_rpy` | `0 0 3.141593` | `urdf/ava.common.xacro` |
| Wrist camera extrinsics | All zeros: **PLACEHOLDER**, not calibrated | `urdf/ava.common.xacro` |

## 3. Joints

All six actuated joints are `revolute` (`urdf/ava.kinematics.xacro`, generated, local only).
Position limits come from `urdf/ava.control.xacro`, which mirrors the CAD export. Degree
values are converted from the radian values.

| # | Joint | Moves | Lower (rad / °) | Upper (rad / °) | Servo | URDF effort limit (N·m) |
|---|---|---|---|---|---|---|
| 1 | `shoulder_pan` | whole arm about vertical axis | −1.570796 / −90.0 | 1.570796 / 90.0 | MG996R | 1.079 |
| 2 | `shoulder_lift` | upper arm | −0.212536 / −12.2 | 2.094395 / 120.0 | MG996R (direct drive) | 1.079 |
| 3 | `elbow_flex` | lower arm | −1.523672 / −87.3 | 1.617920 / 92.7 | MG996R (was SG90; CAD still shows SG90 until redrawn) | 1.079 |
| 4 | `wrist_flex` | wrist pitch | −2.617994 / −150.0 | 0.523599 / 30.0 | SG90 | 0.157 |
| 5 | `wrist_roll` | gripper roll | −1.963495 / −112.5 | 1.178097 / 67.5 | SG90 | 0.157 |
| 6 | `gripper` | moving jaw | −0.594434 / −34.1 | 0.197952 / 11.3 | SG90 | 0.157 |

Servo-per-joint sources: `urdf/ava.control.xacro`, `tools/torque_monitor.py` (`JOINT_SERVO`).

### MoveIt velocity / acceleration limits

From `ava_moveit_config/config/joint_limits.yaml`. `CHANGELOG.md` §3 says these are **still
SO-101 values** and need re-tuning for hobby servos.

| Joint | max_velocity (rad/s) | max_acceleration (rad/s²) |
|---|---|---|
| shoulder_pan, shoulder_lift, elbow_flex | 1.0 | 2.0 |
| wrist_flex, wrist_roll, gripper | 1.5 | 3.0 |

### Named poses (SRDF)

From `ava_moveit_config/config/ava.srdf`, where they are marked "verified in-limit and
self-collision-free against the current CAD export".

| Pose | Group | Values |
|---|---|---|
| `home` | arm | all arm joints 0 |
| `pose_1` | arm | shoulder_pan 1.5, shoulder_lift 1.0, elbow_flex 0, wrist_flex −1.0, wrist_roll 0 |
| `open` | gripper | gripper −0.59 |
| `closed` | gripper | gripper 0.19 |

## 4. Actuators

| Item | MG996R | SG90 | Source |
|---|---|---|---|
| Quantity | 3 (pan, lift, elbow) | 3 (wrist flex, wrist roll, gripper) | `ava.control.xacro`, `tools/torque_monitor.py` |
| Stall torque used in torque checks | 11.0 kg·cm = 1.079 N·m (TowerPro datasheet at 6 V; 9.4 kg·cm at 4.8 V) | 1.6 kg·cm = 0.157 N·m (datasheet of the units in stock, 6 V) | `tools/torque_monitor.py`, `ava.control.xacro` |
| Continuous threshold used (stall / 2) | 0.539 N·m | 0.078 N·m | `tools/torque_monitor.py` (`status()`) |
| Datasheet of purchased units | Planned: dratek 123001 (plastic gears): 13 kg·cm @ 4.8 V, 15 kg·cm @ 6 V, 0.13 s/60° @ 6 V. Shop figures, unverified | In stock: dratek 897 ("TowerPro SG-90"): 1.2 kg·cm @ 4.8 V, 1.6 kg·cm @ 6 V, 0.12 s/60°, 14 g | Shop pages / datasheet PDF |
| Operating voltage | 6.0 V planned (4.8–7.2 V rated) | 6.0 V planned (3.0–7.2 V rated) | shop pages; not final |
| Supplier / part number | TBD | TBD | `hardware/bom.md` is empty |

## 5. Electronics and hardware interface

| Item | Value | Notes |
|---|---|---|
| Microcontroller | ESP32 DevKit (SDA GPIO21, SCL GPIO22) | `CHANGELOG.md`, `firmware/ava_servo_driver/src/main.cpp` |
| Servo driver board | PCA9685, I2C 0x40, 50 Hz (to be bought) | decided with the user |
| Firmware | `firmware/ava_servo_driver` (PlatformIO, Arduino): serial protocol `?` / `P u0..u5` / `D`, channels 0–5 = J1–J6. Written, parser host-tested, never flashed | |
| Power supply | TBD | |
| Wiring / schematic | TBD | `hardware/bom.md` is empty |
| Position feedback | TBD | |
| Emergency stop (physical) | TBD | Dashboard has an e-stop state (`AGENTS.md`); no physical e-stop recorded |

## 6. Software interfaces

| Item | Value | Source |
|---|---|---|
| ros2_control backends | `real`, `gazebo`, `mock_components`, `mujoco` (arg `ros2_control_hardware_type`, default `mock_components`) | `urdf/ava.urdf.xacro`, `urdf/ava.ros2control.xacro` |
| `real` plugin | `ava_hardware/AvaServoSystem`, USB serial `/dev/ttyUSB0` @ 115200 to the ESP32. Package parked (`COLCON_IGNORE`), not built yet | `urdf/ava.ros2control.xacro`, `CHANGELOG.md` |
| `gazebo` plugin | `gz_ros2_control/GazeboSimSystem` | `urdf/ava.ros2control.xacro` |
| Controllers (Gazebo) | `joint_state_broadcaster`, `joint_trajectory_controller` (position command), update_rate 100 Hz | `ava_description/config/gazebo_controllers.yaml` |
| Command topic | `/joint_trajectory_controller/joint_trajectory` | `AGENTS.md` |
| State topic | `/joint_states` | `AGENTS.md`, `tools/torque_monitor.py` |
| MoveIt groups | `arm` (chain `base_link` → `end_effector_link`), `gripper` (joint `gripper`) | `ava.srdf` |
| Dashboard link | rosbridge WebSocket, port 9090 | `AGENTS.md` |

## 7. Sensors

| Sensor | Status | Source |
|---|---|---|
| Overhead camera (ArUco, `ava_vision`) | Unused; extrinsics invalid after root-frame flip | `AGENTS.md`, `CHANGELOG.md` |
| Wrist camera | Frames only, placeholder extrinsics, no mount part in CAD | `urdf/ava.common.xacro` |
| Joint encoders / current sensing | TBD | |

## 8. Source conflicts (resolve before trusting either value)

| Topic | Conflict |
|---|---|
| Servo voltage | Supply voltage not final (6.0 V recommended); all torque values in the repo assume 6 V. |
| Hardware layer | `ava.ros2control.xacro` selects `ava_hardware/AvaServoSystem`, but that package is parked and unbuilt; firmware is written but never flashed. |
| `ava.urdf` tracking | `ava_description/README.md` says "Committed to git"; `.gitignore` and `AGENTS.md` say it is git-ignored. |
| Tool script names | `AGENTS.md` lists `urdf_diff.py` and `mesh_measure.py`; the repo has `tools/compare_urdf.py` and `tools/measure_mashes.py`, both empty. |
| Servo model name | `CHANGELOG.md` §2 mentions re-measuring an "MG995" drawing; every other source says MG996R. |
