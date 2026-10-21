# ava-arm

[![CI](https://github.com/TomasSirotek/ava-arm/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/TomasSirotek/ava-arm/actions/workflows/ci.yml)
[![Dashboard CI](https://github.com/TomasSirotek/ava-dashboard/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/TomasSirotek/ava-dashboard/actions/workflows/ci.yml)

A small 6-DOF desktop arm running ROS 2 Jazzy. In-house Fusion 360 design with 3× MG996R
(shoulder pan, shoulder lift, elbow) and 3× SG90 (wrist flex, wrist roll, gripper).

> **Status:** runs in RViz, Gazebo, MoveIt and the web dashboard. The CAD is still changing
> and no hardware is connected yet. See [CHANGELOG.md](CHANGELOG.md).

## Model files are not in the repo

The CAD export, the URDF/xacro generated from it, the meshes and the STL/STEP files are
git-ignored, so a fresh clone will not launch the robot. See
[ava_description](ros2_ws/src/ava_description/README.md).

## Layout

| Path | What |
|---|---|
| `ros2_ws/src/ava_description` | URDF/xacro, meshes. Depends on nothing. |
| `ros2_ws/src/ava_control` | Gamepad teleop nodes. |
| `ros2_ws/src/ava_moveit_config` | MoveIt 2 config. |
| `ros2_ws/src/ava_vision` | ArUco perception. |
| `ros2_ws/src/ava_bringup` | Top-level launch (stub). |
| `ros2_ws/src/ava_hardware` | ros2_control plugin for the real arm (not built yet). |
| `LICENSES/` | Third-party license texts (see [ATTRIBUTION.md](ATTRIBUTION.md)). |
| `dashboard/` | Web dashboard (git submodule, [ava-dashboard](https://github.com/TomasSirotek/ava-dashboard)). |
| `firmware/` | ESP32-C3 servo driver (PlatformIO): serial protocol v1 → PCA9685 pulses, with PC unit tests. |
| `hardware/` | Bill of materials (`bom.md`). |
| `tools/` | Torque monitor and helper scripts. |
| `tests/` | Repo-level checks that the configs of all packages agree (pytest, no ROS needed). |

## Clone and build

```bash
git clone --recurse-submodules git@github.com:TomasSirotek/ava-arm.git
# already cloned without submodules: git submodule update --init
cd ava-arm/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Run

Use a normal terminal, not VS Code's integrated one (the VS Code snap breaks RViz/Gazebo).
Source both setups first: `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`.

| What | Command |
|---|---|
| Model + sliders in RViz | `ros2 launch ava_description display.launch.py` |
| Gazebo + MoveIt + RViz | `QT_QPA_PLATFORM=xcb ros2 launch ava_moveit_config gazebo_moveit.launch.py rviz_use_sim_time:=true` |
| Bridge for the dashboard (needs `ros-jazzy-rosbridge-suite`) | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1` |
| Dashboard | `cd dashboard && pnpm dev` |

## Tests and CI

Config checks (no ROS needed):

```bash
pip install -r tests/requirements.txt
python3 -m pytest tests
```

Firmware unit tests (on the PC, no board needed):

```bash
cd firmware/ava_servo_driver
pip install -r requirements.txt
pio test -e native
```

GitHub Actions runs the config tests, the dashboard unit tests against this repo, and the firmware
tests plus an ESP32-C3 build. Dependabot keeps the dashboard submodule, actions and PlatformIO current.

## Security

- Meant for one machine: set `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `~/.bashrc`.
- Bind rosbridge to `127.0.0.1`; never expose port 9090 or ROS traffic to a network (no authentication).
- Software stops are not an emergency stop: keep the power switch within reach when hardware is connected.

## License

Copyright 2026 Tomas Sirotek. Licensed under the [Apache License 2.0](LICENSE), except where a file
or package states otherwise (`ava_description` is MIT). Upstream sources and their licenses:
[ATTRIBUTION.md](ATTRIBUTION.md), texts in [LICENSES/](LICENSES/).
