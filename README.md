# ava-arm

[![CI](https://github.com/TomasSirotek/ava-arm/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/TomasSirotek/ava-arm/actions/workflows/ci.yml)
[![Dashboard CI](https://github.com/TomasSirotek/ava-dashboard/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/TomasSirotek/ava-dashboard/actions/workflows/ci.yml)

A small 6-DOF desktop arm running ROS 2 Jazzy, built in Gazebo first and on hardware
second.

The arm started on an off-the-shelf SO-101 model. That model failed the Gazebo load
test — the weight and the payload sit at the end of the arm, so the shoulder and elbow
wanted bigger and more expensive servos than the budget allowed, and shortening the
links barely helped. So the arm was **redrawn from scratch in Fusion 360** (with help
from a Fusion designer) around hobby servos: **3× MG996R** on the shoulder and elbow,
**3× SG90** from the wrist out. (The first budget was 2× MG996R + 4× SG90; the elbow SG90
could not hold the arm in Gazebo, so the elbow is now an MG996R. The simulation already
uses it; the CAD is being redrawn.)

> **Status:** the new model runs in RViz, Gazebo and MoveIt, and the web dashboard shows
> and drives it live. The 3× MG996R + 3× SG90 layout holds a **20 g** payload in
> simulation. The Fusion 360 design is still under construction (elbow mount being
> redrawn, more changes to come). No hardware connected yet. See
> [CHANGELOG.md](CHANGELOG.md).


## The arm model files are supplied locally

The model is kept outside the repo, so these files are git-ignored:

| Not committed | What it is |
|---|---|
| `ava_description/urdf/ava_robotic_arm.urdf` | the Fusion 360 export |
| `ava_description/urdf/ava.kinematics.xacro` | generated from the export |
| `ava_description/urdf/ava.urdf` | generated from the xacro |
| `ava_description/meshes/*.stl` | the part meshes |
| `hardware/stl/`, `hardware/step/` | printable and CAD exports |

The two generated files are excluded alongside the export, because they carry the same
geometry.

Everything else is tracked, including all the hand-written xacro (`ava.urdf.xacro`,
`ava.common.xacro`, `ava.control.xacro`, `ava.ros2control.xacro`, `ava.gazebo.xacro`).
**A fresh clone will not launch the robot** until the model files are dropped in and
regenerated — see [ava_description](ros2_ws/src/ava_description/README.md).

## Layout

| Path | What |
|---|---|
| `ros2_ws/src/ava_description` | URDF/xacro, meshes. Depends on nothing. |
| `ros2_ws/src/ava_control` | Gamepad teleop nodes. |
| `ros2_ws/src/ava_moveit_config` | MoveIt 2 config. |
| `ros2_ws/src/ava_vision` | ArUco perception. |
| `ros2_ws/src/ava_bringup` | Top-level launch (stub). |
| `ros2_ws/src/ava_hardware` | ros2_control plugin for the real arm (ESP32 + PCA9685 over USB serial); parked, not built yet. |
| `LICENSES/` | Third-party license texts (see [ATTRIBUTION.md](ATTRIBUTION.md)). |
| `dashboard/` | Web dashboard (git submodule, [ava-dashboard](https://github.com/TomasSirotek/ava-dashboard)). |
| `firmware/` | ESP32-C3 servo driver: empty PlatformIO project plus a guide to what it must do; to be written by hand once the parts arrive. |
| `hardware/` | Bill of materials (`bom.md`); STEP/STL exports and the KiCad project are local, not tracked. |
| `tools/` | Offline scripts: torque monitor (`compare_urdf.py`, `measure_mashes.py` are empty placeholders). |
| `tests/` | Repo-level checks that the configs of all packages agree (pytest, no ROS needed). |

Dependency direction: bringup -> control -> kinematics -> nothing.

## Clone

The dashboard is a submodule, so clone with `--recurse-submodules`:

```bash
git clone --recurse-submodules git@github.com:TomasSirotek/ava-arm.git
```

Already cloned without it:

```bash
git submodule update --init
```

Pull the latest of both:

```bash
git pull --recurse-submodules
```

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Run

Use a normal terminal, not VS Code's integrated one (the VS Code snap breaks RViz/Gazebo).
Source both setups first: `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`.

| What | Command | Details |
|---|---|---|
| Model + sliders in RViz | `ros2 launch ava_description display.launch.py` | [ava_description](ros2_ws/src/ava_description/README.md) |
| Gazebo + MoveIt + RViz | `QT_QPA_PLATFORM=xcb ros2 launch ava_moveit_config gazebo_moveit.launch.py rviz_use_sim_time:=true` | [ava_moveit_config](ros2_ws/src/ava_moveit_config/README.md) |
| Bridge for the dashboard | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` | needs `ros-jazzy-rosbridge-suite` |
| Dashboard | `cd dashboard && pnpm dev` | [dashboard](dashboard/README.md) |
| Torque check under load | `ros2 launch ava_description gazebo.launch.py payload_kg:=0.1` then `python3 tools/torque_monitor.py` | the current load test |

## Tests and CI

Repo-level checks (no ROS needed): the configs of all packages must agree.

```bash
pip install -r tests/requirements.txt
python3 -m pytest tests
```

| Test file | Checks |
|---|---|
| `tests/test_joint_config.py` | the same six joints in every controller config (Gazebo, MoveIt, real arm); MoveIt velocity limits within the hardware driver's rate limit; SRDF named poses inside the joint limits |
| `tests/test_servo_calibration.py` | the real-arm calibration in `ava.control.xacro`, with the driver's maths: every joint's whole range maps inside the 500–2500 µs pulse clamp, channels 0–5 = J1–J6, directions ±1 |
| `tests/test_torque_monitor.py` | `tools/torque_monitor.py` uses the URDF effort limits as stall torque; OK / HOT / FAIL boundaries |

GitHub Actions ([`.github/workflows/ci.yml`](.github/workflows/ci.yml)) runs on every push to
`main` and on pull requests:

| Job | What |
|---|---|
| Config checks | the pytest suite above |
| Dashboard against this repo | the dashboard submodule's unit tests, including its ROS-contract tests read from `ros2_ws/src/ava_description` (they are skipped in the dashboard repo's own CI); fails if those were skipped |

**Dashboard pointer:** Dependabot ([`.github/dependabot.yml`](.github/dependabot.yml)) checks
ava-dashboard's `main` daily and opens a PR moving `dashboard/` to its newest commit. Both jobs
run on it and [`dependabot-auto-merge.yml`](.github/workflows/dependabot-auto-merge.yml) merges it
when they pass; a failing update stays open for a look. Workflow-action updates come weekly and
are merged by hand.

**Branch protection** ([`.github/rulesets/main.json`](.github/rulesets/main.json), applied in the
repo settings, same as the dashboard): `main` takes pull requests with both jobs green, no
force-push or deletion; repository admins can bypass and push directly.

Not in CI yet: building the ROS packages with colcon (they need the git-ignored model files) and
the firmware (written by hand later).

## License

Upstream sources and their licenses: [ATTRIBUTION.md](ATTRIBUTION.md), texts in [LICENSES/](LICENSES/).
