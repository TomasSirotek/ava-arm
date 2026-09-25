# ava-arm

A small 6-DOF desktop arm (SO-101-style kinematics) running ROS 2 Jazzy, built in
Gazebo first and on hardware second.

> Status: SO-101 model runs in RViz, Gazebo and MoveIt; the web dashboard shows and
> drives it live. Hardware (ESP32 + MG996R/SG90) not connected yet. See [CHANGELOG.md](CHANGELOG.md).

## Layout

| Path | What |
|---|---|
| `ros2_ws/src/ava_description` | URDF/xacro, meshes. Depends on nothing. |
| `ros2_ws/src/ava_control` | Control nodes: gamepad teleop, hardware bridges. |
| `ros2_ws/src/ava_moveit_config` | MoveIt 2 config. |
| `ros2_ws/src/ava_vision` | ArUco perception. |
| `ros2_ws/src/ava_bringup` | Top-level launch (stub). |
| `LICENSES/` | Third-party license texts (see [ATTRIBUTION.md](ATTRIBUTION.md)). |
| `dashboard/` | Web dashboard (git submodule, [ava-dashboard](https://github.com/TomasSirotek/ava-dashboard)). |
| `firmware/` | Microcontroller servo driver. |
| `hardware/` | BOM, STEP/STL, electronics. |
| `docs/` | Design notes and decision records. |
| `tools/` | Offline scripts (URDF diff, mesh measurement). |

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

## License

Upstream sources and their licenses: [ATTRIBUTION.md](ATTRIBUTION.md), texts in [LICENSES/](LICENSES/).
