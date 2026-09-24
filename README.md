# ava-arm

A small 6-DOF desktop arm (SO-101-style kinematics) running ROS 2 Jazzy, built in
Gazebo first and on hardware second.

> Status: repo structure only. URDF, kinematics and nodes are not written yet.

## Layout

| Path | What |
|---|---|
| `ros2_ws/src/ava_description` | URDF/xacro, meshes. Depends on nothing. |
| `ros2_ws/src/ava_control` | Control nodes: gamepad teleop, hardware bridges. |
| `ros2_ws/src/ava_moveit_config` | MoveIt 2 config. |
| `ros2_ws/src/ava_vision` | ArUco perception. |
| `ros2_ws/src/ava_bringup` | Top-level launch (stub). |
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

See [ATTRIBUTION.md](ATTRIBUTION.md) for upstream sources and licenses.
