# ava_description

URDF/Xacro model, STL meshes and RViz/Gazebo bringup for the AVA arm 5-DOF arm.
This is the package built in **Part 2** of the series:
[ROS 2 URDF Tutorial — Visualize and Simulate a Robot Arm](https://omartronics.com/ros2-urdf-tutorial-robot-arm-rviz-gazebo/).

## Contents

| Path | Purpose |
|---|---|
| `urdf/ava.xacro` | Main robot description: links, joints, limits, the `tcp` grasp frame, `ros2_control` block |
| `urdf/ava.gazebo` / `.ros2control` / `materials.xacro` | Gazebo plugins, controller hardware interface, colors |
| `meshes/*.stl` | 8 visual/collision meshes exported from Fusion 360 |
| `config/gazebo_controllers.yaml` | `joint_state_broadcaster` + `joint_trajectory_controller` (+ PID gains) for Gazebo |
| `config/*.rviz` | Preconfigured RViz views |
| `launch/display.launch.py` | Model + joint sliders in RViz |
| `launch/gazebo.launch.py` | Gazebo Harmonic: spawn robot, load controllers, clock bridge |
| `launch/rviz_sim.launch.py` | RViz-only kinematic simulation |
| `gripper_mimic_bridge.py` | Mirrors `Revolute 6` into `Revolute 7` for topic-based gripper commands that omit the second finger |

## Usage

```bash
# Look at the model, move joints with sliders
ros2 launch ava_description display.launch.py

# Full physics simulation (Gazebo Harmonic)
ros2 launch ava_description gazebo.launch.py
```

## Notes on the model

- Joint names are `Revolute 1`–`Revolute 5` (arm) and `Revolute 6`/`Revolute 7` (gripper
  fingers). `world` == `base_link`; the table top is at `z = 0`.
- The fixed **`tcp` frame** sits at the fingertip midpoint (9 cm along the gripper X axis)
  and is rotated so **tcp +Z is the finger/approach axis** — the pick-and-place IK
  (Part 4) aligns this axis vertically for top-down grasps.
- Gripper joint limits are `-1.6 … 1.6 rad` so both the simulation gripper poses and the
  full real servo range (0–180°) are representable.
