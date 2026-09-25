# ava_description

URDF/xacro model and STL meshes for **Ava**, a 6-DOF SO-101 (SO-ARM100 family) arm.
This package describes the robot and nothing else: it depends on no other `ava_*` package.

## URDF files

Edit the xacro files. Never edit `ava.urdf` by hand.

| File | What it does |
|---|---|
| `urdf/ava.urdf.xacro` | **Entry point.** Declares the xacro args, all links (with meshes) and joints, and includes the files below. This is the file you pass to `xacro`. |
| `urdf/ava.common.xacro` | Geometry constants: joint origins, colours, wrist-camera poses. |
| `urdf/ava.control.xacro` | Servo parameters: motor IDs, PID gains, deadband, torque, joint limits. |
| `urdf/ava.ros2control.xacro` | The `<ros2_control>` block. **The only part that differs between sim and real**, selected by `ros2_control_hardware_type` (`real`, `gazebo`, `mock_components`, `mujoco`). |
| `urdf/ava.gazebo.xacro` | gz sim plugin that runs ros2_control inside Gazebo. Only included when `ros2_control_hardware_type:=gazebo`. |
| `urdf/ava.urdf` | **Generated** plain URDF (`mock_components`) for tools that cannot run xacro: web dashboard, MuJoCo, viewers. Committed to git. |

Regenerate `ava.urdf` after any xacro change (it runs `check_urdf` and refuses to write an invalid file):

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2_ws/src/ava_description/scripts/generate_urdf.sh
```

## Joints

| Joint | Moves | Servo ID |
|---|---|---|
| `shoulder_pan` | whole arm about the vertical axis | 1 |
| `shoulder_lift` | upper arm | 2 |
| `elbow_flex` | lower arm | 3 |
| `wrist_flex` | wrist pitch | 4 |
| `wrist_roll` | gripper roll | 5 |
| `gripper` | moving jaw | 6 |

Links use a `_link` suffix (`shoulder_link`, `upper_arm_link`, …). `end_effector_link` is the tool frame.

## Other contents

| Path | Purpose |
|---|---|
| `meshes/*.stl` | SO-101 visual/collision meshes (from TheRobotStudio SO-ARM100, Apache-2.0) |
| `config/gazebo_controllers.yaml` | Controllers loaded by the Gazebo plugin |
| `config/*.rviz` | Saved RViz views |
| `launch/display.launch.py` | Model + joint sliders in RViz |
| `launch/rviz_sim.launch.py` | RViz-only kinematic view |
| `launch/gazebo.launch.py` | Gazebo Harmonic: spawn robot, load controllers (moves to `ava_bringup` once sim is verified) |

```bash
ros2 launch ava_description display.launch.py
```
