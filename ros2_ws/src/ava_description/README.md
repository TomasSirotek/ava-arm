# ava_description

URDF/xacro model and STL meshes for **Ava**, a 6-DOF desktop arm. The model is an
in-house Fusion 360 design with MG996R servos on the shoulder and elbow and SG90s from the
wrist out (elbow mass approximated until the CAD is updated).
This package describes the robot and nothing else: it depends on no other `ava_*` package.

## URDF files

`urdf/ava_robotic_arm.urdf` is the CAD export; `urdf/ava.kinematics.xacro` and
`urdf/ava.urdf` are generated from it. Never edit those three by hand. The other xacro
files are hand-edited and add everything the exporter does not model.

| File | What it does |
|---|---|
| `urdf/ava_robotic_arm.urdf` | **CAD export** (Fusion 360). 45 links, 44 joints, all 45 STLs. Replaced wholesale on every design change. Not used directly by ROS. |
| `urdf/ava.kinematics.xacro` | **Generated** from the export by `scripts/generate_kinematics_xacro.py`: mesh paths become `package://`, cosmetic collisions are stripped, joint limits become references into `ava.control.xacro`. |
| `urdf/ava.urdf.xacro` | **Entry point.** Declares the xacro args, includes the kinematics, and adds the frames the exporter does not model. This is the file you pass to `xacro`. |
| `urdf/ava.common.xacro` | Geometry constants: colours, base mount height, tool frame, wrist-camera poses. |
| `urdf/ava.control.xacro` | Servo parameters: PCA9685 channels, pulse calibration, velocity limits, effort ceilings and joint limits. |
| `urdf/ava.ros2control.xacro` | The `<ros2_control>` block, selected by `ros2_control_hardware_type` (`mock_components` default, `real`, `gazebo`, `mujoco`). |
| `urdf/ava.gazebo.xacro` | gz sim plugin that runs ros2_control inside Gazebo. Only included when `ros2_control_hardware_type:=gazebo`. |
| `urdf/ava.urdf` | **Generated** plain URDF (`mock_components`) for tools that cannot run xacro: web dashboard, MuJoCo, viewers. Not tracked in git. |

### After a new CAD export

Drop the new `ava_robotic_arm.urdf` and STLs in, then regenerate both derived files:

```bash
# 1. CAD export -> ava.kinematics.xacro (no ROS needed)
ros2_ws/src/ava_description/scripts/generate_kinematics_xacro.py

# 2. xacro -> ava.urdf (runs check_urdf and refuses to write an invalid file)
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
colcon build --packages-select ava_description   # $(find) resolves to the install space
ros2_ws/src/ava_description/scripts/generate_urdf.sh
```

Then check by hand:

- **Joint limits:** copy changed limits into `ava.control.xacro`; SRDF named poses must stay inside them.
- **Root frame:** the export is Z-down; `world_to_base` rotates π about X and lifts by `${base_mount_z}`.
- **`STRUCTURAL_LINKS`** in the generator: the links that keep collision geometry.
- **SRDF collision matrix**, if link adjacency changed.

`move_group`'s 38 "no collision geometry" warnings are expected.

Regenerate `ava.urdf` after any hand edit to the xacro too (step 2 alone).

## Joints

| Joint | Moves | Servo | PCA9685 channel |
|---|---|---|---|
| `shoulder_pan` | whole arm about the vertical axis | MG996R | 0 |
| `shoulder_lift` | upper arm | MG996R | 1 |
| `elbow_flex` | lower arm | MG996R | 2 |
| `wrist_flex` | wrist pitch | SG90 | 3 |
| `wrist_roll` | gripper roll | SG90 | 4 |
| `gripper` | moving jaw | SG90 | 5 |

Joint limits are asymmetric and come from CAD (e.g. `shoulder_lift` −0.21…+2.09 rad).

## Other contents

| Path | Purpose |
|---|---|
| `meshes/*.stl` | Visual/collision meshes from the Fusion 360 model (supplied locally, not tracked) |
| `scripts/generate_kinematics_xacro.py` | CAD export → `ava.kinematics.xacro` |
| `scripts/generate_urdf.sh` | xacro → generated `ava.urdf` |
| `config/gazebo_controllers.yaml` | Controllers loaded by the Gazebo plugin |
| `config/*.rviz` | Saved RViz views |
| `launch/display.launch.py` | Model + joint sliders in RViz |
| `launch/rviz_sim.launch.py` | RViz-only kinematic view |
| `launch/gazebo.launch.py` | Gazebo Harmonic: spawn robot, load controllers |

```bash
ros2 launch ava_description display.launch.py
```
