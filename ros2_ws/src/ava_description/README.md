# ava_description

URDF/xacro model and STL meshes for **Ava**, a 6-DOF desktop arm. The model is an
in-house Fusion 360 design built around 2× MG996R (shoulder) + 4× SG90 (elbow out).
This package describes the robot and nothing else: it depends on no other `ava_*` package.

## URDF files

Two files are generated and two are hand-edited. `urdf/ava_robotic_arm.urdf` comes from
CAD and `urdf/ava.kinematics.xacro` + `urdf/ava.urdf` are derived from it — never edit
any of those three by hand.

The CAD export is the source of truth for links, joints and limits. Everything the
exporter does not model — the root frame correction, the tool frame, `base_footprint`,
the payload, the wrist-camera frames, `ros2_control` — lives in the hand-edited xacro.

| File | What it does |
|---|---|
| `urdf/ava_robotic_arm.urdf` | **CAD export** (Fusion 360). 45 links, 44 joints, all 45 STLs. Replaced wholesale on every design change. Not used directly by ROS. |
| `urdf/ava.kinematics.xacro` | **Generated** from the export by `scripts/generate_kinematics_xacro.py`: mesh paths become `package://`, cosmetic collisions are stripped, joint limits become references into `ava.control.xacro`. |
| `urdf/ava.urdf.xacro` | **Entry point.** Declares the xacro args, includes the kinematics, and adds the frames the exporter does not model. This is the file you pass to `xacro`. |
| `urdf/ava.common.xacro` | Geometry constants: colours, base mount height, tool frame, wrist-camera poses. |
| `urdf/ava.control.xacro` | Servo parameters: motor IDs, PID gains, deadband, torque, joint limits. |
| `urdf/ava.ros2control.xacro` | The `<ros2_control>` block. **The only part that differs between sim and real**, selected by `ros2_control_hardware_type` (`real`, `gazebo`, `mock_components`, `mujoco`). |
| `urdf/ava.gazebo.xacro` | gz sim plugin that runs ros2_control inside Gazebo. Only included when `ros2_control_hardware_type:=gazebo`. |
| `urdf/ava.urdf` | **Generated** plain URDF (`mock_components`) for tools that cannot run xacro: web dashboard, MuJoCo, viewers. Committed to git. |

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

Then check by hand, because the exporter cannot know these:

- **Joint limits.** `generate_kinematics_xacro.py` points the six joints at
  `${..._lower}/${..._upper}` in `ava.control.xacro`; if the export's limits changed,
  copy the new values across. Named poses in `ava_moveit_config/config/ava.srdf` must
  stay inside them.
- **The root frame.** The export is authored **Z-down**, which `world_to_base` corrects
  by rotating π about X and lifting the base `${base_mount_z}`. Re-measure that offset if
  the base plate changed.
- **`STRUCTURAL_LINKS`** in the generator — the links that keep collision geometry. The
  script fails loudly if a CAD rename makes one of them disappear.
- **The SRDF collision matrix**, if link adjacency changed.

### Expected warnings

`move_group` logs 38 warnings like `Link bearing_outer_01 has visual geometry but no
collision geometry`. That is intended, not a bug: the generator strips collision from the
cosmetic links on purpose (see `STRUCTURAL_LINKS`). Adding collision geometry to silence
them would put ~990 pairs into the SRDF and slow planning down for parts that cannot
drive a collision anyway.

Regenerate `ava.urdf` after any hand edit to the xacro too (step 2 alone).

## Joints

| Joint | Moves | Servo ID |
|---|---|---|
| `shoulder_pan` | whole arm about the vertical axis | 1 |
| `shoulder_lift` | upper arm | 2 |
| `elbow_flex` | lower arm | 3 |
| `wrist_flex` | wrist pitch | 4 |
| `wrist_roll` | gripper roll | 5 |
| `gripper` | moving jaw | 6 |

Joint limits are asymmetric and come from CAD — notably `shoulder_lift` runs
−0.21…+2.09 rad, so poses written for the old model do not carry over.

The 7 structural links are `base_link`, `shoulder_link`, `upper_arm_link`,
`forearm_link`, `wrist_link`, `gripper_base_link`, `gripper_jaw_link`; these are the only
ones with collision geometry. The other 38 (bearings, spacers, servo bodies, horns,
covers) are visual-only. Each actuated joint's child in the export is an `axis_*`/`horn_*`
stub that the structural link is fixed to. `base_footprint` is the ground frame and
`end_effector_link` the tool frame (the grasp point between the jaws).

## Other contents

| Path | Purpose |
|---|---|
| `meshes/*.stl` | Visual/collision meshes exported from the in-house Fusion 360 model |
| `scripts/generate_kinematics_xacro.py` | CAD export → `ava.kinematics.xacro` |
| `scripts/generate_urdf.sh` | xacro → generated `ava.urdf` |
| `config/gazebo_controllers.yaml` | Controllers loaded by the Gazebo plugin |
| `config/*.rviz` | Saved RViz views |
| `launch/display.launch.py` | Model + joint sliders in RViz |
| `launch/rviz_sim.launch.py` | RViz-only kinematic view |
| `launch/gazebo.launch.py` | Gazebo Harmonic: spawn robot, load controllers (moves to `ava_bringup` once sim is verified) |

```bash
ros2 launch ava_description display.launch.py
```
