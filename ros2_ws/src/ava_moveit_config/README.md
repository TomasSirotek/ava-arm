# ava_moveit_config

> **SO-101 migration in progress.** `.setup_assistant`, `ava.srdf` and `joint_limits.yaml`
> are updated for SO-101 and Gazebo + MoveIt runs. Files marked `TODO(so101-migration)`
> (`arm_ik.py`, `cup_pick_and_place.py`) still describe the old 5-DOF OmArm arm.

MoveIt 2 configuration for the AVA arm **plus** the complete computer-vision
pick-and-place task. Built in **Part 3**
([MoveIt 2 Tutorial](https://omartronics.com/moveit2-motion-planning-real-robot-tutorial/))
and extended in **Part 4** (`docs/part4-computer-vision-pick-and-place.md`).

## MoveIt configuration (Part 3)

| File | Purpose |
|---|---|
| `config/ava.srdf` | Groups `arm` (Rev 1–5) and `gripper` (Rev 6–7), named states `home`, `open`, `closed` |
| `config/kinematics.yaml` | KDL with `position_only_ik: true` (a 5-DOF arm cannot satisfy full 6-DoF poses) |
| `config/joint_limits.yaml` | Planning limits — gripper range covers sim poses *and* the real servo range |
| `config/ros2_controllers.yaml`, `config/moveit_controllers.yaml` | One `joint_trajectory_controller` drives all 7 joints |
| `launch/demo.launch.py` | MoveIt + RViz with mock hardware |
| `launch/gazebo_moveit.launch.py` | MoveIt on top of the Gazebo simulation |
| `launch/real_moveit.launch.py` | MoveIt + the ESP32 hardware bridge (real arm) |

> The SRDF named states `open`/`closed` are calibrated for the **real** gripper
> (open = servo 180°, closed = servo 0°). The simulation task uses its own values from
> the parameter file instead.

## Pick-and-place task (Part 4)

| File | Purpose |
|---|---|
| `scripts/cup_pick_and_place.py` | The task: waits for `/cup_pose`, flies over the cube, descends **linearly**, grasps, lifts linearly, places at the live-detected marker (with reachability tolerance), settles the cube on the table, returns home |
| `scripts/arm_ik.py` | Custom 5-DOF IK: position (3) + vertical approach axis (2) via damped least squares; prefers the neutral wrist-roll family; `quick=True` feasibility mode for scans |
| `scripts/gazebo_grasp_attach.py` | Simulation grasp: the cube model follows the `tcp` frame (50 Hz, via the bridged `SetEntityPose` service) while the gripper is commanded closed |
| `scripts/spawn_cube.py` | Spawns the Gazebo cube at the first `/cup_pose` from the real camera |
| `scripts/planned_path_line.py`, `scripts/publish_cup_marker.py` | RViz visualization helpers |
| `models/table/`, `models/cube_marker/` | Gazebo SDF models of the table (with corner markers) and the 3×3×10 cm cube |
| `config/pick_place_params.yaml` | **Simulation** task parameters |
| `config/pick_place_params_real.yaml` | **Real robot** task parameters (calibrated gripper, safe home, marker place target) |

### Run it — simulation (Gazebo robot + REAL camera)

```bash
ros2 launch ava_moveit_config cube_gazebo.launch.py \
    camera_info_url:=<path>/calibration/camera_calibration_<stamp>.yaml \
    run_demo:=true
```

This starts Gazebo + MoveIt + RViz, spawns the table and a cube at the position where
your **real overhead camera** sees the real cube, and runs the full pick cycle.
(`run_demo:=false` starts everything except the task — useful for checking `/cup_pose`
first. `cube_demo.launch.py` is an alternative with RViz mock hardware only.)

### Run it — real robot

Three terminals (see Part 4 for the full walk-through):

```bash
# 1 — robot (MoveIt + ESP32 bridge; arm moves to the ROS-zero pose on start)
ros2 launch ava_moveit_config real_moveit.launch.py \
    transport:=serial use_rviz:=true start_at_ros_zero:=true home_on_start:=false

# 2 — vision (real overhead camera -> /cup_pose)
ros2 launch ava_vision vision_bringup.launch.py \
    camera_info_url:=<path-to-your-intrinsics>.yaml

# 3 — the task, ALWAYS with the real parameter file
ros2 run ava_moveit_config cup_pick_and_place.py --ros-args \
    --params-file src/ava_moveit_config/config/pick_place_params_real.yaml
```

`cube_real.launch.py` bundles all three (it uses the real parameter file).

### The two parameter files — do not mix them

| Parameter | sim (`pick_place_params.yaml`) | real (`pick_place_params_real.yaml`) |
|---|---|---|
| `gripper_open` / `gripper_closed` | `-1.2` / `0.16` (Gazebo-tuned) | `-1.57` / `1.57` (= servo 180°/0°, full range) |
| `home_joints` | `0,0,0,0,0` | `0.8,-0.8,-1.2,-0.8,-2.6` — **safe** pose; all-zero drives real servos into their end stops |
| `place_z` (settle height) | `0.07` | `0.01` (fingers slide the cube firmly onto the table before opening) |
| `place_marker_frame` | — (fixed point) | `aruco_marker_4` — places at the live-detected marker, or as close as reachable |

### How the motion works (short version)

1. **Adaptive fly-over** — searches downward from `grasp_z + approach_height` for the
   highest vertically-reachable pre-grasp height (quick IK scan, full-solver fallback).
2. **Linear descent** — z is sampled in 1 cm steps, each solved with the previous step
   as IK seed and sent as its own single-point trajectory. The tcp stays exactly on the
   vertical line over the cube; the cube is never touched before the fingers close.
3. **Grasp, linear lift**, transport, **fly-over the place target**, linear settle,
   open, linear retreat, home.
4. Failure handling: unreachable positions abort *before any motion* with a clear
   message; Gazebo's transient `START_STATE_INVALID` (-26) is retried automatically.
