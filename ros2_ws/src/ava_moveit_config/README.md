# ava_moveit_config

MoveIt 2 configuration for Ava, plus an ArUco pick-and-place demo derived from
OmArm Zero (see `ATTRIBUTION.md`).

## Configuration

| File | Purpose |
|---|---|
| `config/ava.srdf` | Groups `arm` (`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`; tip `end_effector_link`) and `gripper` (`gripper`); named states `home`, `pose_1`, `open`, `closed` |
| `config/kinematics.yaml` | KDL solver with `position_only_ik: true` |
| `config/joint_limits.yaml` | Planning velocity/acceleration limits |
| `config/ros2_controllers.yaml`, `config/moveit_controllers.yaml` | One `joint_trajectory_controller` for all 6 joints |
| `launch/demo.launch.py` | MoveIt + RViz with mock hardware |
| `launch/gazebo_moveit.launch.py` | MoveIt on top of the Gazebo simulation |
| `launch/real_moveit.launch.py` | MoveIt on the real arm (needs `ava_hardware`, not built yet) |

```bash
ros2 launch ava_moveit_config demo.launch.py
```

## Pick-and-place demo

| File | Purpose |
|---|---|
| `scripts/cup_pick_and_place.py` | The pick-and-place task (waits for `/cup_pose`) |
| `scripts/arm_ik.py` | Custom IK used by the task |
| `scripts/gazebo_grasp_attach.py` | Attaches the cube to the gripper in Gazebo |
| `scripts/spawn_cube.py` | Spawns the Gazebo cube at the detected position |
| `scripts/planned_path_line.py`, `scripts/publish_cup_marker.py` | RViz helpers |
| `models/table/`, `models/cube_marker/` | Gazebo models of the table and cube |
| `config/pick_place_params.yaml` | Simulation parameters |
| `config/pick_place_params_real.yaml` | Real-arm parameters |

Simulation (Gazebo robot, real overhead camera from `ava_vision`):

```bash
ros2 launch ava_moveit_config cube_gazebo.launch.py \
    camera_info_url:=<path>/calibration/camera_calibration_<stamp>.yaml \
    run_demo:=true
```

`cube_real.launch.py` runs the demo on the real arm with `pick_place_params_real.yaml`.
Never use the simulation parameter file on the real arm.

> **Safety:** on activation the real arm moves straight to the all-zero pose; clear the
> workspace first.

## Known limitations

- Pick-and-place scripts (`arm_ik.py`, `cup_pick_and_place.py`) are not yet ported to
  the current 6-joint arm.
- Velocity/acceleration limits in `joint_limits.yaml` are not yet tuned for the servos.
