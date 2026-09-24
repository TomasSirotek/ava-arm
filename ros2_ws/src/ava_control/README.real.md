# ava_control

The bridge between ROS 2 and the real AVA arm hardware (ESP32 + PCA9685 servo board).
Built in **Part 3** of the series:
[MoveIt 2 Tutorial — Motion Planning and Real Robot Control](https://omartronics.com/moveit2-motion-planning-real-robot-tutorial/).

## Architecture

```
MoveIt (/move_action)
   └─ follow_joint_trajectory_bridge      # exposes the FollowJointTrajectory action
        └─ topic /joint_trajectory_controller/joint_trajectory
             └─ hardware_bridge_node      # takes the trajectory's FINAL point
                  └─ resource_manager     # serial (or WiFi/HTTP) transport
                       └─ ESP32 firmware  # MOVE:d1,..,d6 / HOME / SPEED:n  → servos
```

- The firmware ramps every servo toward its target at `movementSpeed` degrees per
  80 ms tick. **That ramp — not MoveIt's trajectory timing — sets the real speed.**
- The firmware `SPEED` setting is global for all servos, so the bridge switches it per
  command: joint (arm) trajectories run at the slow, smooth speed **5**; gripper-only
  trajectories run at speed **20** so the gripper snaps open/closed quickly.
- Without encoders, the bridge publishes its *commanded* state on `/joint_states`.
  `start_at_ros_zero:=true` therefore sends the arm to the ROS-zero pose on startup so
  the software state and the physical arm agree.

## Key files

| File | Purpose |
|---|---|
| `hardware_bridge_node.py` | Trajectory topic → servo commands, `/joint_states`, startup pose, per-move SPEED |
| `follow_joint_trajectory_bridge.py` | Action façade so MoveIt can execute on hardware |
| `resource_manager.py` / `servo_calibration.py` | Transport + rad ↔ servo-degree mapping (`deg = 90 + dir·(rad − zero)·57.3`) |
| `config/hardware.yaml` | Serial port, servo calibration (`zero_rad`, `direction`, `scale`) |

## Usage

Normally started through `ava_moveit_config/launch/real_moveit.launch.py`:

```bash
ros2 launch ava_moveit_config real_moveit.launch.py \
    transport:=serial use_rviz:=true start_at_ros_zero:=true home_on_start:=false
```

## Practical notes

- The serial port (`/dev/ttyUSB0`) is **exclusive**. If the bridge dies right after
  start ("process has died"), something else holds the port — the Arduino IDE serial
  monitor is the classic culprit. Check with `fuser /dev/ttyUSB0`.
- After a robot-side power loss, restart the launch: the bridge re-syncs the arm to the
  ROS-zero pose and re-applies the speed setting.
- `hardware.yaml → servo_calibration` maps ROS joint angles to servo degrees. The
  gripper (`J6`) uses `direction: -1`; with the SRDF poses this maps *open* to servo
  180° and *closed* to servo 0° (full mechanical range).
