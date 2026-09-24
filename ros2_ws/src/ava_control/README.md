# ava_control

Gamepad teleoperation for the AVA arm (USB/Bluetooth controllers via `joy`).

```bash
ros2 launch ava_control gamepad_control.launch.py
```

- Left stick / right stick / triggers move the arm in joint space, buttons open and
  close the gripper — see `config/gamepad.yaml` for the axis/button mapping and speeds.
- Works against any running controller stack: the Gazebo simulation
  (`ava_description gazebo.launch.py`) or the real arm
  (`ava_moveit_config real_moveit.launch.py`).
- `gamepad_calibrate_node` (run manually) helps identify the axis indices of your
  specific controller and writes them into the config.
