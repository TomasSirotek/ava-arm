# ava_control

Gamepad teleoperation for the AVA arm (USB/Bluetooth controllers via `joy`).

```bash
ros2 launch ava_control gamepad_control.launch.py
```

- Left stick / right stick / triggers move the arm in joint space, buttons open and
  close the gripper — see `config/gamepad.yaml` for the axis/button mapping and speeds.
- Works against any running controller stack, e.g. the Gazebo simulation
  (`ava_description gazebo.launch.py`).
- `gamepad_calibrate_node` (run manually) helps identify the axis indices of your
  specific controller and writes them into the config.

## Real hardware

This package contains no hardware drivers. Real-arm use needs the ros2_control plugin in
`ava_hardware`, which is not built yet (`COLCON_IGNORE`).
