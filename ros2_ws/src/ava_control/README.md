# ava_control

Gamepad teleoperation for the AVA arm (USB/Bluetooth controllers via `joy`).

```bash
ros2 launch ava_control gamepad_control.launch.py
```

- Left stick / right stick / triggers move the arm in joint space, buttons open and
  close the gripper — see `config/gamepad.yaml` for the axis/button mapping and speeds.
- Works against any running controller stack: the Gazebo simulation
  (`ava_description gazebo.launch.py`) or the real arm (see below).
- `gamepad_calibrate_node` (run manually) helps identify the axis indices of your
  specific controller and writes them into the config.

## Real hardware

This package contains no hardware drivers. The real arm (PWM servos on a PCA9685
driven by an ESP32 over USB serial) is controlled through the ros2_control plugin
in `ava_hardware`:

```bash
# ros2_control + joint_trajectory_controller only
ros2 launch ava_hardware real_robot.launch.py serial_port:=/dev/ttyUSB0
# or with MoveIt (includes the launch above)
ros2 launch ava_moveit_config real_moveit.launch.py serial_port:=/dev/ttyUSB0
```

Controller names and topics are identical to the simulation, so the gamepad node
works unchanged.
