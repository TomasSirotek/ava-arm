# ava_servo_driver: firmware guide

This folder is an empty PlatformIO project skeleton (`platformio.ini`, `src/main.cpp`, `lib/`,
`test/`) on purpose: the firmware is yours to write once the parts are here. This page says
**what** it has to do so that the ROS side can drive it, not **how** to write it.

## Where it sits

```
PC (ROS 2)                         ESP32-C3-DevKitM-1            PCA9685            servos
ava_hardware/AvaServoSystem  --USB serial-->  your firmware  --I2C-->  16-ch PWM  --PWM--> ch 0-5
(joint angles -> pulse widths)     115200 8N1, text lines         0x40, 50 Hz
```

The ROS driver (`ros2_ws/src/ava_hardware`) already turns joint angles into pulse widths in
microseconds and clamps them; the firmware only has to put those pulses on the right PCA9685
channels. The servos report nothing back, so there is nothing to read from them.

## Wiring

| ESP32-C3 | PCA9685 |
|---|---|
| GPIO0 | SDA |
| GPIO1 | SCL |
| 3V3 | VCC (logic) |
| GND | GND |

The servos get 6.0 V from the step-down converter into the PCA9685 **V+** screw terminal, never
from the ESP32 or USB. The ESP32 is powered by the USB cable only. Channel = joint:

| PCA9685 channel | Joint | Servo |
|---|---|---|
| 0 | J1 shoulder_pan | MG996R |
| 1 | J2 shoulder_lift | MG996R |
| 2 | J3 elbow_flex | MG996R |
| 3 | J4 wrist_flex | SG90 |
| 4 | J5 wrist_roll | SG90 |
| 5 | J6 gripper | SG90 |

## The serial protocol the ROS driver speaks (v1)

USB serial at **115200 baud**, ASCII lines ending in `\n` (ignore `\r`). The driver's settings
are in `ros2_ws/src/ava_description/urdf/ava.urdf.xacro` (`serial_port`, default
`/dev/ttyUSB0`, `baud_rate`) and `ava.control.xacro`.

| PC sends | Firmware does | Firmware replies |
|---|---|---|
| `?` | nothing else | `AVA_SERVO 1 6` (protocol version 1, 6 channels) |
| `P u0 u1 u2 u3 u4 u5` | sets the pulse of channels 0..5, in microseconds; `0` = channel off (no pulse) | nothing |
| `D` | switches all channels off (servos go limp) | nothing |
| anything else | ignores it | `ERR <short reason>` (the driver logs it as a warning) |

Behaviour the driver relies on:

- **At boot all channels are off** and the firmware prints `AVA_SERVO 1 6 READY` once.
- **Handshake:** opening the port resets the ESP32, so the driver keeps sending `?` every 250 ms
  for up to 4 s (`servo_handshake_timeout_ms`) until either reply line arrives. No reply means
  "Is the ESP32 connected to this port and flashed with the Ava servo firmware?".
- **The last `P` pulses are held** until the next `P`, a `D` or power-off. The driver only sends
  `P` when a pulse changes, so the firmware must not time out and switch off on its own.
- **If the board restarts** while running (USB glitch, brown-out) it prints the `READY` line again;
  the driver then re-sends the last pulses.
- **Pulse range:** the driver sends 500–2500 µs (or 0); clamping again on the firmware side is a
  good safety net.

## Things to learn along the way

1. PlatformIO: create the project for the board (`esp32-c3-devkitm-1`, Arduino framework), build,
   upload, serial monitor at 115200.
2. Serial: read characters into a line buffer, react on `\n`, split the line into words.
3. I2C on the C3: start it on GPIO0/GPIO1 (the C3 has no default pins like the classic ESP32).
4. PCA9685: 50 Hz means a 20 ms period split into 4096 steps, so one step is about 4.9 µs;
   a pulse of `us` microseconds is `us * 4096 / 20000` steps. The board's oscillator is not
   exactly 25 MHz; measuring it (or trimming by ear on a servo) makes the pulses accurate.
   A library such as Adafruit's PWM Servo Driver does the register work for you.
5. Bring-up order: one SG90 on channel 0 with the arm not attached; `?` by hand in the serial
   monitor; then `P 1500 0 0 0 0 0`; then `D`; only then the ROS driver
   (`ros2 launch ava_hardware real_robot.launch.py`, once `ava_hardware` is un-parked).

Testing on the PC: keep the line parsing in a plain C++ file without Arduino includes and it can be
unit-tested with PlatformIO's `native` environment (`pio test -e native`).
