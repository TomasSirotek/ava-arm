# ava_servo_driver: ESP32-C3 firmware

Puts the pulse widths the ROS driver sends over USB serial onto the PCA9685's servo channels.
No calibration or motion logic; the ROS driver (`ava_hardware`) does that.

## Where it sits

```
PC (ROS 2)                         ESP32-C3-DevKitM-1            PCA9685            servos
ava_hardware/AvaServoSystem  --USB serial-->  this firmware  --I2C-->  16-ch PWM  --PWM--> ch 0-5
(joint angles -> pulse widths)     115200 8N1, text lines         0x40, 50 Hz
```

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

## Serial protocol (v1)

USB serial at **115200 baud**, ASCII lines ending in `\n` (`\r` is ignored). The driver's settings
are in `ros2_ws/src/ava_description/urdf/ava.urdf.xacro` (`serial_port`, default
`/dev/ttyUSB0`, `baud_rate`) and `ava.control.xacro`.

| PC sends | Firmware does | Firmware replies |
|---|---|---|
| `?` | nothing else | `AVA_SERVO 1 6` (protocol version 1, 6 channels) |
| `P u0 u1 u2 u3 u4 u5` | sets the pulse of channels 0..5, in microseconds; `0` = channel off (no pulse) | nothing |
| `D` | switches all channels off (servos go limp) | nothing |
| anything else | ignores it | `ERR <short reason>` (the driver logs it as a warning) |

- At boot all channels are off and the firmware prints `AVA_SERVO 1 6 READY` once.
- The last `P` pulses are held until the next `P`, a `D` or power-off.
- The firmware clamps pulses to 500–2500 µs.

## Code structure

```
ava_servo_driver/
├── platformio.ini                    board env (esp32c3) + PC test env (native)
├── include/board_config.h            this board: baud, I2C pins GPIO0/1, PCA9685 0x40, 50 Hz, oscillator
├── src/main.cpp                      the only place the parts are created and connected
├── lib/
│   ├── ava_protocol/                 plain C++: protocol v1 text -> commands
│   │   └── src/ava_protocol/
│   │       ├── protocol_config.h     version 1, 6 channels, 500-2500 µs clamp, max line length
│   │       ├── command.h             the Command type (Query / SetPulses / Disable / Invalid)
│   │       ├── parser.*              one line -> Command, pulse clamping
│   │       └── line_buffer.*         characters -> lines, ignores \r, flags too-long lines
│   ├── ava_firmware/                 plain C++: what the firmware does
│   │   └── src/ava_firmware/
│   │       ├── pwm_output.h          interface to the PWM hardware (no Arduino types)
│   │       └── servo_firmware.*      boot (all off, READY), replies, pulses, ERR lines
│   └── pca9685_output/               Arduino: PwmOutput on a PCA9685
│       └── src/pca9685_output/
│           ├── pca9685_config.h      Pca9685Config: pins, address, frequency, oscillator
│           └── pca9685_output.*      Adafruit PWM Servo Driver; reports a missing chip
└── test/
    ├── test_protocol/                parser + line buffer
    └── test_firmware/                behaviour against a fake PwmOutput
```

| To change | Edit |
|---|---|
| ESP32 pins, PCA9685 address, oscillator calibration, baud | `include/board_config.h` |
| Number of channels, pulse clamp (must match `ava_hardware`) | `lib/ava_protocol/src/ava_protocol/protocol_config.h` |
| Other PWM hardware | a new class implementing `ava_firmware/pwm_output.h`, created in `src/main.cpp` instead of `Pca9685Output` |
| More than 16 channels | a second `Pca9685Config` (e.g. address 0x41) and a small `PwmOutput` that splits channels across the two boards |
| A new command | `command.h` + `parser.cpp`, then `servo_firmware.cpp`, with tests in both suites |

## Build, test, flash

```bash
cd firmware/ava_servo_driver
pio test -e native            # unit tests on the PC, no board needed
pio run -e esp32c3            # build
pio run -e esp32c3 -t upload  # flash (close any serial monitor and the ROS launch first)
pio device monitor --echo     # talk to it by hand at 115200
```

## Bring-up

1. Wiring as above, servos unplugged. PCA9685 V+ from the servo supply, **measured first**:
   5 V for SG90-only tests, 6.0 V (XL4016) once MG996Rs are connected. Never above 6.0 V.
2. Flash, open the monitor, press RESET: `AVA_SERVO 1 6 READY`. An `ERR PCA9685 not found` line
   means the I2C wiring (SDA/SCL swapped, VCC or GND missing) or a different address.
3. One servo, horn only: `P 1500 0 0 0 0 0` (channel 0), then `P 1000 0 0 0 0 0`,
   `P 2000 0 0 0 0 0`, then `D`. For another channel move the number: `P 0 0 0 1500 0 0` = ch 3.
4. Oscillator calibration: find the commanded value at which a servo reaches a known true end point,
   then set `oscillator_hz` in `board::kPca9685` (`include/board_config.h`) to `25000000 × commanded / true`.
5. Then the ROS driver: `ros2 launch ava_hardware real_robot.launch.py serial_port:=<port>`.
   Per-joint calibration is in `ros2_ws/src/ava_description/urdf/ava.control.xacro`.
