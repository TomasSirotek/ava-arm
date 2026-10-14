# AVA Hardware Validation

Status: **Not started.** No hardware is connected (`README.md`, `CHANGELOG.md` §3).

Fill in each section as tests are run. Record measured values and the instrument used.
`TBD` = not yet measured or decided. See `robot_specification.md` for nominal values and
known source conflicts.

## 1. Blockers before hardware testing

Taken from `CHANGELOG.md` ("Probably next, once the servos are settled"):

- [ ] Servo layout passes simulation validation (`simulation_validation.md`).
- [ ] Hardware layer finished: un-park `ros2_ws/src/ava_hardware` (remove `COLCON_IGNORE`),
      build and smoke-test it (see the CHANGELOG TODO).
- [ ] Servos bought and wired (PCA9685 channels 0–5 = J1–J6, 6.0 V servo supply).
- [ ] Firmware flashed: `firmware/ava_servo_driver` (written, never flashed).
- [ ] BOM recorded: `hardware/bom.md` is currently empty.

## 2. Test setup

| Item | Value |
|---|---|
| Date | TBD |
| Tester | TBD |
| Git commit | TBD |
| Microcontroller | ESP32 (per `CHANGELOG.md`); board/variant TBD |
| Servo driver | TBD |
| Firmware version | TBD |
| Power supply (model, voltage, current limit) | TBD |
| Servo supply voltage measured at the servo | TBD |
| Emergency stop method | TBD |
| ROS launch command used | TBD |

## 3. Safety checklist (before each session)

- [ ] Power supply current limit set: TBD A
- [ ] Emergency stop tested and reachable
- [ ] Arm clamped / base secured
- [ ] Work area clear inside full reach (reach: TBD)
- [ ] Only the joint under test is powered (single-joint bring-up)

## 4. Per-joint bring-up

`CHANGELOG.md` plans to bring up one joint at a time against the ESP32.
Software limits are from `urdf/ava.control.xacro`.

| Joint | Servo | Channel / ID | Direction | Zero offset (rad) | SW limits (rad) | Measured range (rad) | Moves smoothly? | Result |
|---|---|---|---|---|---|---|---|---|
| `shoulder_pan` | MG996R | TBD | TBD | TBD | −1.570796 … 1.570796 | TBD | TBD | TBD |
| `shoulder_lift` | MG996R | TBD | TBD | TBD | −0.212536 … 2.094395 | TBD | TBD | TBD |
| `elbow_flex` | SG90 | TBD | TBD | TBD | −1.523672 … 1.617920 | TBD | TBD | TBD |
| `wrist_flex` | SG90 | TBD | TBD | TBD | −2.617994 … 0.523599 | TBD | TBD | TBD |
| `wrist_roll` | SG90 | TBD | TBD | TBD | −1.963495 … 1.178097 | TBD | TBD | TBD |
| `gripper` | SG90 | TBD | TBD | TBD | −0.594434 … 0.197952 | TBD | TBD | TBD |

## 5. Named-pose check

Poses from `ava_moveit_config/config/ava.srdf`. Compare commanded vs. physical.

| Pose | Commanded | Physical error | Self-collision observed? | Result |
|---|---|---|---|---|
| `home` | all 0 | TBD | TBD | TBD |
| `pose_1` | pan 1.5, lift 1.0, elbow 0, wrist_flex −1.0, roll 0 | TBD | TBD | TBD |
| gripper `open` | −0.59 | TBD | TBD | TBD |
| gripper `closed` | 0.19 | TBD | TBD | TBD |

## 6. Load test (hardware)

Compare against the simulation results for the same pose and payload.

| Pose | Payload (kg) | Joint | Holds position? | Servo temperature after TBD s | Supply current (A) | Sim status (OK/HOT/FAIL) | Result |
|---|---|---|---|---|---|---|---|
| TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

## 7. Calibration results

| Item | Value | Method |
|---|---|---|
| Per-joint zero offsets | TBD | TBD |
| Per-joint direction | TBD | TBD |
| Tool frame (`end_effector_link`) vs. physical grasp point | TBD | `ava.urdf.xacro` says to verify against hardware before fine placement |
| Wrist camera extrinsics | TBD | No mount part in CAD yet (`CHANGELOG.md`) |
| Overhead camera extrinsics | TBD | Existing `camera_extrinsics.yaml` is invalid after the root-frame flip (`CHANGELOG.md`) |

## 8. Issues found

| # | Date | Joint / subsystem | Description | Action | Status |
|---|---|---|---|---|---|
| | | | | | |
