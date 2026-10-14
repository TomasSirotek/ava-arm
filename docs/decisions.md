# AVA Design Decisions

Lightweight decision log. Entries below are taken from `CHANGELOG.md`, `README.md` and source
comments; dates not recorded in the repo are `TBD`. `docs/decisions/0001-why-jazz.md` also
exists but is empty.

Status values: `Proposed` · `Accepted` · `Under validation` · `Superseded`.

## Template

```markdown
## D-NNN: <title>
- Date: YYYY-MM-DD
- Status: Proposed | Accepted | Under validation | Superseded by D-NNN
- Context: <problem and constraints>
- Decision: <what was chosen>
- Alternatives considered: <options rejected and why>
- Consequences: <what this changes, what must be updated>
- Evidence: <files, test results, measurements>
```

---

## D-001: Use ROS 2 Jazzy

- Date: 2026-09-28
- Status: Accepted
- Context: TBD (`docs/decisions/0001-why-jazz.md` is empty)
- Decision: ROS 2 Jazzy for the whole stack.
- Alternatives considered: TBD
- Consequences: Build/run commands source `/opt/ros/jazzy/setup.bash`.
- Evidence: `README.md`, `AGENTS.md`

## D-002: Replace the SO-101 model with a custom design

- Date: 2026-09-28
- Status: Accepted
- Context: The SO-101 model failed the Gazebo load test. Weight and payload sit at the end
  of the arm, so the shoulder and elbow needed servos beyond the budget. Shortening the
  links barely helped (confirmed with `tools/torque_monitor.py`). No ready-made model for
  2× MG996R + 4× SG90 was found.
- Decision: Redraw the arm from scratch in Fusion 360 around the fixed servo budget.
- Alternatives considered: Shorter SO-101 links (rejected: barely helped); another
  ready-made model (none found).
- Consequences: Joint names kept (`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
  wrist_roll, gripper`); link names and all joint limits changed; named poses re-verified.
- Evidence: `CHANGELOG.md` §1–2, `README.md`

## D-003: Servo layout 2× MG996R + 4× SG90

- Date: 2026-09-28
- Status: **Superseded by D-009** (elbow moved to MG996R)
- Context: Fixed servo budget.
- Decision: MG996R on `shoulder_pan` and `shoulder_lift`; SG90 on `elbow_flex`,
  `wrist_flex`, `wrist_roll`, `gripper`. Per-joint URDF effort limits replace the shared
  `max_torque_nm`.
- Alternatives considered (if the shoulder will not hold): smaller payload target,
  counterweight or spring, geared shoulder. Decide before buying anything.
- Consequences: Updated in URDF effort limits, `tools/torque_monitor.py`, dashboard
  torque card and Parts tab.
- Evidence: `urdf/ava.control.xacro`, `tools/torque_monitor.py`, `CHANGELOG.md` §2–3
- Resolved: no joint is geared; `shoulder_lift` uses the plain MG996R limit (1.079 N·m).

## D-004: Correct the Z-down CAD export in `world_to_base`

- Date: 2026-10-13
- Status: Accepted
- Context: The Fusion export is authored Z-down; ROS expects Z-up (REP-103). Without
  correction the arm hangs below the ground plane.
- Decision: `world_to_base` rotates π about X and lifts the base by `base_mount_z` = 0.0337 m.
- Consequences: `world` is now `base_footprint`; `base_link` sits 33.7 mm above it, rotated
  π about X. `ava_vision` assumptions and any existing `camera_extrinsics.yaml` are broken.
- Evidence: `urdf/ava.urdf.xacro`, `urdf/ava.common.xacro`, `CHANGELOG.md`

## D-005: Collision geometry on structural links only

- Date: 2026-10-13
- Status: Accepted
- Context: 45 links, of which 38 are cosmetic (bearings, spacers, servo bodies, horns, covers).
- Decision: Keep collision geometry only on the 7 structural links: `base_link`,
  `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_link`, `gripper_base_link`,
  `gripper_jaw_link`.
- Alternatives considered: Collision on every link (rejected: ~990 SRDF pairs, slower planning).
- Consequences: `move_group` logs 38 expected "visual geometry but no collision geometry" warnings.
- Evidence: `ava_description/README.md`, `CHANGELOG.md`

## D-006: SRDF collision matrix: disable adjacent pairs only

- Date: 2026-10-13
- Status: Accepted
- Context: Sampling 600 configurations showed the arm can fold onto itself
  (`shoulder_link`↔`gripper_base_link` 1.3 mm, `upper_arm_link`↔`gripper_jaw_link` 1.6 mm).
- Decision: Disable only the 6 adjacent pairs; leave the other 15 enabled.
- Consequences: The planner checks real self-collisions that the SO-101 SRDF had marked "Never".
- Evidence: `ava_moveit_config/config/ava.srdf`, `CHANGELOG.md`

## D-007: Keep model files out of git

- Date: 2026-10-12
- Status: Accepted
- Context: The CAD export, generated xacro/URDF and meshes carry the same geometry.
- Decision: Git-ignore `ava_robotic_arm.urdf`, `ava.kinematics.xacro`, `ava.urdf`,
  `meshes/*.stl`, `hardware/stl/*`, `hardware/step/*`; keep hand-written xacro tracked.
- Consequences: A fresh clone cannot launch the robot until model files are supplied and
  regenerated.
- Evidence: `.gitignore`, `README.md`

## D-008: Dashboard as a separate repo (git submodule) over rosbridge

- Date: 2026-09-29
- Status: Accepted
- Decision: `dashboard/` is a submodule
  ([ava-dashboard](https://github.com/TomasSirotek/ava-dashboard)), a static SPA connected
  through rosbridge WebSocket.
- Evidence: `.gitmodules`, `AGENTS.md`

## D-009: Elbow servo SG90 → MG996R (3× MG996R + 3× SG90)

- Date: 2026-10-12
- Status: Accepted (simulation updated; CAD redraw pending)
- Context: Gazebo and a static torque calculation from the CAD masses show the elbow needs
  ~0.11 N·m to hold the home pose and up to ~0.26 N·m stretched (10 g payload). The 9g
  servos in stock stall at 0.118 N·m (4.8 V) / 0.157 N·m (6 V).
- Decision: MG996R on `elbow_flex`. SG90s stay on `wrist_flex`, `wrist_roll`, `gripper`.
- Alternatives considered: lighter arm (would need ~70 % less elbow load), shorter forearm,
  spring counterbalance, MG90S micro servo (still short of the stretched-pose load).
- Consequences: Elbow effort limit 1.079 N·m; `torque_monitor.py` maps the elbow to MG996R;
  a TEMPORARY 46.8 g mass stub on `upper_arm_link` (`ava.urdf.xacro`) until the redrawn
  export arrives. Shoulder load rises (worst case ~0.63 N·m with 10 g, 59 % of MG996R stall).
  Buy 3 MG996R, not 2. Dashboard servo list (submodule) not yet updated.
- Evidence: `CHANGELOG.md` §3, `simulation_validation.md`

## D-010: Hardware path — ESP32 + PCA9685 over USB serial, ros2_control plugin

- Date: 2026-10-12
- Status: Accepted; implementation parked (plugin unbuilt, firmware never flashed)
- Context: The old `real` backend targeted STS3215 serial-bus servos; the OmArm Python
  bridges never sent a joint move and bypassed ros2_control.
- Decision: ESP32 drives a PCA9685 (I2C 0x40, 50 Hz, channels 0–5 = J1–J6); the PC talks to
  it over USB serial (`?`, `P u0..u5`, `D`); a C++ ros2_control plugin
  (`ava_hardware/AvaServoSystem`) maps joint angles to pulse widths, so the real arm uses the
  same controllers and topics as Gazebo.
- Consequences: Legacy bridges and STS parameters removed. No servo feedback: state = last
  command. Remaining work listed in the CHANGELOG TODO.
- Evidence: `firmware/ava_servo_driver`, `ros2_ws/src/ava_hardware`, `ava.ros2control.xacro`

---

## Open decisions

| # | Question | Options recorded | Blocks | Source |
|---|---|---|---|---|
| O-1 | Finish the hardware layer | Decided in D-010; plugin build/test parked | All hardware testing | `CHANGELOG.md` |
| O-2 | Shoulder fallback if MG996R does not hold | Smaller payload target, counterweight or spring, geared shoulder | Buying servos | `CHANGELOG.md` §3 |
| O-3 | Target payload | 20g (for now) | O-2 | `CHANGELOG.md` §3 |
| O-4 | MoveIt velocity/acceleration limits for hobby servos | TBD | Realistic trajectories | `CHANGELOG.md` §3 |
| O-5 | Wrist camera mount | Design a mount in CAD first | Wrist-camera extrinsics | `CHANGELOG.md` |
| O-6 | Contents of `ava_bringup` | TBD (currently an empty stub) | Single-command startup | `CHANGELOG.md` |
