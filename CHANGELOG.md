# Changelog

All notable changes to this repo. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Where this is going

Three stages so far. Each one is finished unless marked otherwise.

1. **Started on an off-the-shelf SO-101 model** — fastest way to get something moving in
   RViz, Gazebo and MoveIt, and it worked.
2. **It failed the load test, so the arm is now my own design.** ✅ The Fusion 360 design is
   still under construction: the elbow MG996R mount is being redrawn and more changes will
   follow, so dimensions and masses in the current export are not final.
3. **Testing the servo layout (now 3× MG996R + 3× SG90).** ⬅ *current stage* — Gazebo
   sweep done: holds up to a 20 g payload (results in `docs/simulation_validation.md`).

### 1. Why the SO-101 model was dropped
- Most of the weight and the payload sit at the end of the arm, so the shoulder and elbow
  needed bigger, stronger, more expensive servos than the budget allowed.
- The torque math (`tools/torque_monitor.py`) confirmed it: shortening the links barely helps.
- Looked for another ready-made model built for 2× MG996R + 4× SG90 and found nothing that
  fit, so the arm got drawn from scratch instead.

### 2. Own model ✅
- **Redrawn in Fusion 360**, with help from a Fusion designer, around a fixed servo budget
  rather than fitting servos to someone else's geometry. The MG995 drawing had no usable
  dimensions — documentation and the available models all disagreed — so the servo was
  re-measured from the manufacturer's details and redrawn.
- **Model files kept local.** The CAD export, the meshes and the two files generated from
  them are git-ignored; a fresh clone cannot launch the robot without them. See the README.
- **Imported into `ava_description`**: the export is `urdf/ava_robotic_arm.urdf`
  (45 links / 44 joints, 45 STLs). The six joint names are unchanged, so controllers,
  teleop and the dashboard's joint plumbing carried over; link names and every joint limit
  did not. `scripts/generate_kinematics_xacro.py` adapts the raw export to the stack:
  - The export is authored **Z-down** — `world_to_base` rotates π about X and lifts the base
    by `base_mount_z`, or the whole arm hangs below the ground plane.
  - Collision geometry kept on the 7 structural links only; the 38 cosmetic bearing, spacer,
    servo-body and horn links stay visual-only so planning and Gazebo are not swamped.
  - Per-joint effort limits replace the single shared `max_torque_nm`: MG996R at the shoulder
    (1.079 / 2.157 N·m), SG90 from the elbow out (0.177 N·m).
  - `end_effector_link` re-derived from the gripper meshes; `base_footprint`, payload and the
    wrist-camera frames re-added on top of the export.
  - SRDF rebuilt: the collision matrix was recomputed by sampling 600 configurations, which
    shows this arm **can** fold onto itself (shoulder↔gripper closes to 1.3 mm), so only the 6
    adjacent pairs are disabled where the SO-101 SRDF could disable 10. Named poses re-verified
    against the new, asymmetric limits (`shoulder_lift` runs −0.21…+2.09 rad).
- Servo layout corrected to **2× MG996R + 4× SG90** everywhere it was recorded: the URDF
  effort limits, `tools/torque_monitor.py` and the dashboard (torque card and Parts tab had
  3× MG996R).

### 3. Testing the servos ⬅ current stage

The open question is whether the servo layout actually holds up, since the servo budget is
what forced the redesign in the first place.

- **Elbow moved from SG90 to MG996R.** In Gazebo the elbow SG90 saturated: the elbow needs
  ~0.11 N·m to hold the home pose and ~0.26 N·m stretched, against 0.118–0.157 N·m stall for
  the 9g servos in stock (1.2 / 1.6 kg·cm at 4.8 / 6 V). Layout is now 3× MG996R (pan, lift,
  elbow) + 3× SG90 (wrist flex, wrist roll, gripper). Simulation updated: elbow effort limit
  1.079 N·m, `torque_monitor.py`, and a TEMPORARY 46.8 g mass stub on `upper_arm_link`
  standing in for the heavier servo until the CAD is redrawn.

- [ ] Re-run the Gazebo load test on the new model:
      `ros2 launch ava_description gazebo.launch.py payload_kg:=0.1` with
      `python3 tools/torque_monitor.py` alongside.
- [ ] Sweep the payload until something reads FAIL, and record the real usable payload.
- [ ] Check the worst case, not just the home pose: arm fully extended, which is where the
      SO-101 model failed.
- [ ] If the shoulder MG996R will not hold, the options are a smaller payload target, a
      counterweight or spring, or a geared shoulder — decide before buying anything.
- [ ] Re-tune `joint_limits.yaml` velocities and accelerations; they are still the SO-101
      values (1.0–1.5 rad/s, well below the servos' no-load ~8 rad/s).
- Servo ratings now match the real parts at the planned 6.0 V: SG90 1.6 kg·cm (datasheet of
  the 9g servos in stock), MG996R 11 kg·cm (TowerPro datasheet; the shop lists 15).
  `shoulder_lift`'s 2.157 N·m "geared" limit is gone: no joint has a gearbox (the bearings
  carry side loads, they do not add torque).
- Testing guide: `docs/simulation_testing_guide.md` (RViz, Gazebo load test, MoveIt, math).

### Probably next, once the servos are settled
- **TODO — finish the hardware layer (parked).** Decided: ESP32 + PCA9685 (I2C 0x40,
  50 Hz), USB serial, ros2_control plugin. Done: ESP32 firmware
  (`firmware/ava_servo_driver`, protocol parser host-tested, never flashed); the broken
  OmArm Python bridges and their configs/launch files are removed; `ava.ros2control.xacro`
  `real` now selects `ava_hardware/AvaServoSystem` with PWM calibration in
  `ava.control.xacro`. Remaining: `ros2_ws/src/ava_hardware` is written but parked with
  `COLCON_IGNORE` — remove it, re-add `<exec_depend>ava_hardware</exec_depend>` to
  `ava_moveit_config/package.xml`, build, smoke-test against a fake serial device, write its
  README (wiring, flashing, calibration). Until then `real_moveit.launch.py` and
  `cube_real.launch.py` cannot start.
- **Buy and wire the servos**, then bring up one joint at a time against the ESP32.
- **Re-measure the wrist-camera extrinsics.** The redesign has no camera mount part, so the
  frames in `ava.common.xacro` are placeholders (`wrist_camera_urdf:=false` omits them).
  Needs a mount designed in CAD first.
- **Recalibrate the overhead camera.** `ava_vision` assumes `world == base_link` and that
  `base_link` XY lies on the table at z=0. The root flip broke both: `world` is now
  `base_footprint` and `base_link` sits 33.7 mm above it, rotated pi about X. Any existing
  `camera_extrinsics.yaml` is off by that transform.
- **Port the stale scripts.** `arm_ik.py` and `cup_pick_and_place.py` are still written for
  the original 5-DOF OmArm arm (`TODO(cad-redesign)`), so pick-and-place does not run.
- **Finish the dashboard placeholders**: Current pose, Solve IK, camera feed, voice.
- **Fill `ava_bringup`**, which is still an empty stub, so one launch file starts everything.

### Added
- `tools/torque_monitor.py`: live torque per joint vs its servo (OK / HOT / FAIL); Gazebo `payload_kg:=` option to test with a weight in the gripper.
- SO-101 robot model in `ava_description` (from [so_arm_ros2](https://github.com/adityakamath/so_arm_ros2), meshes from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)), with a single entry point `ava.urdf.xacro`.
- `ava.ros2control.xacro`: one switch for `real` / `gazebo` / `mock_components` / `mujoco`.
- `scripts/generate_urdf.sh`: builds the plain `urdf/ava.urdf` (for the dashboard and other tools) and validates it with `check_urdf`.
- Gazebo: robot anchored to the ground, mesh search path set, `headless:=true` launch option.
- MoveIt on Gazebo: `gazebo_moveit.launch.py` publishes the `world` frame; SRDF and joint limits rewritten for SO-101.
- `ava_bringup` package (empty stub).
- `LICENSES/` with the MIT (OmArm Zero) and Apache-2.0 (SO-ARM100, so_arm_ros2) texts.

### Changed
- Joint names follow SO-101: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` (were `Revolute 1–7`).
- Packages renamed from `omarm_zero_*` to `ava_*`; gamepad and real-hardware control merged into `ava_control`; `computer_vision_aruco` is now `ava_vision`.
- `ATTRIBUTION.md` rewritten: correct licenses and what was taken from where.

### Removed
- The old OmArm Zero model (`ava.xacro`, `ava.gazebo`, `ava.ros2control`, `materials.xacro`) and its meshes.
- Legacy OmArm ESP32 firmware and electronics files.
- `ava_control` legacy hardware bridges (`hardware_bridge_node`, `esp32_servo_bridge_node`,
  `follow_joint_trajectory_bridge`, `resource_manager`, `servo_calibration`), their configs
  (`hardware.yaml`, `real_robot.yaml`, `controllers.yaml`), `real_robot_*.launch.py` and
  `README.real.md`. They never sent a joint move (calibration key mismatch) and bypassed
  ros2_control.
- STS3215 serial-bus parameters (motor IDs, EEPROM PID, `/dev/ttySERVO`) from the xacro.

### Known issues
- `arm_ik.py` and `cup_pick_and_place.py` still describe the old 5-DOF arm (`TODO(so101-migration)`).
- Real hardware cannot run yet: the `ava_hardware` plugin is parked (see the TODO above).

## 2026-09-24 — Initial import
- Repo created from the OmArm Zero ROS 2 workspace.
