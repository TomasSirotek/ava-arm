# AVA Simulation Validation

Status: **In progress.** This is the current project stage (`CHANGELOG.md` §3). No results are
recorded in the repo yet.

Question under test (`CHANGELOG.md` §3): does the servo layout hold under load? The original
2× MG996R + 4× SG90 failed at the elbow; the layout under test is now 3× MG996R (pan, lift,
elbow) + 3× SG90 (wrist flex, wrist roll, gripper).

## 1. Environment

| Item | Value | Source |
|---|---|---|
| ROS distribution | Jazzy | `README.md` |
| Simulator | Gazebo Harmonic (gz sim), world `empty.sdf` | `ava_description/README.md`, `launch/gazebo.launch.py` |
| ros2_control plugin | `gz_ros2_control/GazeboSimSystem` | `urdf/ava.ros2control.xacro` |
| Controller update rate | 100 Hz | `config/gazebo_controllers.yaml` |
| Model files present locally | Required: CAD export, generated xacro/URDF, meshes are git-ignored | `.gitignore`, `README.md` |
| Git commit tested | `94f620b` + uncommitted working tree of 2026-10-12 (3× MG996R + 3× SG90, elbow mass stub, SG90 1.6 kg·cm) | |
| Host machine | sir-tomas-labs, Intel i5-13420H, headless gz sim, real-time factor 0.82–0.85 | |

## 2. Method

```bash
# terminal 1
ros2 launch ava_description gazebo.launch.py payload_kg:=<kg>   # headless:=true optional
# terminal 2
python3 tools/torque_monitor.py           # live table
python3 tools/torque_monitor.py --once    # 2 s average, print once
```

- `payload_kg` adds a point mass (`payload_link`) fixed to `end_effector_link`; default 0
  (`urdf/ava.urdf.xacro`).
- `torque_monitor.py` reads `|effort|` from `/joint_states`, smoothed with factor 0.2, and grades it
  against the stall torque of the servo assigned to each joint:

| Status | Rule | MG996R threshold (N·m) | SG90 threshold (N·m) |
|---|---|---|---|
| OK | ≤ stall / 2 | ≤ 0.539 | ≤ 0.078 |
| HOT | ≤ stall | ≤ 1.079 | ≤ 0.157 |
| FAIL | > stall | > 1.079 | > 0.157 |

Stall values at the planned 6.0 V: MG996R 11.0 kg·cm (TowerPro datasheet), SG90 1.6 kg·cm
(datasheet of the 9g servos in stock), from `tools/torque_monitor.py`. The URDF effort limits
sit just above these, so a joint pinned at its limit reads FAIL. How to run the tests:
`docs/simulation_testing_guide.md`.

## 3. Test plan

From `CHANGELOG.md` §3:

- [x] T1: Load test on the new model at `payload_kg:=0.1`, home pose.
- [x] T2: Sweep the payload until a joint reads FAIL; record the real usable payload.
- [x] T3: Worst case: arm fully extended (where the SO-101 model failed).
- [ ] T4: Re-tune `joint_limits.yaml` velocities/accelerations (currently SO-101 values).
- [ ] T5: If the shoulder MG996R will not hold, choose between a smaller payload target,
      a counterweight or spring, or a geared shoulder **before buying anything**
      (record in `decisions.md`).

## 4. Results (2026-10-13)

**Run setup.** Headless Gazebo (`gazebo.launch.py headless:=true payload_kg:=<kg>`), one fresh
launch per payload. A throwaway probe sent each pose on
`/joint_trajectory_controller/joint_trajectory` (3 s move), waited 6 s sim time, then averaged
`/joint_states` for 2 s sim time (200 samples). Sequence per payload: home → front pick →
home → `pose_1` → home → stretched → home. Effort = |held torque| in N·m; % = of stall
(MG996R 1.079, SG90 0.157). Position error = actual − commanded.

Poses (`[pan, lift, elbow, wrist_flex, wrist_roll, gripper]`, rad):

| Pose | Values |
|---|---|
| home | `[0, 0, 0, 0, 0, 0]` |
| front pick | `[0, -0.19, 1.59, -1.17, 0, 0]` (tool low, in front) |
| `pose_1` (SRDF) | `[1.5, 1.0, 0, -1.0, 0, 0]` |
| stretched (T3 worst case) | `[0, 2.07, 0.05, -0.17, 0, 0]` (arm reaching out horizontally) |

`shoulder_pan`, `wrist_roll` and `gripper` stayed ≤ 0.007 N·m in every run and are omitted.

### T1 / T3 / T2: held torque per pose and payload

Format: effort N·m (% stall). All position errors < 0.001 rad unless the cell says FAIL.

| Payload | Pose | `shoulder_lift` (MG996R) | `elbow_flex` (MG996R) | `wrist_flex` (SG90) |
|---|---|---|---|---|
| 0 g | home | 0.212 (20 %) OK | 0.096 (9 %) OK | 0.027 (17 %) OK |
| 0 g | front pick | 0.288 (27 %) OK | 0.111 (10 %) OK | 0.013 (8 %) OK |
| 0 g | `pose_1` | 0.273 (25 %) OK | 0.054 (5 %) OK | 0.027 (17 %) OK |
| 0 g | stretched | 0.584 (54 %) HOT | 0.235 (22 %) OK | 0.078 (50 %) OK |
|---|---|---|---|---|
| 10 g | home | 0.230 (21 %) OK | 0.109 (10 %) OK | 0.035 (22 %) OK |
| 10 g | front pick | 0.300 (28 %) OK | 0.116 (11 %) OK | 0.017 (11 %) OK |
| 10 g | `pose_1` | 0.281 (26 %) OK | 0.053 (5 %) OK | 0.035 (22 %) OK |
| 10 g | stretched | 0.627 (58 %) HOT | 0.263 (24 %) OK | 0.095 (61 %) HOT |
|---|---|---|---|---|
| 20 g | home | 0.247 (23 %) OK | 0.122 (11 %) OK | 0.042 (27 %) OK |
| 20 g | front pick | 0.312 (29 %) OK | 0.120 (11 %) OK | 0.021 (13 %) OK |
| 20 g | `pose_1` | 0.288 (27 %) OK | 0.051 (5 %) OK | 0.042 (27 %) OK |
| 20 g | stretched | 0.670 (62 %) HOT | 0.292 (27 %) OK | 0.112 (71 %) HOT |
|---|---|---|---|---|
| 50 g | home (first) | 0.300 (28 %) OK | 0.160 (15 %) OK | 0.064 (41 %) OK |
| 50 g | front pick | 0.349 (32 %) OK | 0.135 (13 %) OK | 0.035 (22 %) OK |
| 50 g | home (after front pick) | 0.392 OK | 0.252 OK | **0.157 saturated, sagged 1.41 rad — FAIL** |
| 50 g | `pose_1` | 0.228 OK | 0.036 OK | **sagged 1.62 rad — FAIL** |
| 50 g | stretched | 0.776 (72 %) HOT | 0.354 (33 %) OK | **ended 0.69 rad off, at its limit — FAIL** |
|---|---|---|---|---|
| 100 g | home (T1) | 0.387 (36 %) OK | 0.223 (21 %) OK | 0.102 (65 %) HOT |
| 100 g | front pick | 0.410 (38 %) OK | 0.160 (15 %) OK | 0.056 (36 %) OK |
| 100 g | `pose_1` | 0.228 OK | 0.082 OK | **sagged 1.62 rad — FAIL** |
| 100 g | stretched | 0.556 HOT | 0.061 OK | **sagged 2.45 rad — FAIL** |

Every "home" row between poses at 0–20 g repeated the first home row exactly (the joints
always recovered). At 50 g the wrist recovered at the next home; at 100 g it did not.
Held torques at 0–20 g match the static calculation in `simulation_testing_guide.md` to
within 0.003 N·m.

### T2: payload sweep summary

| Payload | First joint to reach HOT | First joint to reach FAIL |
|---|---|---|
| 0 g | `shoulder_lift`, stretched (54 %) | none |
| 10 g | `shoulder_lift` + `wrist_flex`, stretched | none |
| 20 g | `shoulder_lift` + `wrist_flex` (71 %), stretched | none |
| 50 g | same | `wrist_flex`: saturates while moving out of the front pick and drops |
| 100 g | `wrist_flex` already HOT at home (65 %) | `wrist_flex` in `pose_1`, stretched and on the way home |

**Usable payload (simulation): 20 g** (all poses held, zero position error). At 50 g the
`wrist_flex` SG90 holds the static poses but drops during moves. The MG996R joints never
exceeded 72 % (`shoulder_lift`) and 33 % (`elbow_flex`) of stall, even at 50 g.

### Test artefact found (not a servo result)

A first sweep used the guide's poses `front pick [0, -0.21, 1.62, -1.17]` and
`stretched [0, 2.09, 0.05, -0.17]` back-to-back. `1.62` is above `elbow_flex`'s upper limit
(1.6179) and `2.09` sits within 0.004 rad of `shoulder_lift`'s (2.0944). A joint that touched
its limit stop stayed pinned there for the next pose (elbow stuck at 1.618 rad holding only
0.21 N·m, i.e. 19 % of stall — not a torque failure). [INFERENCE] This is how Gazebo's DART
engine handles a velocity-driven joint resting on its limit. Keep test poses ≥ 0.02 rad inside
the joint limits; the clean sweep above does.

### T4: joint limits after re-tune

| Joint | max_velocity (rad/s) | max_acceleration (rad/s²) | Basis |
|---|---|---|---|
| (all) | TBD | TBD | TBD |

## 5. Known simulation limitations

- Gazebo effort is noisy while moving (`torque_monitor.py` comment); read held poses.
- Mass and inertia come from the CAD export. Whether they match printed parts: TBD.
- Servo dynamics (speed, backlash, deadband of hobby PWM servos) are not modelled: TBD.
- Simulation results do not validate the hardware layer (plugin parked, firmware never flashed).

## 6. Conclusion

| Item | Value |
|---|---|
| Layout holds at target payload? | **Yes** for small cubes up to 20 g (all poses held). Not for 50 g+ |
| Weakest joint | `wrist_flex` (SG90): 71 % of stall stretched at 20 g; fails while moving at 50 g |
| Margin at worst case (20 g, stretched) | `shoulder_lift` 38 %, `elbow_flex` 73 %, `wrist_flex` 29 % of stall left |
| Caveats | Simulation gives full stall torque at any speed and holds position perfectly; real hobby servos will do worse. Avoid holding a cube with the arm stretched for long (2 joints HOT). |
| Decision recorded in `decisions.md` | Layout kept (D-009); payload target 20 g for now (O-3) |
