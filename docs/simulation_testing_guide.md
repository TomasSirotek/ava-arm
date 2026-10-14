# AVA Simulation Testing Guide

How to look at the arm in RViz, load-test the servos in Gazebo, plan moves with MoveIt, and
read the results. Everything here is simulation only; real hardware is not usable yet (see
the hardware-layer TODO in `CHANGELOG.md`).

Servo layout under test: **3× MG996R** (`shoulder_pan`, `shoulder_lift`, `elbow_flex`) +
**3× SG90** (`wrist_flex`, `wrist_roll`, `gripper`), planned 6.0 V supply.

## 0. Before you start

- Use a normal terminal, not the VS Code integrated one (the VS Code snap breaks RViz/Gazebo).
- The model files are local only (git-ignored): `ava_robotic_arm.urdf`, the generated
  `ava.kinematics.xacro`/`ava.urdf` and the meshes must exist in `ros2_ws/src/ava_description/`.
- Every terminal needs both environments:

```bash
cd ~/Desktop/Work/ava-arm
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

- Rebuild only after adding/removing files or packages
  (`cd ros2_ws && colcon build --symlink-install`). Edits to existing xacro/yaml/python files
  are picked up without a rebuild because the workspace is built with `--symlink-install`.

## 1. Concepts you need

| Term | Meaning |
|---|---|
| Torque | Turning force on a joint: **weight × horizontal distance** from the joint axis. Unit N·m. 1 kg·cm = 0.0981 N·m. |
| Stall torque | The most a servo can push while held still at full power. Draws max current and heats up — seconds only. |
| Continuous | What a servo can hold for a long time. Rule of thumb used here: **½ of stall**. |
| Saturated | In Gazebo a joint can never push more than its effort limit. A reading equal to the limit means the joint is pushing as hard as it can and is probably sagging. |

Stall values used by the simulation (`ava.control.xacro`) and `tools/torque_monitor.py`:

| Servo | Stall @ 6 V | OK (≤ ½ stall) | HOT | FAIL |
|---|---|---|---|---|
| MG996R | 11 kg·cm = 1.079 N·m | ≤ 0.539 N·m | ≤ 1.079 N·m | > 1.079 or pinned at the limit |
| SG90 (9g) | 1.6 kg·cm = 0.157 N·m | ≤ 0.078 N·m | ≤ 0.157 N·m | > 0.157 or pinned at the limit |

The MG996R shop listing says 15 kg·cm @ 6 V for the plastic-gear units; 11 (TowerPro
datasheet) is used until real servos are measured.

## 2. RViz: look at the model and choose safe poses (no physics)

```bash
QT_QPA_PLATFORM=xcb ros2 launch ava_description display.launch.py
```

Two windows open: **RViz** and **Joint State Publisher** (one slider per joint).

1. **Displays panel** (left): Global Options → **Fixed Frame = `base_footprint`**. Tick
   **RobotModel**. Tick **TF** to see every joint's axes (untick *Show Names* if cluttered).
2. **Camera**: left-drag rotates, Shift+drag or middle-drag pans, scroll zooms.
3. **Sliders**: move a joint and watch the arm. **Center** = all joints 0 (home).
   **Randomize** = random angles (may be invalid).
4. **Self-collision check is your job here**: RViz does not warn. If links pass through
   each other, the pose is bad — do not use it in Gazebo (Gazebo has no self-collision for
   this model, see §6).
5. **Measure tool** (toolbar): click two points on the model to read the distance.
6. Write down the slider values: they are the `positions` you send to Gazebo in §3.

## 3. Gazebo load test (torque check)

### Start

```bash
# Terminal 1 — Gazebo with a 10 g test mass in the gripper (0.01 kg)
ros2 launch ava_description gazebo.launch.py payload_kg:=0.01
#   headless:=true runs without the Gazebo window
#   wait ~10 s: the robot spawns at 5 s, controllers load at 8 s and 10 s

# Terminal 2 — torque table
python3 tools/torque_monitor.py           # live, Ctrl+C to quit
python3 tools/torque_monitor.py --once    # 2 s average, prints once
```

### Send a pose

```bash
# Terminal 3 — positions are in the joint_names order (radians)
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper],
  points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}"

# Did the arm get there? Compare 'position' with what you sent (names are listed in the message)
ros2 topic echo /joint_states --once
```

### Test poses and expected values (10 g payload)

Predicted from the CAD masses (Gazebo matched these to 3 decimals in an earlier run).

| Pose | positions `[pan, lift, elbow, wrist_flex, roll, gripper]` | lift | elbow | wrist_flex |
|---|---|---|---|---|
| home | `[0, 0, 0, 0, 0, 0]` | 0.230 OK | 0.109 OK | 0.035 OK |
| front pick | `[0, -0.19, 1.59, -1.17, 0, 0]` | 0.297 OK | 0.106 OK | 0.022 OK |
| `pose_1` (SRDF, collision-free) | `[1.5, 1.0, 0, -1.0, 0, 0]` | 0.281 OK | 0.053 OK | 0.035 OK |
| stretched | `[0, 2.07, 0.05, -0.17, 0, 0]` | 0.627 HOT | 0.264 OK | 0.095 HOT |

Do **not** use `[0, 2.07, -1.52, -1.56, 0, 0]`: it folds the arm into itself.

### What to check for every pose

1. **Position**: `|actual − commanded|` below ~0.02 rad → the joint holds. A large error → the
   servo is too weak (sag) or the move was blocked.
2. **Torque**: read the `now N·m` column and the status. Use the `now` column; the `peak`
   column keeps the highest value since the monitor started.
3. **Saturation**: a value sitting exactly at 1.079 (MG996R) or 0.157 (SG90) = pinned at the
   limit → treat as FAIL even if the arm looks fine.
4. Restart Gazebo if a joint got stuck at a limit; later readings are then meaningless.

### Payload sweep

Stop Gazebo (Ctrl+C), relaunch with another mass, repeat the poses:

```bash
ros2 launch ava_description gazebo.launch.py payload_kg:=0.02
ros2 launch ava_description gazebo.launch.py payload_kg:=0.05
```

Predicted worst case anywhere in reach (arm above the table):

| Payload | `shoulder_lift` | `elbow_flex` | `wrist_flex` |
|---|---|---|---|
| 0 g | 0.590 (55 %) | 0.235 (22 %) | 0.078 (50 %) |
| 10 g | 0.633 (59 %) | 0.264 (24 %) | 0.095 (61 %) |
| 20 g | 0.677 (63 %) | 0.293 (27 %) | 0.113 (72 %) |
| 50 g | 0.809 (75 %) | 0.380 (35 %) | 0.165 (105 %) ✗ |

Small plastic cubes: solid PLA cube mass = 1.24 g/cm³ × side³ → 20 mm = 9.9 g, 25 mm = 19.4 g,
30 mm = 33.5 g (infill makes them lighter). Up to ~20 g is fine; avoid holding a cube with
the arm fully stretched for long (the wrist gets HOT).

## 4. MoveIt + RViz driving Gazebo (the arm plans its own moves)

```bash
QT_QPA_PLATFORM=xcb ros2 launch ava_moveit_config gazebo_moveit.launch.py rviz_use_sim_time:=true
python3 tools/torque_monitor.py     # second terminal
```

1. **MotionPlanning** panel → **Planning** tab → **Planning Group = `arm`**.
2. **Start State = `<current>`**.
3. **Goal State**: `home`, `pose_1`, or **`<random valid>`** (MoveIt picks a random
   collision-free pose — the easiest way to get safe test poses). Or drag the marker at the
   gripper; the orange arm is the goal.
4. A **red** goal arm = in collision or unreachable.
5. **Plan** shows the motion; **Execute** sends it to Gazebo; watch the torque monitor.
6. Gripper: Planning Group = `gripper`, Goal = `open` / `closed`.

Notes:
- This launch always starts Gazebo with **no payload**; for payload tests use §3.
- `position_only_ik: true` (`ava_moveit_config/config/kinematics.yaml`): MoveIt places the
  tool point but ignores the marker's orientation.

## 5. Recommended routine

1. RViz sliders (§2) or MoveIt `<random valid>` (§4) → pick poses that do not fold.
2. Gazebo + torque monitor (§3) → check position error and torque per pose.
3. Payload sweep (§3) → find where a joint turns HOT/FAIL.
4. Record results in `docs/simulation_validation.md`.

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Arm folds into itself | Gazebo creates no collision shapes for this model (log: "Mesh construction from an SDF has not been implemented yet for dartsim"). Choose poses in RViz/MoveIt first. |
| A joint shows exactly its limit (1.079 / 0.157) in every pose | It saturated once and is stuck or still catching up. Restart Gazebo, send home first, read the `now` column. |
| `ros2 topic pub` does nothing | Controllers not loaded yet (wait ~10 s after launch) or a typo in a joint name. |
| RViz window errors / blank | Prefix `QT_QPA_PLATFORM=xcb`, use a normal terminal. |
| Motion looks very fast | Simulated speed limits are off (URDF velocity `1e6`); held-pose torque checks are unaffected. |

## 7. The math behind it

### Dimensions (home pose) TBD - making it lighter

![Side view of the arm](media/arm_side_view_v1.svg)

Measured by chaining all 44 joint transforms of the CAD export (`ava_robotic_arm.urdf`) from
the table frame (`base_footprint`, including the Z-down → Z-up flip in `world_to_base`), then
measuring between joint axes in the arm's plane.

| Item | Value |
|---|---|
| Pan axis position (x, y) | (−34.7, −5.0) mm |
| Lift axis height above table | 157.8 mm (on the pan axis) |
| Arm-plane sideways offset `d` | 20.3 mm |
| `l1` lift → elbow | 150.1 mm, 70.8° from horizontal at home |
| `l2` elbow → wrist_flex | 119.9 mm, 64.0° |
| `l3` wrist_flex → tool point | 176.7 mm, 64.4° |
| Moving mass | ~266 g (upper arm group 68 g + elbow MG996R stub, forearm 57 g, wrist + gripper 95 g) |

Positive `shoulder_lift` tilts the arm towards −x; at table height the tool reaches 220 mm in
front of the pan axis and 430 mm behind it.

### Holding torque

$$\tau_j = g \sum_k m_k\, x_k$$

$m_k$ = mass of each part beyond joint $j$, $x_k$ = its horizontal distance from the joint
axis, $g = 9.81$ m/s². Example — elbow at home with 10 g payload:

$$
\tau = 9.81,(0.0565 \times 0.0357 + 0.0949 \times 0.0820 + 0.010 \times 0.1289) = 0.109\ \text{N}\cdot\text{m}
$$

(forearm 56.5 g at 35.7 mm, wrist + gripper 94.9 g at 82.0 mm, cube at 128.9 mm).

### Forward kinematics (arm plane)

With $q_1$ pan, $q_2$ lift, $q_3$ elbow, $q_4$ wrist_flex (the elbow axis points the other way,
hence $-q_3$):

$$\alpha_1 = 70.8^\circ + q_2,\quad \alpha_2 = 64.0^\circ + q_2 - q_3,\quad \alpha_3 = 64.4^\circ + q_2 - q_3 + q_4$$

$$\rho = \sum_k l_k\cos\alpha_k,\qquad z = 157.8\ \text{mm} + \sum_k l_k\sin\alpha_k$$

### Inverse kinematics

Given a target $(x, y, z)$ and a tool pitch $\psi$ (−90° = gripper pointing down):

1. Pan: $R^2=(x-x_p)^2+(y-y_p)^2$, $\rho=\pm\sqrt{R^2-d^2}$,
   $q_1=\operatorname{atan2}(y-y_p,\,x-x_p)-\operatorname{atan2}(d,\rho)$ (sign of $\rho$: lean
   forward or backward).
2. Wrist point: $\rho_W=\rho-l_3\cos\psi$, $z_W=z-l_3\sin\psi$.
3. Law of cosines with $\Delta\rho=\rho_W$, $\Delta z=z_W-157.8$:
   $\cos\gamma=\dfrac{\Delta\rho^2+\Delta z^2-l_1^2-l_2^2}{2l_1l_2}$, $\gamma=\pm\arccos(\cdot)$
   (elbow up / down).
4. $\alpha_1=\operatorname{atan2}(\Delta z,\Delta\rho)-\operatorname{atan2}(l_2\sin\gamma,\,l_1+l_2\cos\gamma)$,
   $\alpha_2=\alpha_1+\gamma$.
5. $q_2=\alpha_1-70.8^\circ$, $q_3=64.0^\circ+q_2-\alpha_2$, $q_4=\psi-64.4^\circ-q_2+q_3$.
6. Keep the solution inside the joint limits.

Checked against the exact CAD model on 300 random reachable targets: 287 solved, tool error
3.1 mm median, 7.7 mm max (the CAD axes are tilted ~0.7°, which this planar model ignores).
MoveIt does not use these formulas: it solves IK numerically with KDL, which is exact.
