#!/usr/bin/env python3
"""
Cup pick-and-place demo for ava (ROS 2 Jazzy)
Publishes joint trajectories to the joint_trajectory_controller
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
import threading
import time

import arm_ik   # custom 5-DOF IK (installed next to this script)

ARM_JOINTS = ['Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4', 'Revolute 5']


class CupPickAndPlace(Node):
    def __init__(self):
        super().__init__('cup_pick_and_place')

        # --- Configurable parameters (see config/pick_place_params.yaml) ---
        # Side grasp on the 'tcp' link (fingertip frame, ~9 cm from gripper_base_1).
        # We target the TCP POSITION; orientation is left to position-only IK.
        # Top-down grasp via custom 5-DOF IK: the tcp +Z axis is aligned with
        # 'approach_dir' (default straight down) so it lies on the cube/marker Z.
        self.declare_parameter('grasp_z', 0.04)           # tcp height at grasp
        self.declare_parameter('approach_height', 0.10)   # pre-grasp height above grasp_z
        self.declare_parameter('lift', 0.06)              # lift after closing (+Z)
        self.declare_parameter('approach_dir', [0.0, 0.0, -1.0])  # tcp +Z target (world)
        self.declare_parameter('place_x', 0.15)
        self.declare_parameter('place_y', 0.10)
        self.declare_parameter('place_z', 0.10)
        self.declare_parameter('gripper_open', [-1.2, -1.2])
        self.declare_parameter('gripper_closed', [0.9, 0.9])
        self.declare_parameter('gripper_time_sec', 1.0)
        self.declare_parameter('gripper_joints', ['Revolute 6', 'Revolute 7'])
        # goto_only: only drive the TCP to the marker-0 position (no grasp/place).
        self.declare_parameter('goto_only', False)
        # home_joints: end-of-demo arm pose. Default [0..0] is for the SIM. On the REAL
        # robot [0,0,0,0,0] maps to the servo END STOPS (deg = 90 + dir*(rad-zero)*57.3)
        # -> stall! Real runs MUST override this (safe proven pose: 0.8 -0.8 -1.2 -0.8 -2.6).
        self.declare_parameter('home_joints', [0.0, 0.0, 0.0, 0.0, 0.0])

        self.grasp_z = self.get_parameter('grasp_z').value
        self.approach_height = self.get_parameter('approach_height').value
        self.lift = self.get_parameter('lift').value
        self.approach_dir = list(self.get_parameter('approach_dir').value)
        self.place_x = self.get_parameter('place_x').value
        self.place_y = self.get_parameter('place_y').value
        self.place_z = self.get_parameter('place_z').value
        self.gripper_open_pos = list(self.get_parameter('gripper_open').value)
        self.gripper_closed_pos = list(self.get_parameter('gripper_closed').value)
        self.gripper_time_sec = self.get_parameter('gripper_time_sec').value
        self.gripper_joints = list(self.get_parameter('gripper_joints').value)
        self.goto_only = bool(self.get_parameter('goto_only').value)
        self.home_joints = [float(v) for v in self.get_parameter('home_joints').value]
        # place_marker_frame: if set (e.g. 'aruco_marker_4'), the PLACE target is the
        # LIVE-detected pose of that marker (X,Y; tcp height = marker_z + place_z_offset,
        # floor-limited to grasp_z so the cube is set down gently, never pressed into
        # the table). Put a marker print-out where the cube should end up.
        self.declare_parameter('place_marker_frame', '')
        self.declare_parameter('place_z_offset', 0.03)
        self.place_marker_frame = str(self.get_parameter('place_marker_frame').value)
        self.place_z_offset = float(self.get_parameter('place_z_offset').value)
        # Used only by the legacy send_pose_goal() fallback (PositionConstraint path).
        self.tcp_link = 'tcp'
        self.pos_tol = 0.05

        # 5-DOF IK solver (loads the URDF chain base_link -> tcp).
        self.get_logger().info('Loading 5-DOF IK (arm_ik)...')
        self.ik = arm_ik.ArmIK()

        # Current arm joint state (IK seed + MoveGroup start state).
        self.joint_pos = {}
        # /joint_states is published BEST_EFFORT by the real hardware_bridge (and
        # RELIABLE in sim). A BEST_EFFORT subscription is compatible with both.
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb,
                                 qos_profile_sensor_data)

        # Subscription: current cup pose
        self.cup_pose = None
        self.create_subscription(PoseStamped, '/cup_pose', self._cup_pose_cb, 10)

        # TF listener for the live place-marker lookup (world -> place_marker_frame).
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveGroup action client
        self._mg_client = None
        self._mg_lock = threading.Lock()

        # Gripper: direct FollowJointTrajectory action (identical path in
        # sim [ros2_control JTC] and real [follow_joint_trajectory_bridge]).
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

        # Tells the cube_pose_publisher to remove the cube collision object
        # during the grasp (it would block the plan otherwise). Latched so
        # the publisher reliably receives the last value.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._grasp_pub = self.create_publisher(Bool, '/grasp_active', latched)
        self._set_grasp_active(False)
        # Command-based grasp signal for the Gazebo attach helper (reliable, not
        # dependent on the gripper joint physically reaching a threshold).
        self._gripper_closed_pub = self.create_publisher(Bool, '/gripper_closed', latched)
        self._gripper_closed_pub.publish(Bool(data=False))

        # Direct arm-trajectory publisher for the LINEAR descent/ascent: each 1-cm step
        # is sent as its own single-point command, so sim (JTC topic interface) and real
        # (hardware bridge forwards each as one MOVE) follow the same straight tcp line.
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.get_logger().info('Cup pick-and-place demo initialized')

    def _set_grasp_active(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self._grasp_pub.publish(msg)

    def _tcp_pose(self, frame, x, y, z):
        """Build a PoseStamped target for the TCP link (orientation identity;
        ignored by position-only IK / the PositionConstraint)."""
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation.w = 1.0
        return p

    def _spin_for(self, seconds):
        """Spin the node for 'seconds' seconds (processes subscriptions)."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def execute_demo(self):
        # IMPORTANT: this method is called directly from main() (NOT from
        # a timer callback). Otherwise rclpy.spin_until_future_complete() would
        # re-spin the node from within its own callback
        # (re-entrancy) and action responses would get lost.
        try:
            self.get_logger().info('Waiting for full initialization...')
            self._spin_for(3.0)

            self.get_logger().info('\n=== 🤖 Cup Pick-and-Place Demo GESTARTET ===\n')

            # 2) Wait for cup pose (per spin, nicht per sleep). REAL time — spin_once
            # returns instantly while other topics (e.g. /joint_states @50 Hz) are
            # busy, so counting iterations would time out after milliseconds.
            self.get_logger().info('2️⃣  Waiting for /cup_pose (max 15 s) ...')
            t_end = time.time() + 15.0
            while self.cup_pose is None and time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.2)

            if self.cup_pose is None:
                self.get_logger().error('No /cup_pose received — aborting the demo')
                return

            self.get_logger().info('   ✓ Cup pose received. Planning a vertical top-down grasp (5-DOF)...')

            cube = self.cup_pose
            cx = cube.pose.position.x
            cy = cube.pose.position.y
            # Top-down: the tcp +Z (grasp/approach axis) is aligned with approach_dir
            # (straight down), i.e. tcp-Z lies ON the cube/marker-0 Z axis
            # (collinear); rotation about that Z axis is free.

            # Disable the cube collision object (the cube_pose_publisher removes
            # it) if active; wait briefly until the planning scene has it.
            self._set_grasp_active(True)
            self._spin_for(0.8)

            # 1) Open the gripper first.
            self.get_logger().info('1️⃣  Opening gripper (start state)...')
            self.send_gripper(self.gripper_open_pos)

            # 2) Pre-grasp: vertically ABOVE the cube (target: approach_height above
            #    grasp_z, real = ~7 cm above the marker-0 top). At border positions
            #    the full height is not always vertically reachable -> ADAPTIVELY
            #    pick the highest reachable height; abort below 5 cm clearance.
            pre_z, q_pre = None, None
            z_min = self.grasp_z + 0.05
            # First pass quick (fast feasibility), second pass FULL solver: the quick
            # mode can miss borderline-but-valid heights (false negative) and would
            # abort a perfectly pickable position.
            for use_quick in (True, False):
                z_try = self.grasp_z + self.approach_height
                while z_try >= z_min - 1e-6:
                    q_try = self.ik.ik_solve((cx, cy, z_try),
                                             approach_dir=self.approach_dir,
                                             align_axis=2, seed=self._arm_seed(),
                                             w_orient=1.0, ang_tol=12.0,
                                             quick=use_quick)
                    if q_try is not None:
                        pre_z, q_pre = z_try, q_try
                        break
                    z_try -= 0.01
                if pre_z is not None:
                    break
            if pre_z is None:
                self.get_logger().error(
                    f'Pre-grasp: no vertically reachable height >= {z_min:.2f} '
                    f'above ({cx:.3f},{cy:.3f}).')
                self.get_logger().error(
                    '   ' + self._suggest_reachable_spot(
                        cx, cy, self.grasp_z + self.approach_height, self.grasp_z))
                return
            # UPFRONT check of the WHOLE descent chain (pre_z -> grasp_z): patchy
            # vertical-reach zones can have a gap at an intermediate height even
            # when pre_z itself is fine. Cheap now (solver early exit) — and it
            # aborts BEFORE the arm drives anywhere, telling the user WHERE to
            # push the cube instead. Seed with the PRE-GRASP solution (q_pre): the
            # chain must start from the branch the arm will actually be in — a
            # home-pose seed can wander into a branch that dead-ends mid-descent.
            if self._solve_linear_chain(cx, cy, pre_z, self.grasp_z,
                                        q_pre or self._arm_seed(),
                                        label='Descent') is None:
                self.get_logger().error(
                    '   ' + self._suggest_reachable_spot(cx, cy, pre_z, self.grasp_z))
                return
            self.get_logger().info(
                f'2️⃣  Pre-grasp (vertically above the cube, z={pre_z:.2f} '
                f'= {(pre_z - 0.10)*100:.0f} cm above the marker)...')
            if not self.goto_tcp(cx, cy, pre_z):
                self.get_logger().error('Pre-grasp: IK/planning failed')
                return
            time.sleep(0.3)

            # 3) Grasp: LINEAR vertical descent (no joint-space arc!) — the cube
            #    stays untouched and centered between the fingers until they close.
            self.get_logger().info('3️⃣  Grasp (LINEAR vertical descent)...')
            if not self.move_linear_z(cx, cy, pre_z, self.grasp_z, label='Descent'):
                self.get_logger().error('Grasp: linear descent failed')
                return

            # 4) Close the gripper
            self.get_logger().info('4️⃣  Closing gripper...')
            self.send_gripper(self.gripper_closed_pos)
            self.get_logger().info('   ✓ Gripper closed\n')

            # 5) Lift: LINEAR up as well (an arc would drag the cube sideways).
            #    At most up to pre_z — anything higher is provably not vertically
            #    reachable here (pre_z came from the adaptive search).
            lift_z = min(self.grasp_z + self.lift, pre_z)
            self.get_logger().info(f'5️⃣  Lifting (LINEAR up to z={lift_z:.2f})...')
            if not self.move_linear_z(cx, cy, self.grasp_z, lift_z, label='Lift'):
                self.get_logger().warn('Lift failed — continuing to the place')
            time.sleep(0.3)

            # 6) Place: either the LIVE pose of the place marker (place_marker_frame,
            #    e.g. aruco_marker_4 -> the cube ends up on a defined mark) or
            #    the fixed configured point. Always vertical.
            px, py, pz = self.place_x, self.place_y, self.place_z
            if self.place_marker_frame:
                from rclpy.time import Time as RclTime
                from tf2_ros import (LookupException, ConnectivityException,
                                     ExtrapolationException)
                try:
                    tfm = self.tf_buffer.lookup_transform(
                        'world', self.place_marker_frame, RclTime())
                    mt = tfm.transform.translation
                    mx, my = float(mt.x), float(mt.y)
                    # The marker provides the XY place spot; the settle height is the
                    # fixed place_z param (the table is flat; slightly below the grasp
                    # height so the cube firmly STANDS before the fingers open).
                    pz = self.place_z
                    # TOLERANCE (user requirement): the reference marker may be out of
                    # reach — then place as CLOSE to it as possible: slide the target
                    # radially inward on the base->marker line until BOTH the fly-over
                    # (pz+0.05) AND the settle height (pz) are vertically solvable
                    # (otherwise the later place fly-over would abort).
                    # quick=True: feasibility probes take ~1-2 s instead of ~15 s.
                    import math
                    r = math.hypot(mx, my)
                    ux, uy = (mx / r, my / r) if r > 1e-6 else (1.0, 0.0)
                    found = None
                    rr = min(r, 0.22)   # beyond ~22 cm nothing is reachable anyway
                    while rr >= 0.10:
                        tx, ty = ux * rr, uy * rr
                        if (self.ik.ik_solve((tx, ty, pz + 0.05),
                                             approach_dir=self.approach_dir,
                                             align_axis=2, seed=self._arm_seed(),
                                             w_orient=1.0, ang_tol=12.0,
                                             quick=True) is not None
                                and self.ik.ik_solve((tx, ty, pz),
                                                     approach_dir=self.approach_dir,
                                                     align_axis=2,
                                                     seed=self._arm_seed(),
                                                     w_orient=1.0, ang_tol=12.0,
                                                     quick=True) is not None):
                            found = (tx, ty)
                            break
                        rr -= 0.01
                    if found is not None:
                        px, py = found
                        if rr < r - 0.005:
                            self.get_logger().info(
                                f'   Marker "{self.place_marker_frame}" '
                                f'({mx:.3f},{my:.3f}) out of reach — placing '
                                f'{100*(r-rr):.0f} cm short of it (tolerance): '
                                f'({px:.3f},{py:.3f})')
                        else:
                            self.get_logger().info(
                                f'   Place target = marker "{self.place_marker_frame}" '
                                f'LIVE: ({px:.3f},{py:.3f}), tcp height z={pz:.3f}')
                    else:
                        self.get_logger().warn(
                            '   No reachable place toward the marker — '
                            f'using the fixed place point ({px:.2f},{py:.2f})')
                except (LookupException, ConnectivityException,
                        ExtrapolationException):
                    self.get_logger().warn(
                        f'   Marker "{self.place_marker_frame}" not visible — '
                        f'using the fixed place point ({px:.2f},{py:.2f})')
            # SAME logic as the pick: fly OVER the place spot (adaptive height), then
            # descend LINEARLY until the cube rests on the table, open, retreat linearly.
            place_pre = None
            # Nothing sits at the place spot, so a low fly-over (+2 cm) is fine —
            # the carried cube hangs below the tcp and just skims over the table.
            zmin = self.place_z + 0.02
            # quick first, FULL solver as fallback (quick can false-negative).
            for use_quick in (True, False):
                zt = self.place_z + self.approach_height
                while zt >= zmin - 1e-6:
                    if self.ik.ik_solve((px, py, zt), approach_dir=self.approach_dir,
                                        align_axis=2, seed=self._arm_seed(),
                                        w_orient=1.0, ang_tol=12.0,
                                        quick=use_quick) is not None:
                        place_pre = zt
                        break
                    zt -= 0.01
                if place_pre is not None:
                    break
            if place_pre is not None:
                self.get_logger().info(
                    f'6️⃣  Flying OVER the place position (z={place_pre:.2f})...')
                if not self.goto_tcp(px, py, place_pre):
                    self.get_logger().error(
                        'Place: IK/planning failed — is the place marker within '
                        'reach (radius < ~22 cm from the base)?')
                    return
                time.sleep(0.3)
                self.get_logger().info(
                    f'   Lowering LINEARLY to z={pz:.2f} (cube onto the table)...')
                if not self.move_linear_z(px, py, place_pre, pz, label='Lower'):
                    self.get_logger().warn('Lowering incomplete — opening anyway')
            else:
                # No vertical fly-over solvable here: NEVER stop holding the cube
                # mid-air — fall back to driving straight to the settle pose.
                self.get_logger().warn(
                    '6️⃣  No vertical fly-over solvable at the place spot — '
                    f'driving DIRECTLY to the settle height z={pz:.2f}.')
                place_pre = pz
                if not self.goto_tcp(px, py, pz):
                    self.get_logger().error(
                        'Place: direct approach failed too — move the place '
                        'marker closer to the base.')
                    return
                time.sleep(0.3)

            # 7) Open the gripper (place), then retreat LINEARLY (don't knock the
            #    placed cube over with the fingers).
            self.get_logger().info('7️⃣  Opening gripper (placing)...')
            self.send_gripper(self.gripper_open_pos)
            self.get_logger().info('   ✓ Gripper open\n')
            self.move_linear_z(px, py, pz, place_pre, label='Retreat')
            
            # 8) Back to home (joint-space goal to the configured home pose)
            self.get_logger().info('8️⃣  Returning to the home position...')
            # Gazebo's PID can transiently overshoot a joint limit -> the start state
            # is briefly INVALID (MoveIt code -26). Let the sim settle and retry.
            ok = False
            for attempt in range(3):
                ok = self.send_home_goal(execute=True)
                if ok:
                    break
                self.get_logger().warn(
                    f'Home attempt {attempt + 1} failed — settling + retrying...')
                self._spin_for(2.0)
            if not ok:
                self.get_logger().error('Home planning failed (after 3 attempts)')
                return
            time.sleep(0.5)
            self.get_logger().info('   ✓ Home position reached\n')
            
            self.get_logger().info('=== ✅ Pick-and-place demo DONE! ===\n')
            
        except Exception as e:
            self.get_logger().error(f'❌ Fehler: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # Re-enable the cube collision object (the cube_pose_publisher adds
            # it back), no matter whether the demo succeeded or not.
            self._set_grasp_active(False)
            self._spin_for(0.3)

    def goto_marker0(self):
        """GOAL: tcp origin -> marker-0 origin (position is HARD). tcp +Z is aligned
        best-effort with the marker-0 Z axis (vertical, approach_dir) — if
        reachable, otherwise ignored. Flow: open gripper -> drive there ->
        close gripper."""
        try:
            self.get_logger().info('Waiting for full initialization...')
            self._spin_for(3.0)
            self.get_logger().info('Waiting for /cup_pose (max 10 s) — marker 0 must be visible ...')
            waited = 0.0
            while self.cup_pose is None and waited < 10.0:
                rclpy.spin_once(self, timeout_sec=0.5)
                waited += 0.5
            if self.cup_pose is None:
                self.get_logger().error(
                    'No /cup_pose received — is marker 0 (cube) in the camera image? Aborting.')
                return
            cube = self.cup_pose
            cx, cy, cz = (cube.pose.position.x, cube.pose.position.y, cube.pose.position.z)
            self.get_logger().info(
                f'   ✓ /cup_pose: Marker 0 @ ({cx:+.3f}, {cy:+.3f}, {cz:+.3f}) [world==base_link]')

            self._set_grasp_active(True)
            self._spin_for(0.5)

            # 1) Open the gripper first.
            self.get_logger().info('1️⃣  Opening gripper (before moving)...')
            self.send_gripper(self.gripper_open_pos)

            # 2) Pre-grasp ~approach_height above the marker (best effort, if reachable).
            z_pre = cz + self.approach_height
            self.get_logger().info(f'2️⃣  Pre-grasp above marker 0 (z={z_pre:.2f}) ...')
            if not self.goto_tcp(cx, cy, z_pre):
                self.get_logger().warn('Pre-grasp not reachable — driving directly to the target.')
            else:
                time.sleep(0.3)

            # 3) tcp-Ursprung auf Marker-0-Ursprung (Position HART, tcp-Z best-effort).
            self.get_logger().info(
                f'3️⃣  tcp auf Marker-0-Position (x={cx:+.3f}, y={cy:+.3f}, z={cz:+.3f}) ...')
            if not self.goto_tcp(cx, cy, cz):
                self.get_logger().error(
                    f'tcp position ({cx:+.3f},{cy:+.3f},{cz:+.3f}) not reachable — '
                    'place the cube inside the workspace (~0.12–0.24 m radius). NO motion.')
                return

            # 4) Grasp.
            self.get_logger().info('4️⃣  Grasping — closing gripper...')
            self.send_gripper(self.gripper_closed_pos)
            self.get_logger().info(
                f'=== ✅ tcp an Marker-0-Position ({cx:+.3f},{cy:+.3f},{cz:+.3f}), gegriffen. ===')
        except Exception as e:
            self.get_logger().error(f'❌ Fehler: {e}')
            import traceback
            traceback.print_exc()
        finally:
            self._set_grasp_active(False)
            self._spin_for(0.3)

    def _joint_state_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.joint_pos[n] = p

    def _arm_seed(self):
        """Current arm joints [Rev1..Rev5] as IK seed, or None if unknown."""
        if all(j in self.joint_pos for j in ARM_JOINTS):
            return [self.joint_pos[j] for j in ARM_JOINTS]
        return None

    def goto_tcp(self, x, y, z):
        """Move the tcp ORIGIN to (x,y,z) with tcp +Z (the finger/approach axis, after
        the URDF rpy(0,pi/2,0)) aligned straight DOWN (approach_dir) -> tcp-Z lies ON the
        marker-0 Z axis: clean vertical top-down grasp. STRICT: a vertical tcp-Z is
        REQUIRED. If it is not reachable here (e.g. the back-right -x,-y corner, which is
        kinematically impossible for this 5-DOF arm), ABORT with a clear message instead
        of a crooked grasp. Then move via a MoveGroup joint-space goal."""
        seed = self._arm_seed()
        # Position + tcp +Z (finger axis) aligned with approach_dir (straight down).
        # w_orient>default + tight ang_tol -> the alignment is enforced, not best-effort.
        # Tiered position tolerance: prefer <=5 mm (10-mm solutions made the real
        # gripper MISS the 3-cm cube); fall back to <=10 mm with a loud warning
        # (border positions only have coarse vertical solutions).
        q = self.ik.ik_solve((x, y, z), approach_dir=self.approach_dir,
                             align_axis=2, seed=seed, w_orient=1.0, ang_tol=12.0,
                             pos_tol=0.005)
        if q is None:
            q = self.ik.ik_solve((x, y, z), approach_dir=self.approach_dir,
                                 align_axis=2, seed=seed, w_orient=1.0, ang_tol=12.0,
                                 pos_tol=0.01)
            if q is not None:
                self.get_logger().warn(
                    '   Only a COARSE vertical solution here (up to 10 mm) — the grasp '
                    'may sit slightly off. For precision, place the cube in the inner zone.')
        if q is None:
            self.get_logger().error(
                f'   tcp-Z vertical on marker-Z at ({x:.3f},{y:.3f},{z:.3f}) NOT '
                'reachable — place the cube IN FRONT of the robot / to the left (not '
                'the back-right corner with x<0 AND y<0). No tilted grasp.')
            return False
        pe, ae = self.ik._metrics(q, [float(x), float(y), float(z)],
                                  list(self.approach_dir), 2)
        self.get_logger().info(
            f'   IK ok: pos_err={pe*1000:.0f}mm, tcp-Z·marker-Z={ae:.0f}° (vertical)')
        return self.send_arm_joint_goal(q)

    def _suggest_reachable_spot(self, cx, cy, z_top, z_bottom):
        """The cube sits in a vertical-reach gap: probe 8 neighbor spots (3 cm ring,
        then 6 cm) with quick chain checks and tell the user WHERE to push the cube
        instead of leaving them guessing. Returns a human-readable hint or ''."""
        import math
        corners = {1: (0.272, -0.1645), 2: (-0.272, -0.1645),
                   3: (-0.272, 0.1345), 4: (0.272, 0.1345)}
        for radius in (0.03, 0.06):
            for k in range(8):
                a = k * math.pi / 4.0
                tx, ty = cx + radius * math.cos(a), cy + radius * math.sin(a)
                if tx * tx + ty * ty > 0.22 ** 2:
                    continue
                seed, ok = None, True
                for z in (z_top, (z_top + z_bottom) / 2.0, z_bottom):
                    q = self.ik.ik_solve((tx, ty, z), approach_dir=self.approach_dir,
                                         align_axis=2, seed=seed, w_orient=1.0,
                                         ang_tol=12.0, pos_tol=0.01,
                                         quick=(seed is None))
                    if q is None:
                        ok = False
                        break
                    seed = q
                if ok:
                    dx, dy = tx - cx, ty - cy
                    best_id = min(corners, key=lambda i: math.hypot(
                        corners[i][0] - cx - 10 * dx, corners[i][1] - cy - 10 * dy))
                    return (f'Suggestion: push the cube ~{radius*100:.0f} cm toward '
                            f'the marker-{best_id} corner (reachable at '
                            f'({tx:+.2f},{ty:+.2f})).')
        return ('Suggestion: place the cube on the "grasp ring" — an arc '
                '~15-19 cm from the robot base (the vertical grasp is '
                'solvable everywhere on it).')

    def _solve_linear_chain(self, x, y, z_from, z_to, seed, label='Linear',
                            step=0.01):
        """Solve the IK chain for a straight vertical line (seed-chained steps).
        Returns the list of joint solutions, or None (with a log) on a gap.
        Cheap thanks to the solver's early exit — usable as an upfront check
        BEFORE the arm starts moving toward the pre-grasp."""
        z_from, z_to = float(z_from), float(z_to)
        n = max(1, int(round(abs(z_from - z_to) / step)))
        chain = []
        coarse_warned = False
        for i in range(1, n + 1):
            z = z_from + (z_to - z_from) * i / n
            # Tiered tolerance: <=5 mm preferred, <=10 mm fallback, and finally
            # <=20 mm / 15 deg with a warning. The last tier keeps the task going
            # at borderline spots — uncritical in simulation (the grasp attach is
            # command-based) and usually fine on hardware since the remount gave
            # the fingers a wide opening; the warning tells you to watch the grasp.
            q = None
            for tol_pos, tol_ang, coarse in ((0.005, 12.0, False),
                                             (0.01, 12.0, False),
                                             (0.02, 15.0, True)):
                q = self.ik.ik_solve((x, y, z), approach_dir=self.approach_dir,
                                     align_axis=2, seed=seed, w_orient=1.0,
                                     ang_tol=tol_ang, pos_tol=tol_pos)
                if q is not None:
                    if coarse and not coarse_warned:
                        coarse_warned = True
                        self.get_logger().warn(
                            f'   {label}: only a COARSE solution at z={z:.3f} '
                            '(up to 2 cm / 15°) — fine in simulation; watch the grasp on hardware.')
                    break
            if q is None:
                self.get_logger().error(
                    f'   {label}: vertical IK gap at z={z:.3f} — aborting WITHOUT '
                    'motion (move the cube a little).')
                return None
            chain.append(q)
            seed = q
        return chain

    def move_linear_z(self, x, y, z_from, z_to, label='Linear', step=0.01,
                      step_time=0.4):
        """Move the tcp along a STRAIGHT vertical line over (x,y) from z_from to z_to.

        The cube must not be touched during the approach, so the descent may not be a
        joint-space arc: z is sampled in `step` increments, each solved with the
        previous step as IK seed (continuous joint path, tcp stays ON the vertical
        line), and each step is sent as its OWN single-point trajectory command.
        That gives the same straight tcp line in sim (JTC topic interface) and on the
        real robot (the hardware bridge forwards each step as one MOVE; the firmware
        ramp smooths between steps).
        The FULL IK chain is solved BEFORE any motion — an unreachable intermediate
        height aborts cleanly without moving."""
        chain = self._solve_linear_chain(x, y, z_from, z_to, self._arm_seed(),
                                         label=label, step=step)
        if chain is None:
            return False
        z_from, z_to = float(z_from), float(z_to)
        n = len(chain)
        self.get_logger().info(
            f'   {label}: {n} steps of {step*1000:.0f} mm '
            f'(z {z_from:.3f} → {z_to:.3f}), tcp stays vertically above ({x:.3f},{y:.3f})')
        for q in chain:
            m = JointTrajectory()
            m.joint_names = ['Revolute 1', 'Revolute 2', 'Revolute 3',
                             'Revolute 4', 'Revolute 5']
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.time_from_start = MsgDuration(
                sec=int(step_time), nanosec=int((step_time % 1.0) * 1e9))
            m.points.append(pt)
            self._arm_traj_pub.publish(m)
            self._spin_for(step_time + 0.05)
        return True

    def send_arm_joint_goal(self, joint_values, execute=True,
                            accept_timeout=20.0, result_timeout=120.0,
                            retries=3):
        """MoveGroup joint-space goal for the arm (Rev1..Rev5 = joint_values).

        Retries on START_STATE_INVALID (code -26): the Gazebo PID can transiently
        overshoot a joint limit right after a motion, which makes MoveIt reject the
        start state for a moment — settle briefly and try again."""
        for attempt in range(max(1, retries)):
            ok = self._send_arm_joint_goal_once(joint_values, execute,
                                                accept_timeout, result_timeout)
            if ok is not None:
                return ok
            self.get_logger().warn(
                f'   Start state transiently invalid (attempt {attempt + 1}) — '
                'settling + retrying...')
            self._spin_for(1.5)
        return False

    def _send_arm_joint_goal_once(self, joint_values, execute,
                                  accept_timeout, result_timeout):
        """One attempt. Returns True/False for a final verdict, None for
        'retryable' (START_STATE_INVALID)."""
        mgc = self._ensure_movegroup_client()
        if not mgc.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/move_action not available')
            return False
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = 'arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.8
        req.max_acceleration_scaling_factor = 0.8
        req.start_state.is_diff = True
        c = Constraints()
        for jn, val in zip(ARM_JOINTS, joint_values):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal_msg.planning_options.plan_only = not execute

        send_future = mgc.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=accept_timeout)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Joint goal rejected')
            return False
        resf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, resf, timeout_sec=result_timeout)
        if resf.result() is None:
            self.get_logger().error(f'No result after {result_timeout}s (timeout)')
            return False
        res = resf.result().result
        if res.error_code.val == 1:
            return True
        if res.error_code.val == -26:      # START_STATE_INVALID -> retryable
            return None
        self.get_logger().error(f'Joint motion failed, code={res.error_code.val}')
        return False

    def send_home_goal(self, execute=True, accept_timeout=20.0, result_timeout=120.0):
        """Joint-space goal to the configured home pose (param 'home_joints';
        default [0..0] = sim. Real runs MUST override with a safe pose)."""
        mgc = self._ensure_movegroup_client()
        if not mgc.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/move_action not available')
            return False

        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = 'arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.8
        req.max_acceleration_scaling_factor = 0.8
        req.start_state.is_diff = True

        c = Constraints()
        for jn, pos in zip(['Revolute 1', 'Revolute 2', 'Revolute 3',
                            'Revolute 4', 'Revolute 5'], self.home_joints):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal_msg.planning_options.plan_only = not execute
        send_future = mgc.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=accept_timeout)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Home-Goal abgelehnt')
            return False
        resf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, resf, timeout_sec=result_timeout)
        if resf.result() is None:
            self.get_logger().error(f'Kein Home-Ergebnis nach {result_timeout}s (Timeout)')
            return False
        res = resf.result().result
        if res.error_code.val == 1:
            self.get_logger().info('Home motion succeeded')
            return True
        self.get_logger().error(f'Home motion failed, code={res.error_code.val}')
        return False

    def _cup_pose_cb(self, msg: PoseStamped):
        self.cup_pose = msg

    def send_gripper(self, positions, accept_timeout=10.0, result_timeout=15.0):
        """Drive the gripper via the direct FollowJointTrajectory action.

        Sendet ein 2-Joint-Goal (Revolute 6/7). Der joint_trajectory_controller
        akzeptiert Teil-Joint-Goals und haelt die uebrigen Joints. Funktioniert
        identisch in Sim und auf der echten Hardware (Bridge).
        """
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                '/joint_trajectory_controller/follow_joint_trajectory not available')
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(self.gripper_joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        sec = int(self.gripper_time_sec)
        nsec = int((self.gripper_time_sec - sec) * 1e9)
        pt.time_from_start = MsgDuration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        goal.trajectory = traj

        send_future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=accept_timeout)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False
        # Signal the Gazebo attach helper as soon as the goal is ACCEPTED (the attach is
        # COMMAND-based, not tied to the controller result). This must NOT be gated on the
        # result future: in sim the gripper FollowJointTrajectory result can time out even
        # though the joints move -> gating here left the cube attached on release, so it
        # followed the arm to Home instead of being deposited.
        closed = ([round(float(p), 2) for p in positions]
                  == [round(float(p), 2) for p in self.gripper_closed_pos])
        self._gripper_closed_pub.publish(Bool(data=closed))
        resf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, resf, timeout_sec=result_timeout)
        if resf.result() is None:
            self.get_logger().warn('No gripper result (timeout) — command sent, continuing')
        # Small buffer so the servos are safely in position.
        time.sleep(0.3)
        # Yardstick: check the ACTUAL position against the programmed TARGET pose.
        # (0.8 s: the Gazebo PID needs a moment to catch up after the trajectory
        # ends — measuring too early produces false deviation warnings.)
        self._spin_for(0.8)   # frische /joint_states verarbeiten
        ist = [self.joint_pos.get(j) for j in self.gripper_joints]
        if all(v is not None for v in ist):
            soll = [float(p) for p in positions]
            err = max(abs(a - b) for a, b in zip(ist, soll))
            msg = (f'   Gripper ACTUAL=[{ist[0]:+.2f},{ist[1]:+.2f}] '
                   f'TARGET=[{soll[0]:+.2f},{soll[1]:+.2f}]')
            if err > 0.1:
                self.get_logger().warn(msg + f'  (deviation {err:.2f} rad!)')
            else:
                self.get_logger().info(msg + '  ✓')
        return True

    def _ensure_movegroup_client(self):
        with self._mg_lock:
            if self._mg_client is None:
                self._mg_client = ActionClient(self, MoveGroup, '/move_action')
            return self._mg_client

    def send_pose_goal(self, target_pose: PoseStamped, execute=True,
                       accept_timeout=20.0, result_timeout=120.0):
        """Send a pose goal to MoveIt MoveGroup action and optionally execute the resulting trajectory."""
        mgc = self._ensure_movegroup_client()
        self.get_logger().info('Waiting for /move_action...')
        if not mgc.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/move_action not available')
            return False

        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = 'arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = 0.8
        req.max_acceleration_scaling_factor = 0.8

        # Start state: use current joint states if available
        # (MoveIt will use current state if omitted)
        req.start_state.is_diff = True

        # Position constraint via a small bounding box around the target.
        # WICHTIG: keine Orientierungs-Constraint! Der Arm ist 5-DOF mit
        # position_only_ik=true (siehe config/kinematics.yaml) — eine
        # OrientationConstraint kann der Goal-Sampler nie erfuellen.
        # link_name = 'tcp' (fingertip frame): the ~9 cm gripper offset is thus
        # handled by FK, regardless of the orientation the IK finds.
        pc = PositionConstraint()
        pc.header.frame_id = target_pose.header.frame_id
        pc.link_name = self.tcp_link
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        tol = self.pos_tol
        sp.dimensions = [tol, tol, tol]
        bv.primitives.append(sp)
        bv.primitive_poses.append(target_pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)

        req.goal_constraints.append(c)

        # Ein einziger Durchgang: planen UND (optional) ausfuehren.
        # plan_only=False laesst move_group direkt ueber den
        # joint_trajectory_controller ausfuehren (wie RViz "Plan & Execute").
        goal_msg.planning_options.plan_only = not execute
        self.get_logger().info('Sending pose goal to move_group...')
        send_future = mgc.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=accept_timeout)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error('Keine Goal-Antwort von move_group (Timeout)')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('Goal von move_group abgelehnt')
            return False

        # Ausfuehrung kann deutlich laenger dauern als die Planung -> grosszuegiger Timeout
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout)
        if result_future.result() is None:
            self.get_logger().error(f'Kein Ergebnis nach {result_timeout}s (Timeout bei Ausfuehrung)')
            return False
        result = result_future.result().result
        code = result.error_code.val
        if code != 1:
            self.get_logger().error(f'Motion failed, code={code}')
            return False

        self.get_logger().info('Bewegung erfolgreich')
        return True


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CupPickAndPlace()
        if node.goto_only:
            node.goto_marker0()      # only drive the TCP to the marker-0 position
        else:
            node.execute_demo()      # call directly (no timer/no spin -> no re-entrancy)
    except KeyboardInterrupt:
        print('\n✓ Demo unterbrochen vom Benutzer')
    except Exception as e:
        print(f'❌ Fehler in main: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


