#!/usr/bin/env python3
"""
Cube Pose Publisher Node for ava (ROS 2 Jazzy)

Turns the ArUco detection TF tree into a robot-frame pick target:

  * Looks up TF  target_frame (world) -> marker_frame (aruco_marker_0, the cube)
    and republishes it as a PoseStamped on /cup_pose, so the existing
    cup_pick_and_place.py can consume it unchanged.
  * Publishes RViz visualization Markers for the cube and the 4 table markers.
  * Adds/updates a BOX collision object 'cube' in the MoveIt planning scene.
  * Logs detected-vs-expected base_link positions of the 4 table markers
    (IDs 1..4) so the static camera transform (mount_yaw / offsets) can be
    calibrated and verified.

Design notes:
  * TF is looked up at Time() ("latest available"), never at "now" — looking
    up "now" against a transform that arrived a few ms ago throws an
    ExtrapolationException. Freshness is checked manually via the header stamp.
  * The marker sits on TOP of the cube, so the detected pose is the top face;
    cube_top_offset is subtracted along world -Z to get the cube center.
  * Orientation is published as identity: the 5-DOF arm uses position-only IK,
    so cup_pick_and_place uses only a PositionConstraint (no orientation).
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class CubePosePublisher(Node):
    def __init__(self):
        super().__init__('cube_pose_publisher_node')

        # --- Parameters ---
        self.declare_parameter('marker_frame', 'aruco_marker_0')
        self.declare_parameter('target_frame', 'world')
        # Cube dimensions (m): 3x3 footprint, 5 cm tall.
        self.declare_parameter('cube_size_x', 0.03)
        self.declare_parameter('cube_size_y', 0.03)
        self.declare_parameter('cube_size_z', 0.05)
        # base_link XY lies ON the table (z=0), cube sits on the table, so the cube
        # CENTER height is fixed = cube_size_z/2. We publish this fixed z instead of
        # the noisy marker-top z (marker z is only used for visibility/staleness).
        self.declare_parameter('cube_center_z', 0.025)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('pose_timeout_sec', 3.0)
        self.declare_parameter('smoothing_alpha', 0.3)      # EWMA factor (0..1, higher = less smoothing)
        self.declare_parameter('min_position_jump', 0.05)   # reject single-frame jumps larger than this (m)
        self.declare_parameter('collision_padding', 0.004)  # add to cube size for the collision box
        # Cube collision OFF by default: a side grasp wraps the fingers around the
        # cube, so a 'cube' collision object would block the grasp plan.
        self.declare_parameter('enable_planning_scene', False)
        self.declare_parameter('enable_marker', True)
        self.declare_parameter('enable_verification_log', True)
        # Table collision/visual box (base_link XY on table top, z=0).
        self.declare_parameter('enable_table', True)
        self.declare_parameter('table_size_x', 0.60)
        self.declare_parameter('table_size_y', 0.35)
        self.declare_parameter('table_size_z', 0.02)        # thickness (downward from z=0)
        self.declare_parameter('table_center_x', 0.0)
        self.declare_parameter('table_center_y', -0.095)
        self.declare_parameter('table_marker_frames',
                               ['aruco_marker_1', 'aruco_marker_2',
                                'aruco_marker_3', 'aruco_marker_4'])
        # JSON map of expected base_link [x, y] per table-marker ID, e.g.
        # '{"1":[0.20,0.20],"2":[0.20,-0.20],"3":[-0.20,-0.20],"4":[-0.20,0.20]}'
        self.declare_parameter('table_markers_expected', '')

        self.marker_frame = self.get_parameter('marker_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.cube_size_x = self.get_parameter('cube_size_x').value
        self.cube_size_y = self.get_parameter('cube_size_y').value
        self.cube_size_z = self.get_parameter('cube_size_z').value
        self.cube_center_z = self.get_parameter('cube_center_z').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.pose_timeout = self.get_parameter('pose_timeout_sec').value
        self.alpha = self.get_parameter('smoothing_alpha').value
        self.min_jump = self.get_parameter('min_position_jump').value
        self.collision_padding = self.get_parameter('collision_padding').value
        self.enable_planning_scene = self.get_parameter('enable_planning_scene').value
        self.enable_marker = self.get_parameter('enable_marker').value
        self.enable_verification_log = self.get_parameter('enable_verification_log').value
        self.enable_table = self.get_parameter('enable_table').value
        self.table_size = (self.get_parameter('table_size_x').value,
                           self.get_parameter('table_size_y').value,
                           self.get_parameter('table_size_z').value)
        self.table_center = (self.get_parameter('table_center_x').value,
                             self.get_parameter('table_center_y').value)
        self.table_marker_frames = list(self.get_parameter('table_marker_frames').value)
        self._table_added = False

        expected_str = self.get_parameter('table_markers_expected').value
        self.table_expected = {}
        if expected_str:
            try:
                raw = json.loads(expected_str)
                self.table_expected = {str(k): [float(v[0]), float(v[1])]
                                       for k, v in raw.items()}
            except Exception as e:
                self.get_logger().error(
                    f'table_markers_expected parse failed: {e}')

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, '/cup_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # --- Planning scene service (needed for cube AND/OR table collision) ---
        self.ps_client = None
        if self.enable_planning_scene or self.enable_table:
            self.ps_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # --- Grasp gate: while a pick is active, remove the cube collision so it
        # doesn't block the side grasp (latched topic from cup_pick_and_place). ---
        self._grasp_active = False
        self._cube_removed = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/grasp_active', self._grasp_active_cb, latched)

        # --- State ---
        self.smoothed = None        # last smoothed (x, y, z)
        self._last_warn = False
        self._verify_counter = 0
        self._reject_count = 0      # consecutive outlier rejections (re-seed after 5)
        self._sphere_clear_ticks = 30   # send DELETE for old corner spheres for a while

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'CubePosePublisher: {self.target_frame} -> {self.marker_frame} '
            f'=> /cup_pose (cube {self.cube_size_x}x{self.cube_size_y}x'
            f'{self.cube_size_z} m, center_z={self.cube_center_z} m, '
            f'table={self.enable_table}, cube_collision={self.enable_planning_scene})')

    # ------------------------------------------------------------------ #
    def on_timer(self):
        # Add the static table collision box once (when the service is ready).
        if self.enable_table and not self._table_added:
            self.add_table()

        center = self.lookup_cube_center()   # None if cube TF unavailable/stale
        cube_fresh = False
        if center is not None:
            x, y, z = center
            if self.smoothed is not None and self._reject_count < 5:
                jump = math.dist((x, y, z), self.smoothed)
                if jump > self.min_jump:
                    self._reject_count += 1
                    self.get_logger().warn(
                        f'Cube pose jump {jump*100:.1f} cm > {self.min_jump*100:.1f} cm '
                        f'— ignoring outlier ({self._reject_count}/5)',
                        throttle_duration_sec=2.0)
                else:
                    self._reject_count = 0
                    x = self.alpha * x + (1 - self.alpha) * self.smoothed[0]
                    y = self.alpha * y + (1 - self.alpha) * self.smoothed[1]
                    z = self.alpha * z + (1 - self.alpha) * self.smoothed[2]
                    self.smoothed = (x, y, z)
                    cube_fresh = True
            else:
                # First sample, or too many consecutive rejects (cube was moved
                # far on purpose) -> (re)seed to the new position.
                self._reject_count = 0
                self.smoothed = (x, y, z)
                cube_fresh = True

        # /cup_pose + collision object only on a FRESH detection (pick safety:
        # never feed a stale pose to the arm).
        if cube_fresh:
            self.publish_pose(*self.smoothed)
            if self.enable_planning_scene:
                if self._grasp_active:
                    # Pick in progress -> drop the cube object once so it does
                    # not block the grasp; restore it when the pick is done.
                    if not self._cube_removed:
                        self.remove_cube_collision()
                        self._cube_removed = True
                else:
                    self._cube_removed = False
                    self.update_collision_object(*self.smoothed)

        # Markers + verification run EVERY tick — independent of cube visibility —
        # so /visualization_marker_array is never empty (no red "no topic" in RViz)
        # and the table markers + last-known cube stay visible.
        if self.enable_marker:
            self.publish_markers(self.smoothed)   # None -> only table markers
        if self.enable_verification_log:
            self.verify_table_markers()

    # ------------------------------------------------------------------ #
    def lookup_cube_center(self):
        """Return (x, y, z) cube center in target_frame, or None if unavailable/stale."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.marker_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'No TF {self.target_frame} -> {self.marker_frame} yet '
                '(cube not visible?)', throttle_duration_sec=5.0)
            self._last_warn = True
            return None

        # Freshness check against the transform timestamp.
        stamp = Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds > 0:
            age = self.get_clock().now() - stamp
            if age > Duration(seconds=self.pose_timeout):
                self.get_logger().warn(
                    f'Cube TF stale ({age.nanoseconds/1e9:.1f}s) — '
                    'cube probably not visible', throttle_duration_sec=5.0)
                return None

        if self._last_warn:
            self.get_logger().info('Cube TF acquired.')
            self._last_warn = False

        t = tf.transform.translation
        # base_link XY is on the table -> the cube center height is fixed
        # (cube_center_z); the marker-top z is noisy and only used for
        # visibility/staleness, not for the published pose.
        return (t.x, t.y, self.cube_center_z)

    # ------------------------------------------------------------------ #
    def publish_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0   # identity (position-only IK)
        self.pose_pub.publish(msg)

    # ------------------------------------------------------------------ #
    def publish_markers(self, cube_xyz):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # (Table is the wood-brown collision object via object_colors — not here.)

        # Cube marker (last-known position); skipped only if never seen.
        if cube_xyz is not None:
            x, y, z = cube_xyz
            cube = Marker()
            cube.header.frame_id = self.target_frame
            cube.header.stamp = now
            cube.ns = 'cube'
            cube.id = 0
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = z
            cube.pose.orientation.w = 1.0
            cube.scale.x = self.cube_size_x
            cube.scale.y = self.cube_size_y
            cube.scale.z = self.cube_size_z
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.1, 0.6, 1.0, 0.9
            arr.markers.append(cube)

        # One-time-ish: DELETE any old corner-marker spheres still cached in RViz
        # from a previous version (sent for the first few seconds so late RViz
        # subscribers also get it).
        if self._sphere_clear_ticks > 0:
            self._sphere_clear_ticks -= 1
            for i in range(1, len(self.table_marker_frames) + 1):
                d = Marker()
                d.header.frame_id = self.target_frame
                d.ns = 'table_markers'
                d.id = i
                d.action = Marker.DELETE
                arr.markers.append(d)

        self.marker_pub.publish(arr)

    # ------------------------------------------------------------------ #
    def update_collision_object(self, x, y, z):
        if self.ps_client is None or not self.ps_client.service_is_ready():
            return
        co = CollisionObject()
        co.id = 'cube'
        co.header.frame_id = self.target_frame
        co.operation = CollisionObject.ADD   # ADD overwrites an object with same id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        pad = self.collision_padding
        box.dimensions = [self.cube_size_x + pad,
                          self.cube_size_y + pad,
                          self.cube_size_z + pad]

        pose = PoseStamped().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        # Color the cube blue (distinct from the wood-brown table).
        oc = ObjectColor()
        oc.id = 'cube'
        oc.color = ColorRGBA(r=0.10, g=0.55, b=1.0, a=1.0)
        scene.object_colors.append(oc)
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.ps_client.call_async(req)

    # ------------------------------------------------------------------ #
    def remove_cube_collision(self):
        if self.ps_client is None or not self.ps_client.service_is_ready():
            return
        co = CollisionObject()
        co.id = 'cube'
        co.header.frame_id = self.target_frame
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        scene.is_diff = True
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.ps_client.call_async(req)
        self.get_logger().info('Cube collision removed (grasp active).')

    def _grasp_active_cb(self, msg: Bool):
        if msg.data != self._grasp_active:
            self.get_logger().info(f'grasp_active = {msg.data}')
        self._grasp_active = msg.data

    # ------------------------------------------------------------------ #
    def add_table(self):
        """Add the static table collision box once (top at z=0, extending down)."""
        if self.ps_client is None or not self.ps_client.service_is_ready():
            return
        sx, sy, sz = self.table_size
        cx, cy = self.table_center

        co = CollisionObject()
        co.id = 'table'
        co.header.frame_id = self.target_frame
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]

        pose = PoseStamped().pose
        pose.position.x = cx
        pose.position.y = cy
        # Top surface ~1 cm below z=0 so it does not collide with base_link.
        pose.position.z = -0.01 - sz / 2.0
        pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        # Color the collision object wood-brown so RViz Scene Geometry shows it
        # brown instead of the default green (distinct from the blue cube).
        oc = ObjectColor()
        oc.id = 'table'
        oc.color = ColorRGBA(r=0.60, g=0.40, b=0.20, a=1.0)
        scene.object_colors.append(oc)
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.ps_client.call_async(req)
        self._table_added = True
        self.get_logger().info(
            f'Table collision added (wood-brown): {sx}x{sy} m at ({cx},{cy}), top ~z=-0.01')

    # ------------------------------------------------------------------ #
    def verify_table_markers(self):
        if not self.table_expected:
            return
        # Throttle to ~ every 2 s regardless of publish rate.
        self._verify_counter += 1
        if self._verify_counter < max(int(self.publish_rate * 2.0), 1):
            return
        self._verify_counter = 0

        errors = []
        lines = []
        for frame in self.table_marker_frames:
            marker_id = frame.split('_')[-1]
            pos = self.lookup_xyz(frame)
            if pos is None:
                continue
            dx, dy = pos[0], pos[1]
            exp = self.table_expected.get(marker_id)
            if exp is None:
                lines.append(f'  ID {marker_id}: detected ({dx:+.3f},{dy:+.3f}) '
                             '[no expected]')
                continue
            err = math.dist((dx, dy), (exp[0], exp[1]))
            errors.append(err)
            lines.append(
                f'  ID {marker_id}: detected ({dx:+.3f},{dy:+.3f}) '
                f'expected ({exp[0]:+.3f},{exp[1]:+.3f}) err={err*100:.1f} cm')

        if lines:
            rms = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else float('nan')
            self.get_logger().info(
                'Table-marker verification (base_link X/Y):\n' +
                '\n'.join(lines) +
                (f'\n  RMS error = {rms*100:.1f} cm' if errors else ''))

    # ------------------------------------------------------------------ #
    def lookup_xyz(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)


def main(args=None):
    rclpy.init(args=args)
    node = CubePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
