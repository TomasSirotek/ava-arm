#!/usr/bin/env python3
"""Gazebo grasp helper: hold the cube while the gripper is commanded CLOSED.

cup_pick_and_place publishes /gripper_closed (Bool, latched) on every gripper
command (True=closed, False=open). While closed, this node teleports the cube model
to the tcp pose every tick, so the cube follows the gripper through
pick -> lift -> place. On open it pins the cube briefly at the release pose
(bleeds off residual velocity), then stops (cube rests where placed).

Teleporting goes through the ros_gz SetEntityPose SERVICE BRIDGE
(/world/<world>/set_pose, bridged in cube_gazebo.launch.py) with async calls at
rate_hz — millisecond latency, so the cube follows SMOOTHLY. The old subprocess
`gz service` path (~150-300 ms per call, 3-7 Hz effective) made the cube visibly
jump behind the gripper ("falls off and re-grips").

Command-based (not dependent on the gripper joint physically reaching an angle) ->
reliable even when the fingers stall on the cube in sim.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)


class GazeboGraspAttach(Node):
    def __init__(self):
        super().__init__('gazebo_grasp_attach')
        self.declare_parameter('world_name', 'empty')       # gz world (empty.sdf)
        self.declare_parameter('cube_model', 'cube')         # spawned model name
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('tcp_frame', 'tcp')           # fingertip frame
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('z_offset', 0.0)              # cube-center offset below tcp (m)

        self.world = self.get_parameter('world_name').value
        self.cube = self.get_parameter('cube_model').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_frame = self.get_parameter('tcp_frame').value
        self.z_off = float(self.get_parameter('z_offset').value)
        rate = float(self.get_parameter('rate_hz').value)

        self.closed = False
        self.attached = False
        self.last_pose = None      # last cube (x,y,z) while held
        self.settle = 0            # ticks to pin the cube after release (kill velocity)

        # Native ROS service client to the bridged gz set_pose (fast, no subprocess).
        self.cli = self.create_client(SetEntityPose, f'/world/{self.world}/set_pose')
        self._pending = None       # last in-flight async future (fire-and-forget)
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                f'Waiting for service /world/{self.world}/set_pose (ros_gz bridge) ...')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Bool, '/gripper_closed', self._grip_cb, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Gazebo grasp attach active ({rate:.0f} Hz, service bridge): model "
            f"'{self.cube}' follows '{self.tcp_frame}' while /gripper_closed == true.")

    def _grip_cb(self, msg: Bool):
        self.closed = bool(msg.data)

    def _set_pose(self, x, y, z, qx, qy, qz, qw):
        """Async fire-and-forget set_pose; skip if the previous call is still in flight
        (keeps exactly-once ordering, no queue buildup)."""
        if self._pending is not None and not self._pending.done():
            return
        req = SetEntityPose.Request()
        req.entity.name = self.cube
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.x = float(qx)
        req.pose.orientation.y = float(qy)
        req.pose.orientation.z = float(qz)
        req.pose.orientation.w = float(qw)
        self._pending = self.cli.call_async(req)

    def _tick(self):
        if not self.closed:
            if self.attached:
                self.attached = False
                self.settle = 25   # pin at the release pose to bleed off velocity
                self.get_logger().info('Gripper open -> cube released.')
            # After release the cube is gravity-free, so any residual velocity would make
            # it DRIFT. Keep re-setting it to the exact release pose for a few ticks.
            if self.settle > 0 and self.last_pose is not None:
                x, y, z = self.last_pose
                self._set_pose(x, y, z, 0.0, 0.0, 0.0, 1.0)
                self.settle -= 1
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tcp_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        t = tf.transform.translation
        if not self.attached:
            self.attached = True
            self.get_logger().info('Gripper closed -> cube grasped (follows tcp, upright).')
        # Follow the tcp POSITION but keep the cube UPRIGHT (identity orientation, marker
        # on top): the tcp frame is rotated (tcp +Z = finger axis), so inheriting its
        # orientation would tip the cube over. z_off keeps the center below the fingertips.
        self.last_pose = (t.x, t.y, t.z - self.z_off)
        self._set_pose(t.x, t.y, t.z - self.z_off, 0.0, 0.0, 0.0, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGraspAttach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
