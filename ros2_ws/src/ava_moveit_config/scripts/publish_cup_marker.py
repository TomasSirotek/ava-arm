#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

# Cup position: x=0.22 (innerhalb der Armreichweite, max ~0.25), y=0.0, z=0.04 (auf Grund)
CUP_X = 0.22
CUP_Y = 0.0
CUP_Z = 0.04


class CupMarker(Node):
    def __init__(self):
        super().__init__('cup_marker')
        self.pub_marker = self.create_publisher(Marker, 'visualization_marker', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/cup_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)
        self.timer_pose = self.create_timer(0.5, self.publish_pose)

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'demo'
        m.id = 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = Point(x=CUP_X, y=CUP_Y, z=CUP_Z)
        m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        m.scale.x = 0.07
        m.scale.y = 0.07
        m.scale.z = 0.08
        m.color.r = 0.9
        m.color.g = 0.8
        m.color.b = 0.2
        m.color.a = 1.0
        self.pub_marker.publish(m)

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = CUP_X
        pose.pose.position.y = CUP_Y
        pose.pose.position.z = CUP_Z
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = CupMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
