#!/usr/bin/env python3
"""Wait for the first /cup_pose (real-camera cube pose) and spawn the Gazebo cube
model there, so the sim robot grasps a cube at the REAL detected position.
Spawns once (the real cube does not move during a pick), then exits."""
import os
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class SpawnCube(Node):
    def __init__(self):
        super().__init__('spawn_cube')
        self.declare_parameter('cube_sdf', '')
        self.declare_parameter('model_name', 'cube')
        self.declare_parameter('timeout_sec', 40.0)
        p = self.get_parameter('cube_sdf').value
        if not p:
            p = os.path.join(get_package_share_directory('ava_moveit_config'),
                             'models', 'cube_marker', 'cube.sdf')
        self.cube_sdf = p
        self.model_name = self.get_parameter('model_name').value
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.done = False
        self.create_subscription(PoseStamped, '/cup_pose', self._cb, 10)
        self.get_logger().info(
            f'Waiting for /cup_pose to spawn the cube ({self.cube_sdf}) ...')

    def _cb(self, msg: PoseStamped):
        if self.done:
            return
        self.done = True
        x, y, z = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.get_logger().info(f'/cup_pose received -> spawning cube @ ({x:.3f}, {y:.3f}, {z:.3f})')
        try:
            subprocess.run(
                ['ros2', 'run', 'ros_gz_sim', 'create', '-file', self.cube_sdf,
                 '-name', self.model_name,
                 '-x', str(x), '-y', str(y), '-z', str(z)],
                timeout=25.0)
            self.get_logger().info('Cube spawned.')
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnCube()
    try:
        end = node.get_clock().now().nanoseconds + int(node.timeout * 1e9)
        while rclpy.ok() and not node.done and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.done:
            node.get_logger().warn('No /cup_pose within the timeout — no cube spawned.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
