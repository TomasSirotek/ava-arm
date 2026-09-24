#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperMimicBridge(Node):
    """Expands 6-DOF trajectory commands by mirroring Revolute 6 into Revolute 7."""

    def __init__(self) -> None:
        super().__init__('gripper_mimic_bridge')
        self.cmd_topic = '/joint_trajectory_controller/joint_trajectory'
        self._expanded_count = 0
        self.sub = self.create_subscription(
            JointTrajectory,
            self.cmd_topic,
            self._on_cmd,
            10,
        )
        self.pub = self.create_publisher(JointTrajectory, self.cmd_topic, 10)
        self.get_logger().info('Gripper mimic bridge started')

    def _on_cmd(self, msg: JointTrajectory) -> None:
        # Avoid loops: if Revolute 7 is already present, forward nothing.
        if 'Revolute 7' in msg.joint_names:
            return

        if 'Revolute 6' not in msg.joint_names:
            # No gripper command in this message.
            return

        j6_idx = msg.joint_names.index('Revolute 6')
        out = copy.deepcopy(msg)
        out.joint_names.append('Revolute 7')

        for point in out.points:
            self._append_mirror(point, j6_idx)

        self.pub.publish(out)
        self._expanded_count += 1
        if out.points and len(out.points[0].positions) > j6_idx:
            j6_value = out.points[0].positions[j6_idx]
            self.get_logger().info(
                f'Expanded command #{self._expanded_count}: mirrored Revolute 6={j6_value:.3f} to Revolute 7'
            )

    @staticmethod
    def _append_mirror(point: JointTrajectoryPoint, j6_idx: int) -> None:
        if len(point.positions) > j6_idx:
            point.positions.append(point.positions[j6_idx])
        if len(point.velocities) > j6_idx:
            point.velocities.append(point.velocities[j6_idx])
        if len(point.accelerations) > j6_idx:
            point.accelerations.append(point.accelerations[j6_idx])
        if len(point.effort) > j6_idx:
            point.effort.append(point.effort[j6_idx])


def main() -> None:
    rclpy.init()
    node = GripperMimicBridge()
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
