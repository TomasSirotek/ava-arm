#!/usr/bin/env python3
# TODO(so101-migration): written for the OLD OmArm Zero kinematics
# (5-DOF, wrist roll BEFORE wrist flex, two-finger gripper). Joint names were
# renamed to the SO-101 convention, but the second-finger mirroring (SO-101 has one jaw, so this node is unused) still describe the old
# arm and are NOT valid for the SO-101 model in ava_description. Rewrite before use.

import copy

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperMimicBridge(Node):
    """Expands 6-DOF trajectory commands by mirroring gripper into gripper_second_finger."""

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
        # Avoid loops: if gripper_second_finger is already present, forward nothing.
        if 'gripper_second_finger' in msg.joint_names:
            return

        if 'gripper' not in msg.joint_names:
            # No gripper command in this message.
            return

        j6_idx = msg.joint_names.index('gripper')
        out = copy.deepcopy(msg)
        out.joint_names.append('gripper_second_finger')

        for point in out.points:
            self._append_mirror(point, j6_idx)

        self.pub.publish(out)
        self._expanded_count += 1
        if out.points and len(out.points[0].positions) > j6_idx:
            j6_value = out.points[0].positions[j6_idx]
            self.get_logger().info(
                f'Expanded command #{self._expanded_count}: mirrored gripper={j6_value:.3f} to gripper_second_finger'
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
