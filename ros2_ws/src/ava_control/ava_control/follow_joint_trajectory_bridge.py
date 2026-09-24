#!/usr/bin/env python3
"""Bridge MoveIt FollowJointTrajectory actions to JointTrajectory topic commands."""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class FollowJointTrajectoryBridge(Node):
    """Action server that forwards trajectory goals to the hardware bridge topic."""

    def __init__(self):
        super().__init__('follow_joint_trajectory_bridge')

        self.declare_parameter(
            'action_name', '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.declare_parameter(
            'command_topic', '/joint_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter('min_execution_time_sec', 0.2)

        action_name = self.get_parameter('action_name').value
        command_topic = self.get_parameter('command_topic').value
        self.min_execution_time_sec = float(
            self.get_parameter('min_execution_time_sec').value
        )

        self.command_pub = self.create_publisher(JointTrajectory, command_topic, 10)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f'FollowJointTrajectory bridge ready: {action_name} -> {command_topic}'
        )

    def goal_callback(self, goal_request: FollowJointTrajectory.Goal):
        if not goal_request.trajectory.joint_names:
            self.get_logger().warn('Rejecting goal: empty joint_names')
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().warn('Rejecting goal: no trajectory points')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        # Forward the full trajectory. Hardware bridge maps joints it supports.
        self.command_pub.publish(goal.trajectory)

        # Approximate execution duration by last point time.
        last_point = goal.trajectory.points[-1]
        duration_sec = (
            float(last_point.time_from_start.sec)
            + float(last_point.time_from_start.nanosec) / 1e9
        )
        duration_sec = max(self.min_execution_time_sec, duration_sec)

        remaining = duration_sec
        while remaining > 0.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Goal canceled'
                return result

            sleep_step = min(0.1, remaining)
            time.sleep(sleep_step)
            remaining -= sleep_step

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ''
        return result


def main():
    rclpy.init()
    node = FollowJointTrajectoryBridge()
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
