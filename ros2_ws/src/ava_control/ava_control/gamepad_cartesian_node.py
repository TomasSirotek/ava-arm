#!/usr/bin/env python3
"""Direct gamepad control in joint space (no IK/MoveIt planning)."""

from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GamepadCartesianNode(Node):
    def __init__(self) -> None:
        super().__init__('gamepad_cartesian_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('command_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('loop_hz', 20.0)
        self.declare_parameter('joint_step_rad', 0.03)
        self.declare_parameter('gripper_step_rad', 0.02)
        self.declare_parameter('traj_time_sec', 0.20)

        # Requested mapping
        self.declare_parameter('axis_joint1', 2)   # Right stick up/down
        self.declare_parameter('axis_joint2', 3)   # Right stick left/right
        self.declare_parameter('axis_joint3', 1)   # Left stick up/down
        self.declare_parameter('axis_joint4', 0)   # Left stick left/right
        self.declare_parameter('btn_joint5_pos', 10)   # R1+
        self.declare_parameter('axis_joint5_neg', 5)  # R2- as negative trigger-axis input
        self.declare_parameter('btn_joint6_pos', 9)    # L1+
        self.declare_parameter('axis_joint6_neg', 4)  # L2- as negative trigger-axis input

        self.declare_parameter('btn_home', 3)
        self.declare_parameter('joint_names', ['Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4', 'Revolute 5', 'Revolute 6'])
        self.declare_parameter('home_positions', [1.57, -1.57, -3.14, -3.14, -1.57, 0.0])

        self.joy_topic = self.get_parameter('joy_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.loop_hz = float(self.get_parameter('loop_hz').value)
        self.joint_step_rad = float(self.get_parameter('joint_step_rad').value)
        self.gripper_step_rad = float(self.get_parameter('gripper_step_rad').value)
        self.traj_time_sec = float(self.get_parameter('traj_time_sec').value)

        self.axis_joint1 = int(self.get_parameter('axis_joint1').value)
        self.axis_joint2 = int(self.get_parameter('axis_joint2').value)
        self.axis_joint3 = int(self.get_parameter('axis_joint3').value)
        self.axis_joint4 = int(self.get_parameter('axis_joint4').value)
        self.btn_joint5_pos = int(self.get_parameter('btn_joint5_pos').value)
        self.axis_joint5_neg = int(self.get_parameter('axis_joint5_neg').value)
        self.btn_joint6_pos = int(self.get_parameter('btn_joint6_pos').value)
        self.axis_joint6_neg = int(self.get_parameter('axis_joint6_neg').value)
        self.btn_home = int(self.get_parameter('btn_home').value)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.home_positions = list(self.get_parameter('home_positions').value)

        self.latest_axes: List[float] = []
        self.latest_buttons: List[int] = []
        self.prev_buttons: List[int] = []
        self.current_joint_map: Dict[str, float] = {}

        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(Joy, self.joy_topic, self.on_joy, joy_qos)
        self.create_subscription(JointState, self.joint_state_topic, self.on_joint_state, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(JointTrajectory, self.command_topic, 10)
        self.create_timer(1.0 / self.loop_hz, self.control_tick)

        self.get_logger().info('Gamepad joint-space node ready (no IK)')

    def on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_map[name] = pos

    def on_joy(self, msg: Joy) -> None:
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(self.latest_buttons)

    def _axis(self, idx: int) -> float:
        if idx < 0 or idx >= len(self.latest_axes):
            return 0.0
        value = self.latest_axes[idx]
        return 0.0 if abs(value) < self.deadzone else value

    def _btn(self, idx: int) -> int:
        if idx < 0 or idx >= len(self.latest_buttons):
            return 0
        return self.latest_buttons[idx]

    def _axis_negative(self, idx: int) -> float:
        return max(0.0, -self._axis(idx))

    def _axis_positive(self, idx: int) -> float:
        return max(0.0, self._axis(idx))

    def _button_edge(self, idx: int) -> bool:
        if idx < 0 or idx >= len(self.latest_buttons) or idx >= len(self.prev_buttons):
            return False
        return self.latest_buttons[idx] == 1 and self.prev_buttons[idx] == 0

    def _update_prev_buttons(self) -> None:
        if self.latest_buttons:
            self.prev_buttons = self.latest_buttons.copy()

    def _publish_joint_target(self, positions: List[float], sec: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int((sec - int(sec)) * 1e9)

        traj.points = [point]
        self.cmd_pub.publish(traj)

    def control_tick(self) -> None:
        if not self.latest_axes or not self.current_joint_map:
            return

        if self._button_edge(self.btn_home) and len(self.home_positions) == len(self.joint_names):
            self._publish_joint_target(self.home_positions, sec=1.2)
            self.get_logger().info('Home command sent')
            self._update_prev_buttons()
            return

        deltas = [0.0] * len(self.joint_names)
        if len(deltas) > 0:
            deltas[0] = self._axis(self.axis_joint1) * self.joint_step_rad
        if len(deltas) > 1:
            deltas[1] = self._axis(self.axis_joint2) * self.joint_step_rad
        if len(deltas) > 2:
            deltas[2] = self._axis(self.axis_joint3) * self.joint_step_rad
        if len(deltas) > 3:
            deltas[3] = self._axis(self.axis_joint4) * self.joint_step_rad
        if len(deltas) > 4:
            deltas[4] = (float(self._btn(self.btn_joint5_pos)) - self._axis_negative(self.axis_joint5_neg)) * self.joint_step_rad
        if len(deltas) > 5:
            deltas[5] = (float(self._btn(self.btn_joint6_pos)) - self._axis_negative(self.axis_joint6_neg)) * self.gripper_step_rad

        if not any(abs(delta) > 1e-6 for delta in deltas):
            self._update_prev_buttons()
            return

        target = [self.current_joint_map.get(joint, 0.0) + delta for joint, delta in zip(self.joint_names, deltas)]
        self._publish_joint_target(target, self.traj_time_sec)
        self._update_prev_buttons()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GamepadCartesianNode()
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
