#!/usr/bin/env python3
"""Print changed joystick axes/buttons to help configure index mapping."""

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class GamepadCalibrateNode(Node):
    def __init__(self) -> None:
        super().__init__('gamepad_calibrate_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('axis_threshold', 0.20)

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.axis_threshold = self.get_parameter('axis_threshold').get_parameter_value().double_value

        self.prev_axes: List[float] = []
        self.prev_buttons: List[int] = []

        self.create_subscription(Joy, joy_topic, self.on_joy, 20)

        self.get_logger().info('Gamepad calibrator started')
        self.get_logger().info('Move one stick/trigger or press a button; changed indices will be printed.')

    def on_joy(self, msg: Joy) -> None:
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        if not self.prev_axes:
            self.prev_axes = axes
        if not self.prev_buttons:
            self.prev_buttons = buttons

        axis_changes = []
        for i, value in enumerate(axes):
            prev = self.prev_axes[i] if i < len(self.prev_axes) else 0.0
            if abs(value - prev) >= self.axis_threshold:
                axis_changes.append((i, value))

        button_changes = []
        for i, value in enumerate(buttons):
            prev = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
            if value != prev:
                button_changes.append((i, value))

        if axis_changes or button_changes:
            if axis_changes:
                axis_msg = ', '.join([f'axis[{i}]={v:+.3f}' for i, v in axis_changes])
                self.get_logger().info(f'Axes changed: {axis_msg}')
            if button_changes:
                btn_msg = ', '.join([f'button[{i}]={v}' for i, v in button_changes])
                self.get_logger().info(f'Buttons changed: {btn_msg}')

        self.prev_axes = axes
        self.prev_buttons = buttons


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GamepadCalibrateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
