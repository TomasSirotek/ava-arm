#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ava_control'),
                'config',
                'real_robot.yaml',
            ]),
        ),
        Node(
            package='ava_control',
            executable='esp32_servo_bridge_node',
            name='esp32_servo_bridge_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
