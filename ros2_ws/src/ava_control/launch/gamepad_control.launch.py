#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    start_joy_node = LaunchConfiguration('start_joy_node')
    joy_device_id = LaunchConfiguration('joy_device_id')
    joy_deadzone = LaunchConfiguration('joy_deadzone')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_joy_node', default_value='true'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.15'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ava_control'),
                'config',
                'gamepad.yaml',
            ]),
        ),
        Node(
            condition=IfCondition(start_joy_node),
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            parameters=[
                {
                    'device_id': joy_device_id,
                    'deadzone': joy_deadzone,
                    'autorepeat_rate': 20.0,
                }
            ],
        ),
        Node(
            package='ava_control',
            executable='gamepad_cartesian_node',
            name='gamepad_cartesian_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
