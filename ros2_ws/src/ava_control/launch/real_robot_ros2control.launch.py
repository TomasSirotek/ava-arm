#!/usr/bin/env python3
"""ROS2 Hardware Bridge Launch"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    home_on_start = LaunchConfiguration('home_on_start')
    home_wait_sec = LaunchConfiguration('home_wait_sec')
    start_at_ros_zero = LaunchConfiguration('start_at_ros_zero')

    declared_arguments = [
        DeclareLaunchArgument('transport', default_value='http'),
        DeclareLaunchArgument('config_file', default_value='hardware.yaml'),
        DeclareLaunchArgument('home_on_start', default_value='true'),
        DeclareLaunchArgument('home_wait_sec', default_value='1.5'),
        DeclareLaunchArgument('start_at_ros_zero', default_value='false'),
    ]

    config_path = PathJoinSubstitution([
        FindPackageShare('ava_control'),
        'config',
        config_file,
    ])
    
    hardware_bridge = Node(
        package='ava_control',
        executable='hardware_bridge_node',
        name='hardware_bridge',
        output='both',
        emulate_tty=True,
        parameters=[
            {'transport': LaunchConfiguration('transport')},
            {'config_file': config_path},
            {'publish_rate': 50.0},
            {'home_on_start': home_on_start},
            {'home_wait_sec': home_wait_sec},
            {'start_at_ros_zero': start_at_ros_zero},
        ],
    )
    
    ld = LaunchDescription(declared_arguments)
    ld.add_action(hardware_bridge)
    return ld
