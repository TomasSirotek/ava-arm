#!/usr/bin/env python3
"""
ROS2 Hardware Bridge Launch for AVA arm.
Starts Hardware Bridge Node for servo control via ESP32+PCA9685.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    
    declared_arguments = [
        DeclareLaunchArgument(
            'transport',
            default_value='http',
            description='Transport: http or serial'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='hardware.yaml',
            description='Hardware config file'
        ),
    ]
    
    # Substitutions
    transport = LaunchConfiguration('transport')
    config_file = LaunchConfiguration('config_file')
    
    # Hardware Bridge Node
    hardware_bridge = Node(
        package='ava_control',
        executable='hardware_bridge_node',
        name='hardware_bridge',
        output='both',
        emulate_tty=True,
        parameters=[
            {'transport': transport},
            {'config_file': config_file},
            {'publish_rate': 50.0},
        ],
    )
    
    ld = LaunchDescription(declared_arguments)
    ld.add_action(hardware_bridge)
    return ld
