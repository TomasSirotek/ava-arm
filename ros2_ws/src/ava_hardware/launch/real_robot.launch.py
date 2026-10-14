"""Bring up the real Ava arm: ESP32 + PCA9685 PWM servos over USB serial.

    ros2 launch ava_hardware real_robot.launch.py serial_port:=/dev/ttyUSB0 use_rviz:=false

Starts robot_state_publisher (robot_description built with ros2_control_hardware_type:=real),
the controller manager with ava_hardware/AvaServoSystem, joint_state_broadcaster and
joint_trajectory_controller (same names/topics as Gazebo), and optionally RViz.

WARNING: activating the hardware commands every joint to 0 rad, so the arm jumps to the home
pose at full servo speed as soon as the controller manager starts. Keep clear of it.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    description_share = get_package_share_directory('ava_description')
    hardware_share = get_package_share_directory('ava_hardware')

    robot_description_xml = xacro.process_file(
        os.path.join(description_share, 'urdf', 'ava.urdf.xacro'),
        mappings={
            'ros2_control_hardware_type': 'real',
            'serial_port': LaunchConfiguration('serial_port').perform(context),
        }).toxml()
    robot_description = {'robot_description': robot_description_xml}

    controllers_file = os.path.join(hardware_share, 'config', 'real_controllers.yaml')
    rviz_config_file = os.path.join(description_share, 'config', 'display.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Jazzy: the controller manager takes the URDF from the /robot_description topic.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_file],
        remappings=[('~/robot_description', '/robot_description')],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='USB serial port of the ESP32 servo driver'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the ava_description display config'),
        OpaqueFunction(function=launch_setup),
    ])
