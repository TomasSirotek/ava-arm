import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():
    pkg_ros_gz_rbot = get_package_share_directory('ava_description')
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'ava.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description_xml = robot_description_config.toxml()
    robot_description = {'robot_description': robot_description_xml}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'publish_robot_description': True, 'use_sim_time': False}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_config_file = os.path.join(pkg_ros_gz_rbot, 'config', 'display.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    gripper_mimic_bridge = Node(
        package='ava_control',
        executable='gripper_mimic_bridge',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        gripper_mimic_bridge,
        rviz2,
    ])
        rviz2,
    ])
