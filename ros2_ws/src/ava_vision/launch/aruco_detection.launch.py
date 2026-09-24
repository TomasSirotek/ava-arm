#!/usr/bin/env python3
"""
Lightweight debug launch: camera_node + aruco_detector_node only.

Useful for checking marker detection (rqt_image_view /aruco/debug_image)
without the static transform / cube pose pipeline. For the full pick-and-place
perception stack use vision_bringup.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('ava_vision')
    params_file = os.path.join(pkg, 'config', 'aruco_params.yaml')

    camera_id = int(LaunchConfiguration('camera_id').perform(context))
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)

    camera_overrides = {'camera_id': camera_id}
    if camera_info_url != '':
        camera_overrides['camera_info_url'] = camera_info_url

    camera_node = Node(
        package='ava_vision', executable='camera_node',
        name='camera_node', output='screen',
        parameters=[params_file, camera_overrides],
    )
    detector_node = Node(
        package='ava_vision', executable='aruco_detector_node',
        name='aruco_detector_node', output='screen',
        parameters=[params_file],
    )
    return [camera_node, detector_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='2'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        OpaqueFunction(function=_setup),
    ])
