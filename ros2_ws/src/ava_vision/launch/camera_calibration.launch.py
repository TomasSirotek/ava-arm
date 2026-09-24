#!/usr/bin/env python3
"""
Launch file for camera calibration.

This launch file starts the camera and calibration nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='2',
        description='Camera device ID (USB "HD Web Camera" = /dev/video2)'
    )
    
    checkerboard_width_arg = DeclareLaunchArgument(
        'checkerboard_width',
        default_value='9',
        description='Number of inner corners in width'
    )
    
    checkerboard_height_arg = DeclareLaunchArgument(
        'checkerboard_height',
        default_value='6',
        description='Number of inner corners in height'
    )
    
    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='0.025',
        description='Size of checkerboard squares in meters'
    )

    # IMPORTANT: calibrate at the SAME resolution you run detection at
    # (aruco_params.yaml uses 1280x720), otherwise the intrinsics won't match.
    frame_width_arg = DeclareLaunchArgument('frame_width', default_value='1280')
    frame_height_arg = DeclareLaunchArgument('frame_height', default_value='720')

    # Stable USB-webcam path (immune to video0<->video2 index swaps).
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0',
        description='Stable device path; wins over camera_id'
    )

    # Camera node
    camera_node = Node(
        package='ava_vision',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'camera_device': LaunchConfiguration('camera_device'),
            'frame_rate': 30.0,
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
        }]
    )
    
    # Calibration node
    calibration_node = Node(
        package='ava_vision',
        executable='camera_calibration_node',
        name='camera_calibration_node',
        output='screen',
        parameters=[{
            'checkerboard_width': LaunchConfiguration('checkerboard_width'),
            'checkerboard_height': LaunchConfiguration('checkerboard_height'),
            'square_size': LaunchConfiguration('square_size'),
            'min_captures': 20,
            'output_dir': 'calibration',
        }]
    )
    
    return LaunchDescription([
        camera_id_arg,
        checkerboard_width_arg,
        checkerboard_height_arg,
        square_size_arg,
        frame_width_arg,
        frame_height_arg,
        camera_device_arg,
        camera_node,
        calibration_node,
    ])
