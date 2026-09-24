"""Overhead ArUco vision bringup for ava.

Starts:
  * camera_node            — USB webcam capture (image + camera_info)
  * aruco_detector_node    — ArUco detection, broadcasts camera_optical_frame->aruco_marker_<id>
  * static_transform_publisher — base_link -> camera_optical_frame (from camera_extrinsics.yaml)
  * cube_pose_publisher    — TF world->aruco_marker_0 => /cup_pose + RViz markers + collision object

The robot's own TF (world -> base_link -> ...) is expected to come from the
robot_state_publisher started by the MoveIt launch (demo.launch.py /
real_moveit.launch.py).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('ava_vision')

    params_file = LaunchConfiguration('params_file').perform(context)
    extrinsics_file = LaunchConfiguration('extrinsics_file').perform(context)
    camera_id = LaunchConfiguration('camera_id').perform(context)
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)
    mount_yaw = LaunchConfiguration('mount_yaw').perform(context)

    # Load static extrinsics (base_link -> camera_optical_frame).
    with open(extrinsics_file) as f:
        ext = yaml.safe_load(f) or {}
    cx = str(ext.get('camera_x', 0.0))
    cy = str(ext.get('camera_y', 0.0))
    cz = str(ext.get('camera_z', 0.50))
    roll = str(ext.get('camera_roll', 3.14159))
    pitch = str(ext.get('camera_pitch', 0.0))
    # CLI mount_yaw overrides the yaml value when set (non-empty).
    yaw = mount_yaw if mount_yaw != '' else str(ext.get('camera_yaw', 0.0))

    camera_overrides = {'camera_id': int(camera_id)}
    if camera_info_url != '':
        camera_overrides['camera_info_url'] = camera_info_url

    camera_node = Node(
        package='ava_vision',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params_file, camera_overrides],
    )

    detector_node = Node(
        package='ava_vision',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[params_file],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_tf',
        output='screen',
        arguments=[
            '--x', cx, '--y', cy, '--z', cz,
            '--roll', roll, '--pitch', pitch, '--yaw', yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
    )

    cube_pose_node = Node(
        package='ava_vision',
        executable='cube_pose_publisher',
        name='cube_pose_publisher_node',
        output='screen',
        parameters=[params_file],
    )

    return [camera_node, detector_node, static_tf, cube_pose_node]


def generate_launch_description():
    pkg = get_package_share_directory('ava_vision')
    default_params = os.path.join(pkg, 'config', 'aruco_params.yaml')
    default_extrinsics = os.path.join(pkg, 'config', 'camera_extrinsics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('extrinsics_file', default_value=default_extrinsics),
        DeclareLaunchArgument('camera_id', default_value='2'),
        DeclareLaunchArgument('camera_info_url', default_value='',
                              description='Path to calibrated intrinsics YAML'),
        DeclareLaunchArgument('mount_yaw', default_value='',
                              description='Override camera_yaw from extrinsics (radians)'),
        OpaqueFunction(function=_launch_setup),
    ])
