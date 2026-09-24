"""Real-robot pick-and-place with the overhead ArUco camera (ava).

Identical perception half to cube_demo.launch.py, but drives the REAL hardware
via real_moveit.launch.py (hardware_bridge_node + follow_joint_trajectory_bridge
+ move_group) instead of mock hardware. The gripper path is byte-identical
(direct FollowJointTrajectory to /joint_trajectory_controller/...).

Always uses the real camera (use_camera is implied true).

First run WITHOUT run_demo to verify TF / /cup_pose / RViz, then enable it:
  ros2 launch ava_moveit_config cube_real.launch.py \
      camera_id:=0 camera_info_url:=/path/intrinsics.yaml transport:=serial run_demo:=true
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    run_demo = LaunchConfiguration('run_demo')
    transport = LaunchConfiguration('transport')
    config_file = LaunchConfiguration('config_file')
    camera_id = LaunchConfiguration('camera_id')
    camera_info_url = LaunchConfiguration('camera_info_url')
    mount_yaw = LaunchConfiguration('mount_yaw')

    moveit_pkg = FindPackageShare('ava_moveit_config')
    vision_pkg = FindPackageShare('ava_vision')

    real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg, 'launch', 'real_moveit.launch.py'])),
        launch_arguments={
            'use_rviz': use_rviz,
            'transport': transport,
            'config_file': config_file,
        }.items(),
    )

    vision = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vision_pkg, 'launch', 'vision_bringup.launch.py'])),
                launch_arguments={
                    'camera_id': camera_id,
                    'camera_info_url': camera_info_url,
                    'mount_yaw': mount_yaw,
                }.items(),
            )
        ],
    )

    pick_and_place = TimerAction(
        period=16.0,
        actions=[
            ExecuteProcess(
                cmd=['python3',
                     PathJoinSubstitution([moveit_pkg, 'cup_pick_and_place.py']),
                     '--ros-args', '--params-file',
                     # REAL parameter set: real gripper poses, safe real home pose and
                     # the marker-targeted place. The sim file (pick_place_params.yaml)
                     # must NOT be used on the real robot.
                     PathJoinSubstitution([moveit_pkg, 'config',
                                           'pick_place_params_real.yaml'])],
                output='screen',
                condition=IfCondition(run_demo),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_demo', default_value='false'),
        DeclareLaunchArgument('transport', default_value='serial'),
        DeclareLaunchArgument('config_file', default_value='hardware.yaml'),
        DeclareLaunchArgument('camera_id', default_value='2'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('mount_yaw', default_value=''),
        real,
        vision,
        pick_and_place,
    ])
