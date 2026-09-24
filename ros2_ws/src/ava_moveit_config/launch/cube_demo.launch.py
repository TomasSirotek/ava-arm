"""Sim pick-and-place with the overhead ArUco camera (ava).

Includes the mock-hardware MoveIt demo (demo.launch.py: robot_state_publisher,
ros2_control mock, move_group, RViz) and the vision bringup, then optionally
runs the pick-and-place sequence.

NOTE: demo.launch.py uses mock hardware — there is NO Gazebo and NO simulated
camera. Two modes:
  * use_camera:=true  (Mode A, default) — plug in the real USB webcam pointed at
    the real printed cube; the arm is simulated in RViz but perception is real.
  * use_camera:=false (Mode B) — publish a fake hardcoded /cup_pose via
    publish_cup_marker.py to test the MoveIt pick logic without a camera.

Example:
  ros2 launch ava_moveit_config cube_demo.launch.py \
      run_demo:=true use_camera:=true camera_id:=0 camera_info_url:=/path/intrinsics.yaml
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    run_demo = LaunchConfiguration('run_demo')
    use_camera = LaunchConfiguration('use_camera')
    camera_id = LaunchConfiguration('camera_id')
    camera_info_url = LaunchConfiguration('camera_info_url')
    mount_yaw = LaunchConfiguration('mount_yaw')

    moveit_pkg = FindPackageShare('ava_moveit_config')
    vision_pkg = FindPackageShare('ava_vision')

    # 1) Mock-hardware MoveIt demo (arm in RViz, move_group, controllers).
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg, 'launch', 'demo.launch.py'])),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    # 2a) Real camera perception (Mode A).
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
                condition=IfCondition(use_camera),
            )
        ],
    )

    # 2b) Fake pose source (Mode B) — hardcoded /cup_pose for MoveIt-only tests.
    fake_pose = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', PathJoinSubstitution([moveit_pkg, 'publish_cup_marker.py'])],
                output='screen',
                condition=UnlessCondition(use_camera),
            )
        ],
    )

    # 3) Pick-and-place sequence (after everything is up), if run_demo:=true.
    pick_and_place = TimerAction(
        period=14.0,
        actions=[
            ExecuteProcess(
                cmd=['python3',
                     PathJoinSubstitution([moveit_pkg, 'cup_pick_and_place.py']),
                     '--ros-args', '--params-file',
                     PathJoinSubstitution([moveit_pkg, 'config', 'pick_place_params.yaml'])],
                output='screen',
                condition=IfCondition(run_demo),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_demo', default_value='false',
                              description='Run the pick-and-place sequence'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='true=real webcam perception, false=fake /cup_pose'),
        DeclareLaunchArgument('camera_id', default_value='2'),
        DeclareLaunchArgument('camera_info_url', default_value='',
                              description='Path to calibrated intrinsics YAML'),
        DeclareLaunchArgument('mount_yaw', default_value='',
                              description='Override camera mounting yaw (radians)'),
        demo,
        vision,
        fake_pose,
        pick_and_place,
    ])
