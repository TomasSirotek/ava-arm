from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_gamepad = LaunchConfiguration('use_gamepad')
    gamepad_params = LaunchConfiguration('gamepad_params')

    moveit_pkg = FindPackageShare('ava_moveit_config')
    gamepad_pkg = FindPackageShare('ava_control')

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg, 'launch', 'gazebo_moveit.launch.py'])
        ),
        launch_arguments={
            'use_rviz': use_rviz,
        }.items(),
    )

    gamepad_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gamepad_pkg, 'launch', 'gamepad_control.launch.py'])
        ),
        condition=IfCondition(use_gamepad),
        launch_arguments={
            'params_file': gamepad_params,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gamepad', default_value='true'),
        DeclareLaunchArgument(
            'gamepad_params',
            default_value=PathJoinSubstitution([
                gamepad_pkg,
                'config',
                'gamepad.yaml',
            ]),
        ),
        base_stack,
        gamepad_stack,
    ])
