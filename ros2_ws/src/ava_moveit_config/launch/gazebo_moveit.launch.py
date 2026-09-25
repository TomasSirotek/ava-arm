from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")
    rviz_use_sim_time = LaunchConfiguration("rviz_use_sim_time")

    moveit_pkg = FindPackageShare("ava_moveit_config")
    desc_pkg = FindPackageShare("ava_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_software_rendering", default_value="1"),
            DeclareLaunchArgument("rviz_use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([desc_pkg, "launch", "gazebo.launch.py"])
                )
            ),
            # The SRDF's virtual joint makes "world" MoveIt's planning frame, but the URDF
            # starts at base_footprint, so nothing else publishes it. Robot sits at the origin.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_footprint",
                arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
                parameters=[{"use_sim_time": True}],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([moveit_pkg, "launch", "move_group.launch.py"])
                        ),
                        launch_arguments={
                            "use_sim_time": "true",
                            "allow_trajectory_execution": "true",
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([moveit_pkg, "launch", "moveit_rviz.launch.py"])
                        ),
                        condition=IfCondition(use_rviz),
                        launch_arguments={
                            "use_sim_time": rviz_use_sim_time,
                            "rviz_software_rendering": rviz_software_rendering,
                        }.items(),
                    )
                ],
            ),
        ]
    )
