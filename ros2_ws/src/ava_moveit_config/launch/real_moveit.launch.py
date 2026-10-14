"""MoveIt on the real ava arm (PWM servos on PCA9685, ESP32 over USB serial).

Includes ava_hardware/real_robot.launch.py (robot_state_publisher, ros2_control with
ava_hardware/AvaServoSystem, joint_state_broadcaster, joint_trajectory_controller)
and starts move_group plus an optional MoveIt RViz.

  ros2 launch ava_moveit_config real_moveit.launch.py serial_port:=/dev/ttyUSB0 use_rviz:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_nodes(context):
    serial_port = LaunchConfiguration("serial_port").perform(context)
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ava", package_name="ava_moveit_config")
        .robot_description(
            mappings={"ros2_control_hardware_type": "real", "serial_port": serial_port}
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": False,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": rviz_software_rendering,
            "QT_OPENGL": "software",
        },
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("ava_moveit_config"), "config", "moveit.rviz"]),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")

    real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ava_hardware"), "launch", "real_robot.launch.py"]
            )
        ),
        launch_arguments={"serial_port": serial_port, "use_rviz": "false"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_software_rendering", default_value="1"),
            real_robot,
            # The SRDF's virtual joint makes "world" MoveIt's planning frame, but the URDF
            # starts at base_footprint, so nothing else publishes it. Robot sits at the origin.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_footprint",
                arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
            ),
            OpaqueFunction(function=_moveit_nodes),
        ]
    )
