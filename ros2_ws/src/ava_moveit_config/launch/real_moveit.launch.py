from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ava", package_name="ava_moveit_config"
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    transport = LaunchConfiguration("transport")
    config_file = LaunchConfiguration("config_file")
    home_on_start = LaunchConfiguration("home_on_start")
    home_wait_sec = LaunchConfiguration("home_wait_sec")
    start_at_ros_zero = LaunchConfiguration("start_at_ros_zero")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    real_control_pkg = FindPackageShare("ava_control")
    moveit_pkg = FindPackageShare("ava_moveit_config")

    hardware_config_path = PathJoinSubstitution([real_control_pkg, "config", config_file])

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

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    hardware_bridge_node = Node(
        package="ava_control",
        executable="hardware_bridge_node",
        name="hardware_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"transport": transport},
            {"config_file": hardware_config_path},
            {"publish_rate": 50.0},
            {"home_on_start": home_on_start},
            {"home_wait_sec": home_wait_sec},
            {"start_at_ros_zero": start_at_ros_zero},
        ],
    )

    trajectory_action_bridge_node = Node(
        package="ava_control",
        executable="follow_joint_trajectory_bridge",
        name="follow_joint_trajectory_bridge",
        output="screen",
        parameters=[
            {"action_name": "/joint_trajectory_controller/follow_joint_trajectory"},
            {"command_topic": "/joint_trajectory_controller/joint_trajectory"},
            {"min_execution_time_sec": 0.2},
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
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
            PathJoinSubstitution([moveit_pkg, "config", "moveit.rviz"]),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("transport", default_value="serial"),
            DeclareLaunchArgument("config_file", default_value="hardware.yaml"),
            DeclareLaunchArgument("home_on_start", default_value="true"),
            DeclareLaunchArgument("home_wait_sec", default_value="1.5"),
            DeclareLaunchArgument("start_at_ros_zero", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_software_rendering", default_value="1"),
            rsp_node,
            hardware_bridge_node,
            trajectory_action_bridge_node,
            move_group_node,
            rviz_node,
        ]
    )
