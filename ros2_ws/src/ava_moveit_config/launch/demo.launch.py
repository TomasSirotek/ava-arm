import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _fake_hardware_robot_description_xml():
    desc_share = get_package_share_directory("ava_description")
    xacro_file = os.path.join(desc_share, "urdf", "ava.xacro")
    robot_xml = xacro.process_file(xacro_file).toxml()
    # Keep the same model, but replace Gazebo hardware with ros2_control mock hardware.
    robot_xml = robot_xml.replace(
        "gz_ros2_control/GazeboSimSystem", "mock_components/GenericSystem"
    )

    root = ET.fromstring(robot_xml)
    epsilon_upper_joints = {"shoulder_lift", "elbow_flex", "wrist_roll", "wrist_flex"}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        limit = joint.find("limit")
        if limit is None:
            continue
        if name in epsilon_upper_joints:
            limit.set("upper", "0.0005")
        if name == "shoulder_pan":
            limit.set("lower", "-0.0005")

    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ava", package_name="ava_moveit_config"
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    robot_description_xml = _fake_hardware_robot_description_xml()
    robot_description = {"robot_description": robot_description_xml}

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

    moveit_pkg = FindPackageShare("ava_moveit_config")

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            PathJoinSubstitution([moveit_pkg, "config", "ros2_controllers.yaml"]),
        ],
    )

    # Spawner nodes to load and activate controllers in a deterministic order.
    # Generous service-call / controller-manager timeouts so a slow startup
    # (heavy move_group + ros2_control coming up together) doesn't make the
    # spawner give up after 10s, retry, and die with "already loaded" — which
    # would leave the controller loaded-but-not-activated (no action server).
    _spawner_timeouts = [
        "--controller-manager-timeout", "60",
        "--service-call-timeout", "60",
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager"] + _spawner_timeouts,
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager",
                   "/controller_manager"] + _spawner_timeouts,
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            move_group_configuration,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution([moveit_pkg, "config", "moveit.rviz"]),
        ],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            rsp_node,
            ros2_control_node,
            # Start JSB after ros2_control starts.
            RegisterEventHandler(
                OnProcessStart(
                    target_action=ros2_control_node,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
            # Start trajectory controller after JSB is up.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[joint_trajectory_controller_spawner],
                )
            ),
            # Start MoveIt and RViz once controller setup has completed.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_trajectory_controller_spawner,
                    on_exit=[move_group_node, rviz_node],
                )
            ),
        ]
    )
