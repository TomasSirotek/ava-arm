import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('ava_description')


    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'ava.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description_xml = robot_description_config.toxml()
    robot_description = {'robot_description': robot_description_xml}

   
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'publish_robot_description': True,
                'use_sim_time': True,
            },
        ],
    )

   
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
    )

    spawn_robot = TimerAction(
        period=5.0,  
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-string", robot_description_xml,
                "-name", "ava",
                "-allow_renaming", "false",  # prevents "_1" duplicate
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.0",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    ensure_joint_trajectory_controller = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-lc', 'ros2 control load_controller --set-state active joint_trajectory_controller || true'],
            output='screen'
        )]
    )

    # Startpose override disabled: use natural/default joint startup state.
    # set_start_pose = TimerAction(
    #     period=12.0,
    #     actions=[ExecuteProcess(
    #         cmd=[
    #             'bash',
    #             '-lc',
    #             "ros2 topic pub --once /joint_trajectory_controller/joint_trajectory "
    #             "trajectory_msgs/msg/JointTrajectory "
    #             "\"{joint_names: ['Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4', 'Revolute 5', 'Revolute 6'], "
    #             "points: [{positions: [1.57, -1.57, -3.14, -3.14, -1.57, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}\""
    #         ],
    #         output='screen'
    #     )]
    # )

    ensure_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-lc', 'ros2 control load_controller --set-state active joint_state_broadcaster || true'],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    gripper_mimic_bridge = Node(
        package='ava_control',
        executable='gripper_mimic_bridge',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ensure_joint_state_broadcaster,
        ensure_joint_trajectory_controller,
        # set_start_pose,
        ros_gz_bridge,
        gripper_mimic_bridge,
    ])
