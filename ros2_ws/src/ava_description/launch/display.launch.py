from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('ava_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'ava.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file, mappings={'ros2_control_hardware_type': 'mock_components'})
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    # Startpose override disabled: use natural/default joint startup state.
    # home_zeros = {
    #     'shoulder_pan': 1.57,
    #     'shoulder_lift': -1.57,
    #     'elbow_flex': -3.14,
    #     'wrist_roll': -3.14,
    #     'wrist_flex': -1.57,
    #     'gripper': 0.0,
    # }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # parameters=[{'zeros': home_zeros}]
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # parameters=[{'zeros': home_zeros}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
