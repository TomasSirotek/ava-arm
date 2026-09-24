"""Gazebo simulation of the CV pick-and-place: SIM robot + REAL camera.

Robot runs in Gazebo (gz-sim, gz_ros2_control). The REAL overhead camera localizes
the REAL cube (ArUco) -> /cup_pose. A Gazebo cube is spawned at that pose, MoveIt
plans + executes the pick on the sim robot (RViz), the cube is held while the gripper
is closed (gazebo_grasp_attach) and moved to the place point.

  ros2 launch ava_moveit_config cube_gazebo.launch.py \
      camera_info_url:=<path-to-your-workspace>/calibration/camera_calibration_<stamp>.yaml \
      run_demo:=true

camera_info_url points to YOUR camera's intrinsic calibration (create it once with
`ros2 launch ava_vision camera_calibration.launch.py` — see the package README).
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    run_demo = LaunchConfiguration('run_demo')
    camera_info_url = LaunchConfiguration('camera_info_url')

    moveit_pkg = FindPackageShare('ava_moveit_config')
    vision_pkg = FindPackageShare('ava_vision')

    # 1) Gazebo + robot (gz_ros2_control) + controllers + move_group + RViz
    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg, 'launch', 'gazebo_moveit.launch.py'])),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    # 2) spawn the table (with corner-marker plates) into the running world
    spawn_table = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-file', PathJoinSubstitution([moveit_pkg, 'models', 'table', 'table.sdf']),
                 '-name', 'table', '-x', '0', '-y', '0', '-z', '0'],
            output='screen')],
    )

    # 3) REAL camera perception -> /cup_pose (+ base_link->camera_optical static TF)
    vision = TimerAction(
        period=9.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([vision_pkg, 'launch', 'vision_bringup.launch.py'])),
            launch_arguments={'camera_info_url': camera_info_url}.items(),
        )],
    )

    # 4) spawn the Gazebo cube at the detected /cup_pose
    spawn_cube = TimerAction(
        period=11.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ava_moveit_config', 'spawn_cube.py'],
            output='screen')],
    )

    # 5a) service bridge: ROS /world/empty/set_pose -> gz SetEntityPose. The grasp
    #     helper teleports the cube through THIS native service (ms latency); the old
    #     subprocess `gz service` path was 150-300 ms/call -> the cube visibly lagged
    #     behind the gripper ("falls off and re-grips").
    set_pose_bridge = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
            output='screen')],
    )

    # 5b) grasp helper: cube follows the gripper while closed
    grasp_attach = TimerAction(
        period=11.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ava_moveit_config', 'gazebo_grasp_attach.py',
                 '--ros-args', '-p', 'z_offset:=0.02'],   # cube center 2 cm below the tcp
            output='screen')],
    )

    # 6) visualize the planned end-effector path
    planned_path = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ava_moveit_config', 'planned_path_line.py'],
            output='screen')],
    )

    # 7) pick-and-place (optional): tcp -> marker, grasp, move cube
    pick = TimerAction(
        period=16.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ava_moveit_config', 'cup_pick_and_place.py',
                 '--ros-args', '--params-file',
                 PathJoinSubstitution([moveit_pkg, 'config', 'pick_place_params.yaml'])],
            output='screen',
            condition=IfCondition(run_demo))],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_demo', default_value='false',
                              description='Run the pick-and-place after localization'),
        DeclareLaunchArgument('camera_info_url', default_value='',
                              description='Path to calibrated camera intrinsics YAML'),
        gazebo_moveit,
        spawn_table,
        vision,
        spawn_cube,
        set_pose_bridge,
        grasp_attach,
        planned_path,
        pick,
    ])
