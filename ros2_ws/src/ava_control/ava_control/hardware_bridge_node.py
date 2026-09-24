#!/usr/bin/env python3
"""
Hardware bridge node for AVA arm.

Connects ros2_control joint trajectory commands to the ESP32 servo hardware.
- Subscribes to /joint_trajectory_controller/joint_trajectory
- Publishes /joint_states (echo state)
- Sends MOVE commands to the ESP32 (HTTP or serial)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray

import time
import math
import yaml
from typing import Dict, List
from ament_index_python.packages import get_package_share_directory

from .servo_calibration import load_calibrations_from_yaml, ServoCalibration
from .resource_manager import ResourceManager


class HardwareBridgeNode(Node):
    """
    Bridges ROS 2 control commands to the AVA arm ESP32 hardware.
    """
    
    JOINT_NAMES = [
        'Revolute 1', 'Revolute 2', 'Revolute 3',
        'Revolute 4', 'Revolute 5', 'Revolute 6'
    ]
    NUM_JOINTS = 6
    
    def __init__(self):
        super().__init__('hardware_bridge')
        
        # Parameters
        self.declare_parameter('transport', 'http')
        self.declare_parameter('config_file', 'real_robot.yaml')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('command_timeout_sec', 1.0)
        self.declare_parameter('home_on_start', True)
        self.declare_parameter('home_wait_sec', 1.5)
        self.declare_parameter('start_at_ros_zero', False)
        
        # Get parameters
        self.transport = self.get_parameter('transport').value
        config_file = self.get_parameter('config_file').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.command_timeout = self.get_parameter('command_timeout_sec').value
        self.home_on_start = bool(self.get_parameter('home_on_start').value)
        self.home_wait_sec = float(self.get_parameter('home_wait_sec').value)
        self.start_at_ros_zero = bool(self.get_parameter('start_at_ros_zero').value)
        
        self.get_logger().info(f'Hardware Bridge starting: transport={self.transport}')
        
        # State arrays
        self.joint_positions = [0.0] * self.NUM_JOINTS
        self.joint_velocities = [0.0] * self.NUM_JOINTS
        self.joint_efforts = [0.0] * self.NUM_JOINTS
        self.last_command_time = time.time()
        self.command_received = False
        
        # Calibrations
        self.calibrations: Dict[str, ServoCalibration] = {}
        
        # Resource manager
        self.resource_manager: ResourceManager = None
        
        # Load configuration
        if not self._load_configuration(config_file):
            self.get_logger().error('Failed to load configuration')
            raise RuntimeError('Configuration loading failed')
        
        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', qos_profile=qos
        )
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self._on_trajectory,
            qos_profile=qos
        )
        
        # Timer for state publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_state)

        # Defined ramp-speed base state: slow (5) for arm/joint moves — the gripper-only
        # fast speed (20) is switched per-trajectory in _on_trajectory.
        self._current_speed = None
        if self.resource_manager.send_speed_command(5):
            self._current_speed = 5

        # Without encoder feedback, force a known startup pose to avoid drift between
        # real hardware and the internal software state.
        if self.start_at_ros_zero:
            if self.resource_manager.send_move_command(self.joint_positions):
                self.get_logger().info(
                    f'ROS-zero startup pose sent, waiting {self.home_wait_sec:.1f}s'
                )
                time.sleep(max(0.0, self.home_wait_sec))
            else:
                self.get_logger().warn('Failed to send ROS-zero startup pose')
        elif self.home_on_start:
            if self.resource_manager.send_home_command():
                self.get_logger().info(
                    f'HOME command sent on startup, waiting {self.home_wait_sec:.1f}s'
                )
                time.sleep(max(0.0, self.home_wait_sec))
            else:
                self.get_logger().warn('Failed to send HOME command on startup')
        
        self.get_logger().info(
            f'Hardware Bridge ready. Joints: {", ".join(self.JOINT_NAMES)}'
        )
    
    def _load_configuration(self, config_file: str) -> bool:
        """Load the configuration from YAML."""
        try:
            import os

            # If caller already passed absolute path, use it directly.
            if os.path.isabs(config_file) and os.path.exists(config_file):
                config_path = config_file
            else:
                pkg_share = get_package_share_directory('ava_control')
                config_path = os.path.join(pkg_share, 'config', config_file)

            if not os.path.exists(config_path):
                self.get_logger().warn(f'Config file not found: {config_path}')
                return False
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Load calibrations
            servo_cal = config.get('servo_calibration', {})
            self.calibrations = load_calibrations_from_yaml(servo_cal)
            
            # Initialize the resource manager
            self.resource_manager = ResourceManager(
                transport=self.transport,
                config=config,
                calibrations=self.calibrations,
                logger=self.get_logger()
            )
            
            if not self.resource_manager.initialize():
                self.get_logger().error('Resource Manager initialization failed')
                return False
            
            self.get_logger().info(f'Configuration loaded from {config_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Configuration loading error: {e}')
            return False
    
    def _on_trajectory(self, msg: JointTrajectory):
        """
        Callback for joint trajectory commands.
        
        Uses the last waypoint and sends it to the ESP32 immediately.
        """
        try:
            if not msg.points:
                return
            
            # Use the last waypoint (target position)
            target = msg.points[-1]
            
            if len(msg.joint_names) != len(target.positions):
                self.get_logger().warn(
                    'Trajectory invalid: joint_names/positions length mismatch '
                    f'({len(msg.joint_names)} vs {len(target.positions)})'
                )
                return

            # Map incoming joints by name and use supported hardware joints.
            # If a joint is omitted by MoveIt, keep the previous value for that joint.
            position_by_name = dict(zip(msg.joint_names, target.positions))
            new_positions = list(self.joint_positions)
            mapped_any = False
            missing = []

            for idx, joint_name in enumerate(self.JOINT_NAMES):
                if joint_name in position_by_name:
                    new_positions[idx] = position_by_name[joint_name]
                    mapped_any = True
                else:
                    missing.append(joint_name)

            if not mapped_any:
                self.get_logger().warn('Trajectory contains no hardware-supported joints')
                return

            if missing:
                self.get_logger().debug(
                    f'Trajectory missing joints {missing}; using previous values for them'
                )

            self.joint_positions = new_positions
            self.last_command_time = time.time()
            self.command_received = True

            # The firmware SPEED is global: joint trajectories keep the usual
            # slow ramp (5); ONLY gripper-only trajectories run fast (20).
            gripper_only = set(msg.joint_names) <= {'Revolute 6', 'Revolute 7'}
            desired_speed = 20 if gripper_only else 5
            if desired_speed != self._current_speed:
                if self.resource_manager.send_speed_command(desired_speed):
                    self._current_speed = desired_speed
                    self.get_logger().info(
                        f'Firmware SPEED -> {desired_speed} '
                        f'({"gripper fast" if gripper_only else "joints normal"})')

            # Send to the hardware immediately
            success = self.resource_manager.send_move_command(self.joint_positions)
            if success:
                self.get_logger().debug(
                    f'Trajectory received: {[f"{p:.2f}" for p in self.joint_positions]}'
                )
            else:
                self.get_logger().warn('Failed to send trajectory to hardware')
            
        except Exception as e:
            self.get_logger().error(f'Trajectory callback error: {e}')
    
    def _publish_state(self):
        """
        Publiziere Joint State (Echo State).
        
        Da wir kein echtes Feedback haben, publizieren wir die
        zuletzt gesendeten Kommandos.
        """
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.JOINT_NAMES
            msg.position = self.joint_positions
            msg.velocity = [0.0] * self.NUM_JOINTS  # Unknown
            msg.effort = [0.0] * self.NUM_JOINTS     # Unknown
            
            self.joint_state_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'State publishing error: {e}')
    
    def shutdown(self):
        """Shut down the node."""
        if self.resource_manager:
            self.resource_manager.shutdown()


def main():
    """Main entry point."""
    import sys
    import traceback

    rclpy.init()

    node = None
    exit_code = 0
    try:
        node = HardwareBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Make init/spin failures VISIBLE. Previously this printed to a buffered
        # stdout and the process exited 0 ("finished cleanly"), which hid the
        # real cause (e.g. serial port busy / not found) and made the robot
        # silently not move. Now: print to stderr (flushed) and exit non-zero,
        # so the launch logs "process has died, exit code 1".
        print(f'[hardware_bridge] FATAL: {e}', file=sys.stderr, flush=True)
        traceback.print_exc()
        sys.stderr.flush()
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
