#!/usr/bin/env python3
"""Bridge JointTrajectory commands to AVA arm ESP32 firmware commands.

Supported firmware commands (already in your ESP32 sketches):
- MOVE:d1,d2,d3,d4,d5,d6
- HOME
- STATUS
"""

from __future__ import annotations

import json
import math
import socket
import time
import urllib.error
import urllib.request
from typing import Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    serial = None


class Esp32ServoBridgeNode(Node):
    """Convert ROS joint trajectories to ESP32 servo commands."""

    def __init__(self) -> None:
        super().__init__('esp32_servo_bridge_node')

        self.declare_parameter('command_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('transport', 'http')  # http | serial
        self.declare_parameter('min_send_period_sec', 0.05)
        self.declare_parameter('http_host', '192.168.4.1')
        self.declare_parameter('http_port', 80)
        self.declare_parameter('http_api_path', '/api/command')
        self.declare_parameter('http_timeout_sec', 0.25)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('serial_timeout_sec', 0.10)

        self.declare_parameter(
            'joint_names',
            ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_roll', 'wrist_flex', 'gripper'],
        )

        # Servo mechanics and safety windows.
        self.declare_parameter('servo_min_deg', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('servo_max_deg', [180, 180, 180, 180, 180, 180])

        # Calibration model:
        # servo_deg = 90 + direction * (joint_rad - zero_rad) * (180/pi) * scale
        self.declare_parameter('joint_zero_rad', [1.57, -1.57, -3.14, -3.14, -1.57, 0.0])
        self.declare_parameter('joint_direction', [1, 1, 1, 1, 1, 1])
        self.declare_parameter('joint_scale', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.command_topic = str(self.get_parameter('command_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.transport = str(self.get_parameter('transport').value).strip().lower()
        self.min_send_period_sec = float(self.get_parameter('min_send_period_sec').value)

        self.http_host = str(self.get_parameter('http_host').value)
        self.http_port = int(self.get_parameter('http_port').value)
        self.http_api_path = str(self.get_parameter('http_api_path').value)
        self.http_timeout_sec = float(self.get_parameter('http_timeout_sec').value)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.serial_baud = int(self.get_parameter('serial_baud').value)
        self.serial_timeout_sec = float(self.get_parameter('serial_timeout_sec').value)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.servo_min_deg = [int(v) for v in self.get_parameter('servo_min_deg').value]
        self.servo_max_deg = [int(v) for v in self.get_parameter('servo_max_deg').value]
        self.joint_zero_rad = [float(v) for v in self.get_parameter('joint_zero_rad').value]
        self.joint_direction = [int(v) for v in self.get_parameter('joint_direction').value]
        self.joint_scale = [float(v) for v in self.get_parameter('joint_scale').value]

        self._validate_config_lengths()

        self._serial = None
        self._joint_last_rad: Dict[str, float] = {name: zr for name, zr in zip(self.joint_names, self.joint_zero_rad)}
        self._last_send_ts = 0.0

        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.create_subscription(JointTrajectory, self.command_topic, self.on_trajectory, 20)

        if self.transport == 'serial':
            self._open_serial_if_needed()

        self.get_logger().info(
            f'ESP32 servo bridge ready. transport={self.transport}, command_topic={self.command_topic}'
        )
        self.get_logger().info(
            'Servo setup: J1-J3 MG996R, J4-J6 SG90 (limits and mapping configurable via YAML)'
        )

    def _validate_config_lengths(self) -> None:
        expected = len(self.joint_names)
        fields = [
            ('servo_min_deg', self.servo_min_deg),
            ('servo_max_deg', self.servo_max_deg),
            ('joint_zero_rad', self.joint_zero_rad),
            ('joint_direction', self.joint_direction),
            ('joint_scale', self.joint_scale),
        ]
        for name, values in fields:
            if len(values) != expected:
                raise ValueError(f'{name} length {len(values)} != joint_names length {expected}')

    def _open_serial_if_needed(self) -> None:
        if serial is None:
            self.get_logger().error('transport=serial requires python package pyserial')
            return
        if self._serial and getattr(self._serial, 'is_open', False):
            return
        try:
            self._serial = serial.Serial(  # type: ignore[attr-defined]
                self.serial_port,
                self.serial_baud,
                timeout=self.serial_timeout_sec,
            )
            time.sleep(0.2)
            self.get_logger().info(f'Serial connected: {self.serial_port} @ {self.serial_baud}')
        except Exception as exc:
            self.get_logger().error(f'Serial open failed: {exc}')
            self._serial = None

    def on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        now = time.monotonic()
        if (now - self._last_send_ts) < self.min_send_period_sec:
            return
        self._last_send_ts = now

        point = msg.points[-1]
        if not point.positions:
            return

        names = list(msg.joint_names) if msg.joint_names else self.joint_names
        for idx, name in enumerate(names):
            if idx >= len(point.positions):
                break
            if name in self._joint_last_rad:
                self._joint_last_rad[name] = float(point.positions[idx])

        servo_deg = self._compute_servo_degrees()
        cmd = f"MOVE:{','.join(str(v) for v in servo_deg)}"

        ok = self._send_command(cmd)
        if ok:
            self._publish_joint_state(msg.header.stamp)

    def _compute_servo_degrees(self) -> List[int]:
        result: List[int] = []
        for i, name in enumerate(self.joint_names):
            joint_rad = self._joint_last_rad.get(name, self.joint_zero_rad[i])
            deg = 90.0 + self.joint_direction[i] * (joint_rad - self.joint_zero_rad[i]) * (180.0 / math.pi) * self.joint_scale[i]
            deg = max(float(self.servo_min_deg[i]), min(float(self.servo_max_deg[i]), deg))
            result.append(int(round(deg)))
        return result

    def _send_command(self, cmd: str) -> bool:
        if self.transport == 'http':
            return self._send_http_command(cmd)
        if self.transport == 'serial':
            return self._send_serial_command(cmd)

        self.get_logger().error(f'Unsupported transport={self.transport}. Use http or serial.')
        return False

    def _send_http_command(self, cmd: str) -> bool:
        url = f'http://{self.http_host}:{self.http_port}{self.http_api_path}'
        payload = json.dumps({'cmd': cmd}).encode('utf-8')
        req = urllib.request.Request(
            url,
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(req, timeout=self.http_timeout_sec) as resp:
                if resp.status != 200:
                    self.get_logger().warn(f'HTTP command failed status={resp.status}')
                    return False
            return True
        except (urllib.error.URLError, socket.timeout, TimeoutError) as exc:
            self.get_logger().warn(f'HTTP send failed: {exc}')
            return False

    def _send_serial_command(self, cmd: str) -> bool:
        if self._serial is None or not getattr(self._serial, 'is_open', False):
            self._open_serial_if_needed()
            if self._serial is None:
                return False

        try:
            self._serial.write((cmd + '\n').encode('utf-8'))
            return True
        except Exception as exc:
            self.get_logger().warn(f'Serial send failed: {exc}')
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            return False

    def _publish_joint_state(self, stamp: Time) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = [self._joint_last_rad[n] for n in self.joint_names]
        self.joint_state_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Esp32ServoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
