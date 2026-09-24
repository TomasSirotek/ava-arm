"""
Resource manager for the AVA arm hardware.

Manages the communication with the ESP32 via HTTP or serial.
"""

import json
import urllib.request
import urllib.error
import serial
import time
import logging
from typing import Dict, List

from .servo_calibration import ServoCalibration


class ResourceManager:
    """
    Manages the hardware communication.
    
    Supports both HTTP and serial transport.
    """
    
    def __init__(self, transport: str, config: Dict, calibrations: Dict[str, ServoCalibration], logger):
        """
        Initialize the resource manager.
        
        Args:
            transport: 'http' or 'serial'
            config: configuration dictionary
            calibrations: Servo calibration objects
            logger: ROS Logger
        """
        self.transport = transport
        self.config = config
        self.calibrations = calibrations
        self.logger = logger
        
        self.http_conn = None
        self.serial_conn = None
        
        self.logger.info(f'ResourceManager initialized: transport={transport}')
    
    def initialize(self) -> bool:
        """Initialize the transport connection."""
        try:
            if self.transport == 'http':
                return self._init_http()
            elif self.transport == 'serial':
                return self._init_serial()
            else:
                self.logger.error(f'Unknown transport: {self.transport}')
                return False
        except Exception as e:
            self.logger.error(f'Initialize error: {e}')
            return False
    
    def _init_http(self) -> bool:
        """Initialize the HTTP connection."""
        try:
            http_config = self.config.get('http', {})
            self.http_host = http_config.get('host', '192.168.4.1')
            self.http_port = http_config.get('port', 80)
            self.http_path = http_config.get('path', '/api/command')
            self.http_timeout = http_config.get('timeout_sec', 0.5)
            
            self.logger.info(
                f'HTTP configured: {self.http_host}:{self.http_port}{self.http_path} (timeout={self.http_timeout}s)'
            )
            return True
        except Exception as e:
            self.logger.error(f'HTTP init error: {e}')
            return False
    
    def _init_serial(self) -> bool:
        """Initialize the serial connection."""
        try:
            ser_config = self.config.get('serial', {})
            port = ser_config.get('port', '/dev/ttyUSB0')
            baud = ser_config.get('baud', 115200)
            timeout = ser_config.get('timeout_sec', 0.1)
            
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=timeout
            )
            
            self.logger.info(f'Serial connected: {port} @ {baud} baud')
            return True
            
        except Exception as e:
            self.logger.error(f'Serial init error: {e}')
            return False
    
    def send_move_command(self, positions_rad: List[float]) -> bool:
        """
        Send a MOVE command to the ESP32.
        
        Converts ROS positions to servo degrees and sends MOVE:d1,d2,d3,d4,d5,d6
        
        Args:
            positions_rad: 6-element list of positions in radians
            
        Returns:
            True on success
        """
        try:
            if len(positions_rad) != 6:
                self.logger.warn(f'Expected 6 positions, got {len(positions_rad)}')
                return False
            
            # Convert to servo degrees
            servo_degrees = []
            for i, pos_rad in enumerate(positions_rad):
                joint_name = f'Revolute {i+1}'
                cal = self.calibrations.get(joint_name)
                if not cal:
                    self.logger.error(f'No calibration for {joint_name}')
                    return False
                
                servo_deg = cal.rad_to_servo_deg(pos_rad)
                servo_degrees.append(servo_deg)
            
            # Format the command
            command = f"MOVE:{','.join(f'{d:.0f}' for d in servo_degrees)}"
            
            # Send via transport
            if self.transport == 'http':
                return self._send_http(command)
            elif self.transport == 'serial':
                return self._send_serial(command)
            else:
                return False
                
        except Exception as e:
            self.logger.error(f'Move command error: {e}')
            return False
    
    def _send_http(self, command: str) -> bool:
        """Send a command via HTTP POST."""
        try:
            url = f'http://{self.http_host}:{self.http_port}{self.http_path}'
            
            # Web firmware expects JSON key `cmd`.
            # Keep `command` too for compatibility with older bridge variants.
            payload = json.dumps({'cmd': command, 'command': command}).encode('utf-8')
            
            req = urllib.request.Request(
                url,
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            
            with urllib.request.urlopen(req, timeout=self.http_timeout) as response:
                result = response.read().decode('utf-8')
                self.logger.debug(f'HTTP Response: {result}')
                return True
                
        except urllib.error.HTTPError as e:
            self.logger.warn(f'HTTP error {e.code}: {e.reason}')
            return False
        except urllib.error.URLError as e:
            self.logger.warn(f'HTTP URL error: {e.reason}')
            return False
        except Exception as e:
            self.logger.warn(f'HTTP send error: {e}')
            return False
    
    def _send_serial(self, command: str) -> bool:
        """Send a command via serial."""
        try:
            if not self.serial_conn or not self.serial_conn.is_open:
                self.logger.error('Serial connection not open')
                return False
            
            # Send with trailing newline
            cmd_bytes = (command + '\n').encode('utf-8')
            self.serial_conn.write(cmd_bytes)
            self.serial_conn.flush()
            
            self.logger.debug(f'Serial sent: {command}')
            return True
            
        except Exception as e:
            self.logger.error(f'Serial send error: {e}')
            return False
    
    def send_home_command(self) -> bool:
        """Send the HOME command (all servos to 90 deg)."""
        try:
            if self.transport == 'http':
                return self._send_http('HOME')
            elif self.transport == 'serial':
                return self._send_serial('HOME')
            return False
        except Exception as e:
            self.logger.error(f'HOME command error: {e}')
            return False

    def send_speed_command(self, speed: int) -> bool:
        """Set the firmware ramp speed (SPEED:1..20, degrees per 80 ms tick).

        The firmware setting is GLOBAL for all servos, so the bridge sends the
        appropriate value before every motion: slow for joint trajectories (arm as
        before), fast for gripper-only trajectories (only the gripper servo is
        supposed to move fast)."""
        try:
            cmd = f'SPEED:{int(speed)}'
            if self.transport == 'http':
                return self._send_http(cmd)
            elif self.transport == 'serial':
                return self._send_serial(cmd)
            return False
        except Exception as e:
            self.logger.error(f'SPEED command error: {e}')
            return False
    
    def send_status_command(self) -> str:
        """Frage Status vom ESP32 ab."""
        try:
            if self.transport == 'serial' and self.serial_conn:
                self._send_serial('STATUS')
                # Read response
                response = self.serial_conn.readline().decode('utf-8').strip()
                return response
            return None
        except Exception as e:
            self.logger.error(f'STATUS command error: {e}')
            return None
    
    def shutdown(self):
        """Shut down the resource."""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                self.logger.info('Serial connection closed')
        except Exception as e:
            self.logger.error(f'Shutdown error: {e}')
