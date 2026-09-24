"""
Servo calibration and PWM conversion utilities for AVA arm.

Converts between:
- ROS joint angles (radians) 
- Servo physical angles (degrees)
- PCA9685 PWM duty cycles

Based on servo datasheets:
- SG90: 1.0-2.0ms pulse for 0-180°
- MG996R: 1.0-2.0ms pulse for 0-180°
- PCA9685: 50Hz frequency, 12-bit resolution (4096 counts per 20ms period)
"""

import math


class ServoCalibration:
    """Single servo calibration parameters."""
    
    # PWM timing constants (in milliseconds)
    PWM_MIN_MS = 1.0      # Minimum pulse width (0°)
    PWM_MAX_MS = 2.0      # Maximum pulse width (180°)
    PWM_MID_MS = 1.5      # Neutral pulse width (90°)
    PWM_PERIOD_MS = 20.0  # 50Hz period
    
    # PCA9685 12-bit resolution constants
    PCA9685_COUNTS_PER_PERIOD = 4096  # 12-bit resolution
    PCA9685_FREQUENCY = 50  # Hz
    
    # Calculated PWM counts for PCA9685
    PWM_MIN_COUNTS = int(PWM_MIN_MS / PWM_PERIOD_MS * PCA9685_COUNTS_PER_PERIOD)  # ~205
    PWM_MID_COUNTS = int(PWM_MID_MS / PWM_PERIOD_MS * PCA9685_COUNTS_PER_PERIOD)  # ~307
    PWM_MAX_COUNTS = int(PWM_MAX_MS / PWM_PERIOD_MS * PCA9685_COUNTS_PER_PERIOD)  # ~410
    
    def __init__(self, name: str, **kwargs):
        """
        Initialize servo calibration.
        
        Args:
            name: Joint/servo name (e.g., "J1")
            zero_rad: ROS angle (rad) at neutral position (default: 0.0)
            direction: Rotation direction multiplier (1 or -1)
            scale: Scaling factor for angle conversion (default: 1.0)
            servo_min_deg: Minimum mechanical servo angle (default: 0)
            servo_max_deg: Maximum mechanical servo angle (default: 180)
        """
        self.name = name
        self.zero_rad = kwargs.get('zero_rad', 0.0)
        self.direction = kwargs.get('direction', 1)
        self.scale = kwargs.get('scale', 1.0)
        self.servo_min_deg = kwargs.get('servo_min_deg', 0)
        self.servo_max_deg = kwargs.get('servo_max_deg', 180)
    
    def rad_to_servo_deg(self, joint_rad: float) -> float:
        """
        Convert ROS joint angle (radians) to servo physical angle (degrees).
        
        Formula: servo_deg = 90 + direction * (joint_rad - zero_rad) * (180/π) * scale
        
        Args:
            joint_rad: ROS joint angle in radians
            
        Returns:
            Servo angle in degrees (0-180)
        """
        # Relative angle from zero position
        delta_rad = joint_rad - self.zero_rad
        
        # Convert to degrees and apply scaling
        delta_deg = delta_rad * (180.0 / math.pi) * self.scale
        
        # Apply direction and offset to neutral
        servo_deg = 90.0 + self.direction * delta_deg
        
        # Clamp to mechanical limits
        servo_deg = max(self.servo_min_deg, min(self.servo_max_deg, servo_deg))
        
        return servo_deg
    
    def servo_deg_to_pwm_counts(self, servo_deg: float) -> int:
        """
        Convert servo physical angle to PCA9685 PWM counts.
        
        Assumes linear mapping:
        - 0° = 1.0ms (PWM_MIN_COUNTS)
        - 90° = 1.5ms (PWM_MID_COUNTS)
        - 180° = 2.0ms (PWM_MAX_COUNTS)
        
        Args:
            servo_deg: Servo angle in degrees (0-180)
            
        Returns:
            PCA9685 PWM counts (typically 205-410)
        """
        # Clamp to valid servo range
        servo_deg = max(0, min(180, servo_deg))
        
        # Linear interpolation between PWM values
        # At 0°: PWM_MIN, at 180°: PWM_MAX
        pwm_counts = self.PWM_MIN_COUNTS + (servo_deg / 180.0) * (self.PWM_MAX_COUNTS - self.PWM_MIN_COUNTS)
        
        return int(round(pwm_counts))
    
    def rad_to_pwm_counts(self, joint_rad: float) -> int:
        """
        Convert ROS joint angle directly to PCA9685 PWM counts.
        
        Combines rad_to_servo_deg() and servo_deg_to_pwm_counts().
        
        Args:
            joint_rad: ROS joint angle in radians
            
        Returns:
            PCA9685 PWM counts
        """
        servo_deg = self.rad_to_servo_deg(joint_rad)
        return self.servo_deg_to_pwm_counts(servo_deg)
    
    def servo_deg_to_rad(self, servo_deg: float) -> float:
        """
        Reverse conversion: servo degrees to ROS joint radians.
        
        Args:
            servo_deg: Servo angle in degrees (0-180)
            
        Returns:
            ROS joint angle in radians
        """
        # Reverse the neutral offset
        delta_deg = (servo_deg - 90.0) / self.direction
        
        # Convert to radians and apply scaling
        delta_rad = delta_deg * (math.pi / 180.0) / self.scale
        
        # Add zero position offset
        return self.zero_rad + delta_rad


def create_default_calibrations() -> dict:
    """
    Create calibration objects for all AVA arm joints.
    
    Returns:
        Dictionary mapping joint names to ServoCalibration objects
    """
    calibrations = {
        'Revolute 1': ServoCalibration('J1', zero_rad=1.57, direction=1, scale=1.0,
                                       servo_min_deg=0, servo_max_deg=180),
        'Revolute 2': ServoCalibration('J2', zero_rad=-1.57, direction=1, scale=1.0,
                                       servo_min_deg=0, servo_max_deg=180),
        'Revolute 3': ServoCalibration('J3', zero_rad=-1.57, direction=1, scale=1.0,
                                       servo_min_deg=0, servo_max_deg=180),
        'Revolute 4': ServoCalibration('J4', zero_rad=-1.57, direction=1, scale=1.0,
                                       servo_min_deg=0, servo_max_deg=180),
        'Revolute 5': ServoCalibration('J5', zero_rad=-1.57, direction=1, scale=1.0,
                                       servo_min_deg=0, servo_max_deg=180),
        'Revolute 6': ServoCalibration('J6', zero_rad=0.0, direction=1, scale=1.0,
                                       servo_min_deg=0, servo_max_deg=180),
    }
    return calibrations


def load_calibrations_from_yaml(calibration_dict: dict) -> dict:
    """
    Load calibration parameters from YAML dictionary.
    
    Expected YAML structure:
    ```
    servo_calibration:
      zero_rad: [1.57, -1.57, -3.14, -3.14, -1.57, 0.0]
      direction: [1, 1, 1, 1, 1, 1]
      scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
      servo_min_deg: [0, 0, 0, 0, 0, 0]
      servo_max_deg: [180, 180, 180, 180, 180, 180]
    ```
    
    Args:
        calibration_dict: Dictionary with calibration parameters
        
    Returns:
        Dictionary mapping joint names to ServoCalibration objects
    """
    joint_names = ['Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4', 'Revolute 5', 'Revolute 6']
    
    zero_rad = calibration_dict.get('zero_rad', [1.57, -1.57, -1.57, -1.57, -1.57, 0.0])
    direction = calibration_dict.get('direction', [1, 1, 1, 1, 1, 1])
    scale = calibration_dict.get('scale', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    servo_min_deg = calibration_dict.get('servo_min_deg', [0, 0, 0, 0, 0, 0])
    servo_max_deg = calibration_dict.get('servo_max_deg', [180, 180, 180, 180, 180, 180])
    
    calibrations = {}
    for i, name in enumerate(joint_names):
        calibrations[name] = ServoCalibration(
            name,
            zero_rad=zero_rad[i],
            direction=direction[i],
            scale=scale[i],
            servo_min_deg=servo_min_deg[i],
            servo_max_deg=servo_max_deg[i]
        )
    
    return calibrations
