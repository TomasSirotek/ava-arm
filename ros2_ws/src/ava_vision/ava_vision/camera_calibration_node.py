#!/usr/bin/env python3
"""
Camera Calibration Node for ROS 2

This node helps calibrate the camera using a checkerboard pattern.
Proper calibration is essential for accurate pose estimation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
from datetime import datetime


class CameraCalibrationNode(Node):
    """ROS 2 Node for camera calibration using checkerboard pattern."""
    
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # Declare parameters
        self.declare_parameter('checkerboard_width', 9)   # Number of inner corners
        self.declare_parameter('checkerboard_height', 6)
        self.declare_parameter('square_size', 0.025)      # Square size in meters (25mm)
        self.declare_parameter('min_captures', 20)        # Minimum images for calibration
        self.declare_parameter('output_dir', 'calibration')
        
        # Get parameters
        self.board_width = self.get_parameter('checkerboard_width').value
        self.board_height = self.get_parameter('checkerboard_height').value
        self.square_size = self.get_parameter('square_size').value
        self.min_captures = self.get_parameter('min_captures').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Calibration data storage
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        self.image_size = None
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... scaled by square_size
        self.objp = np.zeros((self.board_width * self.board_height, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_width, 0:self.board_height].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # State
        self.current_frame = None
        self.capture_next = False
        self.calibrated = False
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('Camera Calibration Node started')
        self.get_logger().info(f'Checkerboard: {self.board_width}x{self.board_height}')
        self.get_logger().info(f'Square size: {self.square_size * 1000:.1f} mm')
        self.get_logger().info('Press SPACE to capture, C to calibrate, S to save, Q to quit')
    
    def image_callback(self, msg: Image):
        """Process incoming images."""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_size = (self.current_frame.shape[1], self.current_frame.shape[0])
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def find_checkerboard(self, image):
        """Find checkerboard corners in the image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(
            gray, (self.board_width, self.board_height), None)
        
        if ret:
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        return ret, corners
    
    def capture_calibration_image(self):
        """Capture current frame for calibration."""
        if self.current_frame is None:
            self.get_logger().warn('No frame available')
            return False
        
        ret, corners = self.find_checkerboard(self.current_frame)
        
        if ret:
            self.obj_points.append(self.objp)
            self.img_points.append(corners)
            
            self.get_logger().info(f'Captured image {len(self.img_points)}/{self.min_captures}')
            return True
        else:
            self.get_logger().warn('Checkerboard not found in current frame')
            return False
    
    def calibrate_camera(self):
        """Perform camera calibration."""
        if len(self.img_points) < self.min_captures:
            self.get_logger().warn(
                f'Need at least {self.min_captures} images, have {len(self.img_points)}')
            return None
        
        self.get_logger().info('Calibrating camera...')
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.image_size, None, None)
        
        if ret:
            self.get_logger().info('Calibration successful!')
            self.get_logger().info(f'RMS reprojection error: {ret:.4f}')
            self.get_logger().info(f'Camera matrix:\n{camera_matrix}')
            self.get_logger().info(f'Distortion coefficients: {dist_coeffs.flatten()}')
            
            self.calibrated = True
            return {
                'camera_matrix': camera_matrix,
                'dist_coeffs': dist_coeffs,
                'rms_error': ret,
                'image_size': self.image_size,
                'num_images': len(self.img_points),
            }
        else:
            self.get_logger().error('Calibration failed!')
            return None
    
    def save_calibration(self, calibration_data):
        """Save calibration data to files."""
        if calibration_data is None:
            self.get_logger().error('No calibration data to save')
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Save as YAML (for ROS)
        yaml_file = os.path.join(self.output_dir, f'camera_calibration_{timestamp}.yaml')
        self.save_yaml(yaml_file, calibration_data)
        
        # Save as JSON (for general use)
        json_file = os.path.join(self.output_dir, f'camera_calibration_{timestamp}.json')
        self.save_json(json_file, calibration_data)
        
        # Save as NumPy (for Python scripts)
        npz_file = os.path.join(self.output_dir, f'camera_calibration_{timestamp}.npz')
        np.savez(npz_file,
                 camera_matrix=calibration_data['camera_matrix'],
                 dist_coeffs=calibration_data['dist_coeffs'])
        
        self.get_logger().info(f'Calibration saved to {self.output_dir}/')
    
    def save_yaml(self, filepath, data):
        """Save calibration in YAML format (ROS camera_info layout).

        Includes rectification_matrix and projection_matrix so the file loads
        directly in camera_node.get_camera_info (which expects all four fields).
        """
        k = data['camera_matrix'].flatten().tolist()
        # Identity rectification (monocular). Projection P = [K | 0].
        rect = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        proj = [k[0], k[1], k[2], 0.0,
                k[3], k[4], k[5], 0.0,
                k[6], k[7], k[8], 0.0]
        with open(filepath, 'w') as f:
            f.write(f"# Camera calibration file\n")
            f.write(f"# Generated: {datetime.now().isoformat()}\n")
            f.write(f"# RMS reprojection error: {data['rms_error']:.4f}\n")
            f.write(f"# Captures used: {data.get('num_images', 'n/a')}\n")
            f.write(f"image_width: {data['image_size'][0]}\n")
            f.write(f"image_height: {data['image_size'][1]}\n")
            f.write(f"camera_name: webcam\n")
            f.write(f"camera_matrix:\n")
            f.write(f"  rows: 3\n")
            f.write(f"  cols: 3\n")
            f.write(f"  data: {k}\n")
            f.write(f"distortion_model: plumb_bob\n")
            f.write(f"distortion_coefficients:\n")
            f.write(f"  rows: 1\n")
            f.write(f"  cols: {len(data['dist_coeffs'].flatten())}\n")
            f.write(f"  data: {data['dist_coeffs'].flatten().tolist()}\n")
            f.write(f"rectification_matrix:\n")
            f.write(f"  rows: 3\n")
            f.write(f"  cols: 3\n")
            f.write(f"  data: {rect}\n")
            f.write(f"projection_matrix:\n")
            f.write(f"  rows: 3\n")
            f.write(f"  cols: 4\n")
            f.write(f"  data: {proj}\n")
    
    def save_json(self, filepath, data):
        """Save calibration in JSON format."""
        json_data = {
            'image_size': data['image_size'],
            'camera_matrix': data['camera_matrix'].tolist(),
            'dist_coeffs': data['dist_coeffs'].tolist(),
            'rms_error': data['rms_error'],
            'num_images': data.get('num_images'),
        }
        with open(filepath, 'w') as f:
            json.dump(json_data, f, indent=2)
    
    def run_interactive(self):
        """Run interactive calibration loop."""
        calibration_data = None
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.current_frame is None:
                continue
            
            # Find and draw checkerboard
            display = self.current_frame.copy()
            ret, corners = self.find_checkerboard(display)
            
            if ret:
                cv2.drawChessboardCorners(display, (self.board_width, self.board_height),
                                         corners, ret)
                status = "Checkerboard FOUND - Press SPACE to capture"
                color = (0, 255, 0)
            else:
                status = "Checkerboard NOT found"
                color = (0, 0, 255)
            
            # Add status text
            cv2.putText(display, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display, f"Captured: {len(self.img_points)}/{self.min_captures}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display, "SPACE:Capture  C:Calibrate  S:Save  Q:Quit",
                       (10, display.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            cv2.imshow('Camera Calibration', display)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):  # Space - capture
                self.capture_calibration_image()
            elif key == ord('c'):  # C - calibrate
                calibration_data = self.calibrate_camera()
            elif key == ord('s'):  # S - save
                if calibration_data:
                    self.save_calibration(calibration_data)
                else:
                    self.get_logger().warn('Calibrate first before saving!')
            elif key == ord('q'):  # Q - quit
                break
        
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraCalibrationNode()
    
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
