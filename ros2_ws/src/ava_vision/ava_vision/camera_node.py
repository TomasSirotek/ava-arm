#!/usr/bin/env python3
"""
Camera Node for ROS 2

This node captures images from a webcam and publishes them to a ROS 2 topic.
It's designed to work with USB cameras including in WSL environments.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode(Node):
    """ROS 2 Node for webcam image capture and publishing."""
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        # Stable device path (e.g. /dev/v4l/by-id/usb-...-video-index0). When
        # set, it WINS over camera_id — USB numeric indices swap between the
        # integrated and USB camera on replug/reboot, the by-id path does not.
        self.declare_parameter('camera_device', '')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('camera_frame', 'camera_link')
        # Path to calibration YAML (camera_calibration tool output). Empty = use dummy intrinsics.
        self.declare_parameter('camera_info_url', '')
        # Pixel format fourcc. MJPG lets many USB webcams reach 30 fps at 720p+
        # (uncompressed YUYV is bandwidth-limited to ~5 fps at 1280x720).
        # Set to '' to leave the camera's default format untouched.
        self.declare_parameter('fourcc', 'MJPG')

        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.camera_device = self.get_parameter('camera_device').value
        # Source passed to cv2.VideoCapture: stable path if given, else index.
        self.camera_source = self.camera_device if self.camera_device else self.camera_id
        self.frame_rate = self.get_parameter('frame_rate').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.camera_info_url = self.get_parameter('camera_info_url').value
        self.fourcc = self.get_parameter('fourcc').value

        # Load calibration if path was provided
        self.calibrated_info = self._load_calibration(self.camera_info_url)

        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # Initialize camera
        self.cap = None
        self.init_camera()
        
        # Create timer for capturing frames
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Frame counter for logging
        self.frame_count = 0
        
        self.get_logger().info(f'Camera node started with source: {self.camera_source}')
        self.get_logger().info(f'Resolution: {self.frame_width}x{self.frame_height} @ {self.frame_rate} FPS')
    
    def init_camera(self):
        """Initialize the camera capture."""
        # Try different backends for WSL compatibility
        backends = [
            (cv2.CAP_V4L2, 'V4L2'),
            (cv2.CAP_ANY, 'ANY'),
            (cv2.CAP_DSHOW, 'DirectShow'),
        ]
        
        self.get_logger().info(f'Opening camera source: {self.camera_source}')
        for backend, name in backends:
            self.get_logger().info(f'Trying camera backend: {name}')
            self.cap = cv2.VideoCapture(self.camera_source, backend)
            
            if self.cap.isOpened():
                self.get_logger().info(f'Camera opened successfully with {name} backend')
                break
            else:
                self.cap.release()
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera! Check camera connection and permissions.')
            self.get_logger().error('For WSL, ensure usbipd is configured correctly.')
            return
        
        # Set pixel format BEFORE resolution (MJPG enables high fps at 720p+).
        if self.fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*self.fourcc))

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Get actual properties (camera might not support requested values)
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Actual camera settings: {actual_width}x{actual_height} @ {actual_fps} FPS')
    
    def timer_callback(self):
        """Capture and publish camera frames."""
        if self.cap is None or not self.cap.isOpened():
            return
        
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        # Create timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Convert to ROS Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = self.camera_frame
            
            # Publish image
            self.image_pub.publish(image_msg)
            
            # Publish camera info (with default/placeholder values)
            camera_info_msg = self.get_camera_info(frame.shape[1], frame.shape[0], timestamp)
            self.camera_info_pub.publish(camera_info_msg)
            
            self.frame_count += 1
            
            # Log periodically
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Error converting/publishing image: {e}')
    
    def _load_calibration(self, path: str):
        """Load camera_info from a YAML produced by camera_calibration tool.
        Returns None if path empty or file unreadable."""
        if not path:
            self.get_logger().warn(
                'No camera_info_url set — publishing approximate intrinsics. '
                'Pose estimation will be inaccurate. Set camera_info_url=<path>.yaml.'
            )
            return None
        try:
            import os, yaml
            with open(os.path.expanduser(path)) as f:
                data = yaml.safe_load(f)
            self.get_logger().info(
                f"Loaded calibration from {path} "
                f"({data.get('image_width')}x{data.get('image_height')}, "
                f"camera_name='{data.get('camera_name')}')"
            )
            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load camera_info from {path}: {e}')
            return None

    def get_camera_info(self, width: int, height: int, timestamp) -> CameraInfo:
        """Build a CameraInfo message. Use calibrated values if loaded, else fallback."""
        msg = CameraInfo()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.camera_frame
        msg.width = width
        msg.height = height

        if self.calibrated_info:
            c = self.calibrated_info
            k = list(c['camera_matrix']['data'])
            msg.k = k
            msg.d = list(c['distortion_coefficients']['data'])
            # rectification_matrix / projection_matrix may be absent in YAMLs
            # produced by camera_calibration_node — fall back to sane defaults
            # (identity rectification, projection built from K) instead of
            # crashing with a KeyError.
            if 'rectification_matrix' in c:
                msg.r = list(c['rectification_matrix']['data'])
            else:
                msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            if 'projection_matrix' in c:
                msg.p = list(c['projection_matrix']['data'])
            else:
                # P = [K | 0]
                msg.p = [k[0], k[1], k[2], 0.0,
                         k[3], k[4], k[5], 0.0,
                         k[6], k[7], k[8], 0.0]
            msg.distortion_model = c.get('distortion_model', 'plumb_bob')
            return msg

        # Fallback: approximate intrinsics if no calibration loaded
        fx = width * 0.8
        fy = width * 0.8
        cx = width / 2.0
        cy = height / 2.0
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = 'plumb_bob'
        return msg
    
    def destroy_node(self):
        """Clean up resources."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
