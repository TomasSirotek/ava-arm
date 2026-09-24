#!/usr/bin/env python3
"""
ArUco Marker Detector Node for ROS 2

This node subscribes to camera images, detects ArUco markers,
estimates their pose, and publishes the results.
"""

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
import math


class ArucoDetectorNode(Node):
    """ROS 2 Node for ArUco marker detection and pose estimation."""

    def __init__(self):
        super().__init__('aruco_detector_node')

        # Declare parameters
        self.declare_parameter('marker_size', 0.05)  # Fallback size in meters
        # Per-ID marker sizes as JSON string, e.g. '{"0": 0.132, "1": 0.1055, "2": 0.132}'.
        # Wins over the global marker_size when the detected ID is in the map.
        self.declare_parameter('marker_sizes', '')
        self.declare_parameter('dictionary', '4X4_50')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('show_visualization', True)
        # TF child-frame prefix. Frame name = <tf_prefix><marker_id>.
        # Set differently per camera (e.g. 'aruco_marker_' vs 'aruco_rs_marker_')
        # to avoid TF "two parents" conflicts when multiple detectors run.
        self.declare_parameter('tf_prefix', 'aruco_marker_')

        # Get parameters
        self.marker_size = self.get_parameter('marker_size').value
        self.dictionary_name = self.get_parameter('dictionary').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.show_visualization = self.get_parameter('show_visualization').value
        self.tf_prefix = self.get_parameter('tf_prefix').value

        sizes_str = self.get_parameter('marker_sizes').value
        if sizes_str:
            try:
                raw = json.loads(sizes_str)
                self.marker_sizes = {int(k): float(v) for k, v in raw.items()}
                self.get_logger().info(f'Per-ID marker sizes: {self.marker_sizes}')
            except Exception as e:
                self.get_logger().error(
                    f'marker_sizes parse failed: {e}. Using global marker_size only.'
                )
                self.marker_sizes = {}
        else:
            self.marker_sizes = {}
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector (compatible with different OpenCV versions)
        self.aruco_dict = self.get_aruco_dictionary(self.dictionary_name)
        self.aruco_params = self.get_detector_parameters()
        
        # Check OpenCV version for detector initialization
        cv_version = tuple(map(int, cv2.__version__.split('.')[:2]))
        if cv_version >= (4, 7):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        else:
            self.aruco_detector = None
            self.use_new_api = False
        
        # Camera calibration data (will be updated from camera_info topic)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco/pose', 10)
        self.poses_pub = self.create_publisher(PoseArray, 'aruco/poses', 10)
        self.debug_image_pub = self.create_publisher(Image, 'aruco/debug_image', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f'ArUco Detector started')
        self.get_logger().info(f'Dictionary: DICT_{self.dictionary_name}')
        self.get_logger().info(f'Marker size: {self.marker_size} m')
    
    def get_aruco_dictionary(self, dict_name: str):
        """Get the ArUco dictionary based on name.

        WICHTIG: API muss konsistent zur OpenCV-Version sein. Mischen von
        getPredefinedDictionary (>=4.7) mit detectMarkers als freie Funktion
        (<4.7) segfaults in OpenCV 4.6.
        """
        dict_map = {
            '4X4_50': cv2.aruco.DICT_4X4_50,
            '4X4_100': cv2.aruco.DICT_4X4_100,
            '5X5_50': cv2.aruco.DICT_5X5_50,
            '5X5_100': cv2.aruco.DICT_5X5_100,
            '6X6_250': cv2.aruco.DICT_6X6_250,
            '7X7_1000': cv2.aruco.DICT_7X7_1000,
        }
        dict_type = dict_map.get(dict_name, cv2.aruco.DICT_4X4_50)

        cv_version = tuple(map(int, cv2.__version__.split('.')[:2]))
        if cv_version >= (4, 7):
            return cv2.aruco.getPredefinedDictionary(dict_type)
        # OpenCV < 4.7 — strikt alte API verwenden
        return cv2.aruco.Dictionary_get(dict_type)

    def get_detector_parameters(self):
        """Get detector parameters consistent with the OpenCV version.

        Sub-pixel corner refinement noticeably improves pose accuracy for
        small markers seen from a distance (e.g. a 3.1 cm cube marker at
        ~0.5 m overhead).
        """
        cv_version = tuple(map(int, cv2.__version__.split('.')[:2]))
        if cv_version >= (4, 7):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()
        try:
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        except AttributeError:
            pass
        return params
    
    def camera_info_callback(self, msg: CameraInfo):
        """Process camera calibration information."""
        if not self.camera_info_received:
            # Extract camera matrix
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            
            # Extract distortion coefficients
            self.dist_coeffs = np.array(msg.d)
            
            self.camera_info_received = True
            self.get_logger().info('Camera calibration received')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
    
    def image_callback(self, msg: Image):
        """Process incoming images and detect ArUco markers."""
        if not self.camera_info_received:
            self.get_logger().warn('Waiting for camera calibration...', throttle_duration_sec=5.0)
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        # Convert to grayscale for detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers (compatible with different OpenCV versions)
        if self.use_new_api:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)
        
        # Create debug image
        debug_image = cv_image.copy()
        
        if ids is not None and len(ids) > 0:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
            
            # Estimate pose for each marker
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            for i, marker_id in enumerate(ids.flatten()):
                # Look up this marker's physical size (per-ID map, else global default)
                size = self.marker_sizes.get(int(marker_id), self.marker_size)

                # Estimate pose
                rvec, tvec = self.estimate_pose(corners[i], size)

                if rvec is not None and tvec is not None:
                    # Draw axis on debug image
                    cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs,
                                     rvec, tvec, size * 0.5)
                    
                    # Convert to pose
                    pose = self.rvec_tvec_to_pose(rvec, tvec)
                    pose_array.poses.append(pose.pose)
                    
                    # Publish individual pose for first detected marker
                    if i == 0:
                        pose.header = msg.header
                        self.pose_pub.publish(pose)
                    
                    # Publish TF
                    if self.publish_tf:
                        self.publish_marker_tf(rvec, tvec, marker_id, msg.header)
                    
                    # Add pose info to debug image
                    self.add_pose_text(debug_image, corners[i], marker_id, tvec, rvec)
            
            # Publish all poses
            self.poses_pub.publish(pose_array)
            
            self.get_logger().debug(f'Detected {len(ids)} markers: {ids.flatten().tolist()}')
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {e}')
        
        # Show visualization window (if enabled)
        if self.show_visualization:
            cv2.imshow('ArUco Detection', debug_image)
            cv2.waitKey(1)
    
    def estimate_pose(self, corners, marker_size):
        """
        Estimate the pose of a single ArUco marker.

        Args:
            corners: Corner points of the marker
            marker_size: physical side length of this marker in meters

        Returns:
            rvec: Rotation vector
            tvec: Translation vector
        """
        # Define marker points in marker coordinate system
        marker_points = np.array([
            [-marker_size/2,  marker_size/2, 0],
            [ marker_size/2,  marker_size/2, 0],
            [ marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)
        
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            marker_points,
            corners[0],
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if success:
            return rvec, tvec
        return None, None
    
    def rvec_tvec_to_pose(self, rvec, tvec) -> PoseStamped:
        """Convert rotation and translation vectors to PoseStamped."""
        pose = PoseStamped()
        
        # Translation
        pose.pose.position.x = float(tvec[0][0])
        pose.pose.position.y = float(tvec[1][0])
        pose.pose.position.z = float(tvec[2][0])
        
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose
    
    def rotation_matrix_to_quaternion(self, R):
        """Convert a rotation matrix to quaternion (x, y, z, w)."""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]
    
    def publish_marker_tf(self, rvec, tvec, marker_id, header):
        """Publish TF transform for detected marker."""
        t = TransformStamped()
        t.header = header
        t.header.frame_id = self.camera_frame
        t.child_frame_id = f'{self.tf_prefix}{marker_id}'
        
        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[1][0])
        t.transform.translation.z = float(tvec[2][0])
        
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def add_pose_text(self, image, corners, marker_id, tvec, rvec):
        """Add pose information text to the debug image."""
        # Get marker center
        center = corners[0].mean(axis=0).astype(int)

        # Distance from camera
        distance = np.linalg.norm(tvec)

        # Header line: ID + distance
        text = f"ID:{marker_id} D:{distance:.2f}m"
        cv2.putText(image, text, (center[0] - 50, center[1] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Position line (translation in camera frame, meters)
        pos_text = f"X:{tvec[0][0]:.2f} Y:{tvec[1][0]:.2f} Z:{tvec[2][0]:.2f}"
        cv2.putText(image, pos_text, (center[0] - 80, center[1] + 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # Orientation line (roll/pitch/yaw in degrees, ZYX intrinsic)
        rot, _ = cv2.Rodrigues(rvec)
        # Convert to euler (roll=X, pitch=Y, yaw=Z about marker frame)
        sy = math.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2)
        if sy > 1e-6:
            roll = math.atan2(rot[2, 1], rot[2, 2])
            pitch = math.atan2(-rot[2, 0], sy)
            yaw = math.atan2(rot[1, 0], rot[0, 0])
        else:
            roll = math.atan2(-rot[1, 2], rot[1, 1])
            pitch = math.atan2(-rot[2, 0], sy)
            yaw = 0.0
        rpy_text = (
            f"R:{math.degrees(roll):+.0f} "
            f"P:{math.degrees(pitch):+.0f} "
            f"Y:{math.degrees(yaw):+.0f} deg"
        )
        cv2.putText(image, rpy_text, (center[0] - 80, center[1] + 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
    
    def destroy_node(self):
        """Clean up resources."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
