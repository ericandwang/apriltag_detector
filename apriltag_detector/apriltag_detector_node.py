#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import numpy as np

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.bridge = CvBridge()
        self.detector = Detector(families='tag36h11')
        self.camera_matrix = None
        self.dist_coeffs = None
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/right/image_rect_color',
            self.image_callback,
            10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/right/camera_info',
            self.camera_info_callback,
            10)
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:  # Only process once
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('No camera intrinsics yet.')
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        fx = self.camera_matrix[0,0]
        fy = self.camera_matrix[1,1]
        cx = self.camera_matrix[0,2]
        cy = self.camera_matrix[1,2]
        detections = self.detector.detect(gray, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=0.05)

        for det in detections:
            print(f"Detected AprilTag ID: {det.tag_id}")
            if det.pose_t is None:  # Skip invalid poses
                continue
            print("Pose detected!")
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = det.pose_t[0][0]
            pose_msg.pose.position.y = det.pose_t[1][0]
            pose_msg.pose.position.z = det.pose_t[2][0]
            pose_msg.pose.orientation.x = det.pose_R[0][0]
            pose_msg.pose.orientation.y = det.pose_R[1][0]
            pose_msg.pose.orientation.z = det.pose_R[2][0]
            pose_msg.pose.orientation.w = 1.0  # Not a real quaternion; replace with proper conversion
            self.pose_pub.publish(pose_msg)

            # Convert rotation matrix to quaternion
            rotation_matrix = np.eye(4)
            rotation_matrix[:3, :3] = det.pose_R
            q = tf_transformations.quaternion_from_matrix(rotation_matrix)
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = "camera_right"
            t.child_frame_id = f"tag36h11:{det.tag_id}"
            t.transform.translation.x = det.pose_t[2][0]
            t.transform.translation.y = -det.pose_t[0][0]
            t.transform.translation.z = -det.pose_t[1][0]
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
