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
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_transformations
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.bridge = CvBridge()
        self.detector = Detector(families='tag36h11')
        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.last_tag_transforms = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.broadcast_timer = self.create_timer(
            0.1,
            self.broadcast_all_tag_transforms)

    def transform_stamped_to_matrix(self, transform):
        """Convert TransformStamped to 4x4 homogeneous matrix"""
        t = transform.transform.translation
        q = transform.transform.rotation
        translation = np.array([t.x, t.y, t.z])
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        return matrix

    def matrix_to_transform_stamped(self, matrix, header, child_frame_id):
        """Convert 4x4 matrix to TransformStamped"""
        t = TransformStamped()
        t.header = header
        t.child_frame_id = child_frame_id
        t.transform.translation.x = matrix[0, 3]
        t.transform.translation.y = matrix[1, 3]
        t.transform.translation.z = matrix[2, 3]
        quat = R.from_matrix(matrix[:3, :3]).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    def convert_transform_to_map(self, camera_to_tag, map_to_camera):
        """Chain transforms: map -> camera_right -> tag"""
        # Convert to matrices
        map_to_cam_matrix = self.transform_stamped_to_matrix(map_to_camera)
        cam_to_tag_matrix = self.transform_stamped_to_matrix(camera_to_tag)
        
        # Multiply transforms: map_to_tag = map_to_cam * cam_to_tag
        map_to_tag_matrix = np.dot(map_to_cam_matrix, cam_to_tag_matrix)
        
        # Convert back to TransformStamped
        return self.matrix_to_transform_stamped(
            map_to_tag_matrix,
            map_to_camera.header,  # Use map frame header
            camera_to_tag.child_frame_id
        )


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
            #print(f"Detected AprilTag ID: {det.tag_id}")
            if det.pose_t is None:  # Skip invalid poses
                continue
            #print("Pose detected!")
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
            M = np.array([
                [0,  0,  1],
                [-1, 0,  0],
                [0, -1,  0]
            ])
            rotation_matrix = np.eye(4)
            rotation_matrix3x3 = M @ det.pose_R @ np.linalg.inv(M)
            rotation_matrix[:3, :3] = rotation_matrix3x3
            q = tf_transformations.quaternion_from_matrix(rotation_matrix)
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = "camera_right"
            t.child_frame_id = f"tag36h11:{det.tag_id}"
            t.transform.translation.x = det.pose_t[2][0]
            t.transform.translation.y = -det.pose_t[0][0]
            t.transform.translation.z = -det.pose_t[1][0]
            qx, qy, qz, qw = q
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

            print(f"Detected tag ID: {det.tag_id}")
            print(f"  Position: x={t.transform.translation.x:.3f}, y={t.transform.translation.y:.3f}, z={t.transform.translation.z:.3f}")
            print(f"  Orientation (quaternion): x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}")

            try:
                map_to_camera = self.tf_buffer.lookup_transform(
                    "map",
                    "camera_right",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                t_map = self.convert_transform_to_map(t, map_to_camera)
                t_map.child_frame_id = f"tag36h11:{det.tag_id}_map"
                self.last_tag_transforms[det.tag_id] = t_map
            except Exception as e:
                self.get_logger().warn(f"Transform to map failed: {e}")


    def broadcast_all_tag_transforms(self):
        now = self.get_clock().now().to_msg()
        for tag_id, t in self.last_tag_transforms.items():
            t.header.stamp = now
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
