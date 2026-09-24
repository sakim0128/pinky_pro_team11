"""Publish map-frame Pinky poses from an overhead ArUco camera without mock data."""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from .overhead_math import transform_point


class OverheadTracker(Node):
    def __init__(self):
        super().__init__('overhead_tracker')
        self.declare_parameter('image_topic', '/overhead/camera/image/compressed')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pinky1_marker_id', 1)
        self.declare_parameter('pinky2_marker_id', 2)
        # Row-major image-pixel -> map-metre homography. Empty means uncalibrated.
        self.declare_parameter('image_to_map_homography', [])
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        try:
            import cv2
            if not hasattr(cv2, 'aruco'):
                raise RuntimeError('opencv-contrib aruco module unavailable')
        except ImportError as exc:
            raise RuntimeError('python3-opencv is required for overhead tracking') from exc
        self.cv2 = cv2
        raw_h = self.get_parameter('image_to_map_homography').value
        self.homography = np.asarray(raw_h, dtype=float).reshape(3, 3) if len(raw_h) == 9 else None
        if self.homography is None or not np.isfinite(self.homography).all():
            self.get_logger().warning('Overhead tracker is uncalibrated; it will not publish poses')
        dictionary_name = self.get_parameter('aruco_dictionary').value
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise RuntimeError(f'Unknown ArUco dictionary: {dictionary_name}')
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
        self.marker_names = {int(self.get_parameter('pinky1_marker_id').value): 'pinky1',
                             int(self.get_parameter('pinky2_marker_id').value): 'pinky2'}
        self.publishers = {name: self.create_publisher(PoseStamped, f'/{name}/overhead_pose', 10)
                           for name in self.marker_names.values()}
        self.create_subscription(CompressedImage, self.get_parameter('image_topic').value, self.on_image, 10)

    def on_image(self, msg):
        if self.homography is None or 'jpeg' not in msg.format.lower():
            return
        frame = self.cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), self.cv2.IMREAD_GRAYSCALE)
        if frame is None:
            self.get_logger().warning('Invalid overhead JPEG ignored', throttle_duration_sec=5.0)
            return
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return
        for corner, marker_id in zip(corners, ids.flatten()):
            name = self.marker_names.get(int(marker_id))
            if name is None:
                continue
            points = corner.reshape(4, 2)
            center = transform_point(self.homography, points.mean(axis=0))
            # ArUco corners are ordered clockwise; top edge supplies tag heading.
            ahead = transform_point(self.homography, (points[0] + points[1]) / 2)
            if center is None or ahead is None:
                continue
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = self.get_parameter('map_frame').value
            pose.pose.position.x, pose.pose.position.y = center
            yaw = math.atan2(ahead[1] - center[1], ahead[0] - center[0])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.publishers[name].publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OverheadTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
