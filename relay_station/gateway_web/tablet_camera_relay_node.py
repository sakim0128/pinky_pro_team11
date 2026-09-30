#!/usr/bin/env python3
"""
ROS 2 Tablet Camera Relay Node
- 태블릿 카메라 Ingest 버퍼에서 최신 프레임을 가져와 ROS 2 /camera/image_raw 토픽으로 30fps 발행
- 관제 평면(DOMAIN_ID=8) 환경 및 타임스탬프 동기화
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from stream_ingest import TabletStreamIngest
except ImportError:
    from .stream_ingest import TabletStreamIngest


class TabletCameraRelayNode(Node):
    # ⭐ 주소 기본값을 비운다. 이 경로는 `ingest_manager` 가 없을 때만 쓰는 독립
    #    기동인데, 거기에 죽은 주소를 박아 두면 그때도 재접속 루프가 돈다.
    #    (게이트웨이는 늘 ingest_manager 를 넘기므로 평소엔 안 탄다.)
    def __init__(self, ingest_manager=None, primary_url="", fallback_url=""):
        super().__init__('tablet_camera_relay_node')
        
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_fps', 30.0)
        
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        fps = self.get_parameter('publish_fps').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.camera_topic, 10)

        # Ingest Manager
        if ingest_manager is not None:
            self.ingest = ingest_manager
        else:
            self.ingest = TabletStreamIngest(primary_url, fallback_url)
            self.ingest.start()

        timer_period = 1.0 / max(fps, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"TabletCameraRelayNode initialized on topic: {self.camera_topic} ({fps:.1f} FPS)")

    def timer_callback(self):
        frame, stamp, is_connected, _ = self.ingest.get_latest_frame()
        if frame is None:
            return

        try:
            # OpenCV (BGR) -> ROS Image 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'tablet_camera_optical_frame'
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TabletCameraRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
