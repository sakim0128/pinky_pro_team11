#!/usr/bin/env python3
"""핑키 CSI 카메라 → sensor_msgs/CompressedImage (JPEG). picamera2 사용.

카메라는 **거꾸로** 장착되어 있다. 보정(orient)은 여기서 한 번만 하고 관제 파이프라인은
정방향 이미지만 본다. 값은 tools/record_drive.py --snap 으로 확정한다:
    none | rot180 | rot180_mirror | vflip | hflip

stamp 는 캡처 직후 로봇 시계로 찍는다. 관제 PC 는 이 stamp 를 LanePath.source_stamp 로
복사만 하고, 로봇이 now − source_stamp 로 왕복 지연을 잰다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

IMAGE_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       durability=QoSDurabilityPolicy.VOLATILE)


def orient_frame(cv2, frame, orient):
    if orient == 'rot180':
        return cv2.rotate(frame, cv2.ROTATE_180)
    if orient == 'rot180_mirror':
        return cv2.flip(cv2.rotate(frame, cv2.ROTATE_180), 1)
    if orient == 'vflip':
        return cv2.flip(frame, 0)
    if orient == 'hflip':
        return cv2.flip(frame, 1)
    return frame


class CameraNode(Node):

    def __init__(self):
        super().__init__('pinky_camera')
        self.declare_parameter('robot_name', 'pinky1')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('orient', 'rot180')
        self.declare_parameter('topic', '')            # 비우면 /<robot_name>/camera/image/compressed
        self.declare_parameter('frame_id', 'camera_link')

        import cv2
        self._cv2 = cv2
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError('picamera2 가 없습니다 (핑키 이미지에는 기본 설치)') from exc

        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        self._orient = str(self.get_parameter('orient').value)
        self._quality = int(self.get_parameter('jpeg_quality').value)
        self._frame_id = self.get_parameter('frame_id').value
        name = self.get_parameter('robot_name').value
        topic = self.get_parameter('topic').value or f'/{name}/camera/image/compressed'

        self._cam = Picamera2()
        cfg = self._cam.create_video_configuration(main={'size': (w, h), 'format': 'RGB888'})
        self._cam.configure(cfg)
        self._cam.start()

        self._pub = self.create_publisher(CompressedImage, topic, IMAGE_QOS)
        self._seq = 0
        self.create_timer(1.0 / float(self.get_parameter('fps').value), self._tick)
        self.get_logger().info(f'카메라 {w}x{h} orient={self._orient} → {topic}')

    def _tick(self):
        frame = self._cam.capture_array()          # picamera2 RGB888 은 실제로 BGR 순서로 온다
        stamp = self.get_clock().now().to_msg()
        frame = orient_frame(self._cv2, frame, self._orient)
        ok, buf = self._cv2.imencode('.jpg', frame,
                                     [int(self._cv2.IMWRITE_JPEG_QUALITY), self._quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self._pub.publish(msg)
        self._seq += 1

    def destroy_node(self):
        try:
            self._cam.stop()
        except Exception:
            pass
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
