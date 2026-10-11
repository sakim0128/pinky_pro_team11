#!/usr/bin/env python3
"""상부(천장) 웹캠 → sensor_msgs/CompressedImage (JPEG). cv2.VideoCapture 사용. 관제 PC(도메인 8)에서 띄운다.

overhead_tracker_node 가 같은 토픽(/overhead/camera/image/compressed)을 구독해 ArUco 로 로봇 위치를 만들고,
live_web_node 도 같은 토픽을 화면에 보여준다. stamp 는 캡처 직후 관제 시계.

QoS: overhead_tracker_node · live_web_node 가 기본 프로파일(RELIABLE · VOLATILE, depth 10)로 구독한다.
BEST_EFFORT 발행자는 RELIABLE 구독자와 **연결되지 않으므로** 여기서는 RELIABLE · VOLATILE · KEEP_LAST 1 로 낸다
(depth 는 짝이 달라도 된다 — 발행 쪽 1 이면 밀린 프레임을 버린다).

장치 읽기가 실패하면 경고를 내고 reopen_interval 마다 다시 연다. 죽지 않는다.
"""

import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from .overhead_camera import DEFAULT_FRAME_ID, DEFAULT_TOPIC, encode_jpeg, parse_device, validate_params

IMAGE_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE)


class OverheadCameraNode(Node):

    def __init__(self):
        super().__init__('overhead_camera')
        # device 는 int(/dev/video<N>) 도 문자열(경로·파이프라인) 도 받는다 → 동적 타입
        self.declare_parameter('device', 0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('topic', DEFAULT_TOPIC)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.declare_parameter('reopen_interval', 2.0)     # 읽기 실패 뒤 다시 열기까지 (s)

        import cv2
        self._cv2 = cv2

        self._device = parse_device(self.get_parameter('device').value)
        self._width, self._height, self._fps, self._quality = validate_params(
            self.get_parameter('width').value, self.get_parameter('height').value,
            self.get_parameter('fps').value, self.get_parameter('jpeg_quality').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._reopen_interval = max(0.1, float(self.get_parameter('reopen_interval').value))
        topic = str(self.get_parameter('topic').value) or DEFAULT_TOPIC

        self._cap = None
        self._next_open = 0.0
        self._n_published = 0
        self._pub = self.create_publisher(CompressedImage, topic, IMAGE_QOS)
        self._open()
        self.create_timer(1.0 / self._fps, self._tick)
        self.get_logger().info(f'상부 카메라 device={self._device!r} {self._width}x{self._height} '
                               f'@{self._fps:g} Hz q={self._quality} → {topic}')

    # --- 장치 --------------------------------------------------------------

    def _open(self):
        """VideoCapture 를 연다. 실패하면 False (reopen_interval 뒤 다시 시도)."""
        self._release()
        self._next_open = time.monotonic() + self._reopen_interval
        try:
            cap = self._cv2.VideoCapture(self._device)
        except Exception as exc:                                   # noqa: BLE001 — 백엔드 오류가 종류가 많다
            self.get_logger().warning(f'상부 카메라 열기 실패 ({self._device!r}): {exc}',
                                      throttle_duration_sec=5.0)
            return False
        if not cap.isOpened():
            cap.release()
            self.get_logger().warning(f'상부 카메라를 열 수 없다 ({self._device!r}). '
                                      f'{self._reopen_interval:g} s 뒤 재시도', throttle_duration_sec=5.0)
            return False
        cap.set(self._cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(self._cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(self._cv2.CAP_PROP_FPS, self._fps)
        got_w = int(cap.get(self._cv2.CAP_PROP_FRAME_WIDTH) or 0)
        got_h = int(cap.get(self._cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        if (got_w, got_h) != (self._width, self._height):
            self.get_logger().warning(f'상부 카메라가 {self._width}x{self._height} 대신 {got_w}x{got_h} 로 열렸다')
        self._cap = cap
        self.get_logger().info(f'상부 카메라 열림 ({self._device!r}) {got_w}x{got_h}')
        return True

    def _release(self):
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:   # noqa: BLE001
                pass
            self._cap = None

    # --- 주기 --------------------------------------------------------------

    def _tick(self):
        if self._cap is None:
            if time.monotonic() >= self._next_open and not self._open():
                return
            if self._cap is None:
                return
        try:
            ok, frame = self._cap.read()
        except Exception as exc:                                   # noqa: BLE001
            ok, frame = False, None
            self.get_logger().warning(f'상부 카메라 read 예외: {exc}', throttle_duration_sec=5.0)
        if not ok or frame is None:
            self.get_logger().warning(f'상부 카메라 프레임 읽기 실패 ({self._device!r}) — '
                                      f'{self._reopen_interval:g} s 뒤 다시 연다', throttle_duration_sec=5.0)
            self._release()
            self._next_open = time.monotonic() + self._reopen_interval
            return
        stamp = self.get_clock().now().to_msg()
        data = encode_jpeg(self._cv2, frame, self._quality)
        if data is None:
            self.get_logger().warning('JPEG 인코딩 실패, 프레임 버림', throttle_duration_sec=5.0)
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.format = 'jpeg'
        msg.data = data
        if not rclpy.ok():
            return
        try:
            self._pub.publish(msg)
            self._n_published += 1
        except Exception:
            pass

    def destroy_node(self):
        self._release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OverheadCameraNode()
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
