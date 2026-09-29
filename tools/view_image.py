#!/usr/bin/env python3
"""CompressedImage 토픽을 창으로 보는 최소 뷰어 — rqt_image_view 대용 (rqt 가 멈추거나 raw/compressed 를 헷갈릴 때).

    export ROS_DOMAIN_ID=8
    python3 tools/view_image.py /pinky1/lane_debug/compressed                 # 창 하나
    python3 tools/view_image.py /pinky1/lane_debug/compressed /pinky2/lane_debug/compressed   # 창 둘
    python3 tools/view_image.py /pinky1/camera/image/compressed --scale 0.5   # 원본 카메라, 절반 크기

q 또는 ESC 로 끝낸다. 창 제목에 토픽과 수신 fps 를 찍는다. 3 초 동안 프레임이 없으면 제목에 'NO FRAME' 을 붙인다.
"""

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

# 발행자가 RELIABLE(파이프라인 lane_debug) 이든 BEST_EFFORT 든 받게 BEST_EFFORT 로 구독한다 (호환 규칙: 구독이 느슨하면 된다)
QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                 reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)


class Viewer(Node):
    def __init__(self, topics, scale):
        super().__init__('view_image')
        self.scale = scale
        self.latest = {}                       # topic -> (frame, t_recv)
        self.stamps = {t: [] for t in topics}
        for t in topics:
            self.create_subscription(CompressedImage, t, lambda msg, t=t: self._cb(t, msg), QOS)
            cv2.namedWindow(t, cv2.WINDOW_NORMAL)

    def _cb(self, topic, msg):
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f'{topic}: 디코드 실패 (format={msg.format!r}, {len(msg.data)} bytes)')
            return
        now = time.monotonic()
        self.latest[topic] = (img, now)
        s = self.stamps[topic]
        s.append(now)
        del s[:-30]

    def fps(self, topic):
        s = self.stamps[topic]
        return 0.0 if len(s) < 2 else (len(s) - 1) / max(1e-6, s[-1] - s[0])

    def draw(self):
        now = time.monotonic()
        for topic in self.stamps:
            item = self.latest.get(topic)
            if item is None:
                blank = np.zeros((240, 640, 3), dtype=np.uint8)
                cv2.putText(blank, 'waiting for first frame ...', (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                cv2.imshow(topic, blank)
                continue
            img, t = item
            if self.scale != 1.0:
                img = cv2.resize(img, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
            stale = ' NO FRAME %.0fs' % (now - t) if now - t > 3.0 else ''
            cv2.setWindowTitle(topic, f'{topic}  {self.fps(topic):.1f} fps{stale}')
            cv2.imshow(topic, img)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('topics', nargs='+', help='sensor_msgs/CompressedImage 토픽')
    ap.add_argument('--scale', type=float, default=1.0)
    args = ap.parse_args()
    rclpy.init()
    node = Viewer(args.topics, args.scale)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.draw()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
