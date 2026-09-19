#!/usr/bin/env python3
"""천장 USB 웹캠 → 핑키 2대 PoseFix (D8). 관제 PC 에서 돈다.

    ros2 run pinky_lane_station overhead_localizer_node --ros-args -p config:=.../overhead.yaml

출력  /<robot>/pose_fix (PoseFix, stamp = 캡처 시각 = 관제 시계, stamp_is_robot_clock=false)
      /overhead/debug/compressed (검출·좌표 오버레이) · /overhead/status (String JSON)
로봇 pose_fuser_node 가 (수신 시각 − station_latency) 의 odom 에 붙인다.
"""

import json
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from pinky_lane_msgs.msg import PoseFix

from .overhead_localizer import OverheadConfig, OverheadLocalizer

FIX_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.VOLATILE)
BEST_EFFORT_1 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                           reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.VOLATILE)


def open_camera(cam):
    dev = cam.get('device', 0)
    cap = cv2.VideoCapture(int(dev) if str(dev).isdigit() else str(dev))
    if not cap.isOpened():
        raise RuntimeError(f'카메라를 열 수 없습니다: {dev}')
    if 'width' in cam:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam['width']))
    if 'height' in cam:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam['height']))
    if 'fps' in cam:
        cap.set(cv2.CAP_PROP_FPS, float(cam['fps']))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if cam.get('autoexposure') is False:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)       # V4L2: 1 = manual (지원 안 하면 무시)
    return cap


def draw_debug(img, loc, dets, fixes):
    for mid, c in dets.items():
        ref = mid in loc.cfg.reference
        cv2.polylines(img, [c.astype(int).reshape(-1, 1, 2)], True, (0, 200, 255) if ref else (0, 255, 0), 2)
        cv2.putText(img, str(mid), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 200, 255) if ref else (0, 255, 0), 2)
    y = 24
    st = loc.status()
    cv2.putText(img, f"ref {st['reference_seen']}/{len(loc.cfg.reference)} reproj {st['reproj_px']} px",
                (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if not st['homography'] else (255, 255, 0), 2)
    for name, f in fixes.items():
        y += 26
        cv2.putText(img, f'{name}: ({f.x:.2f}, {f.y:.2f}) {f.yaw:.2f} rad', (8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    return img


class OverheadNode(Node):

    def __init__(self):
        super().__init__('overhead_localizer')
        self.declare_parameter('config', '')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_scale', 0.5)
        self.cfg = OverheadConfig.load(self.get_parameter('config').value)
        self.loc = OverheadLocalizer(self.cfg)
        self.cap = open_camera(self.cfg.camera)
        self._fix_pubs = {name: self.create_publisher(PoseFix, f'/{name}/pose_fix', FIX_QOS)
                          for name in self.cfg.robots}
        self._seq = {name: 0 for name in self.cfg.robots}
        self._status_pub = self.create_publisher(String, '/overhead/status', 10)
        self._debug_pub = (self.create_publisher(CompressedImage, '/overhead/debug/compressed', BEST_EFFORT_1)
                           if self.get_parameter('publish_debug_image').value else None)
        self._debug_scale = float(self.get_parameter('debug_scale').value)
        self._t_hist = []
        fps = float(self.cfg.camera.get('fps', 15))
        self.create_timer(1.0 / fps, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f'overhead_localizer 시작: device={self.cfg.camera.get("device")} robots={list(self.cfg.robots)} '
            f'reference={sorted(self.cfg.reference)} latency={self.cfg.station_latency}s')

    def _tick(self):
        ok, img = self.cap.read()
        stamp = self.get_clock().now().to_msg()
        if not ok or img is None:
            self.get_logger().warn('프레임 읽기 실패', throttle_duration_sec=2.0)
            return
        t0 = time.perf_counter()
        fixes = self.loc.update(img)
        self._t_hist = [t for t in self._t_hist if time.time() - t < 2.0] + [time.time()]
        for name, f in fixes.items():
            self._seq[name] += 1
            pf = PoseFix()
            pf.header.stamp = stamp                      # 캡처 시각, 관제 시계
            pf.header.frame_id = 'map'
            pf.stamp_is_robot_clock = False
            pf.robot_name = name
            pf.seq = self._seq[name]
            pf.x, pf.y, pf.yaw = f.x, f.y, f.yaw
            pf.marker_id = int(f.marker_id)
            pf.marker_range = 0.0
            pf.reproj_error = float(f.reproj)
            pf.n_markers = int(f.n_markers)
            pf.pipeline_latency = float(time.perf_counter() - t0)
            self._fix_pubs[name].publish(pf)
        if self._debug_pub is not None:
            dbg = draw_debug(img.copy(), self.loc, self.loc.detect(img), fixes)
            if self._debug_scale != 1.0:
                dbg = cv2.resize(dbg, None, fx=self._debug_scale, fy=self._debug_scale)
            ok, buf = cv2.imencode('.jpg', dbg, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok:
                m = CompressedImage()
                m.header.stamp = stamp
                m.format = 'jpeg'
                m.data = buf.tobytes()
                self._debug_pub.publish(m)

    def _publish_status(self):
        st = self.loc.status()
        st['fps'] = len(self._t_hist) / 2.0
        st['robots'] = list(self.cfg.robots)
        msg = String()
        msg.data = json.dumps(st, ensure_ascii=False)
        self._status_pub.publish(msg)
        if not st['homography']:
            self.get_logger().warn(f"기준 마커 {st['reference_seen']}/{len(self.cfg.reference)} — H 없음, fix 안 나감",
                                   throttle_duration_sec=5.0)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OverheadNode()
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f'[overhead_localizer] 시작 실패: {exc}')
        rclpy.shutdown()
        return
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
