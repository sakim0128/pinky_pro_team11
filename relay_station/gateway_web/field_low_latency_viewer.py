#!/usr/bin/env python3
"""
Field Low-Latency ROS 2 Viewer (HUD Monitor)
- ROS_DOMAIN_ID=10 환경에서 /camera/image_raw 토픽 초저지연 직접 수신
- 송신-수신 구간 지연시간(End-to-End Latency ms) 및 수신 FPS 정밀 측정
- 화면 상단 HUD(Head-Up Display) 계측 정보 실시간 오버레이
"""

import os
import sys
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FieldLowLatencyViewer(Node):
    def __init__(self):
        super().__init__('field_low_latency_viewer')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 1)

        self.last_frame = None
        self.last_latency_ms = 0.0
        self.fps = 0.0
        self.frame_count = 0
        self.last_stat_time = time.time()
        self.last_received_time = time.time()

        self.get_logger().info(f"[Viewer] Subscribed to {self.camera_topic}. Target latency < 10ms (0.01s).")

    def image_callback(self, msg: Image):
        receive_time = self.get_clock().now()
        
        # 헤더 타임스탬프와 현재 시각 차이로 지연시간(ms) 계산
        send_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        recv_time_sec = receive_time.nanoseconds * 1e-9
        latency_sec = recv_time_sec - send_time_sec
        self.last_latency_ms = max(latency_sec * 1000.0, 0.0)

        # OpenCV BGR 변환
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame = cv_img
            self.last_received_time = time.time()
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")

        now = time.time()
        if now - self.last_stat_time >= 1.0:
            self.fps = self.frame_count / (now - self.last_stat_time)
            self.frame_count = 0
            self.last_stat_time = now


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description="Field Low-Latency ROS 2 Viewer")
    parser.add_argument('--no-gui', action='store_true', help="Run in terminal CLI mode without X11 window")
    cli_args, remaining_args = parser.parse_known_args()

    rclpy.init(args=remaining_args)
    node = FieldLowLatencyViewer()

    has_gui = not cli_args.no_gui and os.environ.get('DISPLAY') is not None
    window_name = "Field Relay Low-Latency Monitor (0.01s Target)"

    if has_gui:
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 1024, 576)
        except Exception as e:
            print(f"[WARN] Failed to open X11 window ({e}), falling back to console mode.")
            has_gui = False

    print("==========================================================")
    print(f" 🖥️  Field Relay Low-Latency ROS 2 Viewer Starting [{'GUI' if has_gui else 'CONSOLE'}]")
    print(f" - Target Topic: {node.camera_topic}")
    print(" - Target Latency: < 10ms (< 0.01s)")
    print(" - Exit: Press 'q' or Ctrl+C")
    print("==========================================================")

    last_print_time = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            
            frame = node.last_frame
            latency = node.last_latency_ms
            status_tag = "[PASS < 10ms]" if latency < 10.0 else ("[OK]" if latency < 30.0 else "[HIGH]")

            if frame is not None:
                if has_gui:
                    display_frame = frame.copy()
                    h, w, _ = display_frame.shape

                    # HUD 반투명 탑 바
                    hud_bg = display_frame[:70, :].copy()
                    cv2.rectangle(display_frame, (0, 0), (w, 70), (20, 24, 30), -1)
                    cv2.addWeighted(hud_bg, 0.3, display_frame[:70, :], 0.7, 0, display_frame[:70, :])

                    lat_color = (0, 255, 0) if latency < 10.0 else ((0, 220, 255) if latency < 30.0 else (0, 0, 255))
                    cv2.putText(display_frame, f"ROS 2 FIELD VIEWER (DOMAIN 10)", (15, 26),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(display_frame, f"Latency: {latency:.2f} ms {status_tag}", (15, 56),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, lat_color, 2, cv2.LINE_AA)
                    cv2.putText(display_frame, f"FPS: {node.fps:.1f}", (480, 56),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(display_frame, f"Resolution: {w}x{h}", (700, 56),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1, cv2.LINE_AA)

                    cv2.imshow(window_name, display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:
                        break
                else:
                    # 콘솔 계측 출력 (0.5초 주기)
                    now = time.time()
                    if now - last_print_time >= 0.5:
                        h, w, _ = frame.shape
                        print(f"📊 [FIELD HUD] Latency: {latency:.2f} ms {status_tag} | FPS: {node.fps:.1f} | Res: {w}x{h}")
                        last_print_time = now
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
