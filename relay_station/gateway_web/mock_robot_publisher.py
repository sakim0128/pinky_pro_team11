#!/usr/bin/env python3
"""
Mock Robot Pose Publisher for Field Gateway Validation
- 실제 로봇이 주행 중이 아닐 때도 게이트웨이 및 관제 뷰어 동작을 검증하기 위한 가상 주행 노드
- Robot 1: 미션 경로(1번 원점 -> 2번 지점 -> 3번 지점 -> 1번 원점) 부드러운 순환 주행
- Robot 2: 대기 구역 순찰 주행
- 토픽: /robot1/odom, /robot2/odom (nav_msgs/Odometry, DOMAIN 10)
"""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class MockRobotPublisher(Node):
    def __init__(self):
        super().__init__('mock_robot_publisher')
        self.pub_r1 = self.create_publisher(Odometry, '/robot1/odom', 10)
        self.pub_r2 = self.create_publisher(Odometry, '/robot2/odom', 10)

        # 미션 경유지 정의
        self.waypoints = [
            (0.0, 0.0),    # 1번 원점
            (-2.2, 0.0),   # 2번 지점
            (0.0, 0.0),    # 1번 복귀
            (-2.2, 0.0),   # 2번 지점
            (2.0, -1.8),   # 3번 지점
            (0.0, 0.0),    # 1번 복귀
        ]
        self.wp_idx = 0
        self.progress = 0.0

        self.r1_x = 0.0
        self.r1_y = 0.0
        self.r1_yaw = 0.0

        self.timer = self.create_timer(0.05, self.update_and_publish) # 20Hz
        self.get_logger().info("Mock Robot Publisher started for /robot1/odom & /robot2/odom")

    def update_and_publish(self):
        # Robot 1: 웨이포인트 보간 이동
        p_from = self.waypoints[self.wp_idx]
        p_to = self.waypoints[(self.wp_idx + 1) % len(self.waypoints)]

        self.progress += 0.01
        if self.progress >= 1.0:
            self.progress = 0.0
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)

        self.r1_x = p_from[0] + (p_to[0] - p_from[0]) * self.progress
        self.r1_y = p_from[1] + (p_to[1] - p_from[1]) * self.progress
        self.r1_yaw = math.atan2(p_to[1] - p_from[1], p_to[0] - p_from[0])

        msg1 = self._create_odom_msg('robot1_base_link', self.r1_x, self.r1_y, self.r1_yaw)
        self.pub_r1.publish(msg1)

        # Robot 2: 우측 하단에서 작은 원형 순찰
        t = time.time() * 0.5
        r2_x = 1.5 + 0.6 * math.cos(t)
        r2_y = -0.8 + 0.6 * math.sin(t)
        r2_yaw = t + math.pi / 2
        msg2 = self._create_odom_msg('robot2_base_link', r2_x, r2_y, r2_yaw)
        self.pub_r2.publish(msg2)

    def _create_odom_msg(self, frame_id, x, y, yaw):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = frame_id

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Yaw -> Quaternion
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return msg


def main():
    rclpy.init()
    node = MockRobotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
