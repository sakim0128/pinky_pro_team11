#!/usr/bin/env python3
"""
🩺 도메인 브릿지 워치독 (관제 도메인 8에서 동작)

브릿지는 관제 대시보드와 팀원 미러(도메인 9)를 동시에 공급하는 단일 장애점이다.
브릿지가 죽거나 QoS 불일치로 '조용히 빈 토픽'이 되면 화면상으로는 그냥
로봇이 멈춰 보이기 때문에, 현장에서 원인을 찾는 데 시간을 크게 잃는다.

이 노드는 로봇별 핵심 토픽의 최종 수신 시각을 추적해
/bridge/health (std_msgs/String, JSON) 로 1초마다 발행한다.
team_mirror.yaml 이 이 토픽을 도메인 9로도 넘기므로, 팀원도 자기 자리에서
'지금 어느 로봇의 어떤 경로가 끊겼는지'를 즉시 확인할 수 있다.

    ros2 topic echo /bridge/health        # 도메인 8 또는 9
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

ROBOTS = {
    'robot1': 10,
    'robot2': 11,
    'robot3': 12,
    'robot4': 13,
}

# 이 시간 이상 소식이 없으면 끊긴 것으로 본다 (초)
STALE_SEC = 3.0


def _qos(reliability, depth):
    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class BridgeWatchdog(Node):
    def __init__(self):
        super().__init__('bridge_watchdog')

        self.last_seen = {}
        self.started_at = time.time()

        reliable = _qos(ReliabilityPolicy.RELIABLE, 10)
        best_effort = _qos(ReliabilityPolicy.BEST_EFFORT, 5)

        for name in ROBOTS:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                self._stamp(f'{name}/odom'), reliable)
            self.create_subscription(
                LaserScan, f'/{name}/scan',
                self._stamp(f'{name}/scan'), best_effort)
            self.create_subscription(
                CompressedImage, f'/{name}/camera/image_raw/compressed',
                self._stamp(f'{name}/camera'), best_effort)

        self.pub = self.create_publisher(String, '/bridge/health', reliable)
        self.create_timer(1.0, self._report)

        self.get_logger().info(
            'Bridge watchdog up on domain 8 — publishing /bridge/health')

    def _stamp(self, key):
        def _cb(_msg):
            self.last_seen[key] = time.time()
        return _cb

    def _report(self):
        now = time.time()
        robots = {}
        degraded = []

        for name, domain in ROBOTS.items():
            streams = {}
            for stream in ('odom', 'scan', 'camera'):
                key = f'{name}/{stream}'
                seen = self.last_seen.get(key)
                age = None if seen is None else round(now - seen, 2)
                alive = age is not None and age < STALE_SEC
                streams[stream] = {'alive': alive, 'age_sec': age}
                if not alive:
                    degraded.append(key)
            robots[name] = {
                'domain': domain,
                'bridge_unit': f'pinky-domain-bridge@{name}_control',
                'streams': streams,
                # 세 경로가 전부 죽었으면 브릿지 자체를, 일부만 죽었으면
                # 해당 온보드 노드를 먼저 의심하라는 힌트를 함께 싣는다.
                'suspect': self._suspect(streams),
            }

        payload = {
            'ts': round(now, 2),
            'uptime_sec': round(now - self.started_at, 1),
            'relay_domain': 8,
            'team_mirror_domain': 9,
            'robots': robots,
            'degraded': degraded,
            'ok': not degraded,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

    @staticmethod
    def _suspect(streams):
        alive = [s for s, v in streams.items() if v['alive']]
        if len(alive) == len(streams):
            return None
        if not alive:
            return 'bridge_or_robot_down'
        return 'onboard_node_down'


def main():
    rclpy.init()
    node = BridgeWatchdog()
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
