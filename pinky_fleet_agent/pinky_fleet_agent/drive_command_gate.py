#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/cmd_vel 의 유일한 발행자 — 비상정지 · 수동 조종 · 자율 주행을 한 곳에서 중재한다.

hybrid_robot.launch.xml(Nav2 + 게이트) 에서만 뜬다. 기존 robot.launch.xml · lane_robot.launch.xml
은 이 노드를 쓰지 않는다 (lane_agent_node 는 스스로 /cmd_vel 유일 발행자다).

    입력  /estop            (std_msgs/Bool)        hybrid_agent_node 가 ESTOP·링크 유실에서 true
          /cmd_vel_teleop   (geometry_msgs/Twist)  수동 조종
          /cmd_vel_mission  (geometry_msgs/Twist)  Nav2 (velocity_smoother · behavior_server 출력을 remap)
    출력  /cmd_vel          (geometry_msgs/Twist)  rate_hz 로 늘 낸다
          /drive_gate_status (std_msgs/String)     "source=MISSION vx=0.12 wz=0.00"

우선순위: ESTOP > TELEOP > MISSION > IDLE. 입력이 timeout 동안 없으면 그 소스는 죽은 것으로 보고
다음 소스로 내려간다. 아무것도 없으면 0 을 낸다 — Nav2 가 goal 취소 뒤 마지막 속도를 남겨도
0.5 s 뒤에는 멈춘다.

비상정지가 **바뀌는 순간** 저장된 입력을 버린다. 해제 직후 비상정지 전에 받아 둔 속도가 다시
나가면 안 되기 때문이다 — 해제 뒤에는 새로 들어온 입력만 쓴다. 같은 값이 되풀이되면 버리지
않는다 (에이전트가 래치 동안 true 를 1 s 마다 다시 내므로).

판단은 :class:`CommandGateCore` 에 있고 rclpy 에 의존하지 않는다 —
``test/test_drive_command_gate.py`` 가 ROS 없이 검증한다.
"""

import sys
from enum import Enum
from typing import Optional, Tuple

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    Node = object


class GateSource(str, Enum):
    ESTOP = 'ESTOP'
    TELEOP = 'TELEOP'
    MISSION = 'MISSION'
    IDLE = 'IDLE'


class CommandGateCore:
    """중재 로직. 시각은 초 단위 float 로만 받는다."""

    def __init__(self, teleop_timeout_sec: float = 0.5, mission_timeout_sec: float = 0.5):
        self.teleop_timeout_sec = float(teleop_timeout_sec)
        self.mission_timeout_sec = float(mission_timeout_sec)

        self.estop_active = False
        self.last_teleop_cmd: Optional[Tuple[float, float]] = None
        self.last_teleop_time: Optional[float] = None

        self.last_mission_cmd: Optional[Tuple[float, float]] = None
        self.last_mission_time: Optional[float] = None

    def set_estop(self, active: bool) -> bool:
        """비상정지 상태를 받는다. 상태가 바뀌었으면 True."""
        active = bool(active)
        if active == self.estop_active:
            return False
        self.estop_active = active
        self.last_teleop_cmd = None
        self.last_teleop_time = None
        self.last_mission_cmd = None
        self.last_mission_time = None
        return True

    def feed_teleop(self, linear_x: float, angular_z: float, now: float) -> None:
        self.last_teleop_cmd = (float(linear_x), float(angular_z))
        self.last_teleop_time = float(now)

    def feed_mission(self, linear_x: float, angular_z: float, now: float) -> None:
        self.last_mission_cmd = (float(linear_x), float(angular_z))
        self.last_mission_time = float(now)

    def step(self, now: float) -> Tuple[float, float, GateSource]:
        """(linear_x, angular_z, 이긴 소스) 를 돌려준다."""
        now = float(now)

        if self.estop_active:
            return 0.0, 0.0, GateSource.ESTOP

        if self.last_teleop_time is not None and self.last_teleop_cmd is not None:
            if (now - self.last_teleop_time) <= self.teleop_timeout_sec:
                return self.last_teleop_cmd[0], self.last_teleop_cmd[1], GateSource.TELEOP

        if self.last_mission_time is not None and self.last_mission_cmd is not None:
            if (now - self.last_mission_time) <= self.mission_timeout_sec:
                return self.last_mission_cmd[0], self.last_mission_cmd[1], GateSource.MISSION

        return 0.0, 0.0, GateSource.IDLE


if HAS_RCLPY:
    class DriveCommandGateNode(Node):

        def __init__(self):
            super().__init__('drive_command_gate')

            self.declare_parameter('teleop_timeout_sec', 0.5)
            self.declare_parameter('mission_timeout_sec', 0.5)
            self.declare_parameter('rate_hz', 20.0)
            self.declare_parameter('output_topic', '/cmd_vel')

            t_to = float(self.get_parameter('teleop_timeout_sec').value)
            m_to = float(self.get_parameter('mission_timeout_sec').value)
            rate_hz = float(self.get_parameter('rate_hz').value)
            out_topic = self.get_parameter('output_topic').value

            self.core = CommandGateCore(teleop_timeout_sec=t_to, mission_timeout_sec=m_to)

            self.create_subscription(Bool, '/estop', self._on_estop, 10)
            self.create_subscription(Twist, '/cmd_vel_teleop', self._on_teleop, 10)
            self.create_subscription(Twist, '/cmd_vel_mission', self._on_mission, 10)

            self._cmd_pub = self.create_publisher(Twist, out_topic, 10)
            self._status_pub = self.create_publisher(String, '/drive_gate_status', 10)

            self.create_timer(1.0 / rate_hz, self._loop)
            self.get_logger().info(
                f'drive_command_gate 시작: out={out_topic} rate={rate_hz:g}Hz '
                f'teleop_timeout={t_to:g}s mission_timeout={m_to:g}s')

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_estop(self, msg: Bool):
            if not self.core.set_estop(msg.data):
                return
            if msg.data:
                self.get_logger().warn('비상정지: /cmd_vel 을 0 으로 묶습니다.')
            else:
                self.get_logger().info('비상정지 해제: 새로 들어오는 명령부터 통과시킵니다.')

        def _on_teleop(self, msg: Twist):
            self.core.feed_teleop(msg.linear.x, msg.angular.z, self._now())

        def _on_mission(self, msg: Twist):
            self.core.feed_mission(msg.linear.x, msg.angular.z, self._now())

        def _loop(self):
            vx, wz, source = self.core.step(self._now())

            t = Twist()
            t.linear.x = float(vx)
            t.angular.z = float(wz)
            self._cmd_pub.publish(t)

            self._status_pub.publish(
                String(data=f'source={source.value} vx={vx:.2f} wz={wz:.2f}'))


def main(args=None):
    if not HAS_RCLPY:
        print('rclpy 가 없어 노드로 실행할 수 없습니다.')
        sys.exit(1)
    rclpy.init(args=args)
    node = DriveCommandGateNode()
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
