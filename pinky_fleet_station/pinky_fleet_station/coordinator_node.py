#!/usr/bin/env python3
"""두 핑키의 교착(deadlock)을 감지해 순차 발진시키는 조정 노드.

기본 회피는 각 로봇의 Nav2 로컬 플래너가 담당한다 (서로를 라이다 장애물로 인식).
이 노드는 Nav2 가 스스로 못 빠져나가는 상황 -- 두 로봇이 가까이서 둘 다 멈춰버린
상태 -- 만 감지해서 개입한다.

  * 미션 4번: 만나면 멈춘 뒤 한 대씩 순차 출발
  * 미션 5번: domain_id 가 작은 쪽이 먼저 간다 (큰 쪽이 양보)
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from pinky_fleet_msgs.msg import FleetCommand, RobotState

from .mission_io import MissionError, load_mission

STATE_NORMAL = 'NORMAL'
STATE_YIELD = 'YIELD'

COMMAND_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class Coordinator(Node):

    def __init__(self):
        super().__init__('fleet_coordinator')

        self.declare_parameter('mission', '')
        self.declare_parameter('tick_rate', 10.0)

        mission_path = self.get_parameter('mission').value
        if not mission_path:
            raise MissionError('mission 파라미터에 mission.yaml 경로를 지정해야 합니다.')
        self._mission = load_mission(mission_path)
        cfg = self._mission.coordinator

        self._conflict_distance = cfg['conflict_distance']
        self._clear_distance = cfg['clear_distance']
        self._stall_speed = cfg['stall_speed']
        self._stall_duration = cfg['stall_duration']
        self._resume_timeout = cfg['resume_timeout']
        self._cooldown = cfg['cooldown']
        self._state_timeout = cfg['state_timeout']

        if self._clear_distance <= self._conflict_distance:
            self.get_logger().warn(
                'clear_distance 가 conflict_distance 보다 크지 않습니다. '
                '히스테리시스가 없어 양보/재출발이 반복될 수 있습니다.')

        # 우선순위 순서로 고정해 둔다 (index 0 = domain_id 가 가장 작은 로봇).
        self._robots = self._mission.by_priority()
        if len(self._robots) != 2:
            self.get_logger().warn(
                f'로봇이 {len(self._robots)}대입니다. 교착 판정은 앞의 2대만 사용합니다.')
        self._pair = self._robots[:2]

        self._states = {}        # name -> RobotState
        self._state_times = {}   # name -> 수신 시각(초)

        self._command_pubs = {}
        for robot in self._robots:
            name = robot['name']
            self._command_pubs[name] = self.create_publisher(
                FleetCommand, robot['command_topic'], COMMAND_QOS)
            self.create_subscription(
                RobotState, robot['state_topic'],
                lambda msg, n=name: self._on_state(n, msg), 10)

        self._status_pub = self.create_publisher(String, '/fleet/coordinator_status', 10)

        self._state = STATE_NORMAL
        self._stall_timer = 0.0
        self._yield_start = None
        self._cooldown_until = 0.0
        self._yielder = None
        self._leader = None
        self._reason = ''

        self._tick_period = 1.0 / float(self.get_parameter('tick_rate').value)
        self._last_tick = self._now()
        self.create_timer(self._tick_period, self._tick)

        names = ' < '.join(f"{r['name']}(domain {r['domain_id']})" for r in self._pair)
        self.get_logger().info(f'fleet_coordinator 시작. 우선순위: {names}')

    # ------------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, name, msg: RobotState):
        self._states[name] = msg
        self._state_times[name] = self._now()

    def _fresh_state(self, name, now):
        stamp = self._state_times.get(name)
        if stamp is None or (now - stamp) > self._state_timeout:
            return None
        return self._states.get(name)

    def _publish_command(self, name, command):
        msg = FleetCommand()
        msg.command = command
        self._command_pubs[name].publish(msg)

    # ------------------------------------------------------------------

    def _tick(self):
        now = self._now()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        a = self._fresh_state(self._pair[0]['name'], now)
        b = self._fresh_state(self._pair[1]['name'], now)

        if a is None or b is None:
            # 한쪽 상태가 끊기면 판단하지 않는다. 이미 양보 중이면 안전하게 풀어 준다.
            if self._state == STATE_YIELD:
                self._release('상대 로봇 상태 수신이 끊겨 양보 해제')
            self._stall_timer = 0.0
            self._reason = '로봇 상태 수신 대기 중'
            self._publish_status(None)
            return

        distance = math.hypot(a.x - b.x, a.y - b.y)

        if self._state == STATE_NORMAL:
            self._tick_normal(a, b, distance, dt, now)
        else:
            self._tick_yield(a, b, distance, now)

        self._publish_status(distance)

    def _tick_normal(self, a, b, distance, dt, now):
        if now < self._cooldown_until:
            self._stall_timer = 0.0
            self._reason = '해제 직후 대기(cooldown)'
            return

        both_localized = a.localized and b.localized
        both_active = (a.nav_status == RobotState.NAV_ACTIVE
                       and b.nav_status == RobotState.NAV_ACTIVE)
        both_stalled = (abs(a.linear_velocity) < self._stall_speed
                        and abs(b.linear_velocity) < self._stall_speed)
        close = distance < self._conflict_distance

        if both_localized and both_active and both_stalled and close:
            self._stall_timer += dt
            self._reason = f'근접 정지 {self._stall_timer:.1f}s / {self._stall_duration:.1f}s'
        else:
            self._stall_timer = 0.0
            self._reason = '정상 주행'

        if self._stall_timer >= self._stall_duration:
            # domain_id 가 작은 쪽이 leader (먼저 통과), 큰 쪽이 yielder (양보).
            self._leader = self._pair[0]['name']
            self._yielder = self._pair[1]['name']
            self._publish_command(self._yielder, FleetCommand.CMD_STOP)
            self._state = STATE_YIELD
            self._yield_start = now
            self._stall_timer = 0.0
            self._reason = f'{self._yielder} 양보 대기, {self._leader} 먼저 통과'
            self.get_logger().info(
                f'교착 감지 (거리 {distance:.2f}m) -> {self._yielder} 정지, '
                f'{self._leader} 우선 통과')

    def _tick_yield(self, a, b, distance, now):
        # leader 는 항상 self._pair[0] (domain_id 가 작은 쪽) 이고, a 가 그 상태다.
        # 메시지의 name 필드에 의존하지 않는다 (에이전트 robot_name 오타에 안전).
        del b
        leader_state = a
        waited = now - (self._yield_start or now)

        if distance > self._clear_distance:
            self._release(f'거리 확보 ({distance:.2f}m)')
        elif leader_state.nav_status in (
                RobotState.NAV_SUCCEEDED, RobotState.NAV_ABORTED, RobotState.NAV_CANCELED):
            self._release(f'{self._leader} 주행 종료')
        elif waited > self._resume_timeout:
            self._release(f'양보 대기 시간 초과 ({waited:.0f}s)')
        else:
            self._reason = (f'{self._yielder} 양보 중 ({waited:.0f}s / '
                            f'{self._resume_timeout:.0f}s), 거리 {distance:.2f}m')

    def _release(self, reason):
        if self._yielder is not None:
            self._publish_command(self._yielder, FleetCommand.CMD_RESUME)
            self.get_logger().info(f'{self._yielder} 재출발: {reason}')
        self._state = STATE_NORMAL
        self._stall_timer = 0.0
        self._yield_start = None
        self._cooldown_until = self._now() + self._cooldown
        self._reason = f'재출발: {reason}'
        self._yielder = None
        self._leader = None

    def _publish_status(self, distance):
        msg = String()
        msg.data = json.dumps({
            'state': self._state,
            'reason': self._reason,
            'distance': None if distance is None else round(distance, 3),
            'leader': self._leader,
            'yielder': self._yielder,
        }, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Coordinator()
    except MissionError as exc:
        print(f'[fleet_coordinator] mission.yaml 오류: {exc}')
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
