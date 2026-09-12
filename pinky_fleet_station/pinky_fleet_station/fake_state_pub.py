#!/usr/bin/env python3
"""실기/시뮬레이터 없이 coordinator 와 GUI 를 검증하기 위한 가짜 로봇 2대.

mission.yaml 을 읽어 각 로봇의 ``state_topic`` 으로 RobotState 를 발행하고
``command_topic`` 의 FleetCommand 를 실제 에이전트와 같은 규칙으로 처리한다.

이동 모델은 아주 단순하다.

  * 목표를 향해 직선으로 max_linear_vel 로 전진한다.
  * 상대 로봇이 ``block_distance`` 안에 있고 상대도 주행 중이면 멈춘다
    (서로 길을 막아 Nav2 가 못 빠져나가는 교착 상황).
  * 상대가 멈춰 있으면(HOLD/도착/대기) 정적 장애물로 보고 옆으로 비켜 간다
    (Nav2 로컬 플래너가 정지한 로봇을 우회하는 동작).

따라서 서로 교차하는 목표를 주면 전체 시나리오가 그대로 재현된다.

    동시 출발 -> 가운데서 맞물려 정지 -> 3초 뒤 coordinator 가 domain_id 가 큰
    pinky2 에 CMD_STOP -> pinky1 이 멈춰 선 pinky2 를 우회 -> 거리 확보 ->
    coordinator 가 pinky2 에 CMD_RESUME -> pinky2 재출발

사용 예:

    ros2 run pinky_fleet_station fake_state_pub --ros-args \
        -p mission:=config/mission.yaml -p auto_start:=true
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from pinky_fleet_msgs.msg import FleetCommand, RobotState

from .mission_io import MissionError, load_mission

COMMAND_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class FakeRobot:

    def __init__(self, spec):
        self.name = spec['name']
        self.domain_id = spec['domain_id']
        self.x = spec['initial_pose']['x']
        self.y = spec['initial_pose']['y']
        self.yaw = spec['initial_pose']['yaw']
        self.max_linear_vel = spec['max_linear_vel']
        self.max_angular_vel = spec['max_angular_vel']
        self.goal = (spec['goal']['x'], spec['goal']['y'], spec['goal']['yaw'])
        self.goal_valid = False
        self.nav_status = RobotState.NAV_IDLE
        self.hold = False
        self.linear_velocity = 0.0


class FakeStatePublisher(Node):

    def __init__(self):
        super().__init__('fake_state_pub')

        self.declare_parameter('mission', '')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('block_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('auto_start', False)

        mission_path = self.get_parameter('mission').value
        if not mission_path:
            raise MissionError('mission 파라미터에 mission.yaml 경로를 지정해야 합니다.')
        mission = load_mission(mission_path)

        self._block_distance = float(self.get_parameter('block_distance').value)
        self._goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self._rate = float(self.get_parameter('rate').value)

        self._robots = {}
        self._pubs = {}
        for spec in mission.robots:
            robot = FakeRobot(spec)
            self._robots[robot.name] = robot
            self._pubs[robot.name] = self.create_publisher(RobotState, spec['state_topic'], 10)
            self.create_subscription(
                FleetCommand, spec['command_topic'],
                lambda msg, n=robot.name: self._on_command(n, msg), COMMAND_QOS)

        if self.get_parameter('auto_start').value:
            for robot in self._robots.values():
                robot.goal_valid = True
                robot.nav_status = RobotState.NAV_ACTIVE

        self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f'fake_state_pub 시작: {list(self._robots)} '
            f'(block_distance={self._block_distance}m)')

    def _on_command(self, name, msg: FleetCommand):
        robot = self._robots[name]
        if msg.command == FleetCommand.CMD_GOTO:
            robot.goal = (msg.x, msg.y, msg.yaw)
            robot.goal_valid = True
            robot.hold = False
            robot.nav_status = RobotState.NAV_ACTIVE
        elif msg.command == FleetCommand.CMD_STOP:
            robot.hold = True
            robot.nav_status = RobotState.NAV_HOLD
            self.get_logger().info(f'{name}: STOP 수신')
        elif msg.command == FleetCommand.CMD_RESUME:
            robot.hold = False
            robot.nav_status = (
                RobotState.NAV_ACTIVE if robot.goal_valid else RobotState.NAV_IDLE)
            self.get_logger().info(f'{name}: RESUME 수신')
        elif msg.command == FleetCommand.CMD_CANCEL:
            robot.hold = False
            robot.goal_valid = False
            robot.nav_status = RobotState.NAV_CANCELED
        elif msg.command == FleetCommand.CMD_SET_INITIAL_POSE:
            robot.x, robot.y, robot.yaw = msg.x, msg.y, msg.yaw
        elif msg.command == FleetCommand.CMD_SET_SPEED:
            if msg.max_linear_vel > 0.0:
                robot.max_linear_vel = msg.max_linear_vel
            if msg.max_angular_vel > 0.0:
                robot.max_angular_vel = msg.max_angular_vel

    def _nearest_blocker(self, robot):
        """``block_distance`` 안에 있는 다른 로봇 중 가장 가까운 것."""
        nearest = None
        nearest_distance = self._block_distance
        for other in self._robots.values():
            if other is robot:
                continue
            distance = math.hypot(other.x - robot.x, other.y - robot.y)
            if distance < nearest_distance:
                nearest, nearest_distance = other, distance
        return nearest

    @staticmethod
    def _detour_heading(robot, blocker, heading):
        """멈춰 있는 상대를 향해 파고드는 성분을 빼서 옆으로 스치듯 지나가게 한다."""
        ox, oy = blocker.x - robot.x, blocker.y - robot.y
        norm = math.hypot(ox, oy)
        if norm < 1e-6:
            return heading + math.pi / 2.0
        ox, oy = ox / norm, oy / norm
        hx, hy = math.cos(heading), math.sin(heading)
        into = hx * ox + hy * oy           # 상대 쪽으로 향하는 성분
        if into <= 0.0:
            return heading                 # 이미 멀어지는 방향이면 그대로
        tx, ty = hx - into * ox, hy - into * oy
        tangent = math.hypot(tx, ty)
        if tangent < 1e-6:
            # 정면으로 마주 본 경우: 한쪽으로 확실히 틀어 준다.
            return heading + math.pi / 2.0
        return math.atan2(ty / tangent, tx / tangent)

    def _tick(self):
        dt = 1.0 / self._rate
        for robot in self._robots.values():
            robot.linear_velocity = 0.0
            if robot.nav_status != RobotState.NAV_ACTIVE or robot.hold:
                continue

            gx, gy, gyaw = robot.goal
            dx, dy = gx - robot.x, gy - robot.y
            distance = math.hypot(dx, dy)
            if distance < self._goal_tolerance:
                robot.yaw = gyaw
                robot.goal_valid = False
                robot.nav_status = RobotState.NAV_SUCCEEDED
                continue

            heading = math.atan2(dy, dx)
            robot.yaw = heading

            blocker = self._nearest_blocker(robot)
            if blocker is not None:
                if blocker.nav_status == RobotState.NAV_ACTIVE:
                    # 상대도 주행 중 -> 서로 길을 막은 교착. 둘 다 멈춘다.
                    continue
                # 상대가 멈춰 있다 -> 정적 장애물로 보고 옆으로 비켜 간다.
                heading = self._detour_heading(robot, blocker, heading)
                robot.yaw = heading

            step = min(robot.max_linear_vel * dt, distance)
            robot.x += step * math.cos(heading)
            robot.y += step * math.sin(heading)
            robot.linear_velocity = step / dt

        now = self.get_clock().now().to_msg()
        for name, robot in self._robots.items():
            msg = RobotState()
            msg.header.stamp = now
            msg.header.frame_id = 'map'
            msg.name = name
            msg.domain_id = robot.domain_id
            msg.localized = True
            msg.x, msg.y, msg.yaw = robot.x, robot.y, robot.yaw
            msg.linear_velocity = robot.linear_velocity
            msg.angular_velocity = 0.0
            msg.nav_status = robot.nav_status
            msg.goal_valid = robot.goal_valid
            msg.goal_x, msg.goal_y, msg.goal_yaw = robot.goal
            msg.max_linear_vel = robot.max_linear_vel
            msg.max_angular_vel = robot.max_angular_vel
            msg.battery_percent = 100.0
            self._pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FakeStatePublisher()
    except MissionError as exc:
        print(f'[fake_state_pub] mission.yaml 오류: {exc}')
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
