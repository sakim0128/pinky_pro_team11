#!/usr/bin/env python3
"""실기 없이 차선 주행 전체를 검증하는 가짜 로봇 N대 (관제 도메인에 직접 띄운다).

실차 lane_agent_node 와 **같은 LaneDriver** 를 쓴다 — 다른 것은 위치(TF 대신 유니사이클
적분), 라이다(가짜 스캔), 카메라(합성 이미지) 뿐이다. 합성 카메라 이미지는 실제 토픽으로
발행되어 lane_pipeline_node → LanePath → 이 노드로 돌아온다. 그래서 메시지·QoS·시각
규칙까지 실차와 같은 경로를 탄다.

    ros2 launch pinky_lane_station fake_lane.launch.xml

가짜 라이다: 다른 가짜 로봇(반지름 0.08) 과 obstacle 파라미터의 원을 본다.
    -p obstacle:="[0.3, -0.45, 0.05]"   # x, y, r (map). 비우면 없음
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from pinky_fleet_msgs.msg import FleetCommand, RobotState
from pinky_lane_msgs.msg import LaneCommand, LanePath, LaneStatus, Route

from pinky_fleet_agent.lane_driver import (CMD_CLEARANCE, CMD_ESTOP, CMD_HEARTBEAT, CMD_RESUME,
                                           CMD_SET_SPEED, CMD_START, CMD_STOP, DriverParams,
                                           LaneDriver)

from .lane_mission import LaneMissionError, load_lane_mission
from .road_graph import RoadGraph
from .synthetic_camera import CameraModel, render_lane_frame

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None

RELIABLE_10 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
ROUTE_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT_1 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                           reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.VOLATILE)

_CMD_MAP = {LaneCommand.CMD_HEARTBEAT: CMD_HEARTBEAT, LaneCommand.CMD_START: CMD_START,
            LaneCommand.CMD_STOP: CMD_STOP, LaneCommand.CMD_ESTOP: CMD_ESTOP,
            LaneCommand.CMD_RESUME: CMD_RESUME, LaneCommand.CMD_SET_SPEED: CMD_SET_SPEED,
            LaneCommand.CMD_CLEARANCE: CMD_CLEARANCE}
ROBOT_RADIUS = 0.08
SCAN_HALF_DEG = 60
SCAN_STEP_DEG = 2


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    """원점 (ox,oy) 방향 (dx,dy) 단위벡터의 광선과 원의 첫 교점 거리. 없으면 None."""
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * c
    if disc < 0:
        return None
    t = (-b - math.sqrt(disc)) / 2.0
    return t if t > 0 else None


class FakeLaneRobot:
    def __init__(self, spec, graph, params):
        self.spec = spec
        self.name = spec['name']
        self.driver = LaneDriver(params)
        self.driver.p.control.v_max = float(spec['max_linear_vel'])
        self.driver.p.control.omega_max = float(spec['max_angular_vel'])
        node = graph.nodes.get(spec['start'])
        self.x, self.y = (node.x, node.y) if node else (0.0, 0.0)
        self.yaw = graph.node_exit_yaw(spec['start']) if node else 0.0
        self.v = self.w = 0.0
        self.out = None
        self.route_seq = 0
        self.cam_seq = 0
        self.moved = False

    def crosswalk_ahead(self):
        f = self.driver.follower
        if f is None:
            return None
        ds = [f.distance_to_idx(i) for i in self.driver.crosswalk_idx]
        ds = [d for d in ds if 0.0 < d < 0.8]
        return min(ds) if ds else None

    def lane_offsets(self):
        f = self.driver.follower
        if f is None:
            return 0.0, 0.0
        return f.lateral, wrap(self.yaw - f.heading_at(f.progress_s))


class FakeLaneFleet(Node):

    def __init__(self):
        super().__init__('fake_lane_robot')
        self.declare_parameter('mission', '')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('camera_fps', 10.0)
        self.declare_parameter('camera', True)
        self.declare_parameter('loopback', False)       # True: 파이프라인 없이 내부에서 LanePath 생성
        self.declare_parameter('obstacle', [0.0, 0.0, 0.0])
        self.declare_parameter('lidar', True)

        mission = load_lane_mission(self.get_parameter('mission').value)
        self.graph = RoadGraph.load(mission.graph_path)
        self._rate = float(self.get_parameter('rate').value)
        self._camera = bool(self.get_parameter('camera').value)
        self._loopback = bool(self.get_parameter('loopback').value)
        self._lidar = bool(self.get_parameter('lidar').value)
        obs = [float(v) for v in self.get_parameter('obstacle').value]
        self._obstacles = [tuple(obs)] if len(obs) == 3 and obs[2] > 0 else []
        self._cam = CameraModel()
        if self._loopback:
            from .detectors import create_detector
            from .lane_target import LaneTargetEstimator
            self._det = create_detector('classic')
            self._ests = {}

        self._robots = {}
        self._state_pubs, self._status_pubs, self._img_pubs = {}, {}, {}
        for spec in mission.robots:
            r = FakeLaneRobot(spec, self.graph, DriverParams())
            self._robots[r.name] = r
            self._state_pubs[r.name] = self.create_publisher(RobotState, spec['state_topic'], 10)
            self._status_pubs[r.name] = self.create_publisher(
                LaneStatus, spec['lane_status_topic'], RELIABLE_10)
            self._img_pubs[r.name] = self.create_publisher(
                CompressedImage, spec['image_topic'], BEST_EFFORT_1)
            self.create_subscription(Route, spec['route_topic'],
                                     lambda m, n=r.name: self._on_route(n, m), ROUTE_QOS)
            self.create_subscription(LaneCommand, spec['lane_command_topic'],
                                     lambda m, n=r.name: self._on_lane_command(n, m), RELIABLE_10)
            self.create_subscription(LanePath, spec['lane_path_topic'],
                                     lambda m, n=r.name: self._on_lane_path(n, m), BEST_EFFORT_1)
            self.create_subscription(FleetCommand, spec['command_topic'],
                                     lambda m, n=r.name: self._on_fleet_command(n, m), RELIABLE_10)
            if self._loopback:
                self._ests[r.name] = LaneTargetEstimator()

        self.create_timer(1.0 / self._rate, self._tick)
        self.create_timer(0.1, self._publish_status)
        if self._camera:
            self.create_timer(1.0 / float(self.get_parameter('camera_fps').value), self._camera_tick)
        self.get_logger().info(
            f'fake_lane_robot 시작: {[(r.name, round(r.x, 2), round(r.y, 2)) for r in self._robots.values()]} '
            f'camera={self._camera} loopback={self._loopback} obstacles={self._obstacles}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 입력

    def _on_route(self, name, msg: Route):
        r = self._robots[name]
        if len(msg.waypoints) < 2:
            return
        r.driver.set_route([(p.x, p.y) for p in msg.waypoints], list(msg.edge_end_idx),
                           list(msg.edge_ids), list(msg.crosswalk_idx), list(msg.junction_idx),
                           int(msg.goal_idx), int(msg.route_seq))
        r.route_seq = int(msg.route_seq)
        if not r.moved:
            # 사람이 로봇을 출발 노드에 경로 방향으로 놓는 것을 흉내낸다
            x0, y0 = msg.waypoints[0].x, msg.waypoints[0].y
            x1, y1 = msg.waypoints[1].x, msg.waypoints[1].y
            r.x, r.y, r.yaw = x0, y0, math.atan2(y1 - y0, x1 - x0)
        self.get_logger().info(f'{name}: Route seq={msg.route_seq} {" → ".join(msg.node_ids)}')

    def _on_lane_command(self, name, msg: LaneCommand):
        cmd = _CMD_MAP.get(msg.command)
        if cmd is None:
            return
        self._robots[name].driver.set_command(
            cmd, self._now(), route_seq=int(msg.route_seq), clear_until=int(msg.clear_until_idx),
            max_v=float(msg.max_linear_vel), max_w=float(msg.max_angular_vel))
        if cmd not in (CMD_HEARTBEAT, CMD_CLEARANCE):
            self.get_logger().info(f'{name}: LaneCommand {cmd}')

    def _on_lane_path(self, name, msg: LanePath):
        if self._loopback:
            return
        self._robots[name].driver.set_lane_path(
            self._now(), msg.source_stamp.sec + msg.source_stamp.nanosec * 1e-9,
            int(msg.quality), float(msg.error_x_norm), bool(msg.crosswalk_detected))

    def _on_fleet_command(self, name, msg: FleetCommand):
        r = self._robots[name]
        if msg.command == FleetCommand.CMD_SET_INITIAL_POSE:
            if not r.moved:
                r.x, r.y, r.yaw = msg.x, msg.y, msg.yaw
        elif msg.command in (FleetCommand.CMD_STOP, FleetCommand.CMD_CANCEL):
            r.driver.set_command(CMD_STOP, self._now())
        elif msg.command == FleetCommand.CMD_RESUME:
            r.driver.set_command(CMD_RESUME, self._now())
        elif msg.command == FleetCommand.CMD_HEARTBEAT:
            r.driver.set_command(CMD_HEARTBEAT, self._now())

    # ------------------------------------------------------------------ 시뮬

    def _synth_scan(self, r):
        circles = list(self._obstacles) + [(o.x, o.y, ROBOT_RADIUS)
                                           for o in self._robots.values() if o is not r]
        n = int(2 * SCAN_HALF_DEG / SCAN_STEP_DEG) + 1
        angle_min = -math.radians(SCAN_HALF_DEG)
        inc = math.radians(SCAN_STEP_DEG)
        ranges = []
        for i in range(n):
            a = r.yaw + angle_min + i * inc
            dx, dy = math.cos(a), math.sin(a)
            best = float('inf')
            for cx, cy, rad in circles:
                t = ray_circle(r.x, r.y, dx, dy, cx, cy, rad)
                if t is not None and t < best:
                    best = t
            ranges.append(best)
        return ranges, angle_min, inc

    def _tick(self):
        now = self._now()
        dt = 1.0 / self._rate
        for r in self._robots.values():
            if self._lidar:
                ranges, amin, inc = self._synth_scan(r)
                r.driver.update_scan(ranges, amin, inc, 0.05, 12.0)
            out = r.driver.tick(now, r.x, r.y, r.yaw)
            r.out = out
            r.v, r.w = out.v, out.omega
            if abs(r.v) > 1e-6 or abs(r.w) > 1e-6:
                r.moved = True
            r.yaw = wrap(r.yaw + r.w * dt)
            r.x += r.v * math.cos(r.yaw) * dt
            r.y += r.v * math.sin(r.yaw) * dt

    def _camera_tick(self):
        stamp = self.get_clock().now().to_msg()
        for r in self._robots.values():
            lateral, heading = r.lane_offsets()
            img = render_lane_frame(lateral=lateral, heading=heading,
                                    crosswalk_ahead=r.crosswalk_ahead(), camera=self._cam)
            if self._loopback:
                res = self._ests[r.name].update(self._det.infer(img), img.shape[1], img.shape[0])
                r.driver.set_lane_path(self._now(), self._now(), res.quality, res.error_x,
                                       res.crosswalk_detected)
                continue
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = f'{r.name}/camera'
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self._img_pubs[r.name].publish(msg)
            r.cam_seq += 1

    # ------------------------------------------------------------------ 발행

    def _publish_status(self):
        stamp = self.get_clock().now().to_msg()
        for r in self._robots.values():
            out = r.out
            st = LaneStatus()
            st.header.stamp = stamp
            st.header.frame_id = 'map'
            st.robot_name = r.name
            if out is not None:
                st.drive_state = int(out.state)
                st.state_reason = out.reason
                st.route_seq = int(r.driver.route_seq)
                st.route_idx = int(out.route_idx)
                st.edge_id = out.edge_id
                st.clear_until_idx = int(out.clear_until)
                st.error_x_norm = float(out.error_x)
                st.lane_quality = int(out.quality)
                st.linear_velocity = float(out.v)
                st.angular_velocity = float(out.omega)
                st.path_age = float(out.path_age) if math.isfinite(out.path_age) else -1.0
                st.lidar_min_range = float(out.lidar_min) if math.isfinite(out.lidar_min) else -1.0
                st.us_range = -1.0
                st.odom_since_state = float(out.odom_since_state)
            self._status_pubs[r.name].publish(st)

            rs = RobotState()
            rs.header.stamp = stamp
            rs.header.frame_id = 'map'
            rs.name = r.name
            rs.domain_id = int(r.spec['domain_id'])
            rs.localized = True
            rs.x, rs.y, rs.yaw = r.x, r.y, r.yaw
            rs.linear_velocity, rs.angular_velocity = r.v, r.w
            if out is None or out.state == LaneStatus.DRIVE_IDLE:
                rs.nav_status = RobotState.NAV_IDLE
            elif out.state == LaneStatus.DRIVE_ARRIVED:
                rs.nav_status = RobotState.NAV_SUCCEEDED
            elif out.state == LaneStatus.DRIVE_LINK_LOST:
                rs.nav_status = RobotState.NAV_LINK_LOST
            elif out.state in (LaneStatus.DRIVE_WAIT_CLEARANCE, LaneStatus.DRIVE_ESTOP):
                rs.nav_status = RobotState.NAV_HOLD
            else:
                rs.nav_status = RobotState.NAV_ACTIVE
            rs.max_linear_vel = r.driver.p.control.v_max
            rs.max_angular_vel = r.driver.p.control.omega_max
            rs.battery_percent = 100.0
            self._state_pubs[r.name].publish(rs)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FakeLaneFleet()
    except (LaneMissionError, FileNotFoundError) as exc:
        print(f'[fake_lane_robot] 시작 실패: {exc}')
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
