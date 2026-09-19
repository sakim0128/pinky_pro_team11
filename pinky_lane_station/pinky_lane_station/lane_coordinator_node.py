#!/usr/bin/env python3
"""차선 주행 관제 — 경로 배정 · 구간 예약 · 출발 순서.

lane_mission.yaml 의 start/goal 로 Route 를 만들어 로봇에 주고, 10 Hz 로 로봇 pose 를
예약기에 넣어 clear_until 을 LaneCommand(CMD_CLEARANCE) 로 보낸다. 이 메시지가 하트비트를
겸한다 — 관제가 죽으면 로봇의 link_watch 가 멈춘다.

GUI 와는 String(JSON) 두 토픽으로 대화한다.
    /fleet/lane/control  ← {"cmd": "assign", "robot": "pinky1", "start": "BL", "goal": "TC"}
                           {"cmd": "start"} {"cmd": "stop"} {"cmd": "resume"} {"cmd": "estop"}
                           {"cmd": "set_delay", "robot": "pinky2", "delay": 6.0} {"cmd": "reset"}
    /fleet/lane/status   → 로봇별 진행·예약·상태, 상호대기 경고
"""

import json
import signal

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from pinky_fleet_msgs.msg import FleetCommand, RobotState
from pinky_lane_msgs.msg import LaneCommand, LaneStatus, Route

from .lane_mission import LaneMissionError, load_lane_mission
from .reservation import Reservation
from .road_graph import RoadGraph, RoadGraphError

RELIABLE_10 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
ROUTE_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

MISSION_IDLE, MISSION_ASSIGNED, MISSION_RUNNING, MISSION_STOPPED, MISSION_DONE = (
    'IDLE', 'ASSIGNED', 'RUNNING', 'STOPPED', 'DONE')
START_RETRY_SECONDS = 1.0
START_RETRY_MAX = 5


class RobotCtx:
    def __init__(self, spec):
        self.spec = spec
        self.name = spec['name']
        self.start = spec['start']
        self.goal = spec['goal']
        self.depart_delay = float(spec['depart_delay'])
        self.route = None
        self.route_seq = 0
        self.state = None            # RobotState
        self.state_time = None
        self.lane_status = None      # LaneStatus
        self.depart_at = None
        self.start_sent = 0
        self.start_sent_at = None
        self.arrived = False


class LaneCoordinator(Node):

    def __init__(self):
        super().__init__('lane_coordinator')
        self.declare_parameter('mission', '')
        self.declare_parameter('control_topic', '/fleet/lane/control')
        self.declare_parameter('status_topic', '/fleet/lane/status')
        self.declare_parameter('auto_start', False)      # mission 의 coordinator.auto_start 와 OR

        path = self.get_parameter('mission').value
        if not path:
            raise LaneMissionError('mission 파라미터에 lane_mission.yaml 경로를 지정해야 합니다')
        self.mission = load_lane_mission(path)
        self.graph = RoadGraph.load(self.mission.graph_path)
        problems = self.mission.validate_against_graph(self.graph)
        for p in problems:
            self.get_logger().warn(f'미션 검증: {p}')
        res = self.mission.reservation
        self.reservation = Reservation(self.graph, res['reserve_ahead'], res['release_behind'],
                                       res['node_stop_margin'])
        cfg = self.mission.coordinator
        self._state_timeout = float(cfg['state_timeout'])
        self._set_initial_pose = bool(cfg['set_initial_pose'])

        self._robots = {}
        self._route_pubs, self._lane_cmd_pubs, self._fleet_cmd_pubs = {}, {}, {}
        for spec in self.mission.robots:
            ctx = RobotCtx(spec)
            self._robots[ctx.name] = ctx
            self._route_pubs[ctx.name] = self.create_publisher(Route, spec['route_topic'], ROUTE_QOS)
            self._lane_cmd_pubs[ctx.name] = self.create_publisher(
                LaneCommand, spec['lane_command_topic'], RELIABLE_10)
            self._fleet_cmd_pubs[ctx.name] = self.create_publisher(
                FleetCommand, spec['command_topic'], RELIABLE_10)
            self.create_subscription(RobotState, spec['state_topic'],
                                     lambda m, n=ctx.name: self._on_state(n, m), 10)
            self.create_subscription(LaneStatus, spec['lane_status_topic'],
                                     lambda m, n=ctx.name: self._on_lane_status(n, m), RELIABLE_10)
        self.create_subscription(String, self.get_parameter('control_topic').value,
                                 self._on_control, 10)
        self._status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)

        self._mission_state = MISSION_IDLE
        self._warning = ''
        self._seq = 0
        tick = float(cfg['tick_rate'])
        self.create_timer(1.0 / tick, self._tick)
        self.create_timer(0.5, self._publish_status)

        for ctx in self._robots.values():
            if ctx.start and ctx.goal:
                self.assign(ctx.name, ctx.start, ctx.goal)
        if bool(cfg['auto_start']) or bool(self.get_parameter('auto_start').value):
            self._auto_start_timer = self.create_timer(2.0, self._auto_start)
        self.get_logger().info(
            f'lane_coordinator 시작: graph={self.mission.graph_path} '
            f'robots={[(c.name, c.start, c.goal, c.depart_delay) for c in self._robots.values()]}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _auto_start(self):
        self._auto_start_timer.cancel()
        self.start()

    # ------------------------------------------------------------------ 입력

    def _on_state(self, name, msg: RobotState):
        ctx = self._robots[name]
        ctx.state, ctx.state_time = msg, self._now()

    def _on_lane_status(self, name, msg: LaneStatus):
        ctx = self._robots[name]
        ctx.lane_status = msg
        if (msg.drive_state == LaneStatus.DRIVE_ARRIVED and msg.route_seq == ctx.route_seq
                and ctx.route is not None and not ctx.arrived):
            ctx.arrived = True
            self.reservation.mark_arrived(name)
            self.get_logger().info(f'{name} 도착 ({ctx.goal})')
            if all(c.arrived for c in self._robots.values() if c.route is not None):
                self._mission_state = MISSION_DONE

    def _on_control(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f'control JSON 파싱 실패: {msg.data!r}')
            return
        kind = str(cmd.get('cmd', ''))
        try:
            if kind == 'assign':
                self.assign(cmd['robot'], cmd['start'], cmd['goal'])
            elif kind == 'set_delay':
                self._robots[cmd['robot']].depart_delay = float(cmd['delay'])
            elif kind == 'start':
                self.start()
            elif kind == 'stop':
                self.broadcast(LaneCommand.CMD_STOP)
                self._mission_state = MISSION_STOPPED
            elif kind == 'resume':
                self.broadcast(LaneCommand.CMD_RESUME)
                self.broadcast(LaneCommand.CMD_START)
                self._mission_state = MISSION_RUNNING
            elif kind == 'estop':
                self.broadcast(LaneCommand.CMD_ESTOP)
                self._mission_state = MISSION_STOPPED
            elif kind == 'reset':
                self.reset()
            else:
                self.get_logger().warn(f'알 수 없는 control cmd: {kind!r}')
        except (KeyError, ValueError, RoadGraphError) as exc:
            self._warning = f'{kind}: {exc}'
            self.get_logger().warn(self._warning)

    # ------------------------------------------------------------------ 동작

    def assign(self, name, start, goal):
        ctx = self._robots[name]
        route = self.graph.shortest_route(start, goal)
        self._seq += 1
        ctx.start, ctx.goal, ctx.route, ctx.route_seq = start, goal, route, self._seq
        ctx.arrived = False
        ctx.depart_at = None
        ctx.start_sent = 0
        self.reservation.register(name, ctx.spec['domain_id'], route)
        self._route_pubs[name].publish(self._route_msg(ctx))
        if self._set_initial_pose:
            fc = FleetCommand()
            fc.command = FleetCommand.CMD_SET_INITIAL_POSE
            fc.x, fc.y = self.graph.nodes[start].x, self.graph.nodes[start].y
            fc.yaw = self.graph.node_exit_yaw(start, route.edge_ids[0])
            self._fleet_cmd_pubs[name].publish(fc)
        if self._mission_state in (MISSION_IDLE, MISSION_DONE):
            self._mission_state = MISSION_ASSIGNED
        self.get_logger().info(
            f'{name}: {start} → {goal} ({route.length:.2f} m, {" ".join(route.edge_ids)}) seq={self._seq}')

    def _route_msg(self, ctx):
        r = ctx.route
        msg = Route()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.graph.frame
        msg.robot_name = ctx.name
        msg.route_seq = ctx.route_seq
        msg.waypoints = [Point(x=float(x), y=float(y), z=0.0) for x, y in r.waypoints]
        msg.edge_ids = list(r.edge_ids)
        msg.edge_forward = [bool(f) for f in r.edge_forward]
        msg.edge_end_idx = [int(i) for i in r.edge_end_idx]
        msg.node_ids = list(r.node_ids)
        msg.node_idx = [int(i) for i in r.node_idx]
        msg.crosswalk_idx = [int(i) for i in r.crosswalk_idx]
        msg.junction_idx = [int(i) for i in r.junction_idx]
        msg.goal_idx = int(r.goal_idx)
        msg.length = float(r.length)
        return msg

    def start(self):
        now = self._now()
        n = 0
        for ctx in self._robots.values():
            if ctx.route is None or ctx.arrived:
                continue
            ctx.depart_at = now + ctx.depart_delay
            ctx.start_sent = 0
            n += 1
        if n:
            self._mission_state = MISSION_RUNNING
            self.get_logger().info(
                '출발: ' + ', '.join(f'{c.name}(+{c.depart_delay:.0f}s)'
                                   for c in self._robots.values() if c.depart_at is not None))

    def reset(self):
        self.broadcast(LaneCommand.CMD_STOP)
        for ctx in self._robots.values():
            self.reservation.remove(ctx.name)
            ctx.route = None
            ctx.depart_at = None
            ctx.arrived = False
        self._mission_state = MISSION_IDLE
        for ctx in self._robots.values():
            if ctx.start and ctx.goal:
                self.assign(ctx.name, ctx.start, ctx.goal)

    def broadcast(self, command):
        for name in self._robots:
            self._send(name, command)

    def _send(self, name, command, clear_until=None):
        ctx = self._robots[name]
        msg = LaneCommand()
        msg.command = int(command)
        msg.route_seq = int(ctx.route_seq)
        if clear_until is not None:
            msg.clear_until_idx = int(clear_until)
        msg.max_linear_vel = float(ctx.spec['max_linear_vel'])
        msg.max_angular_vel = float(ctx.spec['max_angular_vel'])
        self._lane_cmd_pubs[name].publish(msg)

    # ------------------------------------------------------------------ 루프

    def _tick(self):
        now = self._now()
        for ctx in self._robots.values():
            if ctx.route is None:
                self._send(ctx.name, LaneCommand.CMD_HEARTBEAT)
                continue
            st = ctx.state
            fresh = st is not None and ctx.state_time is not None \
                and (now - ctx.state_time) <= self._state_timeout
            if fresh and st.localized:
                self.reservation.update_pose(ctx.name, st.x, st.y)
        clear = self.reservation.step()
        for ctx in self._robots.values():
            if ctx.route is None:
                continue
            self._send(ctx.name, LaneCommand.CMD_CLEARANCE, clear_until=clear.get(ctx.name, 0))
            # 출발 예약
            if ctx.depart_at is not None and now >= ctx.depart_at and ctx.start_sent < START_RETRY_MAX:
                ls = ctx.lane_status
                idle = ls is None or ls.drive_state in (LaneStatus.DRIVE_IDLE,)
                if ctx.start_sent == 0 or (idle and now - ctx.start_sent_at >= START_RETRY_SECONDS):
                    self._send(ctx.name, LaneCommand.CMD_START)
                    ctx.start_sent += 1
                    ctx.start_sent_at = now
                    if ctx.start_sent == 1:
                        self.get_logger().info(f'{ctx.name} CMD_START')
                elif not idle:
                    ctx.start_sent = START_RETRY_MAX     # 움직이기 시작했다 — 그만 보낸다
        pairs = self.reservation.mutual_wait()
        self._warning = ('상호 대기 교착: ' + ', '.join(f'{a}↔{b}' for a, b in pairs)
                         + ' — 한 대를 수동으로 물려야 합니다') if pairs else ''

    def _publish_status(self):
        now = self._now()
        robots = {}
        for ctx in self._robots.values():
            info = {'start': ctx.start, 'goal': ctx.goal, 'route_seq': ctx.route_seq,
                    'edge_ids': list(ctx.route.edge_ids) if ctx.route else [],
                    'length': round(ctx.route.length, 2) if ctx.route else None,
                    'depart_in': (None if ctx.depart_at is None
                                  else round(max(0.0, ctx.depart_at - now), 1)),
                    'arrived': ctx.arrived}
            if ctx.route is not None:
                info.update(self.reservation.status(ctx.name))
            ls = ctx.lane_status
            if ls is not None:
                info.update({'drive_state': int(ls.drive_state), 'reason': ls.state_reason,
                             'route_idx': int(ls.route_idx), 'edge_id': ls.edge_id,
                             'error_x': round(float(ls.error_x_norm), 3),
                             'lane_quality': int(ls.lane_quality),
                             'path_age': round(float(ls.path_age), 3)})
            st = ctx.state
            if st is not None:
                info.update({'x': round(st.x, 3), 'y': round(st.y, 3), 'yaw': round(st.yaw, 3),
                             'localized': bool(st.localized)})
            robots[ctx.name] = info
        msg = String()
        msg.data = json.dumps({'mission': self._mission_state, 'warning': self._warning,
                               'robots': robots}, ensure_ascii=False)
        self._status_pub.publish(msg)

    def shutdown_robots(self):
        self.broadcast(LaneCommand.CMD_STOP)
        self.get_logger().info('종료: 로봇에 STOP 을 보냈습니다 (데드맨이 최종 보장)')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = LaneCoordinator()
    except (LaneMissionError, RoadGraphError, FileNotFoundError) as exc:
        print(f'[lane_coordinator] 시작 실패: {exc}')
        rclpy.shutdown()
        return
    stopping = []

    def _request_stop(_signum, _frame):
        stopping.append(True)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
        if rclpy.ok():
            node.shutdown_robots()
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
