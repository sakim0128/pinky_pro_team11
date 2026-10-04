"""차선 주행 오케스트레이터 — 실차 노드와 가짜 로봇이 같은 코드를 쓴다.

ROS 에 의존하지 않는다. 노드는 메시지를 풀어 ``set_*`` 로 넣고, 20 Hz 로 ``tick`` 을
불러 (v, ω) 와 상태를 받아 발행한다. 시각은 전부 **로봇 시계** 의 float 초.

    d = LaneDriver()
    d.set_route(waypoints, edge_end_idx, edge_ids, crosswalk_idx, junction_idx, goal_idx, route_seq)
    d.set_clearance(route_seq, idx, now)          # LaneCommand CMD_CLEARANCE (하트비트 겸용)
    d.set_lane_path(now, source_stamp, quality, error_x, crosswalk, ...)
    d.update_scan(...) / d.update_us(...)
    out = d.tick(now, x, y, yaw)
"""

import math
from dataclasses import dataclass, field

from dataclasses import replace

from .drive_fsm import IDLE, JUNCTION_PASS, LANE_SEARCH, STATE_NAMES, DriveFsm, FsmParams, Inputs
from .lane_control import (QUALITY_BOTH, QUALITY_JUNCTION, QUALITY_LOST, QUALITY_SINGLE,
                           QUALITY_STALE, ControlParams, LaneController)
from .lane_memory import GroundView, LaneMemory, MemoryParams, OdomBuffer, ViewParams
from .link_watch import LinkWatch
from .maneuver import ManeuverExecutor, ManeuverParams
from .obstacle_guard import GuardParams, ObstacleGuard
from .route_follower import RouteFollower

CMD_HEARTBEAT, CMD_START, CMD_STOP, CMD_ESTOP, CMD_RESUME, CMD_SET_SPEED, CMD_CLEARANCE = range(7)


@dataclass
class DriverParams:
    control: ControlParams = field(default_factory=ControlParams)
    fsm: FsmParams = field(default_factory=FsmParams)
    guard: GuardParams = field(default_factory=GuardParams)
    maneuver: ManeuverParams = field(default_factory=ManeuverParams)
    view: ViewParams = field(default_factory=ViewParams)
    memory: MemoryParams = field(default_factory=MemoryParams)
    lookahead: float = 0.25
    arrive_tolerance: float = 0.10
    clearance_tolerance: float = 0.05     # clear_until 에 이 거리 안이면 "닿았다"
    link_timeout: float = 3.0             # 관제 명령 침묵 허용
    path_timeout: float = 0.9             # LanePath 침묵 허용 (0.3 s × 3)
    path_max_age: float = 0.9             # 이보다 오래된 error_x 는 STALE 취급
    crosswalk_zone: float = 0.45          # 그래프 횡단보도 노드 ± 이 거리 밖의 트리거는 무시
    junction_zone: float = 0.25           # 분기 노드 ± 이 거리는 JUNCTION 취급 (관제가 안 보내도)
    red_line_zone: float = 0.60          # 빨간 선 검출은 경로상 다음 분기 노드가 이 거리 안에 있을 때만 교차로 트리거 (오검출 방어)
    junction_wait_clearance: bool = True  # JUNCTION_STOP 에서 관제 허가(clear_until > 분기 idx) 를 기다린다
    lane_only: bool = False               # 경로·위치 없이 카메라 차선 중앙만 따라간다 (테스트 모드)
    lane_lost_coast: float = 0.6          # lane_only: 차선을 잃고 이만큼(s) 직전 명령 유지 후 정지
    search_omega: float = 0.4             # LANE_SEARCH 제자리 회전 각속도 (rad/s)
    odom_timeout: float = 0.5             # 비전 미션 고정 동작: odom 이 이만큼 끊기면 멈춰 기다린다
    stop_line_min_gap: float = 0.15       # 비전 미션: 흰 정지선을 연달아 셀 때 최소 주행거리 (m) — 한 선을 두 번 세지 않게
    arrive_distance: float = 0.15         # 비전 미션: 도착 지점 벽 ArUco 마커까지 이 거리(m) 이하면 도착 (JunctionPlan 이 주면 그 값)
    marker_slow_distance: float = 0.35    # 비전 미션: 목적지 마커가 이 거리 안이면 감속 (지나치지 않게)
    marker_slow_factor: float = 0.5
    marker_obstacle_distance: float = 0.35  # 목적지 마커가 이 거리 안에 보이는데 장애물 정지면 도착으로 친다 (벽·앞 물체)
    red_line_min_gap: float = 0.05        # lane_only: 빨간 선 정지 뒤 이만큼(m) 달린 다음의 새 선만 다시 세운다
    follow_memory: bool = True            # lane_only: 본 차선 중앙을 odom 에 기억했다가 그 자리에서 따라간다 (카메라 사각지대 보정).
                                          # False = 예전 방식(가장 최근 error_x 로 바로 조향)


@dataclass
class DriverOutput:
    v: float = 0.0
    omega: float = 0.0
    state: int = IDLE
    state_name: str = 'IDLE'
    reason: str = ''
    route_idx: int = 0
    edge_id: str = ''
    clear_until: int = 0
    error_x: float = 0.0
    quality: int = QUALITY_LOST
    path_age: float = float('nan')
    lidar_min: float = float('inf')
    us_range: float = float('nan')
    odom_since_state: float = 0.0
    lateral: float = 0.0


class LaneDriver:
    def __init__(self, params=None):
        self.p = params or DriverParams()
        self.follower = None
        self.route_seq = 0
        self.edge_ids = []
        self.edge_end_idx = []
        self.crosswalk_idx = []
        self.junction_idx = []
        self.goal_idx = 0
        self.clear_until = 0
        self.clearance_reason = ''
        self.started = False
        self.estop = False
        self.controller = LaneController(self.p.control)
        self.fsm = DriveFsm(self.p.fsm)
        self.guard = ObstacleGuard(self.p.guard)
        self.station_link = LinkWatch(self.p.link_timeout)
        self.path_link = LinkWatch(self.p.path_timeout, restore_grace=0.0)
        # 최근 LanePath
        self._lane = {'stamp': None, 'quality': QUALITY_LOST, 'error_x': None, 'crosswalk': False,
                      'barricade': False, 'red_line': False, 'stop_line': False,
                      'left_seen': False, 'right_seen': False, 'markers': {}, 'red_obs': []}
        # 비전 미션 (lane_only + JunctionPlan)
        self._plan = None                  # {'seq', 'steps', 'stop_line_count', 'linear', 'angular', 'goal_marker_id', ...}
        self._maneuver = None
        self._maneuver_finished_pending = False
        self._junction_passed = False
        self._stop_lines_seen = 0
        self._stop_line_prev = False
        self._last_count_travel = None
        self._arrived = False
        self._arrived_reason = ''
        self._pending_search_dir = 0.0
        self._odom = None                  # (x, y, yaw) odom frame
        # 빨간 선 (lane_only): 검출 상승 에지 → RED_LINE_STOP
        self._red_prev = False
        self._red_pending = False
        self._red_stop_travel = None
        self._odom_time = None
        self._search_dir = 0.0
        self._last_valid_error = None      # 마지막 BOTH/SINGLE 의 error_x (LOST 탐색 방향용)
        self._travelled = 0.0
        self._last_xy = None
        self._last_tick = None
        self._last_cmd = (0.0, 0.0)
        self._lost_since = None
        self.view = GroundView(self.p.view)
        self.memory = LaneMemory(self.p.memory)
        self.odom_buf = OdomBuffer()

    # ------------------------------------------------ 입력

    def set_route(self, waypoints, edge_end_idx, edge_ids, crosswalk_idx, junction_idx,
                  goal_idx, route_seq):
        self.follower = RouteFollower(waypoints, goal_idx, lookahead=self.p.lookahead)
        self.edge_end_idx = list(edge_end_idx)
        self.edge_ids = list(edge_ids)
        self.crosswalk_idx = list(crosswalk_idx)
        self.junction_idx = list(junction_idx)
        self.goal_idx = int(goal_idx)
        self.route_seq = int(route_seq)
        self.clear_until = 0
        self.controller.reset()
        self.fsm = DriveFsm(self.p.fsm)

    def set_command(self, cmd, now, route_seq=None, clear_until=None, max_v=None, max_w=None):
        """LaneCommand. 하트비트·clearance 는 link_watch 를 무장시킨다."""
        self.station_link.on_command(now, heartbeat=cmd in (CMD_HEARTBEAT, CMD_CLEARANCE))
        if cmd == CMD_CLEARANCE:
            if route_seq is None or route_seq == self.route_seq:
                self.clear_until = int(clear_until or 0)
        elif cmd == CMD_START:
            if self.station_link.accepts_motion_command(now):
                if not self.started:
                    self.memory.clear()
                self.started = True
        elif cmd == CMD_STOP:
            self.started = False
        elif cmd == CMD_ESTOP:
            self.estop = True
            self.started = False
        elif cmd == CMD_RESUME:
            self.estop = False
        elif cmd == CMD_SET_SPEED:
            if max_v is not None and max_v > 0:
                self.p.control.v_max = float(max_v)
            if max_w is not None and max_w > 0:
                self.p.control.omega_max = float(max_w)

    def set_clearance(self, route_seq, idx, now, reason=''):
        self.clearance_reason = reason
        self.set_command(CMD_CLEARANCE, now, route_seq=route_seq, clear_until=idx)

    def set_plan(self, seq, steps, stop_line_count, linear_speed=0.0, angular_speed=0.0, goal_marker_id=-1,
                 arrive_distance=0.0, skip_clearance=False, arrive_on_obstacle=False):
        """비전 미션 계획 (JunctionPlan). seq 가 바뀌면 새 미션(교차로·도착 진행을 처음부터), 같으면 값만 갱신.

        goal_marker_id >= 0: 교차로 뒤 도착 지점 벽 ArUco 마커(그 id)까지 arrive_distance 이하면 도착.
        goal_marker_id < 0 : 흰 정지선 stop_line_count 번째에서 도착 (예전 방식).
        skip_clearance     : 교차로에서 관제 허가 없이 정지 시간만 채우고 출발 (로봇 1대 시나리오).
        arrive_on_obstacle : 교차로 뒤 장애물 정지 = 도착 (같은 목적지에 앞 로봇이 먼저 도착했다).
        """
        seq = int(seq)
        if self._plan is None or seq != self._plan['seq']:
            self.route_seq = seq
            self.clear_until = 0
            self._maneuver = None
            self._maneuver_finished_pending = False
            self._junction_passed = False
            self._stop_lines_seen = 0
            self._stop_line_prev = False
            self._last_count_travel = None
            self._arrived = False
            self._arrived_reason = ''
            self._pending_search_dir = 0.0
            self._red_pending = False
            self._red_stop_travel = None
            self.memory.clear()
            self.fsm = DriveFsm(self.p.fsm)
        self._plan = {'seq': seq, 'steps': list(steps), 'stop_line_count': int(stop_line_count),
                      'linear': float(linear_speed), 'angular': float(angular_speed),
                      'goal_marker_id': int(goal_marker_id),
                      'arrive_distance': float(arrive_distance) if arrive_distance and arrive_distance > 0
                      else self.p.arrive_distance,
                      'skip_clearance': bool(skip_clearance), 'arrive_on_obstacle': bool(arrive_on_obstacle)}

    def update_odom(self, now, x, y, yaw, stamp=None):
        """stamp: odom 메시지 header.stamp (로봇 시계). 사진 stamp 의 자세를 보간하는 데 쓴다. 없으면 now."""
        self._odom = (float(x), float(y), float(yaw))
        self._odom_time = float(now)
        self.odom_buf.add(now if stamp is None else stamp, x, y, yaw)

    @property
    def mission_stage(self):
        """비전 미션 진행 단계 — LaneStatus.edge_id 로 관제에 보낸다."""
        if self._plan is None:
            return 'lane_only'
        if self._arrived:
            return 'vision:arrived'
        if self._junction_passed:
            return 'vision:after_junction'
        if self._maneuver is not None:
            return 'vision:junction'
        return 'vision:approach'

    def set_lane_path(self, now, source_stamp, quality, error_x, crosswalk=False,
                      barricade=False, left_seen=None, right_seen=None, red_line=False, stop_line=False,
                      markers=None, red_obs=None, target=None, floor=None):
        """markers: {ArUco id: 카메라~마커 거리 m} (LanePath.marker_ids / marker_distances).
        red_obs: 빨간 덩어리 [(x_norm, width_frac, bottom_frac), ...] — 교차로 seek 동작용 (LanePath.red_line_xs/ys/widths).
        target: (target_x, target_y, image_width, image_height, half_lane_px) — 차선 중앙 기억(follow_memory)용.
        floor: (전방 m, 왼쪽 m) — 관제 BEV 가 바닥 좌표로 잰 차로 중앙 목표점 (원점 = 카메라 바로 아래 바닥). 있으면 target 대신 쓴다."""
        self.path_link.on_command(now, heartbeat=True)
        q = int(quality)
        if left_seen is None:            # 옛 호출자: quality 로 추정
            left_seen = q == QUALITY_BOTH
        if right_seen is None:
            right_seen = q == QUALITY_BOTH
        self._lane = {'stamp': float(source_stamp), 'quality': q,
                      'error_x': None if error_x is None else float(error_x),
                      'crosswalk': bool(crosswalk), 'barricade': bool(barricade),
                      'red_line': bool(red_line), 'stop_line': bool(stop_line),
                      'left_seen': bool(left_seen), 'right_seen': bool(right_seen),
                      'markers': {int(k): float(v) for k, v in dict(markers or {}).items()},
                      'red_obs': [(float(b[0]), float(b[1]), float(b[2])) for b in (red_obs or [])],
                      'red_stamp': float(source_stamp) if q != QUALITY_STALE else None}
        if q in (QUALITY_BOTH, QUALITY_SINGLE) and error_x is not None:
            self._last_valid_error = float(error_x)
            self._remember(source_stamp, target, floor)

    def _remember(self, source_stamp, target, floor=None):
        """차선 중앙점을 사진을 찍은 순간의 odom 자세로 기억한다 (교차로 동작 중에는 안 쌓는다).

        관제 BEV 의 바닥 좌표(floor)가 있으면 그것에 구동축 오프셋만 더한다 (캘리브레이션 기반).
        없으면 예전처럼 화면 목표 픽셀을 GroundView 두 점 근사로 바닥에 옮긴다.
        """
        if not (self.p.lane_only and self.p.follow_memory) or (target is None and floor is None):
            return
        if self._maneuver is not None and not self._maneuver.done:
            return
        if floor is not None:
            pt = (float(floor[0]) + self.p.view.axle_to_camera_m, float(floor[1]))
        else:
            pt = self.view.to_robot(*target)
        pose = self.odom_buf.at(source_stamp) if pt is not None else None
        if pose is not None:
            self.memory.add(pt, pose)

    def _memory_curvature(self, now):
        """기억한 차선 중앙을 따라갈 곡률. 끔·odom 끊김·따라갈 점 없음이면 None."""
        p = self.p
        if not p.follow_memory or self._odom_time is None or now - self._odom_time > p.odom_timeout:
            return None
        return self.memory.curvature(self._odom)

    def update_scan(self, ranges, angle_min, angle_increment, range_min=0.05, range_max=12.0):
        self.guard.update_scan(ranges, angle_min, angle_increment, range_min, range_max)

    def update_us(self, rng):
        self.guard.update_us(rng)

    # ------------------------------------------------ 보조

    def _current_edge(self, idx):
        for eid, end in zip(self.edge_ids, self.edge_end_idx):
            if idx <= end:
                return eid
        return self.edge_ids[-1] if self.edge_ids else ''

    def _near_any(self, idx_list, tolerance):
        if not self.follower:
            return False
        s = self.follower.progress_s
        return any(abs(self.follower.cum[i] - s) <= tolerance for i in idx_list
                   if 0 <= i < len(self.follower.cum))

    def _next_junction_idx(self, ahead):
        """진행도 기준 (뒤로 junction_zone 까지 포함해) 가장 가까운 앞쪽 분기 노드의 waypoint idx. 거리 ahead 밖이면 None."""
        if not self.follower or not self.junction_idx:
            return None
        s = self.follower.progress_s
        best = None
        for i in self.junction_idx:
            if not (0 <= i < len(self.follower.cum)):
                continue
            gap = self.follower.cum[i] - s
            if -self.p.junction_zone <= gap <= ahead and (best is None or gap < best[0]):
                best = (gap, i)
        return None if best is None else best[1]

    def _search_direction(self, x, y, yaw):
        """LANE_SEARCH 회전 방향 (+1 좌회전 / −1 우회전).

        경로가 있으면 다음 lookahead 점이 있는 쪽. 없으면 안 보이는 차선 쪽
        (왼쪽만 보이면 도로는 오른쪽 → 우회전). 둘 다 없으면 마지막 error_x 부호, 그것도 없으면
        직전 방향을 유지한다 (탐색 중 방향이 뒤집히지 않게).
        """
        cam_sign = 1.0 if self.p.control.cam_sign >= 0 else -1.0
        if self.follower is not None:
            px, py = self.follower.lookahead_point()
            ang = math.atan2(py - y, px - x) - yaw
            ang = math.atan2(math.sin(ang), math.cos(ang))
            if abs(ang) > 1e-3:
                return 1.0 if ang > 0 else -1.0
        left, right = self._lane['left_seen'], self._lane['right_seen']
        if left != right:
            return (-1.0 if left else 1.0) * cam_sign
        e = self._last_valid_error
        if e is not None and abs(e) > 1e-3:
            return (-1.0 if e > 0 else 1.0) * cam_sign
        return self._search_dir if self._search_dir else cam_sign

    def _apply_search(self, out, x, y, yaw):
        """FSM 이 LANE_SEARCH 이면 v=0, ω=±search_omega (실패면 0). 처리했으면 True."""
        if out.state != LANE_SEARCH:
            self._search_dir = 0.0
            return False
        self.memory.clear()                       # 제자리 회전 — 이전 방향으로 본 점은 버린다
        self.controller.reset()
        self._last_cmd = (0.0, 0.0)
        self._lost_since = None
        if self.fsm.search_failed:
            out.v, out.omega = 0.0, 0.0
            return True
        if not self._search_dir:
            # 교차로 동작 직후면 방금 돈 쪽으로 (차선은 그쪽에 있다), 아니면 보이는 차선·경로로 고른다
            self._search_dir = self._pending_search_dir or self._search_direction(x, y, yaw)
            self._pending_search_dir = 0.0
        out.v, out.omega = 0.0, self._search_dir * self.p.search_omega
        return True

    # ------------------------------------------------ 틱

    def tick(self, now, x, y, yaw):
        p = self.p
        out = DriverOutput()
        dt = 0.05 if self._last_tick is None else max(1e-3, now - self._last_tick)
        self._last_tick = now
        if self._last_xy is not None:
            self._travelled += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)

        self.station_link.poll(now)
        self.path_link.poll(now)
        blocked, obstacle_reason = self.guard.step(now)
        out.lidar_min = self.guard.lidar_min
        out.us_range = self.guard.us_range

        # LanePath 신선도 → quality
        quality = self._lane['quality']
        error_x = self._lane['error_x']
        if self._lane['stamp'] is not None:
            age = now - self._lane['stamp']
            out.path_age = age
            if age < 0 or age > p.path_max_age:
                quality = QUALITY_STALE
        else:
            quality = QUALITY_LOST
        in_junction = bool(self.follower) and self._near_any(self.junction_idx, p.junction_zone)
        if in_junction:
            quality = QUALITY_JUNCTION
        # 빨간 선(카메라) → 교차로 트리거. 경로 모드는 다음 분기 노드가 red_line_zone 안일 때만 (다른 흰 선 오검출 방어)
        next_j = self._next_junction_idx(p.red_line_zone)
        red_line_trigger = bool(self._lane['red_line']) and next_j is not None
        # 관제 허가: clear_until 이 분기 노드 idx 를 넘어야 통과. 분기가 없거나 goal 까지 허가면 True
        junction_clear = (not p.junction_wait_clearance or next_j is None
                          or self.clear_until > next_j or self.clear_until >= self.goal_idx)
        lane_visible = quality in (QUALITY_BOTH, QUALITY_SINGLE, QUALITY_JUNCTION)
        lane_both = quality in (QUALITY_BOTH, QUALITY_JUNCTION)
        out.quality = quality
        out.error_x = error_x if error_x is not None else 0.0

        if self.follower is None and p.lane_only:
            return self._tick_lane_only(now, dt, out, blocked, obstacle_reason, quality, error_x,
                                        lane_visible, lane_both)
        if self.follower is None:
            inp = Inputs(started=False, estop=self.estop, link_ok=self.station_link.alive(now),
                         path_ok=True, obstacle=blocked, obstacle_reason=obstacle_reason,
                         travelled=self._travelled)
            state, _, reason = self.fsm.step(now, inp)
            out.state, out.state_name, out.reason = state, STATE_NAMES[state], reason or '경로 없음'
            return out

        f = self.follower
        f.update(x, y)
        out.route_idx = f.progress_idx
        out.edge_id = self._current_edge(f.progress_idx)
        out.clear_until = self.clear_until
        out.lateral = f.lateral

        limit_idx = min(self.clear_until, self.goal_idx)
        dist_to_limit = f.distance_to_idx(limit_idx)
        at_clearance = (self.clear_until < self.goal_idx
                        and dist_to_limit <= p.clearance_tolerance)
        arrived = f.remaining() <= p.arrive_tolerance and self.clear_until >= self.goal_idx
        crosswalk_trigger = (self._lane['crosswalk']
                             and self._near_any(self.crosswalk_idx, p.crosswalk_zone))

        inp = Inputs(
            started=self.started, estop=self.estop,
            link_ok=self.station_link.alive(now),
            path_ok=self.path_link.alive(now),
            obstacle=blocked, obstacle_reason=obstacle_reason,
            at_clearance=at_clearance, clearance_reason=self.clearance_reason,
            crosswalk_trigger=crosswalk_trigger, barricade=self._lane['barricade'],
            junction_trigger=in_junction or red_line_trigger, junction_clear=junction_clear,
            lane_visible=lane_visible, lane_both=lane_both,
            arrived=arrived, travelled=self._travelled,
        )
        state, speed_factor, reason = self.fsm.step(now, inp)
        out.state, out.state_name, out.reason = state, STATE_NAMES[state], reason
        out.odom_since_state = self.fsm.travelled_since_entry(self._travelled)

        if self._apply_search(out, x, y, yaw):
            return out
        if speed_factor <= 0.0:
            self.controller.reset()
            out.v, out.omega = 0.0, 0.0
            return out

        v_prev = max(self.controller._v_cmd, 0.05)
        omega_route, turn_in_place = f.steer(x, y, yaw, v_prev, limit_idx=limit_idx,
                                             omega_max=p.control.omega_max)
        out.v, out.omega = self.controller.command(
            omega_route, error_x, quality, dt, dist_to_limit, speed_factor, turn_in_place,
            error_stamp=self._lane['stamp'])
        return out

    # ------------------------------------------------ 차선만 (테스트 모드)

    def _count_stop_lines(self):
        """비전 미션: 교차로를 지난 뒤 흰 정지선을 올라가는 모서리마다 센다. 목표 개수면 도착."""
        seen = bool(self._lane['stop_line'])
        armed = (self._plan is not None and self._junction_passed and not self._arrived
                 and self._plan['stop_line_count'] > 0)
        if armed and seen and not self._stop_line_prev:
            if (self._last_count_travel is None
                    or self._travelled - self._last_count_travel >= self.p.stop_line_min_gap):
                self._stop_lines_seen += 1
                self._last_count_travel = self._travelled
                if self._stop_lines_seen >= self._plan['stop_line_count']:
                    self._arrived = True
        self._stop_line_prev = seen

    def _goal_marker_distance(self, quality):
        """목적지 마커까지 거리 (m). 계획이 마커 방식이 아니거나 안 보이면(낡은 LanePath 포함) None."""
        if self._plan is None or self._plan['goal_marker_id'] < 0 or quality == QUALITY_STALE:
            return None
        return self._lane['markers'].get(self._plan['goal_marker_id'])

    def _check_arrival(self, blocked, quality):
        """비전 미션 도착 판정.

        목적지 벽 마커(goal_marker_id) 는 교차로 통과와 **무관하게** 본다 (2026-09-30 사용자 결정 — 빨간 선을 놓쳐 교차로 절차가
        안 걸려도 목적지 앞에서는 선다). 다른 id 는 무시한다. 흰 정지선 세기와 "앞 로봇 뒤 정지 = 도착" 은 교차로를 지난 뒤에만.
        """
        plan = self._plan
        if plan is None or self._arrived:
            return
        if plan['goal_marker_id'] < 0:
            self._count_stop_lines()                  # 교차로 앞에서도 불러 선의 이전 상태를 이어 둔다 (세기는 통과 뒤만)
            if self._arrived:
                self._arrived_reason = f"도착 — 정지선 {self._stop_lines_seen}번째"
        if self._arrived:
            return
        if plan['goal_marker_id'] >= 0:
            dist = self._goal_marker_distance(quality)
            if dist is not None and dist <= plan['arrive_distance']:
                self._arrived = True
                self._arrived_reason = f"도착 — 벽 마커 {plan['goal_marker_id']} {dist:.2f} m"
            elif blocked and dist is not None and dist <= self.p.marker_obstacle_distance:
                self._arrived = True
                self._arrived_reason = f"도착 — 벽 마커 {plan['goal_marker_id']} {dist:.2f} m 앞 장애물 정지"
        if not self._junction_passed or self._arrived:
            return
        if blocked and plan['arrive_on_obstacle']:
            self._arrived = True
            self._arrived_reason = '도착 — 같은 목적지 앞 로봇 뒤 정지'

    def _run_maneuver(self, now, dt, out):
        """허가 뒤 고정 동작 한 틱. 끝나면 다음 틱에 FSM 이 차선 주행(또는 탐색)으로 넘긴다."""
        p = self.p
        self.memory.clear()                       # 교차로 동작 중·직후엔 다른 가지의 점을 따라가지 않게
        if self._maneuver is None:
            mp = replace(p.maneuver)
            if self._plan['linear'] > 0:
                mp.linear_speed = self._plan['linear']
            if self._plan['angular'] > 0:
                mp.angular_speed = self._plan['angular']
            self._maneuver = ManeuverExecutor(self._plan['steps'], mp)
        odom = (self._odom if self._odom_time is not None and now - self._odom_time <= p.odom_timeout
                else None)
        v, w, done, why = self._maneuver.command(now, odom, self._lane['red_obs'], self._lane.get('red_stamp'))
        if done:
            self._junction_passed = True
            self._maneuver_finished_pending = True
            self._pending_search_dir = self._maneuver.last_turn_sign
            v = w = 0.0
        self.controller.reset()
        self._last_cmd = (0.0, 0.0)
        self._lost_since = None
        out.v, out.omega, out.reason = v, w, why
        self._travelled += abs(v) * dt
        return out

    def _red_line_event(self, plan, maneuver_active, maneuver_finished):
        """빨간 선 검출의 상승 에지 → RED_LINE_STOP 이벤트 (lane_only).

        계획 없음: 모든 빨간 선. 계획 있음: 입구 선은 JUNCTION_STOP 이 맡으므로 교차로를 지난 뒤(또는 고정 동작 중) 선만.
        고정 동작 중 에지는 보류했다가 동작이 끝난 다음 틱에 낸다. 직전 정지 뒤 red_line_min_gap 을 달려야 새 선으로 본다.
        """
        seen = bool(self._lane['red_line'])
        edge = seen and not self._red_prev
        self._red_prev = seen
        counts = plan is None or self._junction_passed or maneuver_active
        if edge and counts:
            self._red_pending = True
        if not self._red_pending or maneuver_active or maneuver_finished:
            return False
        if (self._red_stop_travel is not None
                and self._travelled - self._red_stop_travel < self.p.red_line_min_gap):
            self._red_pending = False              # 방금 선 그 선의 재검출 — 무시
            return False
        self._red_pending = False
        self._red_stop_travel = self._travelled
        return True

    def _tick_lane_only(self, now, dt, out, blocked, obstacle_reason, quality, error_x,
                        lane_visible, lane_both):
        """경로·위치 없이 카메라 error_x 만으로 중앙 주행. 횡단보도·바리게이트·장애물·링크 감시는 그대로.

        JunctionPlan(비전 미션) 이 있으면: 빨간 선에서 서고 관제 허가(clear_until ≥ 1) 뒤 고정 동작, 교차로를 지난 뒤
        흰 정지선을 stop_line_count 번째 만나면 도착. 없으면 예전처럼 빨간 선에서 잠깐 서고 차선으로 통과한다.
        """
        p = self.p
        plan = self._plan
        if plan is not None:
            self._check_arrival(blocked, quality)
            junction_trigger = (bool(self._lane['red_line']) and not self._junction_passed
                                and self._maneuver is None)
            junction_clear = plan['skip_clearance'] or self.clear_until >= 1
            maneuver_active = self._maneuver is not None and not self._maneuver.done
        else:
            # 계획 없음(테스트 주행): 빨간 선은 교차로 절차 없이 RED_LINE_STOP 으로만 선다
            junction_trigger, junction_clear, maneuver_active = False, True, False
        finished, self._maneuver_finished_pending = self._maneuver_finished_pending, False
        red_line_event = self._red_line_event(plan, maneuver_active, finished)
        # 카메라 사각지대 보정: 본 차선 중앙을 기억해 두었다면, 지금 차선이 안 보여도 그 점까지는 간다
        kappa = self._memory_curvature(now)
        inp = Inputs(
            started=self.started, estop=self.estop,
            link_ok=self.station_link.alive(now), path_ok=self.path_link.alive(now),
            obstacle=blocked, obstacle_reason=obstacle_reason,
            at_clearance=False, crosswalk_trigger=self._lane['crosswalk'],   # 존 검사 없음 (그래프가 없다)
            barricade=self._lane['barricade'],
            # 빨간 선 → 교차로 정지. 계획이 없으면 허가 없이 정지 시간만 채우고 차선으로 통과한다
            junction_trigger=junction_trigger, junction_clear=junction_clear,
            red_line_event=red_line_event,
            maneuver_active=maneuver_active, maneuver_finished=finished,
            maneuver_reason=self._maneuver.describe() if maneuver_active else '',
            lane_visible=lane_visible or kappa is not None, lane_both=lane_both, arrived=self._arrived,
            arrived_reason=self._arrived_reason, travelled=self._travelled,
        )
        state, speed_factor, reason = self.fsm.step(now, inp)
        out.state, out.state_name, out.reason = state, STATE_NAMES[state], reason
        out.odom_since_state = self.fsm.travelled_since_entry(self._travelled)
        out.edge_id = self.mission_stage
        out.clear_until = self.clear_until
        out.route_idx = self._stop_lines_seen
        if self._apply_search(out, 0.0, 0.0, 0.0):
            return out
        if plan is not None and state == JUNCTION_PASS and not self._junction_passed:
            return self._run_maneuver(now, dt, out)
        if speed_factor <= 0.0:
            self.controller.reset()
            self._last_cmd = (0.0, 0.0)
            self._lost_since = None
            out.v, out.omega = 0.0, 0.0
            return out
        if not lane_visible and kappa is None:
            # 맵이 없으니 서행 계속은 불가 — 잠깐 직전 명령을 유지하고 선다
            if self._lost_since is None:
                self._lost_since = now
            if now - self._lost_since <= p.lane_lost_coast:
                out.v, out.omega = self._last_cmd
                out.reason = f'차선 없음 — {now - self._lost_since:.1f}/{p.lane_lost_coast:.1f}s 유지'
                self._travelled += abs(out.v) * dt
            else:
                self.controller.reset()
                self._last_cmd = (0.0, 0.0)
                out.v, out.omega = 0.0, 0.0
                out.reason = '차선 없음 — 정지'
            return out
        self._lost_since = None
        goal_dist = self._goal_marker_distance(quality)             # 교차로 통과와 무관 (도착 판정과 같은 조건)
        if goal_dist is not None and goal_dist <= p.marker_slow_distance:
            speed_factor *= p.marker_slow_factor
            out.reason = f"{reason} — 벽 마커 {plan['goal_marker_id']} {goal_dist:.2f} m"
        if kappa is not None:
            out.v, out.omega = self.controller.follow_curvature(kappa, dt, speed_factor)
            if not lane_visible:
                out.reason = f'{reason} — 차선 안 보임, 기억한 중앙 따라감'
        else:
            out.v, out.omega = self.controller.command(0.0, error_x, quality, dt, None, speed_factor, False,
                                                       error_stamp=self._lane['stamp'])
        self._last_cmd = (out.v, out.omega)
        # 위치가 없으니 주행거리는 명령 속도로 추측한다 (횡단보도 재래치 거리 판정용)
        self._travelled += abs(out.v) * dt
        return out
