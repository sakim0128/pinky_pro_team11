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

from .drive_fsm import IDLE, LANE_SEARCH, STATE_NAMES, DriveFsm, FsmParams, Inputs
from .lane_control import (QUALITY_BOTH, QUALITY_JUNCTION, QUALITY_LOST, QUALITY_SINGLE,
                           QUALITY_STALE, ControlParams, LaneController)
from .link_watch import LinkWatch
from .obstacle_guard import GuardParams, ObstacleGuard
from .route_follower import RouteFollower

CMD_HEARTBEAT, CMD_START, CMD_STOP, CMD_ESTOP, CMD_RESUME, CMD_SET_SPEED, CMD_CLEARANCE = range(7)


@dataclass
class DriverParams:
    control: ControlParams = field(default_factory=ControlParams)
    fsm: FsmParams = field(default_factory=FsmParams)
    guard: GuardParams = field(default_factory=GuardParams)
    lookahead: float = 0.25
    arrive_tolerance: float = 0.10
    clearance_tolerance: float = 0.05     # clear_until 에 이 거리 안이면 "닿았다"
    link_timeout: float = 3.0             # 관제 명령 침묵 허용
    path_timeout: float = 0.9             # LanePath 침묵 허용 (0.3 s × 3)
    path_max_age: float = 0.9             # 이보다 오래된 error_x 는 STALE 취급
    crosswalk_zone: float = 0.45          # 그래프 횡단보도 노드 ± 이 거리 밖의 트리거는 무시
    junction_zone: float = 0.25           # 분기 노드 ± 이 거리는 JUNCTION 취급 (관제가 안 보내도)
    stop_line_zone: float = 0.60          # 정지선 검출은 경로상 다음 분기 노드가 이 거리 안에 있을 때만 교차로 트리거 (오검출 방어)
    junction_wait_clearance: bool = True  # JUNCTION_STOP 에서 관제 허가(clear_until > 분기 idx) 를 기다린다
    lane_only: bool = False               # 경로·위치 없이 카메라 차선 중앙만 따라간다 (테스트 모드)
    lane_lost_coast: float = 0.6          # lane_only: 차선을 잃고 이만큼(s) 직전 명령 유지 후 정지
    search_omega: float = 0.4             # LANE_SEARCH 제자리 회전 각속도 (rad/s)


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
                      'barricade': False, 'stop_line': False, 'left_seen': False, 'right_seen': False}
        self._search_dir = 0.0
        self._last_valid_error = None      # 마지막 BOTH/SINGLE 의 error_x (LOST 탐색 방향용)
        self._travelled = 0.0
        self._last_xy = None
        self._last_tick = None
        self._last_cmd = (0.0, 0.0)
        self._lost_since = None

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

    def set_lane_path(self, now, source_stamp, quality, error_x, crosswalk=False,
                      barricade=False, left_seen=None, right_seen=None, stop_line=False):
        self.path_link.on_command(now, heartbeat=True)
        q = int(quality)
        if left_seen is None:            # 옛 호출자: quality 로 추정
            left_seen = q == QUALITY_BOTH
        if right_seen is None:
            right_seen = q == QUALITY_BOTH
        self._lane = {'stamp': float(source_stamp), 'quality': q,
                      'error_x': None if error_x is None else float(error_x),
                      'crosswalk': bool(crosswalk), 'barricade': bool(barricade),
                      'stop_line': bool(stop_line),
                      'left_seen': bool(left_seen), 'right_seen': bool(right_seen)}
        if q in (QUALITY_BOTH, QUALITY_SINGLE) and error_x is not None:
            self._last_valid_error = float(error_x)

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
        self.controller.reset()
        self._last_cmd = (0.0, 0.0)
        self._lost_since = None
        if self.fsm.search_failed:
            out.v, out.omega = 0.0, 0.0
            return True
        if not self._search_dir:
            self._search_dir = self._search_direction(x, y, yaw)
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
        # 정지선(카메라) → 교차로 트리거. 경로 모드는 다음 분기 노드가 stop_line_zone 안일 때만 (다른 흰 선 오검출 방어)
        next_j = self._next_junction_idx(p.stop_line_zone)
        stop_line_trigger = bool(self._lane['stop_line']) and next_j is not None
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
            junction_trigger=in_junction or stop_line_trigger, junction_clear=junction_clear,
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
            omega_route, error_x, quality, dt, dist_to_limit, speed_factor, turn_in_place)
        return out

    # ------------------------------------------------ 차선만 (테스트 모드)

    def _tick_lane_only(self, now, dt, out, blocked, obstacle_reason, quality, error_x,
                        lane_visible, lane_both):
        """경로·위치 없이 카메라 error_x 만으로 중앙 주행. 횡단보도·바리게이트·장애물·링크 감시는 그대로."""
        p = self.p
        inp = Inputs(
            started=self.started, estop=self.estop,
            link_ok=self.station_link.alive(now), path_ok=self.path_link.alive(now),
            obstacle=blocked, obstacle_reason=obstacle_reason,
            at_clearance=False, crosswalk_trigger=self._lane['crosswalk'],   # 존 검사 없음 (그래프가 없다)
            barricade=self._lane['barricade'],
            # 정지선 → 교차로 정지. 관제·경로가 없으니 허가 없이 정지 시간만 채우고 통과한다
            junction_trigger=bool(self._lane['stop_line']), junction_clear=True,
            lane_visible=lane_visible, lane_both=lane_both, arrived=False, travelled=self._travelled,
        )
        state, speed_factor, reason = self.fsm.step(now, inp)
        out.state, out.state_name, out.reason = state, STATE_NAMES[state], reason
        out.odom_since_state = self.fsm.travelled_since_entry(self._travelled)
        out.edge_id = 'lane_only'
        if self._apply_search(out, 0.0, 0.0, 0.0):
            return out
        if speed_factor <= 0.0:
            self.controller.reset()
            self._last_cmd = (0.0, 0.0)
            self._lost_since = None
            out.v, out.omega = 0.0, 0.0
            return out
        if not lane_visible:
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
        out.v, out.omega = self.controller.command(0.0, error_x, quality, dt, None, speed_factor, False)
        self._last_cmd = (out.v, out.omega)
        # 위치가 없으니 주행거리는 명령 속도로 추측한다 (횡단보도 재래치 거리 판정용)
        self._travelled += abs(out.v) * dt
        return out
