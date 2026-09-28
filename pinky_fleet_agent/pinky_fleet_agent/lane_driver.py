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

from .drive_fsm import IDLE, STATE_NAMES, DriveFsm, FsmParams, Inputs
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
    lane_only: bool = False               # 경로·위치 없이 카메라 차선 중앙만 따라간다 (테스트 모드)
    lane_lost_coast: float = 0.6          # lane_only: 차선을 잃고 이만큼(s) 직전 명령 유지 후 정지
    # lane_only: 흰선(좌/우 차선) 넘지 않기 — 샘플 행에서 선과 화면 중앙 사이 여유 (화면 폭 비율)
    line_margin_frac: float = 0.08        # 이보다 가까우면 정지 후 안쪽으로 제자리 회전
    line_clear_frac: float = 0.12         # 이만큼 벌어지면 회전 끝 (히스테리시스)
    # lane_only: 한쪽 차선만 보이면 제자리 회전으로 반대쪽 차선을 찾는다
    single_hold: float = 0.3              # 한쪽만 보이는 상태가 이만큼(s) 이어지면 탐색 시작
    search_timeout: float = 4.0           # 이 안에 양쪽을 못 찾으면 정지
    both_confirm: int = 2                 # 양쪽이 연속 이만큼(LanePath 프레임) 보이면 탐색 끝
    turn_omega: float = 0.4               # 제자리 회전 속도 (rad/s)


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
                      'sides': None}
        self._both_streak = 0
        self._single_since = None
        self._search = None                  # (시작 시각, 회전 방향 +1 화면 왼쪽 / -1 화면 오른쪽)
        self._line_guard = None              # 너무 가까운 선 'left' / 'right'
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
                      left_seen=None, right_seen=None, left_x=0, right_x=0, width=0):
        """left_seen/right_seen/left_x/right_x/width(px) 를 주면 lane_only 의 선 넘기 방지·차선 탐색이 켜진다."""
        self.path_link.on_command(now, heartbeat=True)
        sides = None
        if left_seen is not None and right_seen is not None and width and width > 0:
            sides = {'left': bool(left_seen), 'right': bool(right_seen),
                     'left_x': float(left_x), 'right_x': float(right_x), 'width': float(width)}
        self._lane = {'stamp': float(source_stamp), 'quality': int(quality),
                      'error_x': None if error_x is None else float(error_x),
                      'crosswalk': bool(crosswalk), 'sides': sides}
        self._both_streak = self._both_streak + 1 if int(quality) == QUALITY_BOTH else 0

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
        if self.follower and self._near_any(self.junction_idx, p.junction_zone):
            quality = QUALITY_JUNCTION
        lane_visible = quality in (QUALITY_BOTH, QUALITY_SINGLE, QUALITY_JUNCTION)
        out.quality = quality
        out.error_x = error_x if error_x is not None else 0.0

        if self.follower is None and p.lane_only:
            return self._tick_lane_only(now, dt, out, blocked, obstacle_reason, quality, error_x,
                                        lane_visible)
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
            crosswalk_trigger=crosswalk_trigger, lane_visible=lane_visible,
            arrived=arrived, travelled=self._travelled,
        )
        state, speed_factor, reason = self.fsm.step(now, inp)
        out.state, out.state_name, out.reason = state, STATE_NAMES[state], reason
        out.odom_since_state = self.fsm.travelled_since_entry(self._travelled)

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

    def _tick_lane_only(self, now, dt, out, blocked, obstacle_reason, quality, error_x, lane_visible):
        """경로·위치 없이 카메라 error_x 만으로 중앙 주행. 횡단보도·장애물·링크 감시는 그대로."""
        p = self.p
        inp = Inputs(
            started=self.started, estop=self.estop,
            link_ok=self.station_link.alive(now), path_ok=self.path_link.alive(now),
            obstacle=blocked, obstacle_reason=obstacle_reason,
            at_clearance=False, crosswalk_trigger=self._lane['crosswalk'],   # 존 검사 없음 (그래프가 없다)
            lane_visible=lane_visible, arrived=False, travelled=self._travelled,
        )
        state, speed_factor, reason = self.fsm.step(now, inp)
        out.state, out.state_name, out.reason = state, STATE_NAMES[state], reason
        out.odom_since_state = self.fsm.travelled_since_entry(self._travelled)
        out.edge_id = 'lane_only'
        if speed_factor <= 0.0:
            self.controller.reset()
            self._last_cmd = (0.0, 0.0)
            self._lost_since = None
            self._single_since = self._search = self._line_guard = None
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
        if self._lane['sides'] is not None:
            turn = self._line_guard_turn(out)
            if turn is None:
                turn = self._search_turn(now, out, quality)
            if turn is not None:
                self.controller.reset()
                out.v, out.omega = 0.0, turn
                self._last_cmd = (out.v, out.omega)
                return out
        out.v, out.omega = self.controller.command(0.0, error_x, quality, dt, None, speed_factor, False)
        self._last_cmd = (out.v, out.omega)
        # 위치가 없으니 주행거리는 명령 속도로 추측한다 (횡단보도 재래치 거리 판정용)
        self._travelled += abs(out.v) * dt
        return out

    def _turn(self, image_dir):
        """화면 기준 방향(+1 왼쪽, -1 오른쪽) → 제자리 회전 ω. cam_sign 으로 실제 방향을 맞춘다."""
        return image_dir * self.p.turn_omega * self.p.control.cam_sign

    def _line_guard_turn(self, out):
        """좌/우 차선이 화면 중앙에 너무 가까우면 정지하고 반대쪽(차선 안쪽)으로 제자리 회전."""
        p, sd = self.p, self._lane['sides']
        cx, w = 0.5 * sd['width'], sd['width']
        margin = {}
        if sd['left']:
            margin['left'] = (cx - sd['left_x']) / w
        if sd['right']:
            margin['right'] = (sd['right_x'] - cx) / w
        if self._line_guard is not None:
            m = margin.get(self._line_guard)
            if m is None or m >= p.line_clear_frac:
                self._line_guard = None
        if self._line_guard is None:
            close = [k for k, m in margin.items() if m < p.line_margin_frac]
            if not close:
                return None
            self._line_guard = min(close, key=lambda k: margin[k])
            self._search = None
        side = self._line_guard
        m = margin.get(side, 0.0)
        out.reason = (f'{"왼쪽" if side == "left" else "오른쪽"} 흰선 근접 {m:.2f} — '
                      f'정지 후 안쪽으로 회전')
        return self._turn(-1.0 if side == 'left' else 1.0)

    def _search_turn(self, now, out, quality):
        """한쪽 차선만 single_hold 이상 보이면 안 보이는 쪽으로 제자리 회전해 양쪽을 찾는다."""
        p, sd = self.p, self._lane['sides']
        if quality != QUALITY_SINGLE:
            self._single_since = None
        elif self._single_since is None:
            self._single_since = now
        if self._search is not None and self._both_streak >= p.both_confirm:
            self._search = None
        if self._search is None:
            if self._single_since is None or now - self._single_since < p.single_hold:
                return None
            # 왼쪽 선만 보이면 오른쪽 선을 찾아 화면 오른쪽으로 돈다
            self._search = (now, -1.0 if sd['left'] else 1.0)
        started, image_dir = self._search
        elapsed = now - started
        if elapsed > p.search_timeout:
            out.reason = f'차선 탐색 실패 {p.search_timeout:.1f}s — 정지'
            return 0.0
        out.reason = (f'한쪽 차선만 보임 — {"오른쪽" if image_dir < 0 else "왼쪽"}으로 회전 탐색 '
                      f'{elapsed:.1f}/{p.search_timeout:.1f}s')
        return self._turn(image_dir)
