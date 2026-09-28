"""주행 상태기계 — 바퀴를 멈출지 말지는 여기서만 정한다.

ROS 에 의존하지 않는다. 매 틱 ``step(now, inputs)`` 에 관측을 넣으면 (state, speed_factor,
reason) 을 돌려준다. speed_factor 0 이면 정지, 1 이면 정상, 0.5 면 서행.

우선순위 (위가 이긴다):
    ESTOP > LINK_LOST > OBSTACLE_WAIT > BARRICADE_WAIT > WAIT_CLEARANCE > CROSSWALK_STOP
          > CROSSWALK_CLEAR > JUNCTION_STOP > JUNCTION_PASS > ARRIVED > LANE_SEARCH > LANE_LOST > CRUISE > IDLE

JUNCTION_STOP / JUNCTION_PASS (D14 작업 3): 항공뷰 위치가 그래프 분기 노드 반경(junction_zone) 에 들어오면
junction_stop_seconds 정지 → 카메라 없이 경로(pure-pursuit) 만으로 junction_speed_factor 로 통과. 반경을 벗어나고
차선 쌍이 junction_exit_confirm 동안 보이면 CRUISE. 재래치는 주행거리(junction_relatch_distance).

LANE_SEARCH (D14 작업 2): 차선 쌍(BOTH)이 search_after 이상 안 보이면 진입. speed_factor 0 이고
드라이버가 제자리 회전을 건다 (방향은 드라이버가 정한다). BOTH 가 search_confirm_seconds 이어지면
CRUISE 복귀, search_max_seconds 를 넘기면 search_failed 를 세우고 정지한 채 머문다.
BARRICADE_WAIT (작업 5): 카메라 바리게이트 트리거. 관제 디바운스가 풀리면(치워짐) 자동 복귀.

횡단보도 재래치는 시간이 아니라 **주행거리** 로 막는다 — 3 초 서 있던 로봇은 정의상 느리다.
빨간불·타임아웃 류의 "오래 기다리면 스스로 출발" 은 넣지 않는다. mini_project_1 의
hold_watchdog 이 같은 논쟁 끝에 "그때 할 일은 재출발이 아니라 정지" 로 결론냈다.
"""

from dataclasses import dataclass, field

IDLE, CRUISE, WAIT_CLEARANCE, CROSSWALK_STOP, CROSSWALK_CLEAR, OBSTACLE_WAIT, \
    LANE_LOST, ARRIVED, ESTOP, LINK_LOST, BARRICADE_WAIT, LANE_SEARCH, \
    JUNCTION_STOP, JUNCTION_PASS = range(14)

STATE_NAMES = {
    IDLE: 'IDLE', CRUISE: 'CRUISE', WAIT_CLEARANCE: 'WAIT_CLEARANCE',
    CROSSWALK_STOP: 'CROSSWALK_STOP', CROSSWALK_CLEAR: 'CROSSWALK_CLEAR',
    OBSTACLE_WAIT: 'OBSTACLE_WAIT', LANE_LOST: 'LANE_LOST', ARRIVED: 'ARRIVED',
    ESTOP: 'ESTOP', LINK_LOST: 'LINK_LOST', BARRICADE_WAIT: 'BARRICADE_WAIT',
    LANE_SEARCH: 'LANE_SEARCH', JUNCTION_STOP: 'JUNCTION_STOP', JUNCTION_PASS: 'JUNCTION_PASS',
}


@dataclass
class FsmParams:
    crosswalk_stop_seconds: float = 3.0
    crosswalk_relatch_distance: float = 0.60     # CLEAR 진입 후 이만큼 가기 전엔 재트리거 무시 (crosswalk_zone 보다 커야 한다)
    lane_lost_speed_factor: float = 0.5
    blind_warn_distance: float = 0.60            # 카메라 없이 이만큼 이상 달리면 reason 에 경고
    lane_search: bool = True                     # 차선 쌍이 안 보이면 제자리 회전 탐색 (D14 작업 2)
    search_on_single: bool = True                # SINGLE 도 탐색 트리거 (False 면 LOST/STALE 만)
    search_after: float = 0.5                    # 쌍이 이만큼(s) 안 보여야 탐색 시작
    search_confirm_seconds: float = 0.3          # 탐색 중 BOTH 가 이만큼 이어지면 복귀
    search_max_seconds: float = 8.0              # 이보다 오래 돌면 실패 → 정지
    junction_stop: bool = True                   # 분기 노드 반경 진입 시 정지 후 통과 (D14 작업 3)
    junction_stop_seconds: float = 1.0
    junction_speed_factor: float = 0.4           # 통과 속도 = v_max × 이 값
    junction_exit_confirm: float = 0.3           # 반경 밖에서 차선 쌍이 이만큼(s) 보이면 CRUISE
    junction_relatch_distance: float = 0.60      # STOP 진입 후 이만큼 가기 전엔 재트리거 무시 (> 2·junction_zone)


@dataclass
class Inputs:
    started: bool = False           # CMD_START 받았고 STOP/ESTOP 아님
    estop: bool = False             # ESTOP 래치 (RESUME 으로만 해제)
    link_ok: bool = True            # 관제 하트비트 살아 있음
    path_ok: bool = True            # LanePath 가 최근에 옴 (STALE/LOST 포함 — 침묵만 False)
    obstacle: bool = False          # 라이다/초음파 확정 감지 (해제 지연은 guard 가 처리)
    obstacle_reason: str = ''
    at_clearance: bool = False      # 진행도가 clear_until 에 닿았고 clear_until < goal
    clearance_reason: str = ''
    crosswalk_trigger: bool = False # 디바운스 통과한 횡단보도 트리거
    barricade: bool = False         # 디바운스 통과한 바리게이트 (관제가 해제할 때까지 True)
    junction_trigger: bool = False  # 위치가 그래프 분기 노드 반경 안 (경로 모드만)
    lane_visible: bool = True       # quality ∈ {BOTH, SINGLE, JUNCTION}
    lane_both: bool = True          # quality ∈ {BOTH, JUNCTION} — 탐색 복귀 조건
    arrived: bool = False
    travelled: float = 0.0          # 누적 주행거리 (m). 상태 진입 후 거리 계산에 쓴다


@dataclass
class DriveFsm:
    params: FsmParams = field(default_factory=FsmParams)
    state: int = IDLE
    entered_at: float = 0.0
    entered_travel: float = 0.0
    reason: str = ''
    search_failed: bool = False     # LANE_SEARCH 가 search_max_seconds 를 넘김 → 드라이버는 회전도 멈춘다
    _crosswalk_clear_from: float = None
    _nonboth_since: float = None
    _both_since: float = None
    _junction_stop_travel: float = None     # 마지막 JUNCTION_STOP 진입 시 주행거리 (재래치용)
    _exit_both_since: float = None

    def _enter(self, state, now, travelled, reason=''):
        if state != self.state:
            self.state = state
            self.entered_at = now
            self.entered_travel = travelled
        self.reason = reason

    def since_entry(self, now):
        return now - self.entered_at

    def travelled_since_entry(self, travelled):
        return travelled - self.entered_travel

    def step(self, now, inp):
        """(state, speed_factor, reason)"""
        p = self.params
        t, d = float(now), float(inp.travelled)

        if inp.estop:
            self._enter(ESTOP, t, d, 'ESTOP 래치 — RESUME 필요')
            return self.state, 0.0, self.reason
        if not inp.started:
            self._enter(IDLE, t, d, '대기')
            return self.state, 0.0, self.reason
        if not inp.link_ok or not inp.path_ok:
            self._enter(LINK_LOST, t, d, '관제 하트비트 끊김' if not inp.link_ok else 'LanePath 끊김')
            return self.state, 0.0, self.reason
        if inp.obstacle:
            self._enter(OBSTACLE_WAIT, t, d, inp.obstacle_reason or '장애물')
            return self.state, 0.0, self.reason
        if inp.barricade:
            self._enter(BARRICADE_WAIT, t, d, '바리게이트 — 치워지면 재출발')
            return self.state, 0.0, self.reason

        # 횡단보도: 정지 유지 중이면 시간이 찰 때까지 그대로
        if self.state == CROSSWALK_STOP:
            if self.since_entry(t) < p.crosswalk_stop_seconds:
                self.reason = f'횡단보도 정지 {self.since_entry(t):.1f}/{p.crosswalk_stop_seconds:.0f}s'
                return self.state, 0.0, self.reason
            self._enter(CROSSWALK_CLEAR, t, d, '횡단보도 통과')
        if inp.at_clearance and self.state != CROSSWALK_CLEAR:
            self._enter(WAIT_CLEARANCE, t, d, inp.clearance_reason or '예약 대기')
            return self.state, 0.0, self.reason
        if inp.at_clearance:
            # 통과 중이라도 예약 경계에 닿으면 선다 (CLEAR 상태는 유지 안 함)
            self._enter(WAIT_CLEARANCE, t, d, inp.clearance_reason or '예약 대기')
            return self.state, 0.0, self.reason

        # 재래치 금지 구간
        relatch_blocked = (self.state == CROSSWALK_CLEAR
                           and self.travelled_since_entry(d) < p.crosswalk_relatch_distance)
        if inp.crosswalk_trigger and not relatch_blocked:
            self._enter(CROSSWALK_STOP, t, d, '횡단보도 정지 0.0/%.0fs' % p.crosswalk_stop_seconds)
            return self.state, 0.0, self.reason
        if relatch_blocked:
            self.reason = f'횡단보도 통과 {self.travelled_since_entry(d):.2f}/{p.crosswalk_relatch_distance:.2f} m'
            return self.state, 1.0, self.reason

        # 교차로: 정지 → 경로만으로 통과 → 반경 밖에서 쌍 확인
        if self.state == JUNCTION_STOP:
            if self.since_entry(t) < p.junction_stop_seconds:
                self.reason = f'교차로 정지 {self.since_entry(t):.1f}/{p.junction_stop_seconds:.0f}s'
                return self.state, 0.0, self.reason
            self._enter(JUNCTION_PASS, t, d, '교차로 통과 — 경로만')
            self._exit_both_since = None
        if self.state == JUNCTION_PASS:
            if inp.junction_trigger:
                self._exit_both_since = None
                self.reason = '교차로 통과 — 경로만'
                return self.state, p.junction_speed_factor, self.reason
            if inp.lane_both:
                if self._exit_both_since is None:
                    self._exit_both_since = t
                if t - self._exit_both_since >= p.junction_exit_confirm:
                    self._enter(CRUISE, t, d, '교차로 벗어남 — 주행')
                    return self.state, 1.0, self.reason
            else:
                self._exit_both_since = None
            if self.travelled_since_entry(d) < p.junction_relatch_distance:
                self.reason = f'교차로 통과 {self.travelled_since_entry(d):.2f}/{p.junction_relatch_distance:.2f} m'
                return self.state, p.junction_speed_factor, self.reason
            self._enter(LANE_LOST, t, d, '교차로 뒤 차선 없음')      # 아래 탐색/서행 규칙으로 넘긴다
        junction_relatch_ok = (self._junction_stop_travel is None
                               or d - self._junction_stop_travel >= p.junction_relatch_distance)
        if p.junction_stop and inp.junction_trigger and junction_relatch_ok:
            self._junction_stop_travel = d
            self._enter(JUNCTION_STOP, t, d, '교차로 정지 0.0/%.0fs' % p.junction_stop_seconds)
            return self.state, 0.0, self.reason

        if inp.arrived:
            self._enter(ARRIVED, t, d, '도착')
            return self.state, 0.0, self.reason

        # 차선 쌍 탐색 (BOTH/JUNCTION 이 아닌 시간을 잰다 — 주행 상태에서만)
        if inp.lane_both:
            self._nonboth_since = None
            if self._both_since is None:
                self._both_since = t
        else:
            self._both_since = None
            if self._nonboth_since is None:
                self._nonboth_since = t
        if self.state == LANE_SEARCH:
            if inp.lane_both and t - self._both_since >= p.search_confirm_seconds:
                self.search_failed = False
                self._enter(CRUISE, t, d, '차선 쌍 회복 — 주행')
                return self.state, 1.0, self.reason
            if self.since_entry(t) >= p.search_max_seconds:
                self.search_failed = True
                self.reason = f'차선 탐색 실패 {p.search_max_seconds:.0f}s — 정지'
            else:
                self.reason = f'차선 탐색 회전 {self.since_entry(t):.1f}/{p.search_max_seconds:.0f}s'
            return self.state, 0.0, self.reason
        want_search = (p.lane_search and not inp.lane_both
                       and (p.search_on_single or not inp.lane_visible)
                       and (t - self._nonboth_since) >= p.search_after)
        if want_search:
            self.search_failed = False
            self._enter(LANE_SEARCH, t, d, '차선 탐색 회전 0.0/%.0fs' % p.search_max_seconds)
            return self.state, 0.0, self.reason

        if not inp.lane_visible:
            self._enter(LANE_LOST, t, d, '차선 없음 — 맵 경로만')
            blind = self.travelled_since_entry(d)
            if blind >= p.blind_warn_distance:
                self.reason = f'차선 없이 {blind:.2f} m 주행 중'
            return self.state, p.lane_lost_speed_factor, self.reason

        self._enter(CRUISE, t, d, '주행')
        return self.state, 1.0, self.reason
