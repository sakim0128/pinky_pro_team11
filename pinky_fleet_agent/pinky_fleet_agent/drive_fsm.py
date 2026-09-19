"""주행 상태기계 — 바퀴를 멈출지 말지는 여기서만 정한다.

ROS 에 의존하지 않는다. 매 틱 ``step(now, inputs)`` 에 관측을 넣으면 (state, speed_factor,
reason) 을 돌려준다. speed_factor 0 이면 정지, 1 이면 정상, 0.5 면 서행.

우선순위 (위가 이긴다):
    ESTOP > LINK_LOST > OBSTACLE_WAIT > WAIT_CLEARANCE > CROSSWALK_STOP > CROSSWALK_CLEAR
          > ARRIVED > LANE_LOST > CRUISE > IDLE

횡단보도 재래치는 시간이 아니라 **주행거리** 로 막는다 — 3 초 서 있던 로봇은 정의상 느리다.
빨간불·타임아웃 류의 "오래 기다리면 스스로 출발" 은 넣지 않는다. mini_project_1 의
hold_watchdog 이 같은 논쟁 끝에 "그때 할 일은 재출발이 아니라 정지" 로 결론냈다.
"""

from dataclasses import dataclass, field

IDLE, CRUISE, WAIT_CLEARANCE, CROSSWALK_STOP, CROSSWALK_CLEAR, OBSTACLE_WAIT, \
    LANE_LOST, ARRIVED, ESTOP, LINK_LOST = range(10)

STATE_NAMES = {
    IDLE: 'IDLE', CRUISE: 'CRUISE', WAIT_CLEARANCE: 'WAIT_CLEARANCE',
    CROSSWALK_STOP: 'CROSSWALK_STOP', CROSSWALK_CLEAR: 'CROSSWALK_CLEAR',
    OBSTACLE_WAIT: 'OBSTACLE_WAIT', LANE_LOST: 'LANE_LOST', ARRIVED: 'ARRIVED',
    ESTOP: 'ESTOP', LINK_LOST: 'LINK_LOST',
}


@dataclass
class FsmParams:
    crosswalk_stop_seconds: float = 3.0
    crosswalk_relatch_distance: float = 0.60     # CLEAR 진입 후 이만큼 가기 전엔 재트리거 무시 (crosswalk_zone 보다 커야 한다)
    lane_lost_speed_factor: float = 0.5
    blind_warn_distance: float = 0.60            # 카메라 없이 이만큼 이상 달리면 reason 에 경고


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
    lane_visible: bool = True       # quality ∈ {BOTH, SINGLE, JUNCTION}
    arrived: bool = False
    travelled: float = 0.0          # 누적 주행거리 (m). 상태 진입 후 거리 계산에 쓴다


@dataclass
class DriveFsm:
    params: FsmParams = field(default_factory=FsmParams)
    state: int = IDLE
    entered_at: float = 0.0
    entered_travel: float = 0.0
    reason: str = ''
    _crosswalk_clear_from: float = None

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

        if inp.arrived:
            self._enter(ARRIVED, t, d, '도착')
            return self.state, 0.0, self.reason

        if not inp.lane_visible:
            self._enter(LANE_LOST, t, d, '차선 없음 — 맵 경로만')
            blind = self.travelled_since_entry(d)
            if blind >= p.blind_warn_distance:
                self.reason = f'차선 없이 {blind:.2f} m 주행 중'
            return self.state, p.lane_lost_speed_factor, self.reason

        self._enter(CRUISE, t, d, '주행')
        return self.state, 1.0, self.reason
