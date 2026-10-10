"""차선 주행 코어 테스트 — 유니사이클 모델로 폐루프를 돌린다 (ROS 없이)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.drive_fsm import (  # noqa: E402
    ARRIVED, BARRICADE_WAIT, CROSSWALK_CLEAR, CROSSWALK_STOP, CRUISE, ESTOP, IDLE, LANE_LOST,
    LANE_SEARCH, LINK_LOST, OBSTACLE_WAIT, WAIT_CLEARANCE, DriveFsm, FsmParams, Inputs,
)
from pinky_fleet_agent.lane_control import (  # noqa: E402
    QUALITY_BOTH, QUALITY_JUNCTION, QUALITY_LOST, QUALITY_SINGLE, ControlParams, LaneController,
)
from pinky_fleet_agent.lane_driver import (  # noqa: E402
    CMD_CLEARANCE, CMD_ESTOP, CMD_RESUME, CMD_START, DriverParams, LaneDriver,
)
from pinky_fleet_agent.obstacle_guard import GuardParams, ObstacleGuard  # noqa: E402
from pinky_fleet_agent.route_follower import RouteFollower  # noqa: E402

DT = 0.05


def straight(n=31, step=0.1):
    return [(i * step, 0.0) for i in range(n)]          # 3.0 m, x 축


def l_shape():
    pts = [(i * 0.1, 0.0) for i in range(11)]           # 1.0 m 직진
    pts += [(1.0, i * 0.1) for i in range(1, 11)]       # 90° 좌회전 후 1.0 m
    return pts


class Sim:
    """LaneDriver 를 유니사이클로 감싼다. 카메라는 중심선 횡오차를 error_x 로 흉내낸다."""

    def __init__(self, waypoints, params=None, cam=True, half_lane=0.10, **route_kw):
        self.d = LaneDriver(params)
        n = len(waypoints)
        kw = dict(edge_end_idx=[n - 1], edge_ids=['e0'], crosswalk_idx=[], junction_idx=[],
                  goal_idx=n - 1, route_seq=1)
        kw.update(route_kw)
        self.d.set_route(waypoints, **kw)
        self.x, self.y, self.yaw = waypoints[0][0], waypoints[0][1], 0.0
        self.t = 0.0
        self.cam = cam
        self.half_lane = half_lane
        self.log = []
        self.crosswalk_visible = False
        self.lane_quality = QUALITY_BOTH

    def start(self, clear_until=None):
        self.d.set_command(CMD_START, self.t)
        self.d.set_clearance(1, self.d.goal_idx if clear_until is None else clear_until, self.t)

    def run(self, seconds, heartbeat=True, cam=None):
        cam = self.cam if cam is None else cam
        for _ in range(int(seconds / DT)):
            self.t += DT
            if heartbeat and int(self.t / DT) % 2 == 0:            # 10 Hz clearance = 하트비트
                self.d.set_clearance(1, self.d.clear_until, self.t, self.d.clearance_reason)
            if cam and int(self.t / DT) % 6 == 0:                  # 0.3 s LanePath
                f = self.d.follower
                lat = f.lateral if f else 0.0
                # 로봇이 왼쪽(lat>0)에 있으면 차선 중앙은 화면 오른쪽 → error_x 양수
                error_x = max(-1.0, min(1.0, lat / self.half_lane))
                crosswalk = False
                if self.crosswalk_visible and f is not None:
                    # 카메라는 앞에 있는 횡단보도만 본다 (노드 0.45 m 앞까지)
                    for i in self.d.crosswalk_idx:
                        ahead = f.cum[i] - f.progress_s
                        if 0.0 <= ahead <= 0.45:
                            crosswalk = True
                self.d.set_lane_path(self.t, self.t, self.lane_quality, error_x, crosswalk)
            out = self.d.tick(self.t, self.x, self.y, self.yaw)
            self.x += out.v * math.cos(self.yaw) * DT
            self.y += out.v * math.sin(self.yaw) * DT
            self.yaw += out.omega * DT
            self.log.append((self.t, out))
        return self.log[-1][1]

    def states(self):
        return [o.state for _, o in self.log]


# ------------------------------------------------------------ route_follower

def test_follower_progress_monotonic_and_lateral_sign():
    f = RouteFollower(straight())
    f.update(0.5, 0.05)
    assert f.progress_s == pytest.approx(0.5) and f.lateral == pytest.approx(0.05)
    f.update(0.3, 0.0)               # 뒤로 튐 → 무시
    assert f.progress_s == pytest.approx(0.5)
    f.update(1.0, -0.02)
    assert f.lateral == pytest.approx(-0.02)
    assert f.distance_to_idx(30) == pytest.approx(2.0)
    assert f.remaining() == pytest.approx(2.0)


def test_follower_steer_turns_toward_lookahead():
    f = RouteFollower(l_shape(), lookahead=0.25)
    f.update(0.9, 0.0)
    omega, turn = f.steer(0.9, 0.0, 0.0, 0.15)
    assert omega > 0 and not turn               # 좌회전
    f.update(1.0, 0.0)
    omega, turn = f.steer(1.0, 0.0, math.pi, 0.15)   # 반대를 보고 있으면 제자리 회전
    assert turn


def test_follower_lookahead_clamped_by_limit():
    f = RouteFollower(straight(), lookahead=0.5)
    f.update(1.0, 0.0)
    assert f.lookahead_point(limit_idx=12) == pytest.approx((1.2, 0.0))


# ------------------------------------------------------------ lane_control

def test_controller_camera_sign_and_blend():
    c = LaneController(ControlParams(kp=1.0, kd=0.0, accel_slew=100))
    # error_x 음수 = 차선 중앙이 화면 왼쪽 = 로봇이 오른쪽 치우침 → 좌회전(ω 양수)
    v, w = c.command(0.0, -0.5, QUALITY_BOTH, DT, None)
    assert w > 0
    v, w = c.command(0.0, 0.5, QUALITY_BOTH, DT, None)
    assert w < 0
    v, w0 = c.command(0.3, 0.5, QUALITY_LOST, DT, None)       # 카메라 무시
    assert w0 == pytest.approx(0.3)
    v, w1 = c.command(0.3, 0.5, QUALITY_SINGLE, DT, None)     # 절반 가중
    assert 0.3 - 0.5 < w1 < 0.3


def test_controller_brakes_before_stop_and_zero_at_tolerance():
    c = LaneController(ControlParams(v_max=0.2, a_decel=0.1, accel_slew=100))
    v_far, _ = c.command(0.0, 0.0, QUALITY_BOTH, DT, 2.0)
    v_near, _ = c.command(0.0, 0.0, QUALITY_BOTH, DT, 0.05)
    v_stop, w = c.command(0.0, 0.0, QUALITY_BOTH, DT, 0.01)
    assert v_far == pytest.approx(0.2) and 0 < v_near < 0.2 and v_stop == 0.0 and w == 0.0


def test_controller_accel_slew():
    c = LaneController(ControlParams(v_max=0.2, accel_slew=0.4))
    v1, _ = c.command(0.0, 0.0, QUALITY_BOTH, 0.05, None)
    assert v1 == pytest.approx(0.02)


# ------------------------------------------------------------ drive_fsm

def run_fsm(fsm, start, stop, step=0.1, **kw):
    t = start
    last = None
    while t < stop:
        last = fsm.step(t, Inputs(**kw))
        t += step
    return last


def test_fsm_priority_and_crosswalk_hold():
    fsm = DriveFsm(FsmParams(crosswalk_stop_seconds=1.0, crosswalk_relatch_distance=0.5))
    assert fsm.step(0, Inputs(started=False))[0] == IDLE
    assert fsm.step(0, Inputs(started=True, estop=True))[0] == ESTOP
    assert fsm.step(0, Inputs(started=True, link_ok=False))[0] == LINK_LOST
    assert fsm.step(0, Inputs(started=True, obstacle=True, at_clearance=True))[0] == OBSTACLE_WAIT
    assert fsm.step(0, Inputs(started=True, at_clearance=True, crosswalk_trigger=True))[0] == WAIT_CLEARANCE
    s, f, _ = fsm.step(1.0, Inputs(started=True, crosswalk_trigger=True, travelled=0.0))
    assert s == CROSSWALK_STOP and f == 0.0
    s, f, _ = fsm.step(1.5, Inputs(started=True, crosswalk_trigger=True, travelled=0.0))
    assert s == CROSSWALK_STOP                       # 1 초 안 찼음
    s, f, _ = fsm.step(2.1, Inputs(started=True, crosswalk_trigger=True, travelled=0.0))
    assert s == CROSSWALK_CLEAR and f == 1.0         # 시간 찼고, 트리거 계속 보여도 재래치 안 됨
    s, f, _ = fsm.step(2.2, Inputs(started=True, crosswalk_trigger=True, travelled=0.3))
    assert s == CROSSWALK_CLEAR
    s, f, _ = fsm.step(2.3, Inputs(started=True, crosswalk_trigger=False, travelled=0.6))
    assert s == CRUISE
    s, f, _ = fsm.step(2.4, Inputs(started=True, crosswalk_trigger=True, travelled=0.7))
    assert s == CROSSWALK_STOP                       # 0.5 m 지났으니 다시 걸린다


def test_fsm_lane_lost_slows_and_arrived_stops():
    fsm = DriveFsm()
    s, f, _ = fsm.step(0, Inputs(started=True, lane_visible=False))
    assert s == LANE_LOST and f == 0.5
    s, f, _ = fsm.step(1, Inputs(started=True, arrived=True))
    assert s == ARRIVED and f == 0.0


# ------------------------------------------------------------ obstacle_guard

def scan_with_point(x, y, n=360):
    """(x, y) 한 점만 보이는 스캔."""
    ranges = [float('inf')] * n
    a = math.atan2(y, x)
    i = int(round((a + math.pi) / (2 * math.pi / n))) % n
    ranges[i] = math.hypot(x, y)
    return ranges, -math.pi, 2 * math.pi / n


def test_guard_confirm_and_clear():
    g = ObstacleGuard(GuardParams(confirm_count=2, clear_seconds=1.0))
    g.update_scan(*scan_with_point(0.12, 0.0))
    assert g.step(0.0)[0] is False                  # 1회는 아직
    g.update_scan(*scan_with_point(0.12, 0.0))
    blocked, reason = g.step(0.1)
    assert blocked and '라이다' in reason
    g.update_scan(*scan_with_point(0.5, 0.0))       # 박스 밖
    assert g.step(0.2)[0] is True                   # 해제 대기
    assert g.step(0.9)[0] is True
    assert g.step(1.3)[0] is False                  # 1.0 s 비었음


def test_guard_ignores_invalid_and_side_points():
    g = ObstacleGuard(GuardParams(confirm_count=1))
    ranges, amin, ainc = scan_with_point(0.12, 0.0)
    ranges = [0.0 if r == float('inf') else r for r in ranges]       # 0.0 은 무효값
    ranges[0] = float('nan')
    g.update_scan(ranges, amin, ainc)
    assert g.step(0)[0] is True
    g2 = ObstacleGuard(GuardParams(confirm_count=1))
    g2.update_scan(*scan_with_point(0.10, 0.30))    # 옆쪽 — 박스 밖
    assert g2.step(0)[0] is False


def test_guard_ultrasonic_median():
    g = ObstacleGuard(GuardParams(confirm_count=1, us_stop=0.2))
    g.update_us(0.05)                               # 튀는 값 하나
    assert g.step(0)[0] is True                     # 첫 샘플은 그대로 중앙값
    g.update_us(0.5)
    g.update_us(0.5)
    assert g.us_range == pytest.approx(0.5)


# ------------------------------------------------------------ lane_driver 폐루프

def test_driver_follows_straight_route_and_arrives():
    sim = Sim(straight())
    sim.start()
    out = sim.run(40)
    assert out.state == ARRIVED, out.reason
    assert abs(sim.y) < 0.03 and sim.x == pytest.approx(3.0, abs=0.12)
    assert CRUISE in sim.states()


def test_driver_turns_corner_and_camera_keeps_center():
    sim = Sim(l_shape(), half_lane=0.10)
    sim.start()
    out = sim.run(50)
    assert out.state == ARRIVED, out.reason
    assert (sim.x, sim.y) == pytest.approx((1.0, 1.0), abs=0.12)
    # 코너 이후 직선 구간에서 횡오차가 작아야 한다
    late = [o.lateral for t, o in sim.log if t > 30]
    assert max(abs(v) for v in late) < 0.05


def test_driver_waits_at_clearance_then_continues():
    sim = Sim(straight())
    sim.start(clear_until=10)                       # 1.0 m 까지만
    out = sim.run(15)
    assert out.state == WAIT_CLEARANCE
    assert sim.x == pytest.approx(1.0, abs=0.06)
    sim.d.set_clearance(1, 30, sim.t)
    out = sim.run(25)
    assert out.state == ARRIVED


def test_driver_ignores_clearance_of_other_route_seq():
    sim = Sim(straight())
    sim.start(clear_until=10)
    sim.d.set_clearance(99, 30, sim.t)
    assert sim.d.clear_until == 10


def test_driver_crosswalk_stop_only_near_crosswalk_node():
    sim = Sim(straight(), crosswalk_idx=[15])       # 1.5 m 지점
    sim.start()
    sim.crosswalk_visible = True                    # 처음부터 계속 보인다고 우겨도
    sim.run(3)
    assert CROSSWALK_STOP not in sim.states()       # 노드 반경 밖이라 무시
    out = sim.run(12)
    stops = [(t, o) for t, o in sim.log if o.state == CROSSWALK_STOP]
    assert stops, '횡단보도 앞에서 서야 한다'
    t_first = stops[0][0]
    held = [t for t, o in sim.log if o.state == CROSSWALK_STOP]
    assert max(held) - min(held) == pytest.approx(3.0, abs=0.15)
    assert 0.9 <= stops[0][1].route_idx * 0.1 < 1.6      # 노드(1.5 m) 앞, 카메라가 보기 시작한 곳
    out = sim.run(25)
    assert out.state == ARRIVED
    # 한 번만 섰다
    entries = sum(1 for (t0, a), (t1, b) in zip(sim.log, sim.log[1:])
                  if b.state == CROSSWALK_STOP and a.state != CROSSWALK_STOP)
    assert entries == 1


def test_driver_obstacle_stops_and_resumes():
    sim = Sim(straight())
    sim.start()
    sim.run(5)
    for _ in range(3):
        sim.d.update_scan(*scan_with_point(0.12, 0.0))
        sim.run(DT)
    assert sim.log[-1][1].state == OBSTACLE_WAIT
    x_stop = sim.x
    sim.run(2)
    assert sim.x == pytest.approx(x_stop, abs=0.02)
    sim.d.update_scan(*scan_with_point(2.0, 0.0))
    sim.run(2)
    assert sim.log[-1][1].state in (CRUISE, ARRIVED)


def test_driver_stops_when_station_silent_and_when_path_silent():
    sim = Sim(straight())
    sim.start()
    sim.run(3)
    sim.run(4, heartbeat=False)
    assert sim.log[-1][1].state == LINK_LOST
    sim2 = Sim(straight())
    sim2.start()
    sim2.run(3)
    sim2.run(2, cam=False)
    assert sim2.log[-1][1].state == LINK_LOST and 'LanePath' in sim2.log[-1][1].reason


def test_driver_lane_lost_keeps_driving_slowly_on_map_route():
    p = DriverParams()
    p.fsm.lane_search = False                       # 탐색 회전을 끄면 옛 동작: 맵 경로만으로 서행
    sim = Sim(straight(), params=p)
    sim.lane_quality = QUALITY_LOST
    sim.start()
    out = sim.run(60)
    assert LANE_LOST in sim.states()
    assert out.state == ARRIVED                     # 맵 경로만으로도 도착한다
    v_max = max(o.v for _, o in sim.log)
    assert v_max <= 0.15 * 0.5 + 1e-6


def test_driver_junction_zone_disables_camera():
    sim = Sim(straight(), junction_idx=[15])
    sim.start()
    sim.run(40)                                        # 교차로 정지 1 s + 서행 통과까지
    q = [o.quality for t, o in sim.log if 1.3 < o.route_idx * 0.1 < 1.7]
    assert q and all(v == QUALITY_JUNCTION for v in q)


def test_driver_estop_and_resume():
    sim = Sim(straight())
    sim.start()
    sim.run(2)
    sim.d.set_command(CMD_ESTOP, sim.t)
    assert sim.run(1).state == ESTOP
    sim.d.set_command(CMD_RESUME, sim.t)
    sim.d.set_command(CMD_START, sim.t)
    assert sim.run(1).state == CRUISE


# ------------------------------------------------------------ lane_only (경로·위치 없이 차선만)

class LaneOnlySim:
    """직선 차선(중심 y = 0) 위 유니사이클. 카메라 error_x 는 횡오차 + 헤딩 성분으로 흉내낸다."""

    def __init__(self, y0=0.04, yaw0=0.0, v_max=0.10, half_lane=0.10, auto_start=True):
        p = DriverParams()
        p.lane_only = True
        p.control.v_max = v_max
        self.d = LaneDriver(p)
        self.d.started = auto_start
        self.x, self.y, self.yaw = 0.0, y0, yaw0
        self.t = 0.0
        self.half_lane = half_lane
        self.quality = QUALITY_BOTH
        self.crosswalk = False
        self.barricade = False
        self.seen = None                                   # (left, right) 강제. None 이면 quality 로
        self.cam = True
        self.log = []

    def run(self, seconds):
        for _ in range(int(seconds / DT)):
            self.t += DT
            if self.cam and int(self.t / DT) % 6 == 0:
                lat = self.y + 0.15 * math.sin(self.yaw)          # 앞쪽 샘플 행에서 본 횡오차
                e = max(-1.0, min(1.0, lat / self.half_lane))
                seen = self.seen or (None, None)
                self.d.set_lane_path(self.t, self.t, self.quality, e, self.crosswalk,
                                     barricade=self.barricade, left_seen=seen[0], right_seen=seen[1])
            out = self.d.tick(self.t, 0.0, 0.0, 0.0)              # 위치는 안 준다
            self.x += out.v * math.cos(self.yaw) * DT
            self.y += out.v * math.sin(self.yaw) * DT
            self.yaw += out.omega * DT
            self.log.append((self.t, out))
        return self.log[-1][1]


def test_lane_only_converges_to_center_without_route():
    s = LaneOnlySim(y0=0.04, yaw0=0.1)
    out = s.run(8.0)
    assert out.state == CRUISE and out.edge_id == 'lane_only'
    assert abs(s.y) < 0.015 and abs(s.yaw) < 0.1
    assert s.x > 0.5                                   # 실제로 전진했다
    assert max(o.v for _, o in s.log) <= 0.10 + 1e-9  # v_max 0.10


def test_lane_only_does_not_move_without_start():
    s = LaneOnlySim(auto_start=False)
    out = s.run(2.0)
    assert out.state == IDLE and s.x == 0.0
    s.d.set_command(CMD_START, s.t)
    assert s.run(2.0).v > 0.0


def test_lane_only_crosswalk_stops_3s_once_without_graph():
    s = LaneOnlySim()
    s.run(2.0)
    s.crosswalk = True
    s.run(1.5)                                          # 확정 → 정지
    s.crosswalk = False
    s.run(9.0)                                          # 0.10 m/s × 9 s > 재래치 거리 0.60 m
    states = [o.state for _, o in s.log]
    stop_t = [t for t, o in s.log if o.state == CROSSWALK_STOP]
    assert stop_t and 2.9 <= stop_t[-1] - stop_t[0] + DT <= 3.3
    assert states[-1] == CRUISE
    # 정지 구간은 한 번뿐
    runs = sum(1 for i in range(1, len(states)) if states[i] == CROSSWALK_STOP != states[i - 1])
    assert runs == 1


def test_lane_only_lost_coasts_then_stops():
    s = LaneOnlySim()
    s.d.p.fsm.lane_search = False                         # 탐색 없이: 잠깐 유지 → 정지
    s.run(3.0)
    s.quality = QUALITY_LOST
    out = s.run(0.4)
    assert out.v > 0.0 and '유지' in out.reason           # 0.6 s 동안 직전 명령 유지
    out = s.run(1.0)
    assert out.v == 0.0 and out.omega == 0.0 and '정지' in out.reason
    s.quality = QUALITY_BOTH
    assert s.run(2.0).v > 0.0                             # 차선이 돌아오면 재출발


def test_lane_only_obstacle_and_camera_silence_stop():
    s = LaneOnlySim()
    s.run(2.0)
    for _ in range(3):
        s.d.update_scan(*scan_with_point(0.10, 0.0))
    out = s.run(0.5)
    assert out.state == OBSTACLE_WAIT and out.v == 0.0
    for _ in range(3):
        s.d.update_scan(*scan_with_point(2.0, 0.0))
    assert s.run(2.0).state == CRUISE
    s.cam = False                                          # LanePath 침묵 → 0.9 s 뒤 정지
    out = s.run(1.5)
    assert out.state == LINK_LOST and out.v == 0.0


# ------------------------------------------------------------ D14 작업 2·5: 탐색 회전 · 바리게이트

def test_fsm_barricade_waits_until_cleared():
    fsm = DriveFsm()
    s, f, r = fsm.step(0, Inputs(started=True, barricade=True))
    assert s == BARRICADE_WAIT and f == 0.0 and '바리게이트' in r
    assert fsm.step(0.5, Inputs(started=True, barricade=True, crosswalk_trigger=True))[0] == BARRICADE_WAIT
    assert fsm.step(1.0, Inputs(started=True, obstacle=True, barricade=True))[0] == OBSTACLE_WAIT
    assert fsm.step(1.5, Inputs(started=True))[0] == CRUISE      # 관제가 해제하면 바로 복귀


def test_fsm_lane_search_enters_after_delay_and_returns_on_both():
    fsm = DriveFsm(FsmParams(search_after=0.5, search_confirm_seconds=0.3, search_max_seconds=8.0))
    single = Inputs(started=True, lane_visible=True, lane_both=False)
    assert fsm.step(0.0, single)[0] == CRUISE                  # SINGLE 은 잠시 그냥 주행
    assert fsm.step(0.4, single)[0] == CRUISE
    s, f, _ = fsm.step(0.6, single)
    assert s == LANE_SEARCH and f == 0.0 and not fsm.search_failed
    both = Inputs(started=True, lane_both=True)
    assert fsm.step(0.7, both)[0] == LANE_SEARCH               # 0.3 s 확인 전
    assert fsm.step(0.8, single)[0] == LANE_SEARCH             # 깜빡임은 무시
    assert fsm.step(1.0, both)[0] == LANE_SEARCH
    s, f, _ = fsm.step(1.35, both)
    assert s == CRUISE and f == 1.0


def test_fsm_lane_search_times_out_then_stays_stopped():
    fsm = DriveFsm(FsmParams(search_after=0.0, search_max_seconds=2.0))
    lost = Inputs(started=True, lane_visible=False, lane_both=False)
    assert fsm.step(0.0, lost)[0] == LANE_SEARCH
    assert fsm.step(1.9, lost)[0] == LANE_SEARCH and not fsm.search_failed
    s, f, r = fsm.step(2.1, lost)
    assert s == LANE_SEARCH and f == 0.0 and fsm.search_failed and '실패' in r
    assert fsm.step(5.0, Inputs(started=True, lane_both=True))[0] == LANE_SEARCH   # 확인 시간 전
    assert fsm.step(5.4, Inputs(started=True, lane_both=True))[0] == CRUISE
    assert not fsm.search_failed


def test_fsm_search_only_on_lost_when_single_disabled():
    fsm = DriveFsm(FsmParams(search_after=0.0, search_on_single=False))
    assert fsm.step(0.0, Inputs(started=True, lane_visible=True, lane_both=False))[0] == CRUISE
    assert fsm.step(0.1, Inputs(started=True, lane_visible=False, lane_both=False))[0] == LANE_SEARCH


def test_fsm_junction_counts_as_pair_and_stop_states_reset_timer():
    fsm = DriveFsm(FsmParams(search_after=0.5))
    fsm.step(0.0, Inputs(started=True, lane_both=False))
    fsm.step(0.4, Inputs(started=True, crosswalk_trigger=True, lane_both=False))     # 정지 상태
    assert fsm.state == CROSSWALK_STOP
    s = fsm.step(3.5, Inputs(started=True, lane_both=False))[0]
    assert s == CROSSWALK_CLEAR                       # 정지 중 시간은 탐색 타이머에 안 쌓인다 → 바로 탐색 아님
    s = fsm.step(4.2, Inputs(started=True, lane_both=False, travelled=1.0))[0]
    assert s == LANE_SEARCH                           # 이제 0.5 s 넘겼다


def test_lane_only_single_left_rotates_right_until_pair_returns():
    """왼쪽 차선만 보이면 도로는 오른쪽 → 우회전(ω<0) 하다가 쌍이 보이면 다시 주행."""
    s = LaneOnlySim(y0=0.0)
    s.run(2.0)
    yaw0 = s.yaw
    s.quality = QUALITY_SINGLE
    s.seen = (True, False)
    out = s.run(1.5)
    assert out.state == LANE_SEARCH and out.v == 0.0 and out.omega < 0.0
    assert s.yaw < yaw0 - 0.2                          # 실제로 우회전했다
    s.quality = QUALITY_BOTH
    s.seen = None
    out = s.run(1.0)
    assert out.state == CRUISE and out.v > 0.0
    assert s.run(6.0).state == CRUISE and abs(s.yaw) < 0.15     # 다시 중앙으로 수렴


def test_lane_only_single_right_rotates_left_and_lost_uses_error_sign():
    s = LaneOnlySim()
    s.run(1.0)
    s.quality = QUALITY_SINGLE
    s.seen = (False, True)
    assert s.run(1.5).omega > 0.0
    s2 = LaneOnlySim(y0=0.0)
    s2.run(1.0)
    s2.y = +0.05                                       # 로봇이 왼쪽 → 마지막 유효 error_x > 0
    s2.run(0.35)
    s2.quality = QUALITY_LOST
    s2.seen = (False, False)
    assert s2.run(1.5).omega < 0.0                     # 차선 중앙이 오른쪽에 있었다 → 우회전


def test_lane_only_search_direction_is_held_and_stops_on_timeout():
    s = LaneOnlySim()
    s.d.p.fsm.search_max_seconds = 2.0
    s.run(1.0)
    s.quality = QUALITY_SINGLE
    s.seen = (True, False)
    s.run(1.0)
    s.seen = (False, True)                            # 회전 중 다른 쪽만 보여도 방향은 유지
    out = s.run(0.5)
    assert out.state == LANE_SEARCH and out.omega < 0.0
    out = s.run(2.0)
    assert out.state == LANE_SEARCH and out.omega == 0.0 and out.v == 0.0 and '실패' in out.reason


def test_lane_only_barricade_stops_and_resumes_when_cleared():
    s = LaneOnlySim()
    s.run(2.0)
    s.barricade = True
    out = s.run(1.0)
    assert out.state == BARRICADE_WAIT and out.v == 0.0 and out.omega == 0.0
    x_stop = s.x
    s.run(3.0)
    assert s.x == x_stop                              # 치워질 때까지 움직이지 않는다
    s.barricade = False
    out = s.run(1.0)
    assert out.state == CRUISE and out.v > 0.0


def test_route_mode_search_turns_toward_lookahead():
    """경로가 있으면 회전 방향은 다음 웨이포인트 쪽: 로봇이 경로 왼쪽(+y)을 보고 있으면 우회전."""
    sim = Sim(straight())
    sim.start()
    sim.run(2)
    sim.yaw = +0.8                                    # 경로(x 축)에서 왼쪽으로 크게 틀어짐
    sim.lane_quality = QUALITY_LOST
    out = sim.run(1.5)
    assert out.state == LANE_SEARCH and out.v == 0.0 and out.omega < 0.0
    sim.lane_quality = QUALITY_BOTH
    out = sim.run(2.0)
    assert out.state == CRUISE and out.v > 0.0


# ------------------------------------------------------------ D14 작업 3: 교차로 정지 → 경로만 통과

def test_fsm_junction_stop_then_pass_then_cruise():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS, JUNCTION_STOP
    fsm = DriveFsm(FsmParams(junction_stop_seconds=1.0, junction_exit_confirm=0.3,
                             junction_relatch_distance=0.6, junction_speed_factor=0.4))
    inside = Inputs(started=True, junction_trigger=True, lane_both=True)     # 반경 안: quality JUNCTION
    s, f, _ = fsm.step(0.0, inside)
    assert s == JUNCTION_STOP and f == 0.0
    assert fsm.step(0.9, inside)[0] == JUNCTION_STOP
    s, f, _ = fsm.step(1.1, inside)
    assert s == JUNCTION_PASS and f == 0.4
    s, f, _ = fsm.step(2.0, Inputs(started=True, lane_both=False, travelled=0.3))   # 반경 밖, 아직 쌍 없음
    assert s == JUNCTION_PASS and f == 0.4
    assert fsm.step(2.1, Inputs(started=True, lane_both=True, travelled=0.35))[0] == JUNCTION_PASS
    s, f, _ = fsm.step(2.5, Inputs(started=True, lane_both=True, travelled=0.4))
    assert s == CRUISE and f == 1.0
    # 재래치: 0.6 m 안에서는 다시 안 선다
    assert fsm.step(2.6, Inputs(started=True, junction_trigger=True, lane_both=True, travelled=0.5))[0] == CRUISE
    assert fsm.step(2.7, Inputs(started=True, junction_trigger=True, lane_both=True, travelled=0.7))[0] == JUNCTION_STOP


def test_fsm_junction_pass_without_pair_falls_to_lane_lost_after_relatch():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS, JUNCTION_STOP
    fsm = DriveFsm(FsmParams(junction_stop_seconds=0.0, junction_relatch_distance=0.6, lane_search=False))
    assert fsm.step(0.0, Inputs(started=True, junction_trigger=True, lane_both=True))[0] == JUNCTION_STOP
    assert fsm.step(0.1, Inputs(started=True, junction_trigger=True, lane_both=True))[0] == JUNCTION_PASS
    assert fsm.step(1.0, Inputs(started=True, lane_visible=False, lane_both=False, travelled=0.5))[0] == JUNCTION_PASS
    assert fsm.step(1.5, Inputs(started=True, lane_visible=False, lane_both=False, travelled=0.7))[0] == LANE_LOST


def test_fsm_junction_priority_below_crosswalk_and_obstacle():
    from pinky_fleet_agent.drive_fsm import JUNCTION_STOP
    fsm = DriveFsm()
    assert fsm.step(0.0, Inputs(started=True, junction_trigger=True, obstacle=True))[0] == OBSTACLE_WAIT
    assert fsm.step(0.1, Inputs(started=True, junction_trigger=True, crosswalk_trigger=True))[0] == CROSSWALK_STOP
    fsm2 = DriveFsm(FsmParams(junction_stop=False))
    assert fsm2.step(0.0, Inputs(started=True, junction_trigger=True, lane_both=True))[0] == CRUISE


def test_driver_stops_at_junction_then_passes_slowly_on_route():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS, JUNCTION_STOP
    sim = Sim(straight(), junction_idx=[15])          # 노드 1.5 m, 반경 0.25
    sim.start()
    out = sim.run(60)
    assert out.state == ARRIVED
    stop = [(t, o) for t, o in sim.log if o.state == JUNCTION_STOP]
    assert stop and all(o.v == 0.0 and o.omega == 0.0 for _, o in stop)
    assert 0.95 <= stop[-1][0] - stop[0][0] + DT <= 1.15                      # 1 s 정지
    assert 1.15 <= stop[0][1].route_idx * 0.1 <= 1.35                         # 반경 진입 지점
    passing = [o for _, o in sim.log if o.state == JUNCTION_PASS]
    assert passing and max(o.v for o in passing) <= 0.15 * 0.4 + 1e-6         # 서행
    assert all(o.quality == QUALITY_JUNCTION for o in passing if 1.3 < o.route_idx * 0.1 < 1.7)
    runs = sum(1 for i in range(1, len(sim.log)) if sim.log[i][1].state == JUNCTION_STOP != sim.log[i - 1][1].state)
    assert runs == 1                                                           # 교차로 정지는 한 번


# ------------------------------------------------------------ 교차로 규칙 (2026-09-29): 빨간 선 트리거 · 관제 허가 대기

def test_fsm_junction_stop_waits_for_clearance_then_passes():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS, JUNCTION_STOP
    fsm = DriveFsm(FsmParams(junction_stop_seconds=1.0))
    blocked = Inputs(started=True, junction_trigger=True, lane_both=True, junction_clear=False,
                     clearance_reason='J1 pinky1 점유')
    assert fsm.step(0.0, blocked)[0] == JUNCTION_STOP
    s, f, r = fsm.step(1.5, blocked)                              # 1 s 지났지만 허가 없음 → 계속 정지
    assert s == JUNCTION_STOP and f == 0.0 and '허가 없음' in r and 'pinky1' in r
    assert fsm.step(30.0, blocked)[0] == JUNCTION_STOP            # 오래 기다려도 스스로 출발하지 않는다
    # 예약 경계(at_clearance)에 닿아도 JUNCTION_STOP 은 WAIT_CLEARANCE 로 바뀌지 않는다
    assert fsm.step(30.1, Inputs(started=True, junction_trigger=True, lane_both=True, junction_clear=False,
                                 at_clearance=True))[0] == JUNCTION_STOP
    s, f, _ = fsm.step(30.2, Inputs(started=True, junction_trigger=True, lane_both=True, junction_clear=True))
    assert s == JUNCTION_PASS and f == 0.4
    # 허가가 바로 있으면 1 s 뒤 통과 (기존 동작)
    fsm2 = DriveFsm(FsmParams(junction_stop_seconds=1.0))
    ok = Inputs(started=True, junction_trigger=True, lane_both=True)
    assert fsm2.step(0.0, ok)[0] == JUNCTION_STOP
    assert fsm2.step(0.9, ok)[0] == JUNCTION_STOP
    assert fsm2.step(1.0, ok)[0] == JUNCTION_PASS


class StopLineSim(Sim):
    """카메라가 빨간 선을 본다: 경로상 분기 노드 0.10 m 앞에 빨간 선이 있고, 그 0.30 m 앞부터 화면 하단에 걸린다."""

    def __init__(self, *a, red_line_ahead=0.30, red_line_offset=0.10, **kw):
        super().__init__(*a, **kw)
        self.red_line_visible = True
        self.red_line_ahead = red_line_ahead
        self.red_line_offset = red_line_offset
        self.fake_red_line = False        # 분기와 무관한 자리에서 빨간 선을 (잘못) 본다

    def run(self, seconds, heartbeat=True, cam=None):
        cam = self.cam if cam is None else cam
        for _ in range(int(seconds / DT)):
            self.t += DT
            if heartbeat and int(self.t / DT) % 2 == 0:
                self.d.set_clearance(1, self.d.clear_until, self.t, self.d.clearance_reason)
            if cam and int(self.t / DT) % 6 == 0:
                f = self.d.follower
                lat = f.lateral if f else 0.0
                error_x = max(-1.0, min(1.0, lat / self.half_lane))
                stop = self.fake_red_line
                if self.red_line_visible and f is not None:
                    for i in self.d.junction_idx:
                        ahead = f.cum[i] - self.red_line_offset - f.progress_s
                        if 0.0 <= ahead <= self.red_line_ahead:
                            stop = True
                self.d.set_lane_path(self.t, self.t, self.lane_quality, error_x, False, red_line=stop)
            out = self.d.tick(self.t, self.x, self.y, self.yaw)
            self.x += out.v * math.cos(self.yaw) * DT
            self.y += out.v * math.sin(self.yaw) * DT
            self.yaw += out.omega * DT
            self.log.append((self.t, out))
        return self.log[-1][1]


def test_driver_red_line_triggers_junction_stop_before_map_zone():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS, JUNCTION_STOP
    p = DriverParams()
    p.junction_zone = 0.05                                 # 맵 반경을 거의 끄고 빨간 선만으로 세운다
    sim = StopLineSim(straight(), params=p, junction_idx=[15])
    sim.start()
    out = sim.run(60)
    assert out.state == ARRIVED
    stop = [(t, o) for t, o in sim.log if o.state == JUNCTION_STOP]
    assert stop and all(o.v == 0.0 for _, o in stop)
    assert 0.95 <= stop[-1][0] - stop[0][0] + DT <= 1.15                     # 허가가 있으니 1 s 만 선다
    assert 1.05 <= stop[0][1].route_idx * 0.1 <= 1.45                        # 빨간 선 앞 (노드 1.5 m 보다 앞)
    assert any(o.state == JUNCTION_PASS for _, o in sim.log)
    runs = sum(1 for i in range(1, len(sim.log)) if sim.log[i][1].state == JUNCTION_STOP != sim.log[i - 1][1].state)
    assert runs == 1


def test_driver_red_line_far_from_any_junction_is_ignored():
    from pinky_fleet_agent.drive_fsm import JUNCTION_STOP
    sim = StopLineSim(straight(), junction_idx=[25])       # 분기 2.5 m — 출발 직후 보이는 빨간 선은 0.6 m 밖
    sim.red_line_visible = False
    sim.fake_red_line = True                              # 처음부터 계속 빨간 선을 본다
    sim.start()
    sim.run(8)
    early = [o for t, o in sim.log if t < 8 and o.route_idx * 0.1 < 1.5]
    assert early and all(o.state != JUNCTION_STOP for o in early)
    # 분기가 없는 경로면 빨간 선을 아무리 봐도 교차로 트리거가 아니다
    sim2 = StopLineSim(straight(), junction_idx=[])
    sim2.fake_red_line = True
    sim2.start()
    assert sim2.run(40).state == ARRIVED and not any(o.state == JUNCTION_STOP for _, o in sim2.log)


def test_driver_waits_at_red_line_until_station_clears_the_junction():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS, JUNCTION_STOP
    sim = StopLineSim(straight(), junction_idx=[15], edge_end_idx=[15, 30], edge_ids=['a', 'b'])
    sim.start(clear_until=13)                               # 관제: 분기 노드 0.2 m 앞까지만 허가
    sim.d.clearance_reason = 'J pinky1 통과 중'
    out = sim.run(12)
    assert out.state == JUNCTION_STOP and out.v == 0.0 and '허가 없음' in out.reason and 'pinky1' in out.reason
    assert 1.0 <= out.route_idx * 0.1 <= 1.45              # 빨간 선 앞에서 멈춘 채
    sim.d.set_clearance(1, 30, sim.t, '')                   # 관제가 분기 너머까지 허가
    out = sim.run(40)
    assert out.state == ARRIVED
    assert any(o.state == JUNCTION_PASS for _, o in sim.log)
    # 허가 없이 서 있던 동안 WAIT_CLEARANCE 로 바뀌지 않았다 (한 상태로 기다린다)
    assert not any(o.state == WAIT_CLEARANCE for t, o in sim.log if t <= 12)


def test_driver_junction_wait_clearance_can_be_disabled():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS
    p = DriverParams()
    p.junction_wait_clearance = False
    sim = StopLineSim(straight(), params=p, junction_idx=[15], edge_end_idx=[15, 30], edge_ids=['a', 'b'])
    sim.start(clear_until=13)
    sim.run(12)
    assert any(o.state == JUNCTION_PASS for _, o in sim.log)          # 허가 없이도 1 s 뒤 통과 (예약 경계에서 선다)
    assert sim.log[-1][1].state == WAIT_CLEARANCE


def test_lane_only_red_line_stops_once_then_continues_without_station():
    from pinky_fleet_agent.drive_fsm import RED_LINE_STOP
    s = LaneOnlySim()

    def run(seconds, red_line):
        for _ in range(int(seconds / DT)):
            s.t += DT
            if int(s.t / DT) % 6 == 0:
                s.d.set_lane_path(s.t, s.t, QUALITY_BOTH, 0.0, False, red_line=red_line)
            out = s.d.tick(s.t, 0.0, 0.0, 0.0)
            s.x += out.v * DT
            s.log.append((s.t, out))
        return s.log[-1][1]

    assert run(2.0, False).v > 0.0
    out = run(0.5, True)
    assert out.state == RED_LINE_STOP and out.v == 0.0
    out = run(1.0, True)                                               # 선이 계속 보여도 다시 서지 않는다
    assert s.log[-1][1].state == CRUISE and out.v > 0.0                # 관제가 없으니 허가 없이 출발
    out = run(3.0, False)
    assert out.v > 0.0
    runs = sum(1 for i in range(1, len(s.log)) if s.log[i][1].state == RED_LINE_STOP != s.log[i - 1][1].state)
    assert runs == 1


# ------------------------------------------------------------ 차선 하나: 회전하지 않고 따라간다 (search_on_single false)

def test_lane_only_single_lane_keeps_driving_without_rotation():
    s = LaneOnlySim(y0=0.0)
    s.d.p.fsm.search_on_single = False
    s.d.p.control.single_weight = 0.8
    s.run(2.0)
    s.quality = QUALITY_SINGLE
    s.seen = (True, False)
    x0 = s.x
    s.run(3.0)
    states = {o.state for _, o in s.log[-60:]}
    assert LANE_SEARCH not in states and states == {CRUISE}
    assert s.x > x0 + 0.2                              # 계속 전진
    assert all(o.v > 0.0 for _, o in s.log[-40:])
    s.quality = QUALITY_LOST                           # 아예 안 보이면 그때만 탐색
    s.seen = (False, False)
    assert s.run(1.5).state == LANE_SEARCH


def test_single_weight_scales_camera_correction():
    from pinky_fleet_agent.lane_control import ControlParams, LaneController
    lo = LaneController(ControlParams(single_weight=0.5))
    hi = LaneController(ControlParams(single_weight=0.8))
    _, w_lo = lo.command(0.0, 0.3, QUALITY_SINGLE, 0.05, None)
    _, w_hi = hi.command(0.0, 0.3, QUALITY_SINGLE, 0.05, None)
    assert w_lo < 0.0 and w_hi < w_lo                  # 같은 오차에 더 세게 (우회전)
    _, w_both = hi.command(0.0, 0.3, QUALITY_BOTH, 0.05, None)
    assert abs(w_both) > abs(w_hi)


def test_agent_yaml_single_lane_rule():
    import yaml
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(here, 'params', 'lane_agent.yaml'), encoding='utf-8') as fh:
        prm = yaml.safe_load(fh)['pinky_lane_agent']['ros__parameters']
    assert prm['fsm']['search_on_single'] is False and prm['fsm']['lane_search'] is True
    assert prm['control']['single_weight'] == 0.8


# ------------------------------------------------------------ 2026-09-30 실차 피드백: D 항 킥 · error 필터 · 빨간 선 매번 정지

def _omega_trace(ctrl, use_stamp, meas):
    """20 Hz 틱, 10 Hz 측정 (meas: 측정 시각별 error). 틱마다 ω."""
    out, t, e, stamp = [], 0.0, 0.0, None
    for k in range(40):
        t = k * 0.05
        if k % 2 == 0:
            e, stamp = meas(t), t
        _, w = ctrl.command(0.0, e, QUALITY_BOTH, 0.05, None, error_stamp=stamp if use_stamp else None)
        out.append(w)
    return out


def test_derivative_uses_measurement_interval_not_control_tick():
    meas = lambda t: 0.02 * (t / 0.1)                     # 0.1 s 마다 0.02 씩 오르는 오차 (de = 0.2 /s)
    old = _omega_trace(LaneController(ControlParams(kp=0.0, kd=1.0)), False, meas)
    new = _omega_trace(LaneController(ControlParams(kp=0.0, kd=1.0)), True, meas)
    # 예전: 새 측정 틱마다 de = 0.02/0.05 = 0.4, 다음 틱 0 → ω 가 0 과 −0.4 사이를 오간다
    assert max(abs(w) for w in old[4:]) == pytest.approx(0.4, abs=1e-6) and min(abs(w) for w in old[4:]) < 1e-9
    # 새: 측정 간격으로 나눈 기울기 0.2 를 다음 측정까지 유지 → 튀지 않는다
    assert all(w == pytest.approx(-0.2, abs=1e-6) for w in new[4:])


def test_error_alpha_filters_step_change():
    c = LaneController(ControlParams(kp=1.0, kd=0.0, error_alpha=0.5))
    c.command(0.0, 0.0, QUALITY_BOTH, 0.05, None, error_stamp=0.0)
    _, w1 = c.command(0.0, 0.2, QUALITY_BOTH, 0.05, None, error_stamp=0.1)
    _, w1b = c.command(0.0, 0.2, QUALITY_BOTH, 0.05, None, error_stamp=0.1)   # 같은 측정 — 필터 한 번만
    _, w2 = c.command(0.0, 0.2, QUALITY_BOTH, 0.05, None, error_stamp=0.2)
    assert w1 == pytest.approx(-0.1) and w1b == pytest.approx(-0.1) and w2 == pytest.approx(-0.15)
    raw = LaneController(ControlParams(kp=1.0, kd=0.0))                        # 기본 1.0 = 필터 없음
    raw.command(0.0, 0.0, QUALITY_BOTH, 0.05, None, error_stamp=0.0)
    assert raw.command(0.0, 0.2, QUALITY_BOTH, 0.05, None, error_stamp=0.1)[1] == pytest.approx(-0.2)


def test_filter_resets_when_camera_weight_is_zero():
    c = LaneController(ControlParams(kp=1.0, kd=0.0, error_alpha=0.5))
    c.command(0.0, 0.4, QUALITY_BOTH, 0.05, None, error_stamp=0.0)
    c.command(0.0, 0.4, QUALITY_LOST, 0.05, None, error_stamp=0.1)            # 끊김 → 필터 초기화
    assert c.command(0.0, -0.2, QUALITY_BOTH, 0.05, None, error_stamp=0.2)[1] == pytest.approx(0.2)


def test_fsm_red_line_event_stops_then_cruises_and_priorities():
    from pinky_fleet_agent.drive_fsm import JUNCTION_PASS, RED_LINE_STOP
    fsm = DriveFsm(FsmParams(red_line_stop_seconds=1.0))
    assert fsm.step(0.0, Inputs(started=True, red_line_event=True))[0] == RED_LINE_STOP
    assert fsm.step(0.5, Inputs(started=True))[:2] == (RED_LINE_STOP, 0.0)
    assert fsm.step(1.05, Inputs(started=True))[:2] == (CRUISE, 1.0)
    assert fsm.step(1.1, Inputs(started=True, red_line_event=True, obstacle=True))[0] == OBSTACLE_WAIT
    fsm2 = DriveFsm(FsmParams(junction_stop_seconds=0.0))
    fsm2.step(0.0, Inputs(started=True, junction_trigger=True))
    assert fsm2.step(0.1, Inputs(started=True, junction_trigger=True))[0] == JUNCTION_PASS
    assert fsm2.step(0.2, Inputs(started=True, red_line_event=True))[0] == RED_LINE_STOP   # 통과 중 출구 선에서도 선다
    fsm3 = DriveFsm()
    assert fsm3.step(0.0, Inputs(started=True, red_line_event=True, maneuver_active=True))[0] == JUNCTION_PASS
    assert DriveFsm(FsmParams(red_line_stop=False)).step(0.0, Inputs(started=True, red_line_event=True))[0] == CRUISE


def test_lane_only_stops_at_every_red_line_even_when_close():
    """교차로 입구·출구 선이 0.3 m 간격 → 둘 다 선다 (예전엔 재래치 0.6 m 때문에 두 번째를 놓쳤다)."""
    from pinky_fleet_agent.drive_fsm import RED_LINE_STOP
    s = LaneOnlySim(y0=0.0)
    lines = (0.5, 0.8)                                      # 각 선이 카메라 정지 행에 보이는 x 구간 [a, a+0.04)
    for _ in range(int(14.0 / DT)):
        s.t += DT
        if int(s.t / DT) % 2 == 0:
            red = any(a <= s.x < a + 0.04 for a in lines)
            s.d.set_lane_path(s.t, s.t, QUALITY_BOTH, 0.0, False, red_line=red)
        out = s.d.tick(s.t, 0.0, 0.0, 0.0)
        s.x += out.v * DT
        s.log.append((s.t, out))
    states = [o.state for _, o in s.log]
    starts = [s.log[i][0] for i in range(1, len(states)) if states[i] == RED_LINE_STOP != states[i - 1]]
    assert len(starts) == 2
    assert s.x > 1.0 and states[-1] == CRUISE


# ---------------------------------------------------------------- 초음파: 붙으면 무효값(< us_min_valid) 이 나온다

def test_ultrasonic_invalid_after_close_reading_keeps_blocking():
    """2026-10-04 pinky2: 앞 로봇에 다가가며 0.17 → 0.09 → 0.04 다음 −0.01~0.02 가 2 s — 무효로 버려 계속 회전했다."""
    from pinky_fleet_agent.obstacle_guard import GuardParams, ObstacleGuard
    g = ObstacleGuard(GuardParams(us_stop=0.10, use_lidar=False))
    for r in (0.17, 0.09, 0.04, 0.019, 0.005, -0.002, -0.006):
        g.update_us(r)
    blocked = [g.step(t)[0] for t in (0.0, 0.05, 0.1)]
    assert blocked[-1]


def test_ultrasonic_invalid_without_close_reading_is_ignored():
    """막 켰을 때처럼 가까운 유효값 없이 나온 무효값은 장애물이 아니다."""
    from pinky_fleet_agent.obstacle_guard import GuardParams, ObstacleGuard
    g = ObstacleGuard(GuardParams(us_stop=0.10, use_lidar=False))
    for r in (0.0, -0.01, 0.0, 0.01):
        g.update_us(r)
    assert not any(g.step(t)[0] for t in (0.0, 0.05, 0.1))
    for r in (0.80, 0.0, 0.0):                           # 멀리 있다가 무효값 — 장애물 아님
        g.update_us(r)
    assert not any(g.step(t)[0] for t in (0.2, 0.25, 0.3))
