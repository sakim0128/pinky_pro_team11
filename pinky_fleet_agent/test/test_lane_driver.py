"""차선 주행 코어 테스트 — 유니사이클 모델로 폐루프를 돌린다 (ROS 없이)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.drive_fsm import (  # noqa: E402
    ARRIVED, CROSSWALK_CLEAR, CROSSWALK_STOP, CRUISE, ESTOP, IDLE, LANE_LOST, LINK_LOST,
    OBSTACLE_WAIT, WAIT_CLEARANCE, DriveFsm, FsmParams, Inputs,
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
    sim = Sim(straight())
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
    sim.run(12)
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
        self.cam = True
        self.log = []

    def run(self, seconds):
        for _ in range(int(seconds / DT)):
            self.t += DT
            if self.cam and int(self.t / DT) % 6 == 0:
                lat = self.y + 0.15 * math.sin(self.yaw)          # 앞쪽 샘플 행에서 본 횡오차
                e = max(-1.0, min(1.0, lat / self.half_lane))
                self.d.set_lane_path(self.t, self.t, self.quality, e, self.crosswalk)
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
    s.run(6.0)
    states = [o.state for _, o in s.log]
    stop_t = [t for t, o in s.log if o.state == CROSSWALK_STOP]
    assert stop_t and 2.9 <= stop_t[-1] - stop_t[0] + DT <= 3.3
    assert states[-1] == CRUISE
    # 정지 구간은 한 번뿐
    runs = sum(1 for i in range(1, len(states)) if states[i] == CROSSWALK_STOP != states[i - 1])
    assert runs == 1


def test_lane_only_lost_coasts_then_stops():
    s = LaneOnlySim()
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
