"""비전 미션 모드 (lane_only + JunctionPlan) — 빨간 선 정지 → 관제 허가 → 고정 동작(odom) → 차선 복귀 → 흰 정지선 세기 → 도착.

ROS 없이 LaneDriver 를 유니사이클로 돌린다. odom = 참 자세. 카메라는 플래그(빨간 선·정지선)와 차선 품질만 흉내낸다.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.drive_fsm import (  # noqa: E402
    ARRIVED, CRUISE, JUNCTION_PASS, JUNCTION_STOP, LANE_SEARCH, OBSTACLE_WAIT,
)
from pinky_fleet_agent.lane_control import QUALITY_BOTH, QUALITY_LOST  # noqa: E402
from pinky_fleet_agent.lane_driver import CMD_CLEARANCE, CMD_START, DriverParams, LaneDriver  # noqa: E402
from pinky_fleet_agent.maneuver import ManeuverExecutor, ManeuverParams, parse_steps  # noqa: E402

DT = 0.05
RIGHT = [('straight', 0.18), ('turn', -90.0), ('straight', 0.10)]


# ------------------------------------------------------------ 고정 동작 실행기

def drive(ex, pose=(0.0, 0.0, 0.0), seconds=20.0, pause=None):
    """실행기를 유니사이클로 돌린다. pause=(t0, t1) 동안은 명령을 버린다(장애물로 선 것처럼)."""
    x, y, yaw = pose
    t = 0.0
    while t < seconds:
        v, w, done, _ = ex.command(t, (x, y, yaw))
        if done:
            return (x, y, yaw), t
        if pause and pause[0] <= t < pause[1]:
            v = w = 0.0
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += w * DT
        t += DT
    raise AssertionError('동작이 끝나지 않았다')


def test_maneuver_right_turn_ends_where_measured():
    (x, y, yaw), _ = drive(ManeuverExecutor(RIGHT))
    assert x == pytest.approx(0.18, abs=0.01)
    assert y == pytest.approx(-0.10, abs=0.01)
    assert math.degrees(yaw) == pytest.approx(-90.0, abs=2.5)


def test_maneuver_left_and_straight_only():
    (x, y, yaw), _ = drive(ManeuverExecutor([('straight', 0.18), ('turn', 90.0), ('straight', 0.10)]))
    assert (x, y) == pytest.approx((0.18, 0.10), abs=0.01) and math.degrees(yaw) == pytest.approx(90.0, abs=2.5)
    (x, y, yaw), _ = drive(ManeuverExecutor([('straight', 0.35)]), pose=(1.0, 2.0, math.pi / 2))
    assert (x, y) == pytest.approx((1.0, 2.35), abs=0.01)


def test_maneuver_resumes_remaining_amount_after_pause():
    """장애물로 도중에 서도 odom 누적이라 남은 양만 이어서 한다 — 끝 자세가 같다."""
    (x, y, yaw), t_pause = drive(ManeuverExecutor(RIGHT), pause=(1.0, 3.0))
    assert (x, y) == pytest.approx((0.18, -0.10), abs=0.01)
    _, t_plain = drive(ManeuverExecutor(RIGHT))
    assert t_pause == pytest.approx(t_plain + 2.0, abs=0.2)


def test_maneuver_waits_without_odom_and_reports_turn_sign():
    ex = ManeuverExecutor(RIGHT)
    assert ex.command(0.0, None)[:3] == (0.0, 0.0, False)
    assert ex.last_turn_sign == -1.0
    assert ManeuverExecutor([('straight', 0.3)]).last_turn_sign == 0.0
    assert ManeuverExecutor([]).done


def test_parse_steps():
    assert parse_steps(['straight', 'TURN'], [0.2, -90]) == [('straight', 0.2), ('turn', -90.0)]
    with pytest.raises(ValueError):
        parse_steps(['jump'], [1.0])
    with pytest.raises(ValueError):
        parse_steps(['straight'], [])


# ------------------------------------------------------------ 드라이버 폐루프

class VisionSim:
    """lane_only 드라이버 + 계획. 빨간 선은 x ≥ red_x 에서 보인다(교차로 통과 뒤로는 안 보인다고 둔다)."""

    def __init__(self, steps=RIGHT, stop_line_count=1, red_x=1.0, params=None, **plan):
        p = params or DriverParams()
        p.lane_only = True
        p.control.v_max = 0.15
        self.d = LaneDriver(p)
        self.seq = 7
        self.d.set_plan(self.seq, steps, stop_line_count, **plan)
        self.x = self.y = self.yaw = 0.0
        self.t = 0.0
        self.clear = 0
        self.red_x = red_x
        self.quality = QUALITY_BOTH
        self.stop_line = False
        self.markers = lambda: {}                                 # 이번 LanePath 의 {id: 거리}
        self.us = 1.0
        self.log = []
        self.d.set_command(CMD_START, self.t)

    def run(self, seconds, until=None):
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            if int(round(self.t / DT)) % 2 == 0:
                self.d.set_command(CMD_CLEARANCE, self.t, route_seq=self.seq, clear_until=self.clear)
            if int(round(self.t / DT)) % 3 == 0:
                red = self.x >= self.red_x and not self.d._junction_passed
                self.d.set_lane_path(self.t, self.t, self.quality, 0.0, False,
                                     red_line=red, stop_line=self.stop_line, markers=self.markers())
            self.d.update_us(self.us)
            self.d.update_odom(self.t, self.x, self.y, self.yaw)
            out = self.d.tick(self.t, 0.0, 0.0, 0.0)
            self.x += out.v * math.cos(self.yaw) * DT
            self.y += out.v * math.sin(self.yaw) * DT
            self.yaw += out.omega * DT
            self.log.append((self.t, out))
            if until is not None and until(out):
                return out
        return self.log[-1][1]

    def states(self):
        return [o.state for _, o in self.log]


def test_stops_at_red_line_and_waits_for_grant():
    s = VisionSim()
    out = s.run(12)
    assert out.state == JUNCTION_STOP and out.v == 0.0 and '허가 없음' in out.reason
    assert out.edge_id == 'vision:approach'
    assert 1.0 <= s.x <= 1.15                                     # 빨간 선 바로 앞에서 섰다
    x_stop = s.x
    s.run(10)
    assert s.x == pytest.approx(x_stop, abs=1e-6)                 # 허가 없이는 영영 안 간다


def test_grant_runs_maneuver_then_lane_search_toward_turn_then_cruise():
    s = VisionSim()
    s.run(12)
    x_stop = s.x
    s.clear = 1
    s.quality = QUALITY_LOST                                     # 동작 직후 카메라가 아직 차선을 못 본다
    s.run(15, until=lambda o: o.state == LANE_SEARCH)
    assert JUNCTION_PASS in s.states()
    assert s.x == pytest.approx(x_stop + 0.18, abs=0.015)
    assert s.y == pytest.approx(-0.10, abs=0.015)
    assert math.degrees(s.yaw) == pytest.approx(-90.0, abs=3.0)
    out = s.run(0.5)
    assert out.state == LANE_SEARCH and out.omega < 0            # 방금 우회전했으니 오른쪽으로 찾는다
    s.quality = QUALITY_BOTH
    out = s.run(2.0)
    assert out.state == CRUISE and out.v > 0 and out.edge_id == 'vision:after_junction'


def test_obstacle_during_maneuver_pauses_and_resumes():
    s = VisionSim()
    s.run(12)
    x_stop = s.x
    s.clear = 1
    s.run(1.0)                                                   # 동작 시작 (직진 중)
    assert s.log[-1][1].state == JUNCTION_PASS
    s.us = 0.05                                                  # 앞에 장애물
    out = s.run(3.0)
    assert out.state == OBSTACLE_WAIT and out.v == 0.0
    s.us = 1.0
    s.run(15, until=lambda o: o.state == CRUISE)
    assert s.x == pytest.approx(x_stop + 0.18, abs=0.015) and s.y == pytest.approx(-0.10, abs=0.015)


def test_counts_white_stop_lines_after_junction_and_arrives_on_nth():
    s = VisionSim(steps=[('straight', 0.20)], stop_line_count=2)
    s.stop_line = True                                           # 출발 지점 정지선 — 교차로 전이라 세지 않는다
    s.run(1.0)
    s.stop_line = False
    s.run(12)
    s.clear = 1
    s.run(10, until=lambda o: o.state == CRUISE)
    s.run(1.0)
    s.stop_line = True                                           # 뒤 정지선 — 첫째, 지나간다
    out = s.run(1.0)
    assert out.state == CRUISE and out.route_idx == 1
    s.stop_line = False
    s.run(2.0)
    s.stop_line = True                                           # 앞 정지선 — 둘째, 도착
    out = s.run(0.5)
    assert out.state == ARRIVED and out.v == 0.0 and out.edge_id == 'vision:arrived'
    s.run(2.0)
    assert s.log[-1][1].state == ARRIVED


def test_same_seq_updates_count_new_seq_resets_mission():
    s = VisionSim(steps=[('straight', 0.20)], stop_line_count=0)
    s.run(12)
    s.clear = 1
    s.d.set_plan(s.seq, [('straight', 0.20)], 1)                 # 허가와 함께 개수만 갱신 (진행 유지)
    s.run(10, until=lambda o: o.state == CRUISE)
    assert s.d._junction_passed
    s.stop_line = True
    assert s.run(0.5).state == ARRIVED
    s.d.set_plan(s.seq + 1, RIGHT, 1)                            # 새 미션 — 처음부터
    assert not s.d._junction_passed and not s.d._arrived and s.d.clear_until == 0 and s.d.route_seq == s.seq + 1


def test_without_plan_lane_only_stops_at_red_line_then_drives():
    """계획 없음(테스트 주행): 빨간 선은 교차로 절차 없이 RED_LINE_STOP 1 s 후 차선 주행 (2026-09-30)."""
    from pinky_fleet_agent.drive_fsm import RED_LINE_STOP
    p = DriverParams()
    p.lane_only = True
    d = LaneDriver(p)
    d.set_command(CMD_START, 0.0)
    t = 0.0
    states = []
    for i in range(120):
        t += DT
        d.set_lane_path(t, t, QUALITY_BOTH, 0.0, False, red_line=20 <= i < 30)
        states.append(d.tick(t, 0.0, 0.0, 0.0).state)
    assert RED_LINE_STOP in states and JUNCTION_STOP not in states and states[-1] == CRUISE
    assert d.mission_stage == 'lane_only'


def test_lane_agent_params_expose_maneuver_group():
    import yaml
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg = yaml.safe_load(open(os.path.join(here, 'params', 'lane_agent.yaml'), encoding='utf-8'))
    params = cfg['pinky_lane_agent']['ros__parameters']
    assert set(params['maneuver']) <= set(ManeuverParams.__dataclass_fields__)
    assert params['guard']['us_stop'] == pytest.approx(0.10)     # 앞면에서 10 cm
    assert params['guard']['box_x'] == pytest.approx(0.18)       # 라이다→앞면 8 cm + 10 cm
    src = open(os.path.join(here, 'pinky_fleet_agent', 'lane_agent_node.py'), encoding='utf-8').read()
    assert "'maneuver': p.maneuver" in src


# ------------------------------------------------------------ 벽 ArUco 마커 도착 · 장애물

def _to_after_junction(s):
    s.run(12)
    s.clear = 1
    s.run(10, until=lambda o: o.state == CRUISE)
    assert s.d._junction_passed
    return s.x


def test_marker_arrival_at_wall_distance():
    s = VisionSim(steps=[('straight', 0.20)], stop_line_count=0, goal_marker_id=1, arrive_distance=0.15)
    wall = [None]
    s.markers = lambda: {} if wall[0] is None else {1: wall[0] - s.x, 3: 0.12}   # 다른 id(3)는 무시
    x0 = _to_after_junction(s)
    wall[0] = x0 + 0.60
    out = s.run(15, until=lambda o: o.state == ARRIVED)
    assert out.state == ARRIVED and '벽 마커 1' in out.reason and out.edge_id == 'vision:arrived'
    assert 0.10 <= wall[0] - s.x <= 0.15
    s.run(2.0)
    assert s.log[-1][1].state == ARRIVED and s.log[-1][1].v == 0.0


def test_marker_before_junction_is_not_arrival():
    s = VisionSim(steps=[('straight', 0.20)], goal_marker_id=2)
    s.markers = lambda: {2: 0.05}                                  # 출발 지점 쪽 마커가 가깝게 보여도
    out = s.run(12)
    assert out.state == JUNCTION_STOP and not s.d._arrived


def test_marker_slows_down_near_wall():
    s = VisionSim(steps=[('straight', 0.20)], goal_marker_id=1, arrive_distance=0.15)
    wall = [None]
    s.markers = lambda: {} if wall[0] is None else {1: wall[0] - s.x}
    x0 = _to_after_junction(s)
    s.run(2.0)
    v_far = s.log[-1][1].v
    wall[0] = s.x + 0.30
    s.run(0.6)
    assert s.log[-1][1].v < v_far * 0.75


def test_obstacle_waits_then_resumes_when_not_arrival():
    s = VisionSim(steps=[('straight', 0.20)], goal_marker_id=1)
    _to_after_junction(s)
    s.us = 0.08
    out = s.run(2.0)
    assert out.state == OBSTACLE_WAIT and out.v == 0.0 and not s.d._arrived
    s.us = 1.0
    out = s.run(2.0)
    assert out.state == CRUISE and out.v > 0


def test_arrive_on_obstacle_after_same_goal_robot_arrived():
    s = VisionSim(steps=[('straight', 0.20)], goal_marker_id=1)
    _to_after_junction(s)
    s.d.set_plan(s.seq, [('straight', 0.20)], 0, goal_marker_id=1, arrive_on_obstacle=True)   # 관제: 앞 로봇 도착
    assert s.d._junction_passed                                    # 같은 seq — 진행 유지
    s.us = 0.08
    out = s.run(1.0, until=lambda o: o.state == ARRIVED)
    assert out.state == ARRIVED and '앞 로봇' in out.reason
    s.us = 1.0                                                     # 앞 로봇이 치워져도 다시 가지 않는다
    s.run(2.0)
    assert s.log[-1][1].state == ARRIVED and s.log[-1][1].v == 0.0


def test_obstacle_with_goal_marker_close_is_arrival():
    s = VisionSim(steps=[('straight', 0.20)], goal_marker_id=1, arrive_distance=0.15)
    _to_after_junction(s)
    s.markers = lambda: {1: 0.30}
    s.us = 0.08
    out = s.run(1.0, until=lambda o: o.state == ARRIVED)
    assert out.state == ARRIVED and '장애물' in out.reason


def test_skip_clearance_goes_after_stop_seconds():
    s = VisionSim(steps=[('straight', 0.20)], skip_clearance=True)
    s.run(15, until=lambda o: o.state == JUNCTION_PASS)
    stops = [t for t, o in s.log if o.state == JUNCTION_STOP]
    assert s.log[-1][1].state == JUNCTION_PASS and s.clear == 0
    assert 0.95 <= max(stops) - min(stops) <= 1.2


def test_ultrasonic_only_ignores_lidar():
    from pinky_fleet_agent.obstacle_guard import GuardParams, ObstacleGuard
    g = ObstacleGuard(GuardParams(use_lidar=False, us_stop=0.10))
    g.update_scan([0.10] * 3, -0.1, 0.1)                           # 라이다 박스 안
    g.update_us(0.50)
    assert g.step(0.0) == (False, '') and g.step(0.1) == (False, '')
    assert g.lidar_min == pytest.approx(0.10)                      # 기록은 한다
    g.update_us(0.09)
    g.update_us(0.09)
    g.step(0.2)
    blocked, reason = g.step(0.3)
    assert blocked and '초음파' in reason and '라이다' not in reason
    g2 = ObstacleGuard(GuardParams(use_lidar=True))
    g2.update_scan([0.10] * 3, -0.1, 0.1)
    g2.step(0.0)
    assert g2.step(0.1)[0]


def test_lane_only_launch_uses_ultrasonic_only():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xml = open(os.path.join(here, 'launch', 'lane_only.launch.xml'), encoding='utf-8').read()
    assert '<arg name="use_lidar" default="False"' in xml
    assert '<param name="guard.use_lidar" value="$(var use_lidar)"/>' in xml


class ExitLineSim(VisionSim):
    """입구 선(x ≥ red_x, 통과 전)에 더해, red_fn(sim) 이 참이면 빨간 선이 보인다 (출구 선)."""

    def __init__(self, red_fn, **kw):
        super().__init__(**kw)
        self.red_fn = red_fn

    def run(self, seconds, until=None):
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            if int(round(self.t / DT)) % 2 == 0:
                self.d.set_command(CMD_CLEARANCE, self.t, route_seq=self.seq, clear_until=self.clear)
            if int(round(self.t / DT)) % 3 == 0:
                red = (self.x >= self.red_x and not self.d._junction_passed and self.d._maneuver is None) \
                    or self.red_fn(self)
                self.d.set_lane_path(self.t, self.t, self.quality, 0.0, False, red_line=red)
            self.d.update_us(self.us)
            self.d.update_odom(self.t, self.x, self.y, self.yaw)
            out = self.d.tick(self.t, 0.0, 0.0, 0.0)
            self.x += out.v * math.cos(self.yaw) * DT
            self.y += out.v * math.sin(self.yaw) * DT
            self.yaw += out.omega * DT
            self.log.append((self.t, out))
            if until is not None and until(out):
                return out
        return self.log[-1][1]


def test_plan_exit_line_after_junction_stops_once():
    from pinky_fleet_agent.drive_fsm import RED_LINE_STOP
    after = {'x0': None}

    def exit_line(sim):                                   # 교차로를 지난 뒤 0.2 m 달렸을 때 출구 선
        if not sim.d._junction_passed:
            return False
        if after['x0'] is None:
            after['x0'] = (sim.x, sim.y)
        return 0.20 <= math.hypot(sim.x - after['x0'][0], sim.y - after['x0'][1]) < 0.24

    s = ExitLineSim(exit_line)
    s.run(12)
    assert s.log[-1][1].state == JUNCTION_STOP                      # 입구 선: 기존 절차 (허가 대기)
    s.clear = 1
    s.run(20)
    states = s.states()
    assert states.count(JUNCTION_STOP) > 0 and RED_LINE_STOP in states
    runs = sum(1 for i in range(1, len(states)) if states[i] == RED_LINE_STOP != states[i - 1])
    assert runs == 1 and states[-1] == CRUISE


def test_plan_exit_line_seen_during_maneuver_stops_after_maneuver():
    from pinky_fleet_agent.drive_fsm import RED_LINE_STOP
    s = ExitLineSim(lambda sim: sim.d._maneuver is not None and not sim.d._maneuver.done and sim.x > 1.12)
    s.run(12)
    s.clear = 1
    s.run(20)
    states = s.states()
    i_pass = max(i for i, st in enumerate(states) if st == JUNCTION_PASS)
    assert RED_LINE_STOP in states[i_pass + 1:i_pass + 3]           # 동작이 끝난 바로 다음 틱에 선다
    assert JUNCTION_PASS not in states[i_pass + 1:] and states[-1] == CRUISE
