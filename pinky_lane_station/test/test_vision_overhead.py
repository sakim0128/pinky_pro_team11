"""비전 미션 항공뷰 관제 — 항공뷰 좌표를 코스 중심선에 겹쳐 교차로 통행권 거리(남은 거리 짧은 로봇 먼저)와
차선 이탈 보정(3 cm 보정 · 8 cm 경고, 교차로 구간 밖에서만)을 만든다. 항공뷰가 끊기면 선착순으로 돌아간다."""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

from pinky_lane_station.vision_mission import (  # noqa: E402
    DRIVE_CRUISE, DRIVE_JUNCTION_PASS, DRIVE_JUNCTION_STOP, JunctionArbiter, ScenarioRun, load_vision_config,
)
from pinky_lane_station.vision_overhead import OverheadParams, OverheadTrack  # noqa: E402
from pinky_lane_station.vision_pose import load_vision_course  # noqa: E402

COURSE = os.path.join(PKG, 'config', 'vision_course.yaml')
CFG = os.path.join(PKG, 'config', 'vision_mission.yaml')


@pytest.fixture(scope='module')
def course():
    return load_vision_course(COURSE)


def at(route, s, left=0.0, dyaw=0.0):
    """코스 s 지점에서 왼쪽으로 left(m) 옮긴 자세."""
    x, y, yaw = route.point_at(s)
    return x - left * math.sin(yaw), y + left * math.cos(yaw), yaw + dyaw


# ------------------------------------------------------------ 코스 위 위치 · 이탈

def test_lateral_and_distance_to_entry(course):
    r = course.route('2', '1')
    tr = OverheadTrack(r)
    tr.feed(*at(r, 0.30, left=0.05), t=0.0)
    ev = tr.evaluate(0.1, DRIVE_CRUISE)
    assert ev['fresh'] and ev['s'] == pytest.approx(0.30, abs=0.01)
    assert ev['lateral'] == pytest.approx(0.05, abs=0.003)
    assert ev['dist_to_entry'] == pytest.approx(r.s_entry - r.params.red_stop_back - 0.30, abs=0.01)
    assert ev['correct'] and not ev['warn'] and not ev['in_junction']
    tr.feed(*at(r, 0.32, left=-0.09), t=0.2)
    ev = tr.evaluate(0.25, DRIVE_CRUISE)
    assert ev['lateral'] == pytest.approx(-0.09, abs=0.003) and ev['correct'] and ev['warn']


def test_heading_error_sign(course):
    r = course.route('3', '1')
    tr = OverheadTrack(r)
    tr.feed(*at(r, 0.4, dyaw=0.2), t=0.0)
    assert tr.evaluate(0.0)['heading'] == pytest.approx(0.2, abs=0.05)


def test_correction_deadband_and_hysteresis(course):
    r = course.route('2', '1')
    tr = OverheadTrack(r)
    tr.feed(*at(r, 0.3, left=0.02), t=0.0)
    assert not tr.evaluate(0.0)['correct']                       # 3 cm 미만 — 보정 없음
    tr.feed(*at(r, 0.31, left=0.035), t=0.1)
    assert tr.evaluate(0.1)['correct']
    tr.feed(*at(r, 0.32, left=0.02), t=0.2)
    assert tr.evaluate(0.2)['correct']                           # 보정 중엔 1.5 cm 아래로 와야 멈춘다
    tr.feed(*at(r, 0.33, left=0.01), t=0.3)
    assert not tr.evaluate(0.3)['correct']


def test_no_correction_inside_junction_or_when_not_cruising(course):
    r = course.route('3', '1')
    tr = OverheadTrack(r)
    mid = (r.s_entry + r.s_exit) / 2
    tr.feed(*at(r, mid, left=0.06), t=0.0)
    ev = tr.evaluate(0.0, DRIVE_CRUISE)
    assert ev['in_junction'] and not ev['correct'] and not ev['warn'] and ev['dist_to_entry'] == 0.0
    tr2 = OverheadTrack(r)
    tr2.feed(*at(r, 0.3, left=0.06), t=0.0)
    assert not tr2.evaluate(0.0, DRIVE_JUNCTION_STOP)['correct']
    assert tr2.evaluate(0.0, DRIVE_CRUISE)['correct']


def test_after_junction_has_no_entry_distance_but_still_corrects(course):
    r = course.route('2', '1')
    tr = OverheadTrack(r)
    tr.feed(*at(r, r.s_exit + 0.30, left=0.05), t=0.0)
    ev = tr.evaluate(0.0, DRIVE_CRUISE)
    assert ev['dist_to_entry'] is None and ev['correct'] and not ev['in_junction']


def test_stale_overhead_gives_nothing(course):
    r = course.route('2', '1')
    tr = OverheadTrack(r, OverheadParams(stale_after=0.5))
    tr.feed(*at(r, 0.3, left=0.06), t=1.0)
    assert tr.evaluate(1.4)['correct']
    ev = tr.evaluate(1.6)
    assert not ev['fresh'] and ev['dist_to_entry'] is None and not ev['correct']


def test_far_off_course_warns_without_correction(course):
    r = course.route('2', '1')
    tr = OverheadTrack(r)
    tr.feed(*at(r, 0.3, left=0.40), t=0.0)
    ev = tr.evaluate(0.0)
    assert ev['warn'] and not ev['correct'] and ev['dist_to_entry'] is None


def test_projection_window_does_not_jump_along_course(course):
    """창 안에서 따라간다 — 앞으로 조금씩 옮기면 s 도 조금씩."""
    r = course.route('1', '3')
    tr = OverheadTrack(r)
    prev = None
    for k in range(40):
        s = 0.05 + 0.05 * k
        tr.feed(*at(r, s, left=0.01), t=k * 0.1)
        assert tr.s == pytest.approx(s, abs=0.02)
        if prev is not None:
            assert tr.s > prev
        prev = tr.s


# ------------------------------------------------------------ 통행권 (남은 거리)

def test_closer_robot_gets_pass_before_stopping():
    a = JunctionArbiter(domains={'pinky1': 10, 'pinky2': 11})
    assert a.tick(0.0, {'pinky1': 0.50, 'pinky2': 0.30}) == 'pinky2' and a.rule == 'distance'
    assert a.tick(0.1, {'pinky1': 0.45, 'pinky2': 0.25}) is None


def test_pass_moves_to_closer_robot_until_holder_stops():
    a = JunctionArbiter(domains={'pinky1': 10, 'pinky2': 11})
    a.tick(0.0, {'pinky1': 0.50, 'pinky2': 0.30})
    assert a.tick(1.0, {'pinky1': 0.10, 'pinky2': 0.25}) == 'pinky1'   # pinky2 가 막혔다 — 더 가까운 로봇에게
    assert a.grant_order == ['pinky1']
    a.request('pinky1', 2.0, 10)                                     # 입구에 섰다 → 잠김
    assert 'pinky1' in a.locked
    assert a.tick(2.1, {'pinky1': 0.0, 'pinky2': 0.0}) is None        # 잠긴 통행권은 넘기지 않는다
    a.release('pinky1')
    assert a.tick(3.0, {'pinky2': 0.0}) == 'pinky2'


def test_distance_tie_goes_to_first_stopped_then_domain():
    a = JunctionArbiter(domains={'pinky1': 10, 'pinky2': 11})
    assert a.tick(0.0, {'pinky1': 0.31, 'pinky2': 0.30}) == 'pinky1'  # 3 cm 안 — domain_id 작은 쪽
    b = JunctionArbiter(domains={'pinky1': 10, 'pinky2': 11})
    b.request('pinky2', 0.0, 11)
    assert b.tick(0.0, {'pinky1': 0.02}) == 'pinky2'                  # 서 있는 로봇(거리 0)이 먼저


def test_stopped_robot_without_overhead_counts_as_zero():
    a = JunctionArbiter(domains={'pinky1': 10, 'pinky2': 11})
    a.request('pinky2', 0.0, 11)                                     # 항공뷰 없이 입구에 선 로봇
    assert a.tick(0.0, {'pinky1': 0.40, 'pinky2': None}) == 'pinky2'
    assert 'pinky2' in a.locked


def test_without_overhead_falls_back_to_first_stop():
    a = JunctionArbiter(tie_window=0.5, domains={'pinky1': 10, 'pinky2': 11})
    a.request('pinky2', 1.0, 11)
    assert a.tick(1.2, {'pinky1': None, 'pinky2': None}) is None     # 같은 순간 보고를 기다린다 (예전 규칙)
    assert a.tick(1.6, {}) == 'pinky2' and a.rule == 'first_stop'


def test_scenario_run_uses_distance_and_ignores_passed_or_not_departed():
    cfg = load_vision_config(CFG)
    run = ScenarioRun(cfg, 's1', t0=0.0, first_seq=1)
    for n in run.robots:
        run.on_status(n, DRIVE_CRUISE, run.seq[n], 'vision:approach', 0.5)
    run.tick(1.0, {'pinky1': 0.60, 'pinky2': 0.20})
    assert run.clearance('pinky2') == 1 and run.clearance('pinky1') == 0
    st = run.status(1.0)
    assert st['holder'] == 'pinky2' and st['grant_rule'] == 'distance' and not st['holder_locked']
    run.on_status('pinky2', DRIVE_JUNCTION_STOP, run.seq['pinky2'], 'vision:approach', 2.0)
    assert run.status(2.0)['holder_locked']
    run.on_status('pinky2', DRIVE_JUNCTION_PASS, run.seq['pinky2'], 'vision:junction', 3.0)
    run.on_status('pinky2', DRIVE_CRUISE, run.seq['pinky2'], 'vision:after_junction', 6.0)
    assert run.arbiter.holder is None
    run.tick(6.1, {'pinky1': 0.05, 'pinky2': 0.0})                   # 지난 로봇은 다시 받지 않는다
    assert run.arbiter.holder == 'pinky1' and run.clearance('pinky2') == 1


def test_scenario_run_waits_for_depart_delay():
    cfg = load_vision_config(CFG)
    run = ScenarioRun(cfg, 's2', t0=0.0, first_seq=1)                # pinky2 는 10 s 뒤 출발
    run.tick(1.0, {'pinky1': 0.9, 'pinky2': 0.1})
    assert run.arbiter.holder == 'pinky1'
