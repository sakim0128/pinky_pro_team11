"""비전 미션 관제 코어 — 설정 검증, 교차로 통행권(선착순·동점 domain_id), 반납 시점, 정지선 수, 출발 지연,
그리고 로봇 드라이버 두 대(LaneDriver, lane_only + 계획)와 묶은 시나리오 폐루프."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'pinky_fleet_agent'))

from pinky_lane_station.vision_mission import (  # noqa: E402
    DRIVE_ARRIVED, DRIVE_CRUISE, DRIVE_JUNCTION_PASS, DRIVE_JUNCTION_STOP, JunctionArbiter, ScenarioRun,
    VisionConfigError, load_vision_config, vision_config_from_dict,
)

CFG_PATH = os.path.join(os.path.dirname(HERE), 'config', 'vision_mission.yaml')


def cfg():
    return load_vision_config(CFG_PATH)


# ------------------------------------------------------------ 설정

def test_config_scenarios_match_course():
    c = cfg()
    s1, s2 = c.scenarios['s1'], c.scenarios['s2']
    assert {(n, p.start, p.goal) for n, p in s1.robots.items()} == {('pinky1', '2', '1'), ('pinky2', '3', '1')}
    assert {(n, p.start, p.goal) for n, p in s2.robots.items()} == {('pinky1', '1', '2'), ('pinky2', '1', '3')}
    assert s1.stop_lines_by_order == [2, 1]                        # 먼저 지난 로봇은 앞(둘째) 정지선
    assert s2.robots['pinky2'].depart_delay == 10.0 and s2.robots['pinky1'].depart_delay == 0.0
    # 방향: 3→1 우회전(−), 1→3 좌회전(+), 2↔1 직진
    assert [v for k, v in c.maneuvers['right_3_to_1'] if k == 'turn'] == [-90.0]
    assert [v for k, v in c.maneuvers['left_1_to_3'] if k == 'turn'] == [90.0]
    assert all(k == 'straight' for k, _ in c.maneuvers['straight_2_to_1'] + c.maneuvers['straight_1_to_2'])
    assert c.robots == {'pinky1': 10, 'pinky2': 11}


@pytest.mark.parametrize('patch, word', [
    ({'maneuvers': {'m': [{'jump': 1}]}}, '알 수 없는'),
    ({'maneuvers': {'m': [{'straight': 5.0}]}}, '범위'),
    ({'scenarios': {'x': {'robots': {'pinky9': {'maneuver': 'm'}}}}}, '모르는 로봇'),
    ({'scenarios': {'x': {'robots': {'pinky1': {'maneuver': 'nope', 'stop_lines': 1}}}}}, 'maneuver'),
    ({'scenarios': {'x': {'robots': {'pinky1': {'maneuver': 'm'}}}}}, 'stop_lines'),
    ({'robots': {'pinky1': {'domain_id': 10}, 'pinky2': {'domain_id': 10}}}, '겹친다'),
])
def test_config_rejects_mistakes(patch, word):
    base = {'robots': {'pinky1': {'domain_id': 10}, 'pinky2': {'domain_id': 11}},
            'maneuvers': {'m': [{'straight': 0.2}]},
            'scenarios': {'ok': {'robots': {'pinky1': {'maneuver': 'm', 'stop_lines': 1}}}}}
    base.update(patch)
    with pytest.raises(VisionConfigError, match=word):
        vision_config_from_dict(base)


# ------------------------------------------------------------ 통행권

def test_first_to_stop_goes_first_even_with_larger_domain_id():
    a = JunctionArbiter(tie_window=0.5)
    a.request('pinky2', 10.0, 11)
    a.request('pinky1', 11.0, 10)
    assert a.tick(10.2) is None                     # 동점 창 안 — 조금 더 기다린다
    assert a.tick(10.6) == 'pinky2'
    assert a.tick(12.0) is None                     # 쥔 로봇이 있으면 다음은 없다
    assert a.release('pinky2') and a.tick(12.1) == 'pinky1'
    assert a.grant_order == ['pinky2', 'pinky1']


def test_simultaneous_stop_smaller_domain_id_first():
    a = JunctionArbiter(tie_window=0.5)
    a.request('pinky2', 10.0, 11)
    a.request('pinky1', 10.3, 10)
    assert a.tick(10.6) == 'pinky1'


def test_request_is_idempotent_and_released_robot_cannot_rerequest():
    a = JunctionArbiter(tie_window=0.0)
    a.request('pinky1', 1.0, 10)
    a.request('pinky1', 5.0, 10)
    assert a.requests['pinky1'][0] == 1.0
    assert a.tick(1.0) == 'pinky1'
    a.request('pinky1', 6.0, 10)
    assert not a.requests
    a.release('pinky1')
    a.request('pinky1', 7.0, 10)
    assert not a.requests and 'pinky1' in a.passed


def test_release_only_when_holder_returns_to_lane_driving_after_junction():
    run = ScenarioRun(cfg(), 's1', t0=0.0, first_seq=1)
    s1, s2 = run.seq['pinky1'], run.seq['pinky2']
    run.on_status('pinky1', DRIVE_JUNCTION_STOP, s1, 'vision:approach', 1.0)
    run.tick(2.0)
    assert run.arbiter.holder == 'pinky1' and run.clearance('pinky1') == 1 and run.clearance('pinky2') == 0
    run.on_status('pinky2', DRIVE_JUNCTION_STOP, s2, 'vision:approach', 2.5)
    run.on_status('pinky1', DRIVE_JUNCTION_PASS, s1, 'vision:junction', 3.0)
    run.on_status('pinky1', 11, s1, 'vision:after_junction', 4.0)            # 차선 탐색 — 아직 반납 아님
    run.tick(4.1)
    assert run.arbiter.holder == 'pinky1'
    run.on_status('pinky1', DRIVE_CRUISE, s1, 'vision:approach', 4.5)        # 교차로 전 CRUISE 는 반납 아님
    assert run.arbiter.holder == 'pinky1'
    run.on_status('pinky1', DRIVE_CRUISE, s1, 'vision:after_junction', 5.0)  # 차선 주행 복귀 → 반납
    run.tick(5.0)
    assert run.arbiter.holder == 'pinky2' and run.clearance('pinky1') == 1 and run.clearance('pinky2') == 1


def test_stop_lines_by_grant_order_and_wrong_seq_ignored():
    run = ScenarioRun(cfg(), 's1', t0=0.0, first_seq=40)
    assert run.plan_fields('pinky1')['stop_line_count'] == 0             # 아직 모른다
    run.on_status('pinky2', DRIVE_JUNCTION_STOP, 999, 'vision:approach', 1.0)   # 옛 번호 — 무시
    assert not run.arbiter.requests
    run.on_status('pinky2', DRIVE_JUNCTION_STOP, run.seq['pinky2'], 'vision:approach', 1.0)
    assert run.tick(2.0) == ['pinky2']
    assert run.plan_fields('pinky2')['stop_line_count'] == 2
    run.on_status('pinky1', DRIVE_JUNCTION_STOP, run.seq['pinky1'], 'vision:approach', 3.0)
    run.on_status('pinky2', DRIVE_CRUISE, run.seq['pinky2'], 'vision:after_junction', 6.0)
    assert run.tick(6.0) == ['pinky1'] and run.plan_fields('pinky1')['stop_line_count'] == 1


def test_scenario2_depart_delay_and_explicit_stop_lines():
    run = ScenarioRun(cfg(), 's2', t0=100.0, first_seq=1)
    assert run.due('pinky1', 100.0) and not run.due('pinky2', 109.9) and run.due('pinky2', 110.0)
    assert run.plan_fields('pinky1')['stop_line_count'] == 1 and run.plan_fields('pinky2')['stop_line_count'] == 1
    f = run.plan_fields('pinky2')
    assert f['step_kind'] == ['straight', 'turn', 'straight'] and f['step_value'] == [0.18, 90.0, 0.10]
    assert f['linear_speed'] == 0.08 and f['maneuver'] == 'left_1_to_3'
    run.on_status('pinky1', DRIVE_ARRIVED, run.seq['pinky1'], 'vision:arrived', 130.0)
    assert not run.done
    run.on_status('pinky2', DRIVE_ARRIVED, run.seq['pinky2'], 'vision:arrived', 140.0)
    assert run.done and run.status(140.0)['done']


def test_unknown_scenario():
    with pytest.raises(VisionConfigError):
        ScenarioRun(cfg(), 's9', 0.0)


# ------------------------------------------------------------ 폐루프: 관제 코어 + 로봇 드라이버 두 대

class Bot:
    """lane_only 드라이버 + 1차원 차선. red_x 에서 빨간 선이 보이고, 교차로 통과 뒤 lines 거리(m)에 흰 정지선."""

    def __init__(self, red_x, lines=(0.30, 0.70)):
        from pinky_fleet_agent.lane_driver import DriverParams, LaneDriver
        p = DriverParams()
        p.lane_only = True
        self.d = LaneDriver(p)
        self.x = self.y = self.yaw = 0.0
        self.red_x = red_x
        self.lines = lines
        self.pass_travel = None
        self.out = None

    def step(self, t, dt, tick_no):
        from pinky_fleet_agent.lane_control import QUALITY_BOTH
        d = self.d
        if d._junction_passed and self.pass_travel is None:
            self.pass_travel = d._travelled
        stop = False
        if self.pass_travel is not None:
            after = d._travelled - self.pass_travel
            stop = any(L <= after <= L + 0.05 for L in self.lines)
        if tick_no % 3 == 0:
            d.set_lane_path(t, t, QUALITY_BOTH, 0.0, False,
                            red_line=self.x >= self.red_x and not d._junction_passed, stop_line=stop)
        d.update_odom(t, self.x, self.y, self.yaw)
        self.out = d.tick(t, 0.0, 0.0, 0.0)
        self.x += self.out.v * math.cos(self.yaw) * dt
        self.y += self.out.v * math.sin(self.yaw) * dt
        self.yaw += self.out.omega * dt


def run_scenario(name, red):
    from pinky_fleet_agent.lane_driver import CMD_CLEARANCE, CMD_START
    c = cfg()
    run = ScenarioRun(c, name, t0=0.0, first_seq=1)
    bots = {n: Bot(red[n]) for n in run.robots}
    for n, b in bots.items():
        f = run.plan_fields(n)
        b.d.set_plan(f['seq'], list(zip(f['step_kind'], f['step_value'])), f['stop_line_count'],
                     f['linear_speed'], f['angular_speed'])
    dt, t = 0.05, 0.0
    started = set()
    history = []
    for i in range(int(90 / dt)):
        t += dt
        if i % 2 == 0:                                          # 관제 10 Hz
            for n in run.tick(t):
                f = run.plan_fields(n)
                bots[n].d.set_plan(f['seq'], list(zip(f['step_kind'], f['step_value'])), f['stop_line_count'])
            for n, b in bots.items():
                if run.due(n, t) and n not in started:
                    b.d.set_command(CMD_START, t)
                    started.add(n)
                b.d.set_command(CMD_CLEARANCE, t, route_seq=run.seq[n], clear_until=run.clearance(n))
        for n, b in bots.items():
            b.step(t, dt, i)
            run.on_status(n, b.out.state, b.d.route_seq, b.out.edge_id, t)
        history.append((t, {n: (b.out.state, b.x) for n, b in bots.items()}, run.arbiter.holder))
        if run.done:
            break
    return run, bots, history


def test_scenario1_closed_loop_first_arrival_passes_first_and_other_waits():
    """pinky2(3번) 가 먼저 빨간 선에 닿는다 — domain 이 커도 먼저 지나고, pinky1 은 그동안 선다."""
    run, bots, history = run_scenario('s1', red={'pinky1': 1.0, 'pinky2': 0.7})
    assert run.done, run.status(history[-1][0])
    assert run.arbiter.grant_order == ['pinky2', 'pinky1']
    # pinky2 가 통행권을 쥔 동안 pinky1 은 교차로 정지 상태로 서 있었다
    waiting = [(t, s) for t, s, holder in history if holder == 'pinky2' and s['pinky1'][0] == DRIVE_JUNCTION_STOP]
    assert waiting and max(t for t, _ in waiting) - min(t for t, _ in waiting) > 1.0
    xs = [s['pinky1'][1] for _, s in waiting]
    assert max(xs) - min(xs) < 1e-6
    # 먼저 지난 로봇은 둘째(앞) 정지선, 나중 로봇은 첫째(뒤) 정지선에서 섰다
    assert run.stop_lines == {'pinky2': 2, 'pinky1': 1}
    assert bots['pinky2'].d._stop_lines_seen == 2 and bots['pinky1'].d._stop_lines_seen == 1
    # 3→1 은 우회전: pinky2 는 오른쪽(−y)으로 돌아 나갔다, 2→1 직진인 pinky1 은 y 그대로
    assert bots['pinky2'].y < -0.05 and abs(bots['pinky1'].y) < 1e-6


def test_scenario1_simultaneous_arrival_smaller_domain_first():
    run, _, _ = run_scenario('s1', red={'pinky1': 1.0, 'pinky2': 1.0})
    assert run.done and run.arbiter.grant_order == ['pinky1', 'pinky2']


def test_scenario2_closed_loop_sequential_departure_no_meeting():
    run, bots, history = run_scenario('s2', red={'pinky1': 1.0, 'pinky2': 1.0})
    assert run.done
    first_move = {n: next(t for t, s, _ in history if s[n][1] > 0.01) for n in ('pinky1', 'pinky2')}
    assert first_move['pinky2'] - first_move['pinky1'] >= 9.5            # 10 s 간격
    # 통행권 대기 없이 각자 지나갔다(만나지 않는다)
    assert not any(h is not None and s['pinky1'][0] == DRIVE_JUNCTION_STOP and s['pinky2'][0] == DRIVE_JUNCTION_STOP
                   and t > 12 for t, s, h in history)
    assert bots['pinky2'].y > 0.05                                      # 1→3 좌회전
    assert run.stop_lines == {'pinky1': 1, 'pinky2': 1}
