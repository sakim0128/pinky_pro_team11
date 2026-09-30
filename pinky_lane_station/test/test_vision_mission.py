"""비전 미션 관제 코어 — 설정 검증, 경로별 방향, 교차로 통행권(선착순·동점 domain_id), 반납 시점, 도착(벽 마커·정지선),
출발 지연, 웹 직접 설정·저장, 그리고 로봇 드라이버(LaneDriver, lane_only + 계획)와 묶은 시나리오 폐루프."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'pinky_fleet_agent'))

import yaml  # noqa: E402

from pinky_lane_station.vision_mission import (  # noqa: E402
    DRIVE_ARRIVED, DRIVE_CRUISE, DRIVE_JUNCTION_PASS, DRIVE_JUNCTION_STOP, DRIVE_OBSTACLE_WAIT, JunctionArbiter,
    ScenarioRun, VisionConfigError, course_dict, delete_user_scenario, load_user_scenarios, load_vision_config,
    save_user_scenario, scenario_from_dict, vision_config_from_dict,
)

CFG_PATH = os.path.join(os.path.dirname(HERE), 'config', 'vision_mission.yaml')


def cfg(mode=None):
    """mode=None: 파일 그대로(벽 마커). 'stop_line': 같은 파일을 흰 정지선 도착으로."""
    if mode is None:
        return load_vision_config(CFG_PATH)
    with open(CFG_PATH, encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    data['arrival']['mode'] = mode
    return vision_config_from_dict(data, CFG_PATH)


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
    assert c.arrival_mode == 'marker' and c.arrive_distance == 0.15 and c.markers == {'1': 1, '2': 2, '3': 3}
    assert {n: p.goal_marker_id for n, p in s2.robots.items()} == {'pinky1': 2, 'pinky2': 3}


def test_routes_give_direction_and_maneuver():
    c = cfg()
    want = {('2', '1'): 'straight', ('1', '2'): 'straight', ('3', '1'): 'right', ('1', '3'): 'left',
            ('2', '3'): 'right', ('3', '2'): 'left'}
    assert {k: v['direction'] for k, v in c.routes.items()} == want
    s = scenario_from_dict(c, 'x', {'robots': {'pinky1': {'start': 2, 'goal': 3}}}, 'custom')
    p = s.robots['pinky1']
    assert p.direction == 'right' and p.steps == c.directions['right'] and p.goal_marker_id == 3
    # 경로 전용 동작이 있으면 그것, 방향을 바꾸면 방향 기본 동작
    s = scenario_from_dict(c, 'x', {'robots': {'pinky1': {'start': 3, 'goal': 1}}})
    assert s.robots['pinky1'].maneuver == 'right_3_to_1'
    s = scenario_from_dict(c, 'x', {'robots': {'pinky1': {'start': 3, 'goal': 1, 'direction': 'straight'}}})
    assert s.robots['pinky1'].direction == 'straight' and s.robots['pinky1'].steps == c.directions['straight']
    # 쓰지 않는 로봇(enabled false)은 빠진다
    s = scenario_from_dict(c, 'x', {'robots': {'pinky1': {'start': 1, 'goal': 2},
                                               'pinky2': {'start': 1, 'goal': 3, 'enabled': False}}})
    assert list(s.robots) == ['pinky1']
    cd = course_dict(c)
    assert cd['points'] == ['1', '2', '3'] and cd['robots'] == ['pinky1', 'pinky2']
    assert {(r['start'], r['goal']): r['direction'] for r in cd['routes']} == want


@pytest.mark.parametrize('robot, word', [
    ({'start': 1, 'goal': 1}, '같다'),
    ({'start': 1, 'goal': 9}, '모르는 지점'),
    ({'start': 1, 'goal': 2, 'direction': 'back'}, '방향'),
    ({'start': 1, 'goal': 2, 'depart_delay': -1}, 'depart_delay'),
])
def test_custom_scenario_rejects_mistakes(robot, word):
    with pytest.raises(VisionConfigError, match=word):
        scenario_from_dict(cfg(), 'x', {'robots': {'pinky1': robot}})


def test_save_load_delete_user_scenarios(tmp_path):
    c = cfg()
    c.user_path = str(tmp_path / 'user.yaml')
    raw = {'label': '3에서 2로', 'robots': {'pinky2': {'start': '3', 'goal': '2', 'depart_delay': 2}}}
    save_user_scenario(c, 'my_run', raw)
    assert c.scenarios['my_run'].source == 'user'
    with pytest.raises(VisionConfigError, match='프리셋'):
        save_user_scenario(c, 's1', raw)
    with pytest.raises(VisionConfigError, match='이름'):
        save_user_scenario(c, 'bad name!', raw)
    c2 = cfg()
    assert load_user_scenarios(c2, c.user_path) == []
    s = c2.scenarios['my_run']
    assert s.source == 'user' and s.label == '3에서 2로' and s.robots['pinky2'].direction == 'left'
    assert s.robots['pinky2'].depart_delay == 2.0
    with pytest.raises(VisionConfigError):
        delete_user_scenario(c2, 's1')                                  # 프리셋은 못 지운다
    delete_user_scenario(c2, 'my_run')
    c3 = cfg()
    load_user_scenarios(c3, c.user_path)
    assert 'my_run' not in c3.scenarios


def test_user_file_skips_bad_and_preset_names(tmp_path):
    path = tmp_path / 'user.yaml'
    path.write_text(yaml.safe_dump({'scenarios': {
        's1': {'robots': {'pinky1': {'start': 1, 'goal': 2}}},
        'bad': {'robots': {'pinky1': {'start': 1, 'goal': 1}}},
        'good': {'robots': {'pinky1': {'start': 1, 'goal': 2}}}}}), encoding='utf-8')
    c = cfg()
    errors = load_user_scenarios(c, str(path))
    assert len(errors) == 2 and c.scenarios['s1'].source == 'preset' and c.scenarios['good'].source == 'user'


def test_single_robot_scenario_skips_clearance():
    c = cfg()
    s = scenario_from_dict(c, 'custom', {'robots': {'pinky1': {'start': 2, 'goal': 1}}}, 'custom')
    run = ScenarioRun(c, s, t0=0.0)
    assert run.skip_clearance and run.plan_fields('pinky1')['skip_clearance'] is True
    run.on_status('pinky1', DRIVE_JUNCTION_STOP, run.seq['pinky1'], 'vision:approach', 1.0)
    run.tick(3.0)
    assert run.arbiter.holder is None and not run.arbiter.requests and run.clearance('pinky1') == 1
    assert ScenarioRun(c, 's1', 0.0).plan_fields('pinky1')['skip_clearance'] is False


def test_same_goal_second_robot_arrives_on_obstacle():
    run = ScenarioRun(cfg(), 's1', t0=0.0)
    f = run.plan_fields('pinky1')
    assert f['goal_marker_id'] == 1 and f['arrive_distance'] == 0.15 and f['arrive_on_obstacle'] is False
    run.on_status('pinky2', DRIVE_ARRIVED, run.seq['pinky2'], 'vision:arrived', 5.0)
    assert run.tick(5.1) == ['pinky1'] and run.plan_fields('pinky1')['arrive_on_obstacle'] is True
    assert run.tick(5.2) == []                                         # 한 번만
    assert run.plan_fields('pinky2')['arrive_on_obstacle'] is False
    run2 = ScenarioRun(cfg(), 's2', t0=0.0)                            # 목적지가 다르면 표시 없음
    run2.on_status('pinky1', DRIVE_ARRIVED, run2.seq['pinky1'], 'vision:arrived', 5.0)
    assert run2.tick(5.1) == []


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
    run = ScenarioRun(cfg('stop_line'), 's1', t0=0.0, first_seq=40)
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
    """lane_only 드라이버 + 1차원 차선. red_x 에서 빨간 선이 보이고, 교차로 통과 뒤 lines 거리(m)에 흰 정지선,
    wall 거리(m)에 벽(목적지 ArUco 마커). blocker() 가 참이면 초음파 8 cm (앞 로봇) 이고 마커도 가린다."""

    def __init__(self, red_x, lines=(0.30, 0.70), wall=0.85):
        from pinky_fleet_agent.lane_driver import DriverParams, LaneDriver
        p = DriverParams()
        p.lane_only = True
        self.d = LaneDriver(p)
        self.x = self.y = self.yaw = 0.0
        self.red_x = red_x
        self.lines = lines
        self.wall = wall
        self.blocker = lambda after: False
        self.pass_travel = None
        self.out = None

    def step(self, t, dt, tick_no):
        from pinky_fleet_agent.lane_control import QUALITY_BOTH
        d = self.d
        if d._junction_passed and self.pass_travel is None:
            self.pass_travel = d._travelled
        stop, markers, blocked = False, {}, False
        if self.pass_travel is not None:
            after = d._travelled - self.pass_travel
            stop = any(L <= after <= L + 0.05 for L in self.lines)
            blocked = self.blocker(after)
            if d._plan and d._plan['goal_marker_id'] >= 0 and not blocked and self.wall - after <= 1.0:
                markers = {d._plan['goal_marker_id']: self.wall - after}
        d.update_us(0.08 if blocked else 1.0)
        if tick_no % 3 == 0:
            d.set_lane_path(t, t, QUALITY_BOTH, 0.0, False,
                            red_line=self.x >= self.red_x and not d._junction_passed, stop_line=stop,
                            markers=markers)
        d.update_odom(t, self.x, self.y, self.yaw)
        self.out = d.tick(t, 0.0, 0.0, 0.0)
        self.x += self.out.v * math.cos(self.yaw) * dt
        self.y += self.out.v * math.sin(self.yaw) * dt
        self.yaw += self.out.omega * dt


def _set_plan(bot, f):
    bot.d.set_plan(f['seq'], list(zip(f['step_kind'], f['step_value'])), f['stop_line_count'],
                   f['linear_speed'], f['angular_speed'], goal_marker_id=f['goal_marker_id'],
                   arrive_distance=f['arrive_distance'], skip_clearance=f['skip_clearance'],
                   arrive_on_obstacle=f['arrive_on_obstacle'])


def run_scenario(name, red, mode=None, scenario=None, send_clearance=True):
    from pinky_fleet_agent.lane_driver import CMD_CLEARANCE, CMD_HEARTBEAT, CMD_START
    c = cfg(mode)
    run = ScenarioRun(c, scenario or name, t0=0.0, first_seq=1)
    bots = {n: Bot(red[n]) for n in run.robots}
    for n, b in bots.items():
        _set_plan(b, run.plan_fields(n))
        # 같은 목적지의 다른 로봇이 벽 앞에 서 있으면 그 뒤 0.20 m 부터 초음파에 걸린다 (앞 로봇 몸)
        b.blocker = (lambda after, me=n: any(
            o != me and ob.d._arrived and run.scenario.robots[o].goal == run.scenario.robots[me].goal
            and after >= ob.wall - ob.d._plan['arrive_distance'] - 0.20 for o, ob in bots.items()))
    dt, t = 0.05, 0.0
    started = set()
    history = []
    for i in range(int(90 / dt)):
        t += dt
        if i % 2 == 0:                                          # 관제 10 Hz
            for n in run.tick(t):
                _set_plan(bots[n], run.plan_fields(n))
            for n, b in bots.items():
                if run.due(n, t) and n not in started:
                    b.d.set_command(CMD_START, t)
                    started.add(n)
                if send_clearance:
                    b.d.set_command(CMD_CLEARANCE, t, route_seq=run.seq[n], clear_until=run.clearance(n))
                else:
                    b.d.set_command(CMD_HEARTBEAT, t)
        for n, b in bots.items():
            b.step(t, dt, i)
            run.on_status(n, b.out.state, b.d.route_seq, b.out.edge_id, t)
        history.append((t, {n: (b.out.state, b.x) for n, b in bots.items()}, run.arbiter.holder))
        if run.done:
            break
    return run, bots, history


def test_scenario1_closed_loop_first_arrival_passes_first_and_other_waits():
    """pinky2(3번) 가 먼저 빨간 선에 닿는다 — domain 이 커도 먼저 지나고, pinky1 은 그동안 선다. (흰 정지선 도착)"""
    run, bots, history = run_scenario('s1', red={'pinky1': 1.0, 'pinky2': 0.7}, mode='stop_line')
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
    run, _, _ = run_scenario('s1', red={'pinky1': 1.0, 'pinky2': 1.0}, mode='stop_line')
    assert run.done and run.arbiter.grant_order == ['pinky1', 'pinky2']


def test_scenario2_closed_loop_sequential_departure_no_meeting():
    run, bots, history = run_scenario('s2', red={'pinky1': 1.0, 'pinky2': 1.0}, mode='stop_line')
    assert run.done
    first_move = {n: next(t for t, s, _ in history if s[n][1] > 0.01) for n in ('pinky1', 'pinky2')}
    assert first_move['pinky2'] - first_move['pinky1'] >= 9.5            # 10 s 간격
    # 통행권 대기 없이 각자 지나갔다(만나지 않는다)
    assert not any(h is not None and s['pinky1'][0] == DRIVE_JUNCTION_STOP and s['pinky2'][0] == DRIVE_JUNCTION_STOP
                   and t > 12 for t, s, h in history)
    assert bots['pinky2'].y > 0.05                                      # 1→3 좌회전
    assert run.stop_lines == {'pinky1': 1, 'pinky2': 1}


# ------------------------------------------------------------ 폐루프: 벽 마커 도착

def test_marker_scenario1_first_stops_at_wall_second_behind_it():
    run, bots, history = run_scenario('s1', red={'pinky1': 1.0, 'pinky2': 0.7})
    assert run.done, run.status(history[-1][0])
    assert run.arbiter.grant_order == ['pinky2', 'pinky1']
    b2, b1 = bots['pinky2'], bots['pinky1']
    gap2 = b2.wall - (b2.d._travelled - b2.pass_travel)
    assert 0.10 <= gap2 <= 0.15 + 1e-6                                  # 벽 마커 15 cm 안, 지나치지 않게
    assert '벽 마커 1' in b2.d._arrived_reason
    assert run.arrive_on_obstacle == {'pinky1': True, 'pinky2': False}
    assert '앞 로봇 뒤' in b1.d._arrived_reason
    gap1 = b1.wall - (b1.d._travelled - b1.pass_travel)
    assert gap1 > 0.30                                                  # 앞 로봇 뒤에 섰다
    assert b1.out.state == DRIVE_ARRIVED and b1.out.v == 0.0


def test_marker_scenario2_each_to_own_wall():
    run, bots, _ = run_scenario('s2', red={'pinky1': 1.0, 'pinky2': 1.0})
    assert run.done
    for b in bots.values():
        assert 0.10 <= b.wall - (b.d._travelled - b.pass_travel) <= 0.15 + 1e-6
    assert not any(run.arrive_on_obstacle.values())


def test_single_robot_goes_after_one_second_without_clearance():
    c = cfg()
    s = scenario_from_dict(c, 'custom', {'robots': {'pinky1': {'start': 2, 'goal': 3}}}, 'custom')
    run, bots, history = run_scenario('custom', red={'pinky1': 0.5}, scenario=s, send_clearance=False)
    assert run.done
    stops = [t for t, st, _ in history if st['pinky1'][0] == DRIVE_JUNCTION_STOP]
    assert stops and 0.9 <= max(stops) - min(stops) <= 1.3            # 1 s 정지 뒤 허가 없이 출발
    assert bots['pinky1'].y < -0.05                                     # 2→3 우회전

