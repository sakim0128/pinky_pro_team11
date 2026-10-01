# -*- coding: utf-8 -*-
"""비전 미션 모드 (2026-09-30) — 게이트웨이 --vision · /api/fleet/scenario · V2 시나리오 버튼 · VisionFleetCoordinator.

마지막 두 시험은 **진짜 VisionFleetCoordinator(__init__ 포함)** 를 로봇 드라이버 두 대(pinky_fleet_agent.LaneDriver, lane_only +
JunctionPlan)와 폐루프로 돌린다. rclpy 가 없는 곳(노트북·CI)에서는 .msg 파일에서 만든 가짜 메시지와 가짜 Node 로 돌고,
시험이 끝나면 sys.modules 를 원래대로 되돌린다(다른 시험이 가짜를 보지 않게). rclpy 가 있으면 가짜를 넣지 않는다.
"""
import importlib
import json
import math
import os
import re
import sys
import types

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RELAY = os.path.join(REPO, 'relay_station')
STATIC = os.path.join(RELAY, 'gateway_web', 'static')
HTML = open(os.path.join(STATIC, 'fleet_control_v2.html'), encoding='utf-8').read()
JS = open(os.path.join(STATIC, 'fleet_control_v2.js'), encoding='utf-8').read()
GW = open(os.path.join(RELAY, 'gateway_web', 'gateway_web_server.py'), encoding='utf-8').read()
VC = open(os.path.join(RELAY, 'fleet', 'vision_coordinator.py'), encoding='utf-8').read()
FC = open(os.path.join(RELAY, 'fleet', 'fleet_coordinator.py'), encoding='utf-8').read()


# ------------------------------------------------------------ 정적: 화면 · 게이트웨이 · 코디네이터

def test_v2_vision_panel_and_scenario_buttons():
    for el in ('id="vision-panel"', 'id="vision-scenarios"', 'id="vision-status"', 'id="vision-robots"'):
        assert el in HTML
    assert 'vision-panel" id="vision-panel" hidden' in HTML                 # 비전 모드가 아니면 숨는다
    assert 'function renderVision()' in JS and 'renderVision();' in JS
    assert '"/api/fleet/scenario"' in JS
    block = JS[JS.index('function renderVision()'):JS.index('async function scenarioAction')]
    assert 'data-moving="1"' in block                                    # 움직이는 조작 — 보기 전용·제어권 잠금
    assert 'f.mode === "vision"' in block


def test_gateway_vision_flag_and_scenario_endpoint_is_gated():
    assert "parser.add_argument('--vision'" in GW
    assert 'from fleet.vision_coordinator import VisionFleetCoordinator' in GW
    i = GW.index("elif parsed.path == '/api/fleet/scenario':")
    block = GW[i:i + 3200]
    assert "_deny_if_cannot_move('remote fleet command scenario'" in block.split('coord = GLOBAL_FLEET_COORDINATOR')[0]
    for reason in ('NO_FLEET_COORDINATOR', 'NOT_VISION_MODE', 'UNKNOWN_SCENARIO'):
        assert reason in block
    assert "wait_control_result(coord, seq0, 'scenario')" in block


def test_v2_vision_mode_hides_aerial_map_and_assign():
    """항공뷰 관제 없음 — 비전 모드면 지도·중계 탑뷰·내비게이션 탭·경로 배정·Overhead 카드를 숨긴다."""
    for anchor in ('<article class="card map-card" data-hide-vision>', '<div class="camera-main" data-hide-vision>',
                   'data-tab="navigation" data-hide-vision', 'id="assign-card" data-hide-vision'):
        assert anchor in HTML, anchor
    i = HTML.index('<h3>Overhead / External</h3>')
    assert 'data-hide-vision' in HTML[HTML.rindex('<article', 0, i):i]
    css = open(os.path.join(STATIC, 'fleet_control_v2.css'), encoding='utf-8').read()
    assert 'body.vision-mode [data-hide-vision] { display: none !important; }' in css
    assert 'document.body.classList.toggle("vision-mode", !!v)' in JS
    # 로봇 카메라는 남는다
    assert '/robot_camera_feed?id=pinky1' in HTML and '/robot_camera_feed?id=pinky2' in HTML


def test_v2_custom_scenario_form_save_and_delete():
    for el in ('id="vision-custom-rows"', 'id="vision-custom-start"', 'id="vision-custom-save"', 'id="vision-custom-name"'):
        assert el in HTML
    for s in ('postJson("/api/fleet/scenario", {custom: sc})', '"/api/fleet/scenario/save"', '"/api/fleet/scenario/delete"'):
        assert s in JS, s
    block = JS[JS.index('function renderVisionScenarios'):JS.index('function renderVisionCustom')]
    assert 'data-scenario-del' in block and 's.source === "user"' in block        # 프리셋은 못 지운다
    assert block.count('data-moving="1"') >= 2
    i = GW.index("elif parsed.path in ('/api/fleet/scenario/save', '/api/fleet/scenario/delete'):")
    assert "_deny_if_cannot_move('remote fleet scenario ' + action" in GW[i:i + 800]


def test_vision_coordinator_never_sends_route_or_fleet_stop_spam():
    assert 'AUTO_ASSIGN = False' in VC
    assert 'if self.AUTO_ASSIGN and ctx.start_node and ctx.goal_node:' in FC
    pub = VC[VC.index('def _publish_vision_commands'):VC.index('# ------------------------------------------------------------------ 상태')]
    assert 'FleetCommand.CMD_HEARTBEAT' in pub and 'FleetCommand.CMD_STOP' not in pub
    assert 'RouteMsg' not in VC and 'route_pubs' not in VC


# ------------------------------------------------------------ 폐루프 (가짜 ROS 또는 진짜)

def _msg_class(pkg, name, path):
    consts, fields = {}, []
    for raw in open(path, encoding='utf-8'):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        typ, rest = line.split(None, 1)
        if '=' in rest:
            k, v = [x.strip() for x in rest.split('=', 1)]
            consts[k] = float(v) if '.' in v else int(v)
        else:
            fields.append((typ, rest.split()[0]))

    def default(typ):
        if typ.endswith(']'):
            return []
        if typ in ('bool',):
            return False
        if typ in ('string',):
            return ''
        if typ.startswith(('float',)):
            return 0.0
        if typ.startswith(('int', 'uint', 'byte', 'char')):
            return 0
        return types.SimpleNamespace(stamp=None, frame_id='')

    def __init__(self, **kw):
        for typ, fname in fields:
            setattr(self, fname, kw.get(fname, default(typ)))

    return type(name, (), dict(consts, __init__=__init__, __module__=f'{pkg}.msg'))


def _install_fake_ros():
    fake = {}

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        fake[name] = m
        return m

    class Pub:
        def __init__(self, topic):
            self.topic, self.published = topic, []

        def publish(self, msg):
            self.published.append(msg)

    class Clock:
        def __init__(self, node):
            self.node = node

        def now(self):
            t = self.node._fake_t
            return types.SimpleNamespace(nanoseconds=int(t * 1e9), to_msg=lambda: types.SimpleNamespace(sec=int(t)))

    class Logger:
        def __init__(self):
            self.lines = []

        def _log(self, *a, **k):
            self.lines.append(str(a[0]) if a else '')
        info = warn = warning = error = debug = _log

    class Node:
        def __init__(self, name, **kw):
            self._fake_t = 100.0
            self._fake_pubs, self._fake_subs, self._fake_timers = {}, {}, []
            self._fake_logger = Logger()

        def create_publisher(self, typ, topic, qos):
            p = Pub(topic)
            self._fake_pubs[topic] = p
            return p

        def create_subscription(self, typ, topic, cb, qos):
            self._fake_subs[topic] = cb

        def create_timer(self, period, cb):
            self._fake_timers.append((period, cb))

        def get_logger(self):
            return self._fake_logger

        def get_clock(self):
            return Clock(self)

    class Enum:
        def __getattr__(self, name):
            return name

    mod('rclpy', init=lambda *a, **k: None, spin=lambda *a, **k: None, shutdown=lambda *a, **k: None, ok=lambda: True)
    mod('rclpy.node', Node=Node)
    mod('rclpy.qos', QoSProfile=lambda **kw: types.SimpleNamespace(**kw), QoSDurabilityPolicy=Enum(),
        QoSHistoryPolicy=Enum(), QoSReliabilityPolicy=Enum())
    mod('geometry_msgs')
    mod('geometry_msgs.msg', Point=lambda **kw: types.SimpleNamespace(x=0.0, y=0.0, z=0.0, **kw))
    mod('std_msgs')
    mod('std_msgs.msg', Header=lambda **kw: types.SimpleNamespace(stamp=None, frame_id=''),
        String=_msg_class('std_msgs', 'String', _write_tmp_msg('string data')))
    for pkg, names in (('pinky_fleet_msgs', ('FleetCommand', 'RobotState')),
                       ('pinky_lane_msgs', ('LaneCommand', 'LaneStatus', 'Route', 'JunctionPlan', 'LanePath'))):
        mod(pkg)
        mod(f'{pkg}.msg', **{n: _msg_class(pkg, n, os.path.join(REPO, pkg, 'msg', n + '.msg')) for n in names})
    return fake


def _write_tmp_msg(text):
    import tempfile
    fd, path = tempfile.mkstemp(suffix='.msg')
    with os.fdopen(fd, 'w') as fh:
        fh.write(text + '\n')
    return path


@pytest.fixture
def vision_coordinator_cls(tmp_path, monkeypatch):
    monkeypatch.setenv('PINKY_RELAY_STATE_DIR', str(tmp_path))               # 실물 상태 폴더를 건드리지 않는다
    monkeypatch.setenv('PINKY_VISION_USER_SCENARIOS', str(tmp_path / 'vision_user.yaml'))   # 저장 시나리오도
    before = set(sys.modules)
    try:
        import rclpy  # noqa: F401
        have_ros = True
    except ImportError:
        have_ros = False
    if not have_ros:
        sys.modules.update(_install_fake_ros())
    for p in (RELAY, os.path.join(REPO, 'pinky_lane_station'), os.path.join(REPO, 'pinky_fleet_agent')):
        if p not in sys.path:
            sys.path.insert(0, p)
    try:
        vc = importlib.import_module('fleet.vision_coordinator')
        if have_ros:
            pytest.skip('rclpy 가 있는 곳에서는 실물 노드 시험(ROS 컨테이너)로 잰다 — 이 폐루프는 가짜 Node 전용')
        yield vc.VisionFleetCoordinator
    finally:
        for name in set(sys.modules) - before:
            del sys.modules[name]


class Bot:
    """lane_only 드라이버 + 1차원 차선: red_x 에서 빨간 선, 교차로 통과 뒤 0.30 m·0.70 m 에 흰 정지선, wall m 에 벽 마커.
    blocker(after) 가 참이면 초음파 8 cm (같은 목적지 앞 로봇) — 마커도 가린다."""

    def __init__(self, name, red_x, wall=0.85):
        from pinky_fleet_agent.lane_driver import DriverParams, LaneDriver
        from pinky_fleet_agent.maneuver import parse_steps
        p = DriverParams()
        p.lane_only = True
        self.name, self.d, self.parse_steps = name, LaneDriver(p), parse_steps
        self.x = self.y = self.yaw = 0.0
        self.red_x = red_x
        self.wall = wall
        self.blocker = lambda after: False
        self.pass_travel = None
        self.out = None
        self.seen = {'lane': 0, 'plan': 0, 'fleet': 0}

    def receive(self, coord, t):
        from pinky_fleet_agent.lane_driver import CMD_HEARTBEAT
        n = self.name
        plans = coord.plan_pubs[n].published
        for m in plans[self.seen['plan']:]:
            self.d.set_plan(m.seq, self.parse_steps(m.step_kind, m.step_value), m.stop_line_count,
                            m.linear_speed, m.angular_speed, goal_marker_id=m.goal_marker_id,
                            arrive_distance=m.arrive_distance, skip_clearance=m.skip_clearance,
                            arrive_on_obstacle=m.arrive_on_obstacle)
        self.seen['plan'] = len(plans)
        lanes = coord.lane_cmd_pubs[n].published
        for m in lanes[self.seen['lane']:]:
            self.d.set_command(int(m.command), t, route_seq=int(m.route_seq), clear_until=int(m.clear_until_idx),
                               max_v=float(m.max_linear_vel), max_w=float(m.max_angular_vel))
        self.seen['lane'] = len(lanes)
        fleet = coord.fleet_cmd_pubs[n].published
        for m in fleet[self.seen['fleet']:]:
            assert int(m.command) == 7, f'비전 모드 FleetCommand 는 하트비트뿐이어야 한다: {m.command}'
            self.d.set_command(CMD_HEARTBEAT, t)
        self.seen['fleet'] = len(fleet)

    def step(self, t, dt, i):
        from pinky_fleet_agent.lane_control import QUALITY_BOTH
        d = self.d
        if d._junction_passed and self.pass_travel is None:
            self.pass_travel = d._travelled
        stop = self.pass_travel is not None and any(L <= d._travelled - self.pass_travel <= L + 0.05 for L in (0.30, 0.70))
        markers, blocked = {}, False
        if self.pass_travel is not None:
            after = d._travelled - self.pass_travel
            blocked = self.blocker(after)
            if d._plan and d._plan['goal_marker_id'] >= 0 and not blocked and self.wall - after <= 1.0:
                markers = {d._plan['goal_marker_id']: self.wall - after}
        d.update_us(0.08 if blocked else 1.0)
        if i % 3 == 0:
            from pinky_fleet_agent.sim_red_lines import junction_red_lines, red_blobs, red_line_detected
            blobs = red_blobs((self.x, self.y, self.yaw), junction_red_lines(self.red_x))
            d.set_lane_path(t, t, QUALITY_BOTH, 0.0, False, red_line=red_line_detected(blobs), stop_line=stop,
                            markers=markers, red_obs=blobs)
        d.update_odom(t, self.x, self.y, self.yaw)
        self.out = d.tick(t, 0.0, 0.0, 0.0)
        self.x += self.out.v * math.cos(self.yaw) * dt
        self.y += self.out.v * math.sin(self.yaw) * dt
        self.yaw += self.out.omega * dt


def _loop(coord, bots, seconds, dt=0.05):
    import pinky_lane_msgs.msg as lm
    import pinky_fleet_msgs.msg as fm
    t0 = coord._fake_t
    for i in range(int(seconds / dt)):
        t = t0 + (i + 1) * dt
        coord._fake_t = t
        if i % 2 == 0:
            coord._loop_tick()
        for b in bots.values():
            b.receive(coord, t)
            b.step(t, dt, i)
            st = lm.LaneStatus()
            st.drive_state, st.route_seq, st.edge_id = int(b.out.state), int(b.d.route_seq), b.out.edge_id
            coord._cb_lane_status(b.name, st)
            if i % 2 == 0:
                rs = fm.RobotState()
                rs.localized = False
                rs.header.frame_id = 'odom'                          # lane_only 는 odom 자세를 보낸다 (위치 표시)
                rs.x, rs.y, rs.yaw = b.x, b.y, b.yaw
                coord._cb_robot_state(b.name, rs)
        if coord.mission_state == 'DONE':
            return t
    return None


def _scenario(coord, name, custom=None):
    import std_msgs.msg as sm
    payload = {'cmd': 'scenario', 'name': name}
    if custom is not None:
        payload['custom'] = custom
    coord._cb_control(sm.String(data=json.dumps(payload)))
    return coord.last_control


def _same_goal_blockers(coord, bots):
    """같은 목적지로 먼저 도착한 로봇 뒤 0.20 m 부터 초음파에 걸린다."""
    for n, b in bots.items():
        b.blocker = (lambda after, me=n: any(
            o != me and ob.d._arrived and coord.vision_run is not None
            and coord.vision_run.scenario.robots[o].goal == coord.vision_run.scenario.robots[me].goal
            and after >= ob.wall - ob.d._plan['arrive_distance'] - 0.20 for o, ob in bots.items()))


def test_vision_coordinator_scenario1_real_init_closed_loop(vision_coordinator_cls):
    coord = vision_coordinator_cls()
    assert coord.mission_state == 'IDLE'
    assert all(c.route is None for c in coord.robots.values())
    assert not any(t.endswith('/route') and p.published for t, p in coord._fake_pubs.items())   # 경로를 내지 않는다
    assert _scenario(coord, 's9')['ok'] is False                                              # 모르는 시나리오
    bots = {'pinky1': Bot('pinky1', 1.0), 'pinky2': Bot('pinky2', 0.7)}                        # pinky2 가 먼저 닿는다
    _same_goal_blockers(coord, bots)
    last = _scenario(coord, 's1')
    assert last['ok'] is True and coord.mission_state == 'RUNNING'
    plan = coord.plan_pubs['pinky1'].published[-1]
    assert plan.goal_marker_id == 40 and plan.arrive_distance == pytest.approx(0.15) and plan.skip_clearance is False
    assert _scenario(coord, 's2')['ok'] is False                                              # 도는 중엔 다른 시나리오 거절
    done_at = _loop(coord, bots, 90)
    assert done_at is not None, coord.get_fleet_status_dict()['vision']
    run = coord.vision_run
    assert run.arbiter.grant_order == ['pinky2', 'pinky1']
    assert run.arrive_on_obstacle == {'pinky1': True, 'pinky2': False}                         # 뒤 로봇은 앞 로봇 뒤 정지
    assert coord.plan_pubs['pinky1'].published[-1].arrive_on_obstacle is True
    assert '벽 마커' in bots['pinky2'].d._arrived_reason and '앞 로봇' in bots['pinky1'].d._arrived_reason
    assert bots['pinky2'].y < -0.05                                                           # 3→1 우회전
    status = coord.get_fleet_status_dict()
    assert status['mode'] == 'vision' and status['vision']['active'] == 's1' and status['vision']['run']['done']
    assert status['robots']['pinky2']['start_node'] == '3' and status['robots']['pinky2']['goal_node'] == '1'
    json.dumps(status)                                                                        # 웹으로 나간다


def test_vision_coordinator_scenario2_departs_ten_seconds_apart_and_stop_resume(vision_coordinator_cls):
    coord = vision_coordinator_cls()
    bots = {'pinky1': Bot('pinky1', 1.0), 'pinky2': Bot('pinky2', 1.0)}
    assert _scenario(coord, 's2')['ok'] is True
    _loop(coord, bots, 3.0)
    assert bots['pinky1'].x > 0.05 and bots['pinky2'].x == 0.0                                # 2 번은 아직 출발 전
    # 일시정지 → 재개: 멈췄다가 START 를 다시 받아 이어 간다
    assert coord.stop_fleet()
    x_stop = bots['pinky1'].x
    _loop(coord, bots, 2.0)
    assert bots['pinky1'].x == pytest.approx(x_stop, abs=0.02)
    assert coord.resume_fleet()
    done_at = _loop(coord, bots, 90)
    assert done_at is not None
    assert all(0.10 <= b.wall - (b.d._travelled - b.pass_travel) <= 0.15 + 1e-6 for b in bots.values())
    assert bots['pinky2'].y > 0.05                                                            # 1→3 좌회전


def test_vision_coordinator_custom_single_robot_skips_clearance(vision_coordinator_cls):
    coord = vision_coordinator_cls()
    assert _scenario(coord, 'custom', {'robots': {'pinky1': {'start': '1', 'goal': '1'}}})['ok'] is False
    assert '같다' in coord.vision_error
    bots = {'pinky1': Bot('pinky1', 0.6)}
    last = _scenario(coord, 'custom', {'label': '2→3 한 대', 'robots': {'pinky1': {'start': '2', 'goal': '3'}}})
    assert last['ok'] is True
    plan = coord.plan_pubs['pinky1'].published[-1]
    assert plan.skip_clearance is True and plan.goal_marker_id == 42 and list(plan.step_kind) == ['seek'] and list(plan.step_value) == [-1.0]   # 2→3 우회전 = 새 빨간 선 찾기
    assert coord.vision_run.robots == ['pinky1']
    done_at = _loop(coord, bots, 60)
    assert done_at is not None
    assert coord.vision_run.arbiter.grant_order == []                                        # 통행권 없이 지났다
    status = coord.get_fleet_status_dict()['vision']
    assert status['run']['source'] == 'custom' and status['run']['skip_clearance'] is True
    assert status['course']['points'] == ['1', '2', '3']
    json.dumps(status)
    # "주행 시작" = 마지막 직접 설정을 다시
    assert coord.vision_scenario == 'custom' and coord.start_fleet() is True


def test_vision_coordinator_save_and_delete_user_scenarios(vision_coordinator_cls, tmp_path):
    coord = vision_coordinator_cls()
    raw = {'label': '3→2', 'robots': {'pinky2': {'start': '3', 'goal': '2'}}}
    ok, msg = coord.save_user_scenario('s1', raw)
    assert ok is False and '프리셋' in msg
    ok, msg = coord.save_user_scenario('run_3to2', raw)
    assert ok is True and os.path.exists(str(tmp_path / 'vision_user.yaml'))
    names = {s['name']: s['source'] for s in coord.get_fleet_status_dict()['vision']['scenarios']}
    assert names['run_3to2'] == 'user' and names['s1'] == 'preset'
    coord2 = vision_coordinator_cls()                                                         # 다시 띄워도 남아 있다
    assert coord2.vision_cfg.scenarios['run_3to2'].robots['pinky2'].direction == 'left'
    assert _scenario(coord2, 'run_3to2')['ok'] is True
    ok, msg = coord2.delete_user_scenario('run_3to2')
    assert ok is False                                                                        # 도는 중엔 못 지운다
    coord2.stop_fleet()
    assert coord2.delete_user_scenario('run_3to2')[0] is True
    assert coord2.delete_user_scenario('s1')[0] is False


def test_vision_coordinator_shows_estimated_positions(vision_coordinator_cls):
    """천장 카메라 없이 odom + 코스로 추정한 위치 — 시나리오 시작 때 출발 지점, 달리면 코스 위를 나아간다, /api/fleet/poses 용."""
    coord = vision_coordinator_cls()
    assert coord.vision_course is not None and coord.vision_course_image.endswith('docs/map5.png')
    assert coord.vision_poses()['robots'] == {}
    bots = {'pinky1': Bot('pinky1', 0.6)}
    assert _scenario(coord, 'custom', {'robots': {'pinky1': {'start': '2', 'goal': '1'}}})['ok'] is True
    p0 = coord.vision_poses()['robots']['pinky1']
    start = coord.vision_course.px_to_map(92, 100)
    assert abs(p0['x'] - start[0]) < 0.02 and abs(p0['y'] - start[1]) < 0.02 and p0['fix'].startswith('출발')
    _loop(coord, bots, 8)
    p1 = coord.vision_poses()['robots']['pinky1']
    assert p1['s'] > 0.3 and p1['age'] is not None and p1['age'] < 1.0
    status = coord.get_fleet_status_dict()['vision']
    assert status['map']['image_url'] == '/api/fleet/vision_map.png' and len(status['map']['lines']) == 6
    json.dumps(coord.vision_poses())


def test_v2_vision_map_card_polls_poses():
    assert 'id="vision-map-card"' in HTML and 'id="vision-map"' in HTML
    assert 'setInterval(refreshPoses, 200)' in JS and '"/api/fleet/poses"' in JS
    assert "elif parsed.path == '/api/fleet/poses':" in GW and "elif parsed.path == '/api/fleet/vision_map.png':" in GW
