# -*- coding: utf-8 -*-
"""개편 1단계 (2026-09-29): 주 대시보드 = 팀11 live 웹(pinky_fleet_station/live_web_node). 중계는 그 계약을 맞춘다.

live 웹은 중계 PC 도메인 8 에서 뜨고, 로봇 토픽은 중계 브리지가 10·11 → 8 로 올린다. 이름·QoS 가 하나라도 어긋나면
live 웹 화면은 에러 없이 "미수신" 으로만 남는다 — 그래서 **live 웹 소스를 직접 읽어** 대조한다(팀원이 이름을 바꾸면
이 시험이 먼저 빨개진다). ROS 없이 돈다.
"""
import ast
import io
import os
import re

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RELAY = os.path.dirname(HERE)
REPO = os.path.dirname(RELAY)
LIVE_NODE = os.path.join(REPO, 'pinky_fleet_station', 'pinky_fleet_station', 'live_web_node.py')
LIVE_JS = os.path.join(REPO, 'pinky_fleet_station', 'pinky_fleet_station', 'live_static', 'app.js')
COORD = os.path.join(RELAY, 'fleet', 'fleet_coordinator.py')
BRIDGE = os.path.join(RELAY, 'domain_bridge', 'configs', 'robot%d_control.yaml')
SCRIPT = os.path.join(RELAY, 'launch_live_web.sh')
ROBOTS = {1: 'pinky1', 2: 'pinky2'}

# live 웹이 로봇마다 구독하는 것 → 누가 내나. overhead_pose 는 중계 PC 에서 상부 추적기가 직접 낸다(브리지 대상 아님).
UPLINK = {
    'state': 'pinky_fleet_msgs/msg/RobotState',
    'lane_status': 'pinky_lane_msgs/msg/LaneStatus',
    'amcl_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    'camera/image/compressed': 'sensor_msgs/msg/CompressedImage',
}
DOWNLINK = {'command': 'pinky_fleet_msgs/msg/FleetCommand'}   # live 웹 SET_SPEED · SET_INITIAL_POSE

pytestmark = pytest.mark.skipif(not os.path.isfile(LIVE_NODE), reason='팀11 pinky_fleet_station 이 옆에 없다')


def _src(path):
    with io.open(path, encoding='utf-8') as f:
        return f.read()


def _bridge(n):
    with io.open(BRIDGE % n, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_live_web_still_subscribes_what_this_test_expects():
    """전제 확인 — live 웹 소스에 이 이름들이 그대로 있다. 바뀌면 아래 표를 같이 고친다."""
    src = _src(LIVE_NODE)
    for suffix in ('state', 'lane_status', 'amcl_pose', 'camera/image/compressed', 'overhead_pose'):
        assert suffix in src, suffix
    assert "f'/{name}/command'" in src
    assert "'/fleet/lane/control'" in src and "'/fleet/lane/status'" in src


@pytest.mark.parametrize('n', sorted(ROBOTS))
def test_bridge_uplinks_every_topic_live_web_reads(n):
    topics = _bridge(n)['topics']
    for suffix, typ in UPLINK.items():
        name = '/%s/%s' % (ROBOTS[n], suffix)
        assert name in topics, '%s 가 브리지에 없다 — live 웹이 미수신' % name
        t = topics[name]
        assert t['type'] == typ
        assert 'from_domain' not in t, '%s 는 업링크(로봇 → 8)다' % name


@pytest.mark.parametrize('n', sorted(ROBOTS))
def test_bridge_downlinks_live_web_commands(n):
    topics = _bridge(n)['topics']
    for suffix, typ in DOWNLINK.items():
        t = topics['/%s/%s' % (ROBOTS[n], suffix)]
        assert t['type'] == typ and t['from_domain'] == 8


@pytest.mark.parametrize('n', sorted(ROBOTS))
def test_camera_bridge_qos_matches_camera_node(n):
    """camera_node 는 BEST_EFFORT depth 1 로 낸다 — 브리지가 reliable 을 요구하면 짝이 안 맞아 빈 토픽."""
    cam = _src(os.path.join(REPO, 'pinky_fleet_agent', 'pinky_fleet_agent', 'camera_node.py'))
    assert 'reliability=QoSReliabilityPolicy.BEST_EFFORT' in cam
    q = _bridge(n)['topics']['/%s/camera/image/compressed' % ROBOTS[n]]['qos']
    assert q['reliability'] == 'best_effort'


def test_coordinator_status_carries_mission_string():
    """live 웹 on_mission: `mission` 이 문자열이 아니면 표본을 버린다. 팀11 lane_coordinator_node 와 같은 이름."""
    src = _src(COORD)
    fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == 'get_fleet_status_dict')
    ret = [n for n in ast.walk(fn) if isinstance(n, ast.Return) and isinstance(n.value, ast.Dict)][-1].value
    keys = {k.value: v for k, v in zip(ret.keys, ret.values) if isinstance(k, ast.Constant)}
    assert 'mission' in keys and 'warning' in keys
    v = keys['mission']
    assert isinstance(v, ast.Call) and getattr(v.func, 'id', '') == 'str', 'mission 은 str(...) 로 — None 이면 live 웹이 버린다'
    assert ast.unparse(v.args[0]) == 'self.mission_state'


def test_coordinator_accepts_every_live_web_mission_command():
    """live 웹 버튼: start · pause(→stop) · resume · reset(→stop + SET_INITIAL_POSE). 코디네이터가 그 cmd 를 받는다."""
    live = _src(LIVE_NODE)
    sent = set(re.findall(r"\{'pause': '(\w+)', 'resume': '(\w+)'\}", live)[0]) | {'start'}
    assert "json.dumps({'cmd': 'stop'})" in live
    coord = _src(COORD)
    for cmd in sent | {'stop'}:
        assert "cmd == '%s'" % cmd in coord, 'relay 코디네이터가 %r 를 받지 않는다' % cmd


def test_launch_live_web_script_uses_relay_domain_and_same_dds():
    s = _src(SCRIPT)
    assert 'export ROS_DOMAIN_ID=8' in s
    assert 'domain_bridge/bridge_env.sh' in s, 'DDS 프로파일은 브리지와 같은 규칙으로'
    assert '-p enable_control:="${LIVE_WEB_CONTROL:-false}"' in s, '미션 버튼은 기본 꺼짐'
    assert '-p host:=0.0.0.0' in s
    assert os.access(SCRIPT, os.X_OK)


def test_live_web_draws_robots_only_on_identical_map():
    """스크립트가 프로파일 지도를 넘기는 이유 — live 웹은 로봇 지도가 원점·크기까지 같을 때만 로봇을 그린다."""
    js = _src(LIVE_JS)
    assert 'state.map_width === mapMetadata.width' in js and 'state.map_origin_x - mapMetadata.origin[0]' in js
    assert 'resolve_active' in _src(SCRIPT)
