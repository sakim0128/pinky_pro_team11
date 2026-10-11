# -*- coding: utf-8 -*-
"""태블릿 좌표 → 팀11 로봇 위치 (2026-09-29). ROS 없이 돈다.

팀11 로봇(lane_robot.launch.xml)의 pose_fuser_node 는 PoseFix 를 받지 않고 /<robot>/overhead_pose
(geometry_msgs/PoseStamped, map)만 받는다. 그래서
  1. 게이트웨이는 /api/vision/pose_fix 로 받은 좌표를 그 이름으로도 낸다(기본 켜짐, RELAY_POSE_FIX_TO_OVERHEAD=0 이면 끔).
  2. 브리지는 그 토픽과 차선 경로(/<robot>/lane_path)를 관제 8 → 로봇 도메인으로 내린다.
이름·타입·QoS 중 하나라도 어긋나면 로봇은 에러 없이 "위치 없음 — 정지" 로만 남는다.
"""
import ast
import io
import math
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RELAY = os.path.dirname(HERE)
REPO = os.path.dirname(RELAY)
GW = os.path.join(RELAY, 'gateway_web')
SERVER = os.path.join(GW, 'gateway_web_server.py')
BRIDGE = os.path.join(RELAY, 'domain_bridge', 'configs', 'pinky%d_control.yaml')
GENERATOR = os.path.join(RELAY, 'domain_bridge', 'generate_configs.sh')
TRACKER = os.path.join(REPO, 'pinky_fleet_station', 'pinky_fleet_station', 'overhead_tracker_node.py')
ROBOTS = {1: ('pinky1', 10), 2: ('pinky2', 11)}

sys.path.insert(0, GW)
import vision_ingest as VI  # noqa: E402


def _src(path):
    with io.open(path, encoding='utf-8') as f:
        return f.read()


def _bridge(n):
    with io.open(BRIDGE % n, encoding='utf-8') as f:
        return yaml.safe_load(f)['topics']


# ── 브리지 ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('n', sorted(ROBOTS))
def test_브리지가_위치를_로봇으로_내린다(n):
    name, dom = ROBOTS[n]
    t = _bridge(n)['/%s/overhead_pose' % name]
    assert t['type'] == 'geometry_msgs/msg/PoseStamped'
    assert (t['from_domain'], t['to_domain']) == (8, dom)
    assert t['qos']['reliability'] == 'reliable' and t['qos']['durability'] == 'volatile'


@pytest.mark.parametrize('n', sorted(ROBOTS))
def test_브리지가_차선_경로를_로봇으로_내린다(n):
    name, dom = ROBOTS[n]
    t = _bridge(n)['/%s/lane_path' % name]
    assert t['type'] == 'pinky_lane_msgs/msg/LanePath'
    assert (t['from_domain'], t['to_domain']) == (8, dom)
    assert t['qos']['reliability'] == 'best_effort' and t['qos']['depth'] == 1


def test_생성기가_정본이다():
    """configs/*.yaml 은 generate_configs.sh 의 산출물 — 손으로만 고치면 다음 생성 때 사라진다."""
    g = _src(GENERATOR)
    assert '/pinky${n}/overhead_pose:' in g and '/pinky${n}/lane_path:' in g


@pytest.mark.skipif(not os.path.isfile(TRACKER), reason='팀11 pinky_fleet_station 이 옆에 없다')
def test_상부_추적기와_같은_이름을_쓴다():
    """팀11 overhead_tracker_node 가 내는 이름과 같다 — 로봇은 출처를 가리지 않는다."""
    assert "f'/{name}/overhead_pose'" in _src(TRACKER)


# ── 게이트웨이 ────────────────────────────────────────────────────────────

def test_기본은_켜짐이고_끌_수_있다():
    assert VI.pose_fix_to_overhead_enabled({}) is True
    assert VI.pose_fix_to_overhead_enabled({'RELAY_POSE_FIX_TO_OVERHEAD': '1'}) is True
    for off in ('0', 'false', 'OFF', ' no '):
        assert VI.pose_fix_to_overhead_enabled({'RELAY_POSE_FIX_TO_OVERHEAD': off}) is False


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -2.5, math.pi])
def test_yaw_쿼터니언은_되돌리면_같은_yaw(yaw):
    z, w = VI.yaw_to_quat_zw(yaw)
    back = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)          # pose_fuser_node.yaw_from_quaternion (x=y=0)
    assert abs(math.atan2(math.sin(back - yaw), math.cos(back - yaw))) < 1e-9
    assert abs(z * z + w * w - 1.0) < 1e-12


def _fn(name):
    tree = ast.parse(_src(SERVER))
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)


def test_게이트웨이는_PoseStamped_를_로봇마다_낸다():
    s = _src(SERVER)
    assert "self.create_publisher(PoseStamped, f'/{name}/overhead_pose', 10)" in s
    assert "for name in ('pinky1', 'pinky2')" in s
    assert 'vision_ingest.pose_fix_to_overhead_enabled()' in s


def test_stamp_는_중계_시계다():
    """태블릿 시계는 어긋날 수 있다(vision_ingest 머리말) — overhead_tracker_node 처럼 중계가 받은 시각을 싣는다."""
    body = ast.unparse(_fn('publish_overhead_pose'))
    assert 'self.get_clock().now().to_msg()' in body
    assert "stamp_sec" not in body
    assert "frame_id = 'map'" in body


def test_pose_fix_요청이_둘_다_낸다():
    """/api/vision/pose_fix 처리에서 PoseFix 와 overhead_pose 를 같이 내고, 응답에 로봇이 받는 토픽과 구독자 수를 싣는다."""
    s = _src(SERVER)
    i = s.index("elif parsed.path == '/api/vision/pose_fix':")
    block = s[i:s.index('elif parsed.path in PROFILE_COMMANDS:', i)]
    assert 'publish_pose_fix(norm)' in block and 'publish_overhead_pose(norm)' in block
    assert block.index('_check_vision_auth') < block.index('publish_overhead_pose')   # 키 검사 뒤
    assert block.index('validate_pose_fix') < block.index('publish_overhead_pose')    # 검증 뒤
    assert "'overhead_topic'" in block and "'overhead_subscribers'" in block
