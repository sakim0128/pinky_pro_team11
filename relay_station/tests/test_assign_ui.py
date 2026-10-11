# -*- coding: utf-8 -*-
"""웹 배정 (2026-09-29): 시작·목적지를 관제 PC 의 lane_mission.yaml 이 아니라 중계 웹(V2 설정 탭)에서 고른다.

재는 것:
- 도로망 요약(RoadGraph.summary): endpoint 노드만 시작·목적지 후보다 — map5 임시 도로망은 BL·BR·TR 셋(교차로 J).
- 코디네이터가 그 요약을 /api/fleet/profiles(profile_status) 에 싣고, 2 Hz 상태 토픽에는 싣지 않는다.
- V2 화면: 배정 카드·로봇별 select·배정 버튼이 있고, 기존 /api/fleet/assign 을 부른다. 움직이는 조작(data-moving)이라
  보기 전용에서 숨는다. drive_state 10~13(D14) 문구가 있다.
"""
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

import importlib.util  # noqa: E402


def _load_road_graph():
    """도로망 구현은 pinky_lane_station.road_graph 하나다(중계 fleet 도 그것을 import 한다) — 파일로 읽는다(ROS 없이 돈다)."""
    path = os.path.join(REPO, 'pinky_lane_station', 'pinky_lane_station', 'road_graph.py')
    spec = importlib.util.spec_from_file_location('lane_road_graph_for_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.RoadGraph


RoadGraph = _load_road_graph()

STATIC = os.path.join(REPO, 'relay_station', 'gateway_web', 'static')
HTML = open(os.path.join(STATIC, 'fleet_control_v2.html'), encoding='utf-8').read()
JS = open(os.path.join(STATIC, 'fleet_control_v2.js'), encoding='utf-8').read()
COORD_SRC = open(os.path.join(REPO, 'relay_station', 'fleet', 'fleet_coordinator.py'), encoding='utf-8').read()
GRAPH = os.path.join(REPO, 'pinky_lane_station', 'config', 'road_graph.yaml')


def test_summary_lists_only_endpoints_as_start_goal_candidates():
    g = RoadGraph.load(GRAPH)
    s = g.summary()
    assert set(s['endpoints']) == {'BL', 'BR', 'TR'}
    assert set(s['junctions']) == {'J'}
    assert s['frame'] == 'map'
    ids = {n['id'] for n in s['nodes']}
    assert ids >= set(s['endpoints']) | set(s['junctions'])
    for n in s['nodes']:
        assert set(n) >= {'id', 'x', 'y', 'type'}


def test_summary_follows_the_graph_file_not_the_screen():
    """맵이 바뀌면 road_graph.yaml 만 고친다 — 화면 코드에 노드 이름이 박혀 있지 않다."""
    block = JS[JS.index('function renderAssign()'):JS.index('async function profileAction')]
    for nid in ('BL', 'BR', 'TR', 'J'):
        assert re.search(r'["\']%s["\']' % nid, block) is None, f'{nid} 가 배정 화면 코드에 박혀 있다 (DEMO 예시 값 밖)'
    assert 'g.endpoints' in block


def test_coordinator_ships_graph_in_profiles_not_in_status_topic():
    assert "'graph': self.graph.summary()" in COORD_SRC
    assert "if k not in ('available', 'graph')" in COORD_SRC


def test_v2_assign_card_wiring():
    for el in ('id="assign-card"', 'id="assign-body"', 'id="assign-msg"', 'id="assign-pill"'):
        assert el in HTML
    assert 'function renderAssign()' in JS
    assert 'renderAssign();' in JS
    assert '"/api/fleet/assign"' in JS
    for attr in ('data-assign-start', 'data-assign-goal', 'data-assign-btn'):
        assert attr in JS
    # 움직이는 조작 — 보기 전용(U-1)·제어권(body.control-held) 잠금이 같은 표식을 본다
    block = JS[JS.index('function renderAssign()'):JS.index('async function assignAction')]
    assert block.count('data-moving="1"') >= 3
    # 시작 == 목적지는 화면에서 먼저 막는다(서버도 거절하지만 왕복을 아낀다)
    assert 'start === goal' in JS


def test_v2_knows_d14_drive_states():
    for code, word in ((10, '바리게이트'), (11, '차선 탐색'), (12, '교차로 정지'), (13, '교차로 통과')):
        assert re.search(r'%d:"%s' % (code, word), JS), f'drive_state {code} 문구 없음'
