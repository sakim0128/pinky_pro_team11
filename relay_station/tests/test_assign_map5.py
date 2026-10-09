# -*- coding: utf-8 -*-
"""map5 임시 도로망(2026-09-29)과 그 위의 임시 미션 — 코디네이터의 배정 규칙(L3)이 두 로봇을 다 받는다. ROS 없이 돈다.

도로망 정본은 pinky_lane_station/config/road_graph.yaml 하나이고 중계 프로파일 team11_map5 는 그 파일을 상대경로로 읽는다.
배정 규칙은 Reservation.assign_conflict 한 곳에 있다(코디네이터 _assign_conflict 가 그대로 위임한다) — 여기서 그 규칙을 잰다:
  * pinky1 BL→TR 뒤 pinky2 BR→BL: pinky1 이 출발 노드 BL 을 잡고 있어도(register) 거절하지 않는다 — 떠나면 놓는다.
  * 도착해 선 로봇의 목표 노드(mark_arrived)로는 배정하지 않는다 · 같은 목표 · 다른 로봇 경로 **안**의 노드도 거절.
"""
import importlib.util
import os
import sys
import types

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FLEET = os.path.join(REPO, 'relay_station', 'fleet')
GRAPH = os.path.join(REPO, 'pinky_lane_station', 'config', 'road_graph.yaml')
PROFILES = os.path.join(FLEET, 'config', 'profiles')

sys.path.insert(0, os.path.join(REPO, 'pinky_lane_station'))
from pinky_lane_station.road_graph import RoadGraph  # noqa: E402


def _reservation_module():
    """relay_station.fleet 의 __init__ 은 rclpy 를 끌어온다 — 패키지 껍데기만 만들고 reservation 을 파일로 읽는다."""
    pkg = 'relay_fleet_for_assign_test'
    if pkg + '.reservation' not in sys.modules:
        shell = types.ModuleType(pkg)
        shell.__path__ = [FLEET]
        sys.modules[pkg] = shell
        spec = importlib.util.spec_from_file_location(f'{pkg}.reservation', os.path.join(FLEET, 'reservation.py'))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[f'{pkg}.reservation'] = mod
        spec.loader.exec_module(mod)
    return sys.modules[pkg + '.reservation']


RES = _reservation_module()


def _profile_mission():
    man = yaml.safe_load(open(os.path.join(PROFILES, 'profiles.yaml'), encoding='utf-8'))
    assert man['default'] == 'team11_map5' and list(man['profiles']) == ['team11_map5']
    spec = man['profiles']['team11_map5']
    mission_path = os.path.normpath(os.path.join(PROFILES, spec['mission']))
    mission = yaml.safe_load(open(mission_path, encoding='utf-8'))
    graph_path = os.path.normpath(os.path.join(os.path.dirname(mission_path), mission['graph']))
    map_path = os.path.normpath(os.path.join(PROFILES, spec['map']))
    return spec, mission, graph_path, map_path


def test_profile_points_at_the_single_graph_and_map5_without_copies():
    spec, mission, graph_path, map_path = _profile_mission()
    assert graph_path == GRAPH, graph_path                     # 복사본이 아니라 정본
    assert map_path == os.path.join(REPO, 'pinky_fleet_station', 'config', 'map5.yaml')
    assert os.path.isfile(map_path) and os.path.isfile(os.path.join(os.path.dirname(map_path), 'maps', 'map5.pgm'))
    assert spec['frame'] == 'team11_map5' and spec['robot_map_name'] == 'map5'
    assert not os.path.exists(os.path.join(PROFILES, 'team11_map4'))
    assert not os.path.exists(os.path.join(FLEET, 'config', 'road_graph.yaml'))
    assert not os.path.exists(os.path.join(FLEET, 'config', 'lane_mission.yaml'))
    assert [(r['name'], r['start'], r['goal'], r['drive_mode']) for r in mission['robots']] == [
        ('pinky1', 'BL', 'TR', 'lane'), ('pinky2', 'BR', 'BL', 'lane')]


def test_placeholder_graph_is_inside_map5_and_endpoints_are_the_assign_choices():
    g = RoadGraph.load(GRAPH)
    s = g.summary()
    assert set(s['endpoints']) == {'BL', 'BR', 'TR'} and s['junctions'] == ['J']
    assert set(g.edges) == {'BL_J', 'J_TR', 'J_BR'} and g.lane_width == 0.2 and g.frame == 'map'
    for n in g.nodes.values():                                # map5: 원점 (-0.01, -0.01), 2.36 × 1.28
        assert -0.01 <= n.x <= 2.35 and -0.01 <= n.y <= 1.27, n
    for src in s['endpoints']:
        for dst in s['endpoints']:
            if src != dst:
                g.shortest_route(src, dst)
    first = open(GRAPH, encoding='utf-8').readline()
    assert '임시 좌표' in first and 'graph_editor' in first


def test_both_mission_assignments_are_accepted_in_mission_order():
    """코디네이터는 미션 순서대로(pinky1 → pinky2) 배정한다. register 가 출발 노드를 잡으므로 pinky2 의 목표 BL 은
    그 순간 pinky1 이 쥐고 있다 — 도착해 선 것이 아니라 거절 사유가 아니다."""
    g = RoadGraph.load(GRAPH)
    r = RES.Reservation(g)
    assert r.assign_conflict('pinky1', 'TR', 'BL') is None
    r.register('pinky1', 10, g.shortest_route('BL', 'TR', step=0.10))
    assert r.node_holder.get('BL') == 'pinky1'
    assert r.assign_conflict('pinky2', 'BL', 'BR') is None
    r.register('pinky2', 11, g.shortest_route('BR', 'BL', step=0.10))
    assert r.robots['pinky1'].route.node_ids == ['BL', 'J', 'TR']
    assert r.robots['pinky2'].route.node_ids == ['BR', 'J', 'BL']
    # 반대 순서로 배정해도 같다
    r2 = RES.Reservation(g)
    r2.register('pinky2', 11, g.shortest_route('BR', 'BL', step=0.10))
    assert r2.assign_conflict('pinky1', 'TR', 'BL') is None


def test_refusals_that_still_hold():
    g = RoadGraph.load(GRAPH)
    r = RES.Reservation(g)
    r.register('pinky1', 10, g.shortest_route('BL', 'TR', step=0.10))
    hit = r.assign_conflict('pinky2', 'TR', 'BR')             # 같은 목표
    assert hit and hit[0] == 'pinky1' and 'TR' in hit[1]
    hit = r.assign_conflict('pinky2', 'J', 'BR')              # 목표가 다른 로봇 경로 안
    assert hit and hit[0] == 'pinky1' and '경로 위' in hit[1]
    r.mark_arrived('pinky1')                                   # 도착해 선 로봇의 자리
    assert r.node_holder.get('TR') == 'pinky1'
    hit = r.assign_conflict('pinky2', 'TR', 'BR')
    assert hit and hit[0] == 'pinky1'
    assert r.assign_conflict('pinky2', 'BL', 'BR') is None    # 떠난 출발 노드는 비었다
    assert r.node_holder.get('BL') is None


def test_shared_edge_is_exclusive_and_the_waiting_robot_gets_through_after_arrival():
    """두 경로가 J 를 지나고 BL_J 를 반대 방향으로 공유한다 — 예약이 한 번에 한 로봇만 들여보낸다.
    pinky1 은 출발부터 BL_J(첫 엣지)를 쥐므로 pinky2 는 J 앞에서 그 엣지가 비기를 기다린다(blocked_by pinky1)."""
    g = RoadGraph.load(GRAPH)
    r1 = g.shortest_route('BL', 'TR', step=0.10)
    r2 = g.shortest_route('BR', 'BL', step=0.10)
    assert dict(RoadGraph.shared_edges(r1, r2)) == {'BL_J': False}
    r = RES.Reservation(g)
    r.register('pinky1', 10, r1)
    r.register('pinky2', 11, r2)
    r.step(tick=1)                                            # 각자 출발 노드·첫 엣지만 잡는다 — J 는 비어 있다
    assert r.node_holder.get('J') is None
    assert r.edge_holder == {'BL_J': 'pinky1', 'J_BR': 'pinky2'}
    assert r.request_next_now('pinky2') == 'BL_J'             # pinky2 의 다음 엣지(J→BL)는 pinky1 이 쥔 엣지다
    r.step(tick=2)
    assert r.node_holder.get('J') is None and r.robots['pinky2'].waiting_for == 'BL_J'
    assert r.status('pinky2').get('blocked_by') == 'pinky1'
    assert r.request_next_now('pinky1') == 'J_TR'
    r.step(tick=3)
    assert r.node_holder.get('J') == 'pinky1' and r.edge_holder.get('J_TR') == 'pinky1'
    assert r.robots['pinky2'].waiting_for == 'BL_J'           # 여전히 기다린다 — J 앞에서 선다
    assert r.clear_until('pinky2') < r.robots['pinky2'].route.node_idx[1]
    r.mark_arrived('pinky1')                                  # pinky1 이 TR 에 도착하면 BL_J·J 를 놓는다
    r.step(tick=4)
    assert r.edge_holder.get('BL_J') == 'pinky2' and r.node_holder.get('J') == 'pinky2'
    assert r.node_holder.get('TR') == 'pinky1'                # 도착한 로봇은 목표 노드만 계속 잡는다
    assert r.clear_until('pinky2') == r.robots['pinky2'].route.goal_idx
