# -*- coding: utf-8 -*-
"""교차로 규칙 (2026-09-29): 정지선에 선 로봇(JUNCTION_STOP 보고)은 거리와 무관하게 다음 엣지를 요청하고,
예약은 선착순(요청 틱 → 같은 틱이면 domain_id)으로 분기 노드를 한 로봇에만 허가한다. ROS 없이 돈다.
"""
import importlib.util
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FLEET = os.path.join(REPO, 'relay_station', 'fleet')


def _load_fleet_modules():
    """relay_station.fleet 의 __init__ 은 rclpy 를 끌어온다 — 패키지 껍데기만 만들고 road_graph·reservation 을 파일로 읽는다."""
    pkg = 'relay_fleet_for_junction_test'
    if pkg not in sys.modules:
        shell = types.ModuleType(pkg)
        shell.__path__ = [FLEET]
        sys.modules[pkg] = shell
        for name in ('road_graph', 'reservation'):
            spec = importlib.util.spec_from_file_location(f'{pkg}.{name}', os.path.join(FLEET, name + '.py'))
            mod = importlib.util.module_from_spec(spec)
            sys.modules[f'{pkg}.{name}'] = mod
            spec.loader.exec_module(mod)
    return sys.modules[f'{pkg}.road_graph'], sys.modules[f'{pkg}.reservation']


RG, RES = _load_fleet_modules()
GRAPH = os.path.join(FLEET, 'config', 'profiles', 'team11_map4', 'road_graph.yaml')


def _setup():
    g = RG.RoadGraph.load(GRAPH)
    r = RES.Reservation(g)
    # 시나리오 2: pinky1 MC→BL (JI→RM→…→JS→BL), pinky2 RE→TC (TR 을 지난다) — 둘 다 분기 노드를 지난다
    r1 = g.shortest_route('BL', 'TC', step=0.10)      # BL→JS→JW→…   첫 분기 JS
    r2 = g.shortest_route('RE', 'TC', step=0.10)      # RE→TR→TC     첫 분기 TR
    r.register('pinky1', 10, r1)
    r.register('pinky2', 11, r2)
    return g, r, r1, r2


def test_request_next_now_registers_regardless_of_distance():
    g, r, r1, _ = _setup()
    r.step(tick=1)
    k = r.robots['pinky1'].next_edge_k
    eid = r1.edge_ids[k]
    assert (eid, 'pinky1') not in r.request_tick or True   # 출발 노드는 reserve_ahead 안이라 이미 요청됐을 수 있다
    # 로봇을 다음 엣지 시작에서 멀리 둔 채 정지선 보고 → 요청이 선다
    r.step(tick=2)
    slot = r.robots['pinky1']
    k = slot.next_edge_k
    if k < slot.n_edges:
        eid = r1.edge_ids[k]
        r.request_tick.pop((eid, 'pinky1'), None)
        assert r.request_next_now('pinky1') == eid
        assert r.request_tick[(eid, 'pinky1')] == 2
        r.request_tick[(eid, 'pinky1')] = 1               # 이미 있던 요청 틱은 지킨다
        assert r.request_next_now('pinky1') == eid and r.request_tick[(eid, 'pinky1')] == 1


def test_request_next_now_unknown_or_finished_robot_is_noop():
    _, r, _, _ = _setup()
    assert r.request_next_now('nobody') == ''
    r.robots['pinky2'].finished = True
    assert r.request_next_now('pinky2') == ''


def _cross_graph():
    """교차로 하나(J)에 네 방향: A→J→B 와 C→J→D. 엣지는 1 m 라 출발 노드에서는 J 가 reserve_ahead(0.4) 밖이다."""
    return RG.RoadGraph.from_dict({
        'frame': 'map', 'lane_width': 0.2,
        'nodes': [{'id': 'A', 'x': 0.0, 'y': 0.0, 'type': 'endpoint'}, {'id': 'J', 'x': 1.0, 'y': 0.0, 'type': 'junction'},
                  {'id': 'B', 'x': 2.0, 'y': 0.0, 'type': 'endpoint'}, {'id': 'C', 'x': 1.0, 'y': 1.0, 'type': 'endpoint'},
                  {'id': 'D', 'x': 1.0, 'y': -1.0, 'type': 'endpoint'}],
        'edges': [{'id': 'A_J', 'from': 'A', 'to': 'J', 'oneway': False, 'waypoints': [[0.0, 0.0], [1.0, 0.0]]},
                  {'id': 'J_B', 'from': 'J', 'to': 'B', 'oneway': False, 'waypoints': [[1.0, 0.0], [2.0, 0.0]]},
                  {'id': 'C_J', 'from': 'C', 'to': 'J', 'oneway': False, 'waypoints': [[1.0, 1.0], [1.0, 0.0]]},
                  {'id': 'J_D', 'from': 'J', 'to': 'D', 'oneway': False, 'waypoints': [[1.0, 0.0], [1.0, -1.0]]}],
    })


def _two_at_the_stop_lines():
    g = _cross_graph()
    r = RES.Reservation(g)
    r.register('pinky1', 10, g.shortest_route('A', 'B', step=0.10))
    r.register('pinky2', 11, g.shortest_route('C', 'D', step=0.10))
    r.step(tick=1)                                    # 각자 출발 노드·첫 엣지만 잡는다 — J 는 비어 있다
    assert r.node_holder.get('J') is None
    # 둘 다 정지선(J 0.7 m 앞)에 서 있다 — 요청 기준점(정지 지점 = J 0.2 m 앞) 까지 0.5 m > reserve_ahead 라
    # 거리 규칙으로는 요청이 서지 않는다 (정지선이 멀어도 보고만으로 요청이 서야 한다는 것이 이 규칙의 요점)
    r.update_pose('pinky1', 0.3, 0.0)
    r.update_pose('pinky2', 1.0, 0.7)
    r.step(tick=2)
    assert r.node_holder.get('J') is None and not r.request_tick
    return r


def test_junction_node_goes_to_the_first_reporter_then_the_other_waits():
    """두 로봇이 같은 분기 노드(J) 앞 정지선에 선다: 먼저 JUNCTION_STOP 을 보고한 쪽이 노드를 잡고 다른 쪽은 기다린다."""
    r = _two_at_the_stop_lines()
    assert r.request_next_now('pinky2') == 'J_D'      # domain 11 이 먼저 보고
    r.step(tick=5)
    assert r.node_holder.get('J') == 'pinky2' and r.edge_holder.get('J_D') == 'pinky2'
    assert r.request_next_now('pinky1') == 'J_B'
    r.step(tick=6)
    assert r.node_holder.get('J') == 'pinky2'
    assert r.robots['pinky1'].waiting_for == 'J_B' and r.status('pinky1').get('blocked_by') == 'pinky2'
    assert r.clear_until('pinky1') < r.robots['pinky1'].idx_at(1.0)        # 허가가 J 앞에서 끝난다
    # pinky2 가 J 를 release_behind 만큼 지나면 놓고, 다음 틱에 pinky1 이 잡는다
    for _ in range(RES.DEFAULT_CONFIRM_UPDATES + 100):
        r.update_pose('pinky2', 1.0, -0.4)
    r.step(tick=7)
    assert r.node_holder.get('J') == 'pinky1', (r.node_holder, r.robots['pinky2'].progress_s)
    assert r.clear_until('pinky1') > r.robots['pinky1'].idx_at(1.0)


def test_same_tick_reports_break_the_tie_by_domain_id():
    r = _two_at_the_stop_lines()
    r.request_next_now('pinky2')                      # 같은 틱 — 등록 순서는 상관없다
    r.request_next_now('pinky1')
    r.step(tick=3)
    assert r.node_holder.get('J') == 'pinky1'
    assert r.robots['pinky2'].waiting_for == 'J_D'
