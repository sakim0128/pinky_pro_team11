"""road_graph 단위 테스트 (ROS 없이 돈다)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.road_graph import (  # noqa: E402
    RoadGraph, RoadGraphError, project_to_polyline, resample,
)

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(PKG_ROOT, 'config', 'road_graph.yaml')


def diamond(**overrides):
    r"""A -> B -> D 위쪽, A -> C -> D 아래쪽. 위쪽이 짧다.

        B (0,1)
       /  \
    A(0,0) D(2,0)
       \  /
        C (1,-2)
    """
    data = {
        'lane_width': 0.2,
        'nodes': [
            {'id': 'A', 'x': 0.0, 'y': 0.0, 'type': 'endpoint'},
            {'id': 'B', 'x': 0.0, 'y': 1.0, 'type': 'junction'},
            {'id': 'C', 'x': 1.0, 'y': -2.0, 'type': 'crosswalk'},
            {'id': 'D', 'x': 2.0, 'y': 0.0, 'type': 'endpoint'},
        ],
        'edges': [
            {'id': 'AB', 'from': 'A', 'to': 'B', 'waypoints': [[0, 0], [0, 1]]},
            {'id': 'BD', 'from': 'B', 'to': 'D', 'waypoints': [[0, 1], [2, 0]]},
            {'id': 'AC', 'from': 'A', 'to': 'C', 'waypoints': [[0, 0], [1, -2]]},
            {'id': 'CD', 'from': 'C', 'to': 'D', 'waypoints': [[1, -2], [2, 0]]},
        ],
    }
    data.update(overrides)
    return data


# ------------------------------------------------------------ 기하

def test_resample_spacing_and_endpoints():
    pts = resample([(0, 0), (1, 0), (1, 1)], step=0.25)
    assert pts[0] == (0, 0) and pts[-1] == (1, 1)
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        assert math.hypot(x1 - x0, y1 - y0) <= 0.25 + 1e-9
    assert len(pts) == 9  # 2.0 m / 0.25 = 8 구간


def test_project_left_positive():
    s, lateral, qx, qy, dist = project_to_polyline(1.0, 0.3, [(0, 0), (2, 0)])
    assert s == pytest.approx(1.0)
    assert lateral == pytest.approx(0.3)      # 진행 방향(+x) 기준 좌측(+y) 이 양수
    assert (qx, qy) == pytest.approx((1.0, 0.0))
    s, lateral, *_ = project_to_polyline(1.0, -0.3, [(0, 0), (2, 0)])
    assert lateral == pytest.approx(-0.3)


def test_project_clamps_to_segment_end():
    s, _, qx, qy, dist = project_to_polyline(5.0, 0.0, [(0, 0), (2, 0)])
    assert s == pytest.approx(2.0) and (qx, qy) == (2.0, 0.0) and dist == pytest.approx(3.0)


# ------------------------------------------------------------ 로더 검증

@pytest.mark.parametrize('mutate, message', [
    (lambda d: d['edges'][0].update({'to': 'ZZ'}), 'to 노드 ZZ'),
    (lambda d: d['edges'][0].update({'waypoints': [[0, 0]]}), '2개 이상'),
    (lambda d: d['edges'][0].update({'waypoints': [[0.5, 0], [0, 1]]}), 'from 노드'),
    (lambda d: d['edges'][0].update({'waypoints': [[0, 0], [0, 0.5]]}), 'to 노드'),
    (lambda d: d['nodes'][0].update({'type': 'castle'}), 'type'),
    (lambda d: d['edges'][0].update({'to': 'A'}), '같습니다'),
])
def test_loader_rejects_bad_graph(mutate, message):
    data = diamond()
    mutate(data)
    with pytest.raises(RoadGraphError, match=message):
        RoadGraph.from_dict(data)


def test_round_trip(tmp_path):
    g = RoadGraph.from_dict(diamond())
    path = tmp_path / 'g.yaml'
    g.save(path)
    g2 = RoadGraph.load(path)
    assert set(g2.nodes) == set(g.nodes) and set(g2.edges) == set(g.edges)
    assert g2.edges['BD'].waypoints == g.edges['BD'].waypoints


# ------------------------------------------------------------ 경로

def test_dijkstra_picks_shorter_branch():
    r = RoadGraph.from_dict(diamond()).shortest_route('A', 'D')
    assert r.edge_ids == ['AB', 'BD']
    assert r.node_ids == ['A', 'B', 'D']
    assert r.junction_idx == [r.node_idx[1]]
    assert r.crosswalk_idx == []
    assert r.goal_idx == len(r.waypoints) - 1
    assert r.length == pytest.approx(1.0 + math.hypot(2, 1), abs=1e-6)


def test_dijkstra_respects_oneway():
    data = diamond()
    data['edges'][1]['oneway'] = True          # BD: B->D 만
    g = RoadGraph.from_dict(data)
    back = g.shortest_route('D', 'A')
    assert back.edge_ids == ['CD', 'AC']      # BD 를 역방향으로 못 쓴다
    assert back.edge_forward == [False, False]


def test_route_backward_edge_reverses_points():
    g = RoadGraph.from_dict(diamond())
    r = g.shortest_route('D', 'A')
    assert r.waypoints[0] == pytest.approx((2.0, 0.0))
    assert r.waypoints[-1] == pytest.approx((0.0, 0.0))


def test_no_route_raises():
    data = diamond()
    data['edges'] = data['edges'][:1]
    with pytest.raises(RoadGraphError, match='경로가 없습니다'):
        RoadGraph.from_dict(data).shortest_route('A', 'D')


def test_edge_end_idx_matches_geometry():
    g = RoadGraph.from_dict(diamond())
    r = g.shortest_route('A', 'C')
    assert r.edge_end_idx == [len(r.waypoints) - 1]
    r = g.shortest_route('A', 'D')
    bx, by = r.waypoints[r.edge_end_idx[0]]
    assert (bx, by) == pytest.approx((0.0, 1.0), abs=0.06)
    assert r.crosswalk_idx == []
    r2 = g.shortest_route('A', 'D', step=0.5)
    assert len(r2.waypoints) < len(r.waypoints)


def test_route_with_crosswalk_marks_index():
    data = diamond()
    data['edges'][1]['oneway'] = True
    g = RoadGraph.from_dict(data)
    r = g.shortest_route('D', 'A')            # D -> C -> A
    assert r.node_ids == ['D', 'C', 'A']
    assert r.crosswalk_idx == [r.node_idx[1]]
    cx, cy = r.waypoints[r.crosswalk_idx[0]]
    assert (cx, cy) == pytest.approx((1.0, -2.0), abs=0.06)


def test_shared_edges_direction():
    g = RoadGraph.from_dict(diamond())
    a = g.shortest_route('A', 'D')            # AB, BD 정방향
    b = g.shortest_route('D', 'A')            # BD 역, AB 역 (양방향이라 위쪽이 여전히 짧다)
    shared = dict(RoadGraph.shared_edges(a, b))
    assert shared == {'AB': False, 'BD': False}
    c = g.build_route([('AB', True)], 'A')
    assert RoadGraph.shared_edges(a, c) == [('AB', True)]


# ------------------------------------------------------------ 조회

def test_nearest_node_and_exit_yaw():
    g = RoadGraph.from_dict(diamond())
    assert g.nearest_node(0.1, -0.1) == 'A'
    assert g.nearest_node(0.1, -0.1, max_dist=0.05) is None
    assert g.nearest_node(0.0, 0.9, types=('junction',)) == 'B'
    assert g.node_exit_yaw('A', 'AB') == pytest.approx(math.pi / 2)
    assert g.node_exit_yaw('B', 'AB') == pytest.approx(-math.pi / 2)   # B 에서 AB 를 거꾸로


def test_graph_project_returns_nearest_edge():
    g = RoadGraph.from_dict(diamond())
    p = g.project(0.05, 0.5)
    assert p.edge_id == 'AB'
    assert p.s == pytest.approx(0.5)
    assert p.lateral == pytest.approx(-0.05)   # +x 쪽은 AB(+y 진행) 기준 우측


def test_isolated_nodes():
    data = diamond()
    data['nodes'].append({'id': 'Z', 'x': 9, 'y': 9})
    assert RoadGraph.from_dict(data).isolated_nodes() == ['Z']


# ------------------------------------------------------------ 실제 파일

def test_example_graph_loads_and_routes():
    g = RoadGraph.load(EXAMPLE)
    assert not g.isolated_nodes()
    endpoints = [n.id for n in g.nodes.values() if n.type == 'endpoint']
    assert len(endpoints) >= 2
    for src in endpoints:
        for dst in endpoints:
            if src != dst:
                g.shortest_route(src, dst)    # 모든 끝점 쌍이 연결되어야 한다
