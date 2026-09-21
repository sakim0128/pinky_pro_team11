"""graph_recorder — 합성 궤적(사각 루프 + 분기)에서 노드 병합·엣지 분할·재샘플·경고 (ROS 불필요)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.graph_recorder import GraphRecorder, RecorderParams  # noqa: E402
from pinky_lane_station.road_graph import RoadGraph  # noqa: E402


def drive(rec, x0, y0, x1, y1, n=20, noise=0.0, t0=0.0, localized=True):
    """직선 구간을 n 개 pose 로. 약간의 잡음(중심선 흔들림)."""
    import random
    rng = random.Random(1)
    for i in range(1, n + 1):
        u = i / n
        x = x0 + (x1 - x0) * u + rng.uniform(-noise, noise)
        y = y0 + (y1 - y0) * u + rng.uniform(-noise, noise)
        rec.add_pose(t0 + i * 0.1, x, y, math.atan2(y1 - y0, x1 - x0), localized)
    return t0 + n * 0.1


def loop_with_branch(noise=0.01):
    """A(0,0) → B(1,0) → C(1,1) → D(0,1) → A, 그리고 B → E(1,-1) 분기. A 는 세 번 찍힌다."""
    rec = GraphRecorder(RecorderParams(step=0.05))
    rec.add_pose(0.0, 0.0, 0.0, 0.0)
    rec.mark('A', 'e')
    t = drive(rec, 0, 0, 1, 0, noise=noise)
    rec.mark('B', 'j')
    t = drive(rec, 1, 0, 1, 1, noise=noise, t0=t)
    rec.mark('C')
    t = drive(rec, 1, 1, 0, 1, noise=noise, t0=t)
    rec.mark('D', 'c')
    t = drive(rec, 0, 1, 0, 0, noise=noise, t0=t)
    rec.mark('A', 'e')                          # 같은 노드 재방문
    t = drive(rec, 0, 0, 1, 0, noise=noise, t0=t)
    rec.mark('B', 'j')                          # A→B 두 번째 = 중복 엣지 (버림)
    t = drive(rec, 1, 0, 1, -1, noise=noise, t0=t)
    rec.mark('E', 'e')
    return rec


def test_nodes_merged_and_typed():
    rec = loop_with_branch()
    g, warnings = rec.build()
    assert set(g.nodes) == {'A', 'B', 'C', 'D', 'E'}
    assert g.nodes['B'].type == 'junction' and g.nodes['D'].type == 'crosswalk'
    assert g.nodes['C'].type == 'waypoint'
    assert math.hypot(g.nodes['A'].x, g.nodes['A'].y) < 0.02       # 세 번 찍은 평균


def test_edges_split_resampled_and_deduped():
    rec = loop_with_branch()
    g, warnings = rec.build()
    assert set(g.edges) == {'A_B', 'B_C', 'C_D', 'D_A', 'B_E'}
    assert any('이미 있음' in w for w in warnings)                  # 두 번째 A→B
    e = g.edges['A_B']
    assert e.waypoints[0] == (g.nodes['A'].x, g.nodes['A'].y)
    assert e.waypoints[-1] == (g.nodes['B'].x, g.nodes['B'].y)
    gaps = [math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(e.waypoints, e.waypoints[1:])]
    assert max(gaps) <= 0.051 and 0.95 <= e.length <= 1.05
    # 평활 후 중심선 흔들림이 잡음보다 작다
    assert max(abs(y) for _, y in e.waypoints) < 0.01


def test_route_works_on_recorded_graph():
    g, _ = loop_with_branch().build()
    r = g.shortest_route('E', 'D')
    assert r.node_ids == ['E', 'B', 'C', 'D'] or r.node_ids == ['E', 'B', 'A', 'D']
    assert r.junction_idx and r.crosswalk_idx


def test_pause_segment_dropped_and_restart_by_remark():
    rec = GraphRecorder()
    rec.add_pose(0, 0, 0, 0)
    rec.mark('A')
    t = drive(rec, 0, 0, 1, 0)
    rec.mark('B')
    rec.pause()
    t = drive(rec, 1, 0, 0.5, 0.5, t0=t)         # 들고 옮김
    rec.resume()
    rec.mark('B')                                 # 재시작점 다시 찍기
    t = drive(rec, 0.5, 0.5, 0.5, 1.5, t0=t)
    rec.mark('C')
    g, warnings = rec.build()
    assert set(g.edges) == {'A_B', 'B_C'}
    assert any('일시정지' in w for w in warnings)
    # B 는 (1,0) 과 (0.5,0.5) 의 평균 — 재배치가 있었다는 뜻이므로 경고는 없어도 좌표는 평균
    assert abs(g.nodes['B'].x - 0.75) < 1e-9


def test_unlocalized_segment_dropped_and_short_segment_ignored():
    rec = GraphRecorder()
    rec.add_pose(0, 0, 0, 0)
    rec.mark('A')
    t = drive(rec, 0, 0, 1, 0, localized=False)
    rec.mark('B')
    assert any('localized=False' in w for w in rec.warnings)
    rec.add_pose(t + 0.1, 1, 0, 0)                 # 위치 복구
    rec.mark('B')
    t = drive(rec, 1, 0, 1.02, 0, n=3, t0=t)
    rec.mark('C')                                  # 2 cm — 너무 짧다
    g, warnings = rec.build()
    assert 'A_B' not in g.edges and 'B_C' not in g.edges
    assert any('너무 짧아' in w for w in warnings)


def test_undo_and_near_node_warning():
    rec = GraphRecorder()
    rec.add_pose(0, 0, 0, 0)
    rec.mark('A')
    drive(rec, 0, 0, 0.05, 0, n=2)
    rec.mark('Aa')                                 # 5 cm 옆에 다른 이름 → 경고
    assert any('같은 노드면' in w for w in rec.warnings)
    assert rec.undo() == 'Aa'
    assert list(rec.node_coords()) == ['A']
    with pytest.raises(ValueError):
        rec.mark('', 'j')
    with pytest.raises(ValueError):
        rec.mark('X', 'z')


def test_saved_yaml_reloads(tmp_path):
    g, _ = loop_with_branch().build()
    p = tmp_path / 'g.yaml'
    g.save(str(p))
    g2 = RoadGraph.load(str(p))
    assert set(g2.edges) == set(g.edges)
    rec = loop_with_branch()
    rec.write_csv(str(tmp_path / 't.csv'))
    assert (tmp_path / 't.csv').read_text(encoding='utf-8').count('\n') == len(rec.poses) + 1
