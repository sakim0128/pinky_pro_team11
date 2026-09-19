"""그래프 편집기 headless 테스트 (QT_QPA_PLATFORM=offscreen, ROS 없이)."""

import os
import sys

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('PyQt5')
pytest.importorskip('numpy')
pytest.importorskip('cv2')

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(PKG_ROOT)
sys.path.insert(0, PKG_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, 'pinky_fleet_station'))

from PyQt5.QtWidgets import QApplication  # noqa: E402

from pinky_lane_station.graph_editor import (  # noqa: E402
    GraphCanvas, GraphEditorWindow, MODE_EDGE, compute_homography, warp_photo_to_map,
)
from pinky_lane_station.road_graph import RoadGraph  # noqa: E402

MAP_YAML = os.path.join(REPO_ROOT, 'pinky_fleet_station', 'config', 'map4.yaml')
GRAPH = os.path.join(PKG_ROOT, 'config', 'road_graph.yaml')
PHOTO = os.path.join(REPO_ROOT, 'docs', 'course_aerial.jpg')


@pytest.fixture(scope='session')
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


def test_window_loads_map_and_graph(qapp):
    win = GraphEditorWindow(MAP_YAML, GRAPH)
    assert win.canvas.map_data.width == 47
    assert len(win.canvas.graph.nodes) >= 10
    win.canvas.resize(800, 500)
    win.canvas.fit_to_view()
    p = win.canvas.world_to_screen(0.0, 0.0)
    x, y = win.canvas.screen_to_world(p)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-9)
    # paintEvent 까지 실제로 돌려 본다 (맵 픽스맵·경로·노드 그리기)
    win.canvas.preview_routes('BL', 'TC', 'BL', 'RE')
    shot = win.canvas.grab()
    assert not shot.isNull() and shot.width() > 0


def test_edit_and_save_round_trip(qapp, tmp_path):
    canvas = GraphCanvas()
    a = canvas.add_node(0.0, 0.0, 'endpoint', label='시작')
    b = canvas.add_node(1.0, 0.0, 'junction')
    c = canvas.add_node(1.0, 1.0, 'endpoint')
    e1 = canvas.add_edge(a, b, [(0.5, 0.05)])
    e2 = canvas.add_edge(b, c)
    canvas.toggle_oneway(e2)
    canvas.move_node(b, 1.1, 0.0)
    assert canvas.graph.edges[e1].waypoints[-1] == (1.1, 0.0)
    assert canvas.graph.edges[e2].waypoints[0] == (1.1, 0.0)
    canvas.move_waypoint(e1, 1, 0.5, 0.1)
    path = tmp_path / 'g.yaml'
    canvas.graph.save(path)
    g = RoadGraph.load(path)
    assert g.edges[e2].oneway is True
    assert g.edges[e1].waypoints[1] == (0.5, 0.1)
    assert g.nodes[a].label == '시작'
    r, _ = canvas.preview_routes(a, c)
    assert r.edge_ids == [e1, e2]
    canvas.delete_node(b)
    assert canvas.graph.edges == {}


def test_edge_mode_state_machine(qapp):
    canvas = GraphCanvas()
    a = canvas.add_node(0.0, 0.0)
    b = canvas.add_node(1.0, 0.0)
    canvas.mode = MODE_EDGE
    canvas.pending_edge = {'from': a, 'points': [(0.5, 0.1)]}
    eid = canvas.add_edge(canvas.pending_edge['from'], b, canvas.pending_edge['points'])
    canvas.pending_edge = None
    assert canvas.graph.edges[eid].waypoints == [(0.0, 0.0), (0.5, 0.1), (1.0, 0.0)]


def test_auto_ids_do_not_collide(qapp):
    canvas = GraphCanvas()
    ids = {canvas.add_node(i * 0.1, 0.0, 'junction') for i in range(3)}
    assert ids == {'J1', 'J2', 'J3'}
    e = canvas.add_edge('J1', 'J2')
    e2 = canvas.add_edge('J1', 'J2')
    assert e != e2


def test_homography_and_warp(qapp):
    from pinky_fleet_station.map_canvas import MapData
    m = MapData(MAP_YAML)
    # 사진 4 모서리를 맵 4 모서리에 대응시키면 사진 전체가 맵에 워프된다
    import cv2
    ph, pw = cv2.imread(PHOTO).shape[:2]
    photo_px = [[0, 0], [pw, 0], [pw, ph], [0, ph]]
    x0, y0 = m.origin_x, m.origin_y
    x1, y1 = x0 + m.width * m.resolution, y0 + m.height * m.resolution
    map_xy = [[x0, y1], [x1, y1], [x1, y0], [x0, y0]]
    H = compute_homography(photo_px, map_xy)
    img = warp_photo_to_map(PHOTO, H, m, upscale=4)
    assert img.width() == m.width * 4 and img.height() == m.height * 4
    with pytest.raises(ValueError):
        compute_homography(photo_px[:3], map_xy[:3])
