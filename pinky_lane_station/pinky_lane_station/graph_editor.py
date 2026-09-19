"""도로망 그래프 편집기 (PyQt5, ROS 불필요).

    python3 -m pinky_lane_station.graph_editor \
        --map   pinky_fleet_station/config/map4.yaml \
        --graph pinky_lane_station/config/road_graph.yaml \
        --photo docs/course_aerial.jpg

라이다 맵을 맵 방향(기존 fleet_gui 와 동일)으로 그리고, 항공 사진을 호모그래피로 맵
프레임에 워프해 반투명으로 겹친다. 그 위에 노드·엣지를 찍고 Ctrl+S 로 저장한다.

모드 (툴바 / 단축키)
  1 선택   노드·중간점 드래그 이동, Del 삭제, O 로 엣지 oneway 토글,
           Shift+클릭 두 노드 → 경로 미리보기 (Ctrl+Shift+클릭 → 두 번째 경로, 공유 엣지 빨강)
  2 노드   빈 곳 클릭 → 노드 추가 (타입은 콤보박스)
  3 엣지   노드 클릭 → 중간점 클릭… → 노드 클릭으로 완성. Esc 취소
  R 정합   사진에서 외벽 모서리 등 기준점을 찍어 맵과 대응시킨다
휠 줌, 오른쪽 드래그 팬, F 화면에 맞추기.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFileDialog,
    QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
    QScrollArea, QToolBar, QVBoxLayout, QWidget,
)

try:
    from pinky_fleet_station.map_canvas import MapData
except ImportError:  # 소스 트리에서 바로 실행할 때
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                    'pinky_fleet_station'))
    from pinky_fleet_station.map_canvas import MapData

from .road_graph import NODE_TYPES, Edge, Node, RoadGraph, RoadGraphError

MODE_SELECT, MODE_NODE, MODE_EDGE = 'select', 'node', 'edge'
NODE_COLOR = {'endpoint': QColor(40, 180, 40), 'junction': QColor(220, 60, 30),
              'crosswalk': QColor(255, 200, 0), 'waypoint': QColor(150, 150, 150)}
EDGE_COLOR = QColor(30, 120, 200)
PICK_PX = 10          # 화면 픽셀 기준 클릭 판정 반경
PHOTO_UPSCALE = 20    # 사진을 맵 픽셀의 몇 배 해상도로 워프할지


# ------------------------------------------------------------------ 사진 정합

def compute_homography(photo_px, map_xy):
    """사진 픽셀 → 맵 좌표 (m) 호모그래피. 4점 이상."""
    import cv2
    src = np.array(photo_px, np.float32)
    dst = np.array(map_xy, np.float32)
    if len(src) < 4:
        raise ValueError('정합에는 대응점이 4개 이상 필요합니다')
    H, _ = cv2.findHomography(src, dst, 0)
    if H is None:
        raise ValueError('호모그래피를 구할 수 없습니다 (점이 한 직선 위에 있나요)')
    return H


def warp_photo_to_map(photo_path, homography, map_data, upscale=PHOTO_UPSCALE):
    """사진을 맵 픽셀 프레임(upscale 배)으로 워프한 QImage 를 돌려준다."""
    import cv2
    photo = cv2.imread(os.path.expanduser(photo_path))
    if photo is None:
        raise ValueError(f'사진을 읽을 수 없습니다: {photo_path}')
    r = map_data.resolution / upscale
    W, H = map_data.width * upscale, map_data.height * upscale
    # 맵 좌표(m) → 출력 픽셀 : px = (x - ox)/r , py = H - (y - oy)/r
    to_px = np.array([[1 / r, 0, -map_data.origin_x / r],
                      [0, -1 / r, H + map_data.origin_y / r],
                      [0, 0, 1]], np.float64)
    warped = cv2.warpPerspective(photo, to_px @ homography, (W, H))
    rgb = cv2.cvtColor(warped, cv2.COLOR_BGR2RGB)
    return QImage(rgb.data, W, H, 3 * W, QImage.Format_RGB888).copy()


class RegisterDialog(QDialog):
    """사진 위에 기준점을 찍고 맵 좌표를 입력한다."""

    def __init__(self, photo_path, map_data, existing=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle('사진 ↔ 맵 정합')
        self.resize(1100, 800)
        self._pix = QPixmap(os.path.expanduser(photo_path))
        self._scale = min(900 / max(1, self._pix.width()), 760 / max(1, self._pix.height()), 1.0)
        self.points = []            # [(photo_x, photo_y)]
        self.rows = []              # [(QDoubleSpinBox x, QDoubleSpinBox y)]

        layout = QHBoxLayout(self)
        self._label = _ClickLabel(self._on_click)
        self._label.setPixmap(self._pix.scaled(
            int(self._pix.width() * self._scale), int(self._pix.height() * self._scale),
            Qt.KeepAspectRatio, Qt.SmoothTransformation))
        scroll = QScrollArea()
        scroll.setWidget(self._label)
        layout.addWidget(scroll, 3)

        side = QVBoxLayout()
        side.addWidget(QLabel('사진을 클릭해 기준점을 찍고 오른쪽에 맵 좌표(m)를 넣으세요.\n'
                              '외벽 안쪽 모서리 4개가 가장 좋습니다. 4점 이상.'))
        self._grid = QGridLayout()
        side.addLayout(self._grid)
        # 기본값: 맵 안쪽 모서리 (좌하 → 우하 → 우상 → 좌상)
        m = map_data
        x0, y0 = m.origin_x + m.resolution, m.origin_y + m.resolution
        x1 = m.origin_x + (m.width - 1) * m.resolution
        y1 = m.origin_y + (m.height - 1) * m.resolution
        self._defaults = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        clear = QPushButton('점 지우기')
        clear.clicked.connect(self._clear)
        side.addWidget(clear)
        side.addStretch(1)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        side.addWidget(buttons)
        layout.addLayout(side, 1)

        if existing:
            for (px, py), (mx, my) in zip(existing.get('photo_px', []), existing.get('map_xy', [])):
                self._add_point(px, py, mx, my)

    def _on_click(self, x, y):
        px, py = x / self._scale, y / self._scale
        i = len(self.points)
        mx, my = self._defaults[i] if i < 4 else (0.0, 0.0)
        self._add_point(px, py, mx, my)

    def _add_point(self, px, py, mx, my):
        i = len(self.points)
        self.points.append((float(px), float(py)))
        sx, sy = QDoubleSpinBox(), QDoubleSpinBox()
        for s, v in ((sx, mx), (sy, my)):
            s.setRange(-50, 50)
            s.setDecimals(3)
            s.setSingleStep(0.01)
            s.setValue(float(v))
        self._grid.addWidget(QLabel(f'#{i + 1} ({px:.0f}, {py:.0f}) →'), i, 0)
        self._grid.addWidget(sx, i, 1)
        self._grid.addWidget(sy, i, 2)
        self.rows.append((sx, sy))
        self._label.marks = [(x * self._scale, y * self._scale) for x, y in self.points]
        self._label.update()

    def _clear(self):
        self.points.clear()
        for i in reversed(range(self._grid.count())):
            w = self._grid.itemAt(i).widget()
            if w:
                w.setParent(None)
        self.rows.clear()
        self._label.marks = []
        self._label.update()

    def registration(self):
        return {
            'photo_px': [[round(x, 1), round(y, 1)] for x, y in self.points],
            'map_xy': [[round(sx.value(), 3), round(sy.value(), 3)] for sx, sy in self.rows],
        }


class _ClickLabel(QLabel):
    def __init__(self, on_click):
        super().__init__()
        self._on_click = on_click
        self.marks = []

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._on_click(event.pos().x(), event.pos().y())

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setPen(QPen(QColor(255, 40, 40), 2))
        for i, (x, y) in enumerate(self.marks):
            painter.drawEllipse(QPointF(x, y), 6, 6)
            painter.drawText(QPointF(x + 8, y - 8), str(i + 1))
        painter.end()


# ------------------------------------------------------------------ 캔버스

class GraphCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 400)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.map_data = None
        self.photo = None               # 워프된 QImage (맵 픽셀 * PHOTO_UPSCALE)
        self.photo_alpha = 0.55
        self.graph = RoadGraph()
        self.mode = MODE_SELECT
        self.node_type = 'waypoint'
        self.scale = 200.0              # 화면 px / m
        self.tx = self.ty = 0.0
        self.selected = None            # ('node', id) | ('edge', id, wp_index|None)
        self.dragging = None
        self.pending_edge = None        # {'from': id, 'points': [(x,y)]}
        self.route_a = self.route_b = None
        self.route_nodes = []           # 미리보기용 선택 노드
        self._pan_from = None
        self.status_cb = lambda text: None
        self.dirty = False

    # ------------------------------------------------ 좌표

    def world_to_screen(self, wx, wy):
        return QPointF(wx * self.scale + self.tx, -wy * self.scale + self.ty)

    def screen_to_world(self, p):
        return (p.x() - self.tx) / self.scale, -(p.y() - self.ty) / self.scale

    def fit_to_view(self):
        if not self.map_data:
            return
        m = self.map_data
        w_m, h_m = m.width * m.resolution, m.height * m.resolution
        self.scale = min(self.width() / w_m, self.height() / h_m) * 0.92
        cx = m.origin_x + w_m / 2
        cy = m.origin_y + h_m / 2
        self.tx = self.width() / 2 - cx * self.scale
        self.ty = self.height() / 2 + cy * self.scale
        self.update()

    # ------------------------------------------------ 그래프 조작 (테스트에서 직접 호출)

    def add_node(self, x, y, node_type=None, node_id=None, label=''):
        node_type = node_type or self.node_type
        if node_id is None:
            prefix = {'endpoint': 'E', 'junction': 'J', 'crosswalk': 'C', 'waypoint': 'W'}[node_type]
            k = 1
            while f'{prefix}{k}' in self.graph.nodes:
                k += 1
            node_id = f'{prefix}{k}'
        self.graph.nodes[node_id] = Node(node_id, float(x), float(y), node_type, label)
        self.dirty = True
        self.update()
        return node_id

    def delete_node(self, node_id):
        for eid in [e.id for e in self.graph.edges.values()
                    if node_id in (e.from_id, e.to_id)]:
            del self.graph.edges[eid]
        self.graph.nodes.pop(node_id, None)
        self.selected = None
        self.dirty = True
        self.update()

    def add_edge(self, from_id, to_id, mid_points=(), edge_id=None, oneway=False):
        a, b = self.graph.nodes[from_id], self.graph.nodes[to_id]
        pts = [(a.x, a.y)] + [tuple(p) for p in mid_points] + [(b.x, b.y)]
        if edge_id is None:
            edge_id = f'{from_id}_{to_id}'
            k = 2
            while edge_id in self.graph.edges:
                edge_id = f'{from_id}_{to_id}_{k}'
                k += 1
        self.graph.edges[edge_id] = Edge(edge_id, from_id, to_id, pts, oneway)
        self.dirty = True
        self.update()
        return edge_id

    def delete_edge(self, edge_id):
        self.graph.edges.pop(edge_id, None)
        self.selected = None
        self.dirty = True
        self.update()

    def move_node(self, node_id, x, y):
        node = self.graph.nodes[node_id]
        node.x, node.y = float(x), float(y)
        for edge in self.graph.edges.values():
            if edge.from_id == node_id:
                edge.waypoints[0] = (node.x, node.y)
            if edge.to_id == node_id:
                edge.waypoints[-1] = (node.x, node.y)
            edge.__post_init__()
        self.dirty = True

    def move_waypoint(self, edge_id, index, x, y):
        edge = self.graph.edges[edge_id]
        if 0 < index < len(edge.waypoints) - 1:
            edge.waypoints[index] = (float(x), float(y))
            edge.__post_init__()
            self.dirty = True

    def toggle_oneway(self, edge_id):
        edge = self.graph.edges[edge_id]
        edge.oneway = not edge.oneway
        self.dirty = True
        self.update()

    def preview_routes(self, a_src, a_dst, b_src=None, b_dst=None):
        self.route_a = self.graph.shortest_route(a_src, a_dst)
        self.route_b = self.graph.shortest_route(b_src, b_dst) if b_src and b_dst else None
        self.update()
        return self.route_a, self.route_b

    # ------------------------------------------------ 클릭 판정

    def _pick_node(self, p):
        for node in self.graph.nodes.values():
            q = self.world_to_screen(node.x, node.y)
            if math.hypot(q.x() - p.x(), q.y() - p.y()) <= PICK_PX:
                return node.id
        return None

    def _pick_waypoint(self, p):
        for edge in self.graph.edges.values():
            for i, (x, y) in enumerate(edge.waypoints[1:-1], start=1):
                q = self.world_to_screen(x, y)
                if math.hypot(q.x() - p.x(), q.y() - p.y()) <= PICK_PX:
                    return edge.id, i
        return None

    def _pick_edge(self, p):
        wx, wy = self.screen_to_world(p)
        proj = self.graph.project(wx, wy) if self.graph.edges else None
        if proj and proj.distance * self.scale <= PICK_PX:
            return proj.edge_id
        return None

    # ------------------------------------------------ 이벤트

    def mousePressEvent(self, event):
        p = event.pos()
        if event.button() == Qt.RightButton:
            self._pan_from = (p, self.tx, self.ty)
            return
        if event.button() != Qt.LeftButton:
            return
        wx, wy = self.screen_to_world(p)
        mods = event.modifiers()
        node = self._pick_node(p)

        if self.mode == MODE_NODE:
            if node:
                self.selected = ('node', node)
                self.dragging = ('node', node)
            else:
                nid = self.add_node(wx, wy)
                self.selected = ('node', nid)
            self.update()
            return

        if self.mode == MODE_EDGE:
            if self.pending_edge is None:
                if node:
                    self.pending_edge = {'from': node, 'points': []}
                    self.status_cb(f'엣지 시작: {node} — 중간점을 찍고 끝 노드를 클릭')
            else:
                if node and node != self.pending_edge['from']:
                    eid = self.add_edge(self.pending_edge['from'], node, self.pending_edge['points'])
                    self.pending_edge = None
                    self.selected = ('edge', eid, None)
                    self.status_cb(f'엣지 추가: {eid}')
                elif not node:
                    self.pending_edge['points'].append((wx, wy))
            self.update()
            return

        # MODE_SELECT
        if node and mods & Qt.ShiftModifier:
            self.route_nodes.append(node)
            if len(self.route_nodes) >= 2:
                a, b = self.route_nodes[-2], self.route_nodes[-1]
                try:
                    if mods & Qt.ControlModifier and self.route_a is not None:
                        self.route_b = self.graph.shortest_route(a, b)
                    else:
                        self.route_a = self.graph.shortest_route(a, b)
                        self.route_b = None
                    self.status_cb(self._route_summary())
                except RoadGraphError as exc:
                    self.status_cb(str(exc))
                self.route_nodes = []
            self.update()
            return
        if node:
            self.selected = ('node', node)
            self.dragging = ('node', node)
        else:
            wp = self._pick_waypoint(p)
            if wp:
                self.selected = ('edge', wp[0], wp[1])
                self.dragging = ('wp', wp[0], wp[1])
            else:
                eid = self._pick_edge(p)
                self.selected = ('edge', eid, None) if eid else None
        self.update()

    def mouseMoveEvent(self, event):
        p = event.pos()
        wx, wy = self.screen_to_world(p)
        if self._pan_from:
            p0, tx0, ty0 = self._pan_from
            self.tx, self.ty = tx0 + (p.x() - p0.x()), ty0 + (p.y() - p0.y())
            self.update()
            return
        if self.dragging and event.buttons() & Qt.LeftButton:
            if self.dragging[0] == 'node':
                self.move_node(self.dragging[1], wx, wy)
            else:
                self.move_waypoint(self.dragging[1], self.dragging[2], wx, wy)
            self.update()
        self.status_cb(f'({wx:.3f}, {wy:.3f}) m  모드={self.mode}  선택={self.selected}')

    def mouseReleaseEvent(self, event):
        self._pan_from = None
        self.dragging = None

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        p = event.pos()
        wx, wy = self.screen_to_world(p)
        self.scale = max(20.0, min(5000.0, self.scale * factor))
        self.tx = p.x() - wx * self.scale
        self.ty = p.y() + wy * self.scale
        self.update()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Escape:
            self.pending_edge = None
            self.route_nodes = []
            self.selected = None
        elif key == Qt.Key_Delete and self.selected:
            if self.selected[0] == 'node':
                self.delete_node(self.selected[1])
            else:
                self.delete_edge(self.selected[1])
        elif key == Qt.Key_O and self.selected and self.selected[0] == 'edge':
            self.toggle_oneway(self.selected[1])
        elif key == Qt.Key_F:
            self.fit_to_view()
        elif key == Qt.Key_1:
            self.mode = MODE_SELECT
        elif key == Qt.Key_2:
            self.mode = MODE_NODE
        elif key == Qt.Key_3:
            self.mode = MODE_EDGE
        else:
            super().keyPressEvent(event)
            return
        self.update()

    def resizeEvent(self, event):
        if self.map_data and event.oldSize().width() <= 0:
            self.fit_to_view()
        super().resizeEvent(event)

    # ------------------------------------------------ 그리기

    def _route_summary(self):
        parts = []
        if self.route_a:
            parts.append(f'A {self.route_a.start_id}→{self.route_a.goal_id} '
                         f'{self.route_a.length:.2f} m: {" ".join(self.route_a.edge_ids)}')
        if self.route_b:
            parts.append(f'B {self.route_b.start_id}→{self.route_b.goal_id} '
                         f'{self.route_b.length:.2f} m')
            shared = RoadGraph.shared_edges(self.route_a, self.route_b)
            opposite = [e for e, same in shared if not same]
            parts.append('반대 방향 공유: ' + (', '.join(opposite) if opposite else '없음'))
        return ' | '.join(parts)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(15, 23, 42))
        painter.setRenderHint(QPainter.Antialiasing)
        if self.map_data:
            m = self.map_data
            top_left = self.world_to_screen(m.origin_x, m.origin_y + m.height * m.resolution)
            w = m.width * m.resolution * self.scale
            h = m.height * m.resolution * self.scale
            target = (top_left.x(), top_left.y(), w, h)
            painter.drawPixmap(int(target[0]), int(target[1]), int(w), int(h), m.pixmap)
            if self.photo is not None:
                painter.setOpacity(self.photo_alpha)
                painter.drawImage(int(target[0]), int(target[1]),
                                  self.photo.scaled(int(w), int(h), Qt.IgnoreAspectRatio,
                                                    Qt.SmoothTransformation))
                painter.setOpacity(1.0)

        # 엣지
        for edge in self.graph.edges.values():
            selected = self.selected and self.selected[0] == 'edge' and self.selected[1] == edge.id
            color = QColor(255, 220, 60) if selected else EDGE_COLOR
            pen = QPen(color, 3 if selected else 2)
            if edge.oneway:
                pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            pts = [self.world_to_screen(x, y) for x, y in edge.waypoints]
            for a, b in zip(pts, pts[1:]):
                painter.drawLine(a, b)
            if edge.oneway:
                self._draw_arrow(painter, pts[-2], pts[-1], color)
            painter.setBrush(QColor(200, 200, 200))
            painter.setPen(Qt.NoPen)
            for q in pts[1:-1]:
                painter.drawEllipse(q, 3, 3)
            mid = pts[len(pts) // 2]
            painter.setPen(QColor(30, 90, 160))
            painter.drawText(mid + QPointF(4, -4), edge.id)

        # 경로 미리보기
        for route, color, width in ((self.route_a, QColor(60, 220, 120), 6),
                                    (self.route_b, QColor(220, 120, 255), 6)):
            if route:
                painter.setPen(QPen(color, width, Qt.SolidLine, Qt.RoundCap))
                pts = [self.world_to_screen(x, y) for x, y in route.waypoints]
                for a, b in zip(pts, pts[1:]):
                    painter.drawLine(a, b)
        if self.route_a and self.route_b:
            opposite = {e for e, same in RoadGraph.shared_edges(self.route_a, self.route_b) if not same}
            painter.setPen(QPen(QColor(255, 40, 40), 8, Qt.SolidLine, Qt.RoundCap))
            for eid in opposite:
                pts = [self.world_to_screen(x, y) for x, y in self.graph.edges[eid].waypoints]
                for a, b in zip(pts, pts[1:]):
                    painter.drawLine(a, b)

        # 작성 중인 엣지
        if self.pending_edge:
            a = self.graph.nodes[self.pending_edge['from']]
            pts = [self.world_to_screen(a.x, a.y)] + \
                  [self.world_to_screen(x, y) for x, y in self.pending_edge['points']]
            painter.setPen(QPen(QColor(255, 220, 60), 2, Qt.DashLine))
            for p, q in zip(pts, pts[1:]):
                painter.drawLine(p, q)

        # 노드
        for node in self.graph.nodes.values():
            q = self.world_to_screen(node.x, node.y)
            selected = self.selected and self.selected[0] == 'node' and self.selected[1] == node.id
            painter.setBrush(NODE_COLOR[node.type])
            painter.setPen(QPen(QColor(255, 255, 255) if selected else QColor(0, 0, 0), 2))
            painter.drawEllipse(q, 7, 7)
            text = node.id + (f' {node.label}' if node.label else '')
            painter.setPen(QColor(15, 23, 42))
            painter.drawText(q + QPointF(10, 5), text)
        painter.end()

    @staticmethod
    def _draw_arrow(painter, a, b, color):
        ang = math.atan2(b.y() - a.y(), b.x() - a.x())
        painter.setBrush(color)
        painter.setPen(Qt.NoPen)
        tip = b
        left = tip + QPointF(-12 * math.cos(ang - 0.4), -12 * math.sin(ang - 0.4))
        right = tip + QPointF(-12 * math.cos(ang + 0.4), -12 * math.sin(ang + 0.4))
        painter.drawPolygon(QPolygonF([tip, left, right]))


# ------------------------------------------------------------------ 창

class GraphEditorWindow(QMainWindow):
    def __init__(self, map_path, graph_path=None, photo_path=None):
        super().__init__()
        self.setWindowTitle('도로망 그래프 편집기')
        self.resize(1300, 800)
        self.canvas = GraphCanvas()
        self.setCentralWidget(self.canvas)
        self.canvas.status_cb = self.statusBar().showMessage
        self.graph_path = graph_path
        self.photo_path = photo_path

        self.canvas.map_data = MapData(map_path)
        if graph_path and os.path.isfile(graph_path):
            self.canvas.graph = RoadGraph.load(graph_path)
        self._apply_registration()

        bar = QToolBar('도구')
        self.addToolBar(bar)
        for text, mode in (('1 선택', MODE_SELECT), ('2 노드', MODE_NODE), ('3 엣지', MODE_EDGE)):
            act = bar.addAction(text)
            act.triggered.connect(lambda _=False, m=mode: self._set_mode(m))
        bar.addSeparator()
        bar.addWidget(QLabel(' 노드 타입 '))
        self.type_box = QComboBox()
        self.type_box.addItems(NODE_TYPES)
        self.type_box.currentTextChanged.connect(lambda t: setattr(self.canvas, 'node_type', t))
        bar.addWidget(self.type_box)
        bar.addWidget(QLabel(' 라벨 '))
        self.label_edit = QLineEdit()
        self.label_edit.setMaximumWidth(160)
        self.label_edit.editingFinished.connect(self._apply_label)
        bar.addWidget(self.label_edit)
        bar.addSeparator()
        act = bar.addAction('R 사진 정합')
        act.triggered.connect(self._register)
        act = bar.addAction('사진 열기')
        act.triggered.connect(self._open_photo)
        act = bar.addAction('F 맞추기')
        act.triggered.connect(self.canvas.fit_to_view)
        act = bar.addAction('Ctrl+S 저장')
        act.setShortcut('Ctrl+S')
        act.triggered.connect(self.save)
        act = bar.addAction('검증')
        act.triggered.connect(self._validate)

    # ------------------------------------------------ 동작

    def _set_mode(self, mode):
        self.canvas.mode = mode
        self.canvas.pending_edge = None
        self.canvas.update()
        self.statusBar().showMessage(f'모드: {mode}')

    def _apply_label(self):
        sel = self.canvas.selected
        if sel and sel[0] == 'node':
            self.canvas.graph.nodes[sel[1]].label = self.label_edit.text()
            self.canvas.dirty = True
            self.canvas.update()

    def _apply_registration(self):
        reg = self.canvas.graph.registration
        photo = self.photo_path or (reg or {}).get('photo')
        if not (reg and photo and reg.get('photo_px') and reg.get('map_xy')):
            return
        if not os.path.isabs(photo) and self.graph_path:
            photo = os.path.join(os.path.dirname(self.graph_path), photo)
        try:
            H = compute_homography(reg['photo_px'], reg['map_xy'])
            self.canvas.photo = warp_photo_to_map(photo, H, self.canvas.map_data)
            self.canvas.update()
        except (ValueError, ImportError) as exc:  # noqa: BLE001
            self.statusBar().showMessage(f'사진 정합 실패: {exc}')

    def _open_photo(self):
        path, _ = QFileDialog.getOpenFileName(self, '항공 사진', '', 'Images (*.jpg *.jpeg *.png)')
        if path:
            self.photo_path = path
            self._register()

    def _register(self):
        if not self.photo_path and not (self.canvas.graph.registration or {}).get('photo'):
            self._open_photo()
            return
        photo = self.photo_path or self.canvas.graph.registration['photo']
        if not os.path.isabs(photo) and self.graph_path:
            photo = os.path.join(os.path.dirname(self.graph_path), photo)
        dlg = RegisterDialog(photo, self.canvas.map_data, self.canvas.graph.registration, self)
        if dlg.exec_() != QDialog.Accepted:
            return
        reg = dlg.registration()
        if len(reg['photo_px']) < 4:
            QMessageBox.warning(self, '정합', '대응점이 4개 이상 필요합니다')
            return
        rel = os.path.relpath(photo, os.path.dirname(self.graph_path)) if self.graph_path else photo
        reg['photo'] = rel
        self.canvas.graph.registration = reg
        self.canvas.dirty = True
        self._apply_registration()

    def _validate(self):
        try:
            self.canvas.graph.validate()
            iso = self.canvas.graph.isolated_nodes()
            msg = '문제 없음' if not iso else f'고립 노드: {", ".join(iso)}'
        except RoadGraphError as exc:
            msg = str(exc)
        QMessageBox.information(self, '검증', msg)

    def save(self):
        if not self.graph_path:
            path, _ = QFileDialog.getSaveFileName(self, '저장', 'road_graph.yaml', 'YAML (*.yaml)')
            if not path:
                return
            self.graph_path = path
        try:
            self.canvas.graph.validate()
        except RoadGraphError as exc:
            QMessageBox.warning(self, '저장', f'검증 실패 — 저장하지 않았습니다\n{exc}')
            return
        self.canvas.graph.save(self.graph_path)
        self.canvas.dirty = False
        self.statusBar().showMessage(f'저장: {self.graph_path}')

    def closeEvent(self, event):
        if self.canvas.dirty:
            ans = QMessageBox.question(self, '종료', '저장하지 않은 변경이 있습니다. 저장할까요?',
                                       QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            if ans == QMessageBox.Cancel:
                event.ignore()
                return
            if ans == QMessageBox.Yes:
                self.save()
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description='도로망 그래프 편집기')
    parser.add_argument('--map', required=True, help='nav2 맵 yaml')
    parser.add_argument('--graph', default='', help='road_graph.yaml (없으면 새로 만든다)')
    parser.add_argument('--photo', default='', help='항공 사진 (선택)')
    args = parser.parse_args(argv)
    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    win = GraphEditorWindow(args.map, args.graph or None, args.photo or None)
    win.show()
    win.canvas.fit_to_view()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
