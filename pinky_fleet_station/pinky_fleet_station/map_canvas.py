"""맵 렌더링 + 클릭/드래그로 좌표 지정하는 PyQt5 위젯.

좌표 변환 (map yaml 의 origin 은 이미지 좌하단 기준):

    world -> pixel :  px = (wx - ox) / r,   py = H - (wy - oy) / r
    pixel -> world :  wx = ox + px * r,     wy = oy + (H - py) * r

여기에 화면 배율 ``scale`` 과 팬 오프셋 ``(tx, ty)`` 가 더해진다.
origin 의 yaw 는 0 이라고 가정한다 (핑키 맵 두 종류 모두 0).
"""

import math
import os

import numpy as np
import yaml
from PyQt5.QtCore import QPointF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import QWidget

MODE_NONE = None
MODE_GOAL = 'goal'
MODE_INITIALPOSE = 'initialpose'

COLOR_UNKNOWN = (75, 85, 99)
COLOR_FREE = (255, 255, 255)
COLOR_OCCUPIED = (20, 20, 20)


class MapLoadError(RuntimeError):
    pass


class MapData:
    """map yaml + 이미지 한 벌."""

    def __init__(self, yaml_path):
        yaml_path = os.path.expandvars(os.path.expanduser(str(yaml_path)))
        if not os.path.isfile(yaml_path):
            raise MapLoadError(f'맵 yaml 을 찾을 수 없습니다: {yaml_path}')
        with open(yaml_path, encoding='utf-8') as handle:
            meta = yaml.safe_load(handle) or {}

        image_name = meta.get('image')
        if not image_name:
            raise MapLoadError(f'맵 yaml 에 image 항목이 없습니다: {yaml_path}')
        image_path = image_name
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)
        if not os.path.isfile(image_path):
            raise MapLoadError(f'맵 이미지를 찾을 수 없습니다: {image_path}')

        self.yaml_path = yaml_path
        self.image_path = image_path
        self.resolution = float(meta.get('resolution', 0.05))
        origin = meta.get('origin') or [0.0, 0.0, 0.0]
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.negate = int(meta.get('negate', 0))
        self.occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        self.free_thresh = float(meta.get('free_thresh', 0.25))

        source = QImage(image_path)
        if source.isNull():
            raise MapLoadError(f'맵 이미지를 읽을 수 없습니다: {image_path}')
        self.width = source.width()
        self.height = source.height()
        self.image = self._colorize(source)
        # QPixmap 은 QApplication 이 있어야 만들 수 있다. 맵 로딩 자체를 GUI 기동과
        # 분리해 두려고 첫 렌더링 때까지 미룬다 (단위 테스트도 이 덕분에 가능).
        self._pixmap = None

    def _colorize(self, source):
        """nav2_map_server 의 trinary 규칙대로 점유/자유/미지를 색칠한다."""
        gray = source.convertToFormat(QImage.Format_Grayscale8)
        width, height = gray.width(), gray.height()
        buffer = np.frombuffer(gray.constBits().asstring(gray.sizeInBytes()), dtype=np.uint8)
        buffer = buffer.reshape(height, gray.bytesPerLine())[:, :width]

        value = buffer.astype(np.float32) / 255.0
        occ = value if self.negate else (1.0 - value)

        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[...] = np.array(COLOR_UNKNOWN, dtype=np.uint8)
        rgb[occ < self.free_thresh] = COLOR_FREE
        rgb[occ > self.occupied_thresh] = COLOR_OCCUPIED

        rgb = np.ascontiguousarray(rgb)
        image = QImage(rgb.data, width, height, 3 * width, QImage.Format_RGB888)
        return image.copy()          # numpy 버퍼와 수명을 끊는다

    @property
    def pixmap(self):
        if self._pixmap is None:
            self._pixmap = QPixmap.fromImage(self.image)
        return self._pixmap

    @property
    def name(self):
        """확장자를 뗀 맵 이름. 로봇에는 이 이름만 보낸다."""
        return os.path.splitext(os.path.basename(self.yaml_path))[0]

    def summary(self):
        """GUI 라벨/상태바에 쓰는 한 줄 요약."""
        return (f'{os.path.basename(self.yaml_path)} · '
                f'{self.width}x{self.height} px · {self.resolution:g} m/px')

    # --- 좌표 변환 ---------------------------------------------------

    def world_to_pixel(self, wx, wy):
        return ((wx - self.origin_x) / self.resolution,
                self.height - (wy - self.origin_y) / self.resolution)

    def pixel_to_world(self, px, py):
        return (self.origin_x + px * self.resolution,
                self.origin_y + (self.height - py) * self.resolution)


def map_mismatches(map_data, resolution=None, width=None, height=None,
                   origin_x=None, origin_y=None, name=None, tol=1e-6):
    """GUI 가 연 맵과 로봇이 로드한 맵의 차이를 사람이 읽을 문자열로 돌려준다.

    빈 리스트면 같은 맵이다. 파일 경로가 아니라 이름과 규격을 비교하므로, 경로가
    달라도 내용이 같으면 통과하고 경로가 같아도 내용이 다르면 잡힌다.

    각 인자는 ``None`` 이면 건너뛴다. 이름과 규격은 출처가 달라서 (이름은 에이전트가
    기억하는 값, 규격은 map 토픽) 한쪽만 알 수 있는 상황이 있다 — 예를 들어 Nav2 가
    아직 안 떠서 map 토픽을 못 받았어도 이름은 알 수 있다.

    ``resolution`` 은 OccupancyGrid 에서 float32 로 오기 때문에 (0.05 가
    0.05000000074505806 로 들어온다) 반드시 허용 오차를 두고 비교해야 한다.
    """
    issues = []
    if name and name != map_data.name:
        issues.append(f'맵 이름 {map_data.name} != {name}')
    if resolution is not None and abs(float(resolution) - map_data.resolution) > tol:
        issues.append(f'해상도 {map_data.resolution:g} != {float(resolution):g}')
    if width is not None and height is not None and (
            int(width) != map_data.width or int(height) != map_data.height):
        issues.append(
            f'크기 {map_data.width}x{map_data.height} != {int(width)}x{int(height)}')
    if origin_x is not None and abs(float(origin_x) - map_data.origin_x) > tol:
        issues.append(f'원점 x {map_data.origin_x:g} != {float(origin_x):g}')
    if origin_y is not None and abs(float(origin_y) - map_data.origin_y) > tol:
        issues.append(f'원점 y {map_data.origin_y:g} != {float(origin_y):g}')
    return issues


class MapCanvas(QWidget):
    """맵 위에서 press -> drag -> release 로 (x, y, yaw) 를 고르는 캔버스."""

    poseSelected = pyqtSignal(str, str, float, float, float)   # mode, robot, x, y, yaw
    hoverMoved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 480)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self._map = None
        self._scale = 1.0
        self._tx = 0.0
        self._ty = 0.0
        self._fit_pending = True

        self._mode = MODE_NONE
        self._mode_robot = None
        self._drag_start = None       # world 좌표
        self._drag_now = None

        self._panning = False
        self._pan_anchor = None

        self.robots = {}              # name -> dict(color,x,y,yaw,goal,...)
        self.paths = {}               # name -> [(x, y), ...]
        self.show_paths = True
        self.show_goals = True
        self.show_distance = True

    # --- 맵 -----------------------------------------------------------

    def set_map(self, map_data):
        self._map = map_data
        self._fit_pending = True
        self.update()

    def has_map(self):
        return self._map is not None

    def map_data(self):
        return self._map

    def set_mode(self, mode, robot_name=None):
        self._mode = mode
        self._mode_robot = robot_name
        self._drag_start = None
        self._drag_now = None
        self.setCursor(Qt.CrossCursor if mode else Qt.ArrowCursor)
        self.update()

    def fit_to_view(self):
        self._fit_pending = True
        self.update()

    # --- 좌표 변환 (world <-> 위젯) -----------------------------------

    def _world_to_screen(self, wx, wy):
        px, py = self._map.world_to_pixel(wx, wy)
        return QPointF(px * self._scale + self._tx, py * self._scale + self._ty)

    def _screen_to_world(self, point):
        px = (point.x() - self._tx) / self._scale
        py = (point.y() - self._ty) / self._scale
        return self._map.pixel_to_world(px, py)

    def _apply_fit(self):
        margin = 12
        available_w = max(1, self.width() - 2 * margin)
        available_h = max(1, self.height() - 2 * margin)
        self._scale = min(available_w / self._map.width, available_h / self._map.height)
        self._tx = (self.width() - self._map.width * self._scale) / 2.0
        self._ty = (self.height() - self._map.height * self._scale) / 2.0
        self._fit_pending = False

    # --- 마우스 -------------------------------------------------------

    def mousePressEvent(self, event):
        if self._map is None:
            return
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self._panning = True
            self._pan_anchor = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            return
        if event.button() == Qt.LeftButton and self._mode:
            self._drag_start = self._screen_to_world(event.pos())
            self._drag_now = self._drag_start
            self.update()

    def mouseMoveEvent(self, event):
        if self._map is None:
            return
        if self._panning and self._pan_anchor is not None:
            delta = event.pos() - self._pan_anchor
            self._tx += delta.x()
            self._ty += delta.y()
            self._pan_anchor = event.pos()
            self.update()
            return
        wx, wy = self._screen_to_world(event.pos())
        self.hoverMoved.emit(wx, wy)
        if self._drag_start is not None:
            self._drag_now = (wx, wy)
            self.update()

    def mouseReleaseEvent(self, event):
        if self._map is None:
            return
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self._panning = False
            self._pan_anchor = None
            self.setCursor(Qt.CrossCursor if self._mode else Qt.ArrowCursor)
            return
        if event.button() != Qt.LeftButton or self._drag_start is None:
            return

        end = self._screen_to_world(event.pos())
        dx = end[0] - self._drag_start[0]
        dy = end[1] - self._drag_start[1]
        # 드래그가 거의 없으면 방향을 바꾸지 않고 0 rad 으로 둔다.
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-3 else 0.0
        mode, robot = self._mode, self._mode_robot
        x, y = self._drag_start
        self._drag_start = None
        self._drag_now = None
        self.update()
        if mode and robot:
            self.poseSelected.emit(mode, robot, x, y, yaw)

    def wheelEvent(self, event):
        if self._map is None:
            return
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        new_scale = max(0.2, min(80.0, self._scale * factor))
        if new_scale == self._scale:
            return
        anchor = event.pos()
        # 커서 아래 지점이 그대로 있도록 오프셋을 보정한다.
        self._tx = anchor.x() - (anchor.x() - self._tx) * (new_scale / self._scale)
        self._ty = anchor.y() - (anchor.y() - self._ty) * (new_scale / self._scale)
        self._scale = new_scale
        self.update()

    # --- 렌더링 -------------------------------------------------------

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#111827'))

        if self._map is None:
            painter.setPen(QColor('#9ca3af'))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             '맵이 없습니다.\n오른쪽 [맵 열기] 로 맵 yaml 을 선택하세요.')
            return

        if self._fit_pending:
            self._apply_fit()

        painter.setRenderHint(QPainter.Antialiasing, True)
        target = self._map.pixmap.rect()
        painter.save()
        painter.translate(self._tx, self._ty)
        painter.scale(self._scale, self._scale)
        painter.drawPixmap(target, self._map.pixmap)
        painter.restore()

        if self.show_paths:
            self._draw_paths(painter)
        if self.show_distance:
            self._draw_distance(painter)
        if self.show_goals:
            self._draw_goals(painter)
        self._draw_robots(painter)
        self._draw_drag(painter)

    def _draw_paths(self, painter):
        for name, points in self.paths.items():
            if len(points) < 2:
                continue
            color = QColor(self.robots.get(name, {}).get('color', '#22c55e'))
            color.setAlpha(160)
            painter.setPen(QPen(color, 2))
            polygon = QPolygonF([self._world_to_screen(x, y) for x, y in points])
            painter.drawPolyline(polygon)

    def _draw_goals(self, painter):
        for name, info in self.robots.items():
            goal = info.get('goal')
            if not goal or not info.get('goal_valid', False):
                continue
            gx, gy, gyaw = goal
            center = self._world_to_screen(gx, gy)
            color = QColor(info.get('color', '#ff5a7a'))
            painter.setPen(QPen(color, 2, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(center, 9, 9)
            painter.drawLine(
                center,
                center + QPointF(14 * math.cos(-gyaw), 14 * math.sin(-gyaw)))
            painter.setPen(color)
            painter.drawText(center + QPointF(12, -10), name)

    def _draw_robots(self, painter):
        for name, info in self.robots.items():
            if info.get('x') is None:
                continue
            center = self._world_to_screen(info['x'], info['y'])
            color = QColor(info.get('color', '#ff5a7a'))
            if not info.get('localized', True):
                color.setAlpha(90)
            painter.setPen(QPen(QColor('#0b1120'), 2))
            painter.setBrush(color)
            painter.drawEllipse(center, 8, 8)
            # 화면 y 축은 아래로 증가하므로 heading 은 -yaw 로 그린다.
            painter.setPen(QPen(QColor('#fde68a'), 2))
            painter.drawLine(
                center,
                center + QPointF(18 * math.cos(-info['yaw']), 18 * math.sin(-info['yaw'])))
            painter.setPen(QColor('#e5e7eb'))
            painter.drawText(center + QPointF(11, 16), name)

    def _draw_distance(self, painter):
        located = [i for i in self.robots.values() if i.get('x') is not None]
        if len(located) != 2:
            return
        a, b = located
        pa = self._world_to_screen(a['x'], a['y'])
        pb = self._world_to_screen(b['x'], b['y'])
        distance = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
        pen = QPen(QColor('#64748b'), 1, Qt.DotLine)
        painter.setPen(pen)
        painter.drawLine(pa, pb)
        painter.setPen(QColor('#94a3b8'))
        painter.drawText((pa + pb) / 2 + QPointF(6, -6), f'{distance:.2f} m')

    def _draw_drag(self, painter):
        if self._drag_start is None or self._drag_now is None:
            return
        color = QColor('#22c55e' if self._mode == MODE_INITIALPOSE else '#fb7185')
        start = self._world_to_screen(*self._drag_start)
        end = self._world_to_screen(*self._drag_now)
        painter.setPen(QPen(color, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(start, 7, 7)
        painter.drawLine(start, end)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._map is not None and self._fit_pending:
            self.update()
