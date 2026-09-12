#!/usr/bin/env python3
"""관제 PC 의 PyQt5 GUI.

맵을 띄워 두 핑키의 위치를 실시간으로 보여 주고, 맵 위 클릭/드래그로
목표 좌표와 초기 위치를 지정한다. 좌표/속도는 mission.yaml 로 주고받는다.

rclpy 와 Qt 는 한 프로세스에서 돈다. 별도 스레드 대신 QTimer 로
``rclpy.spin_once(timeout_sec=0)`` 를 돌려 모든 ROS 콜백이 Qt 메인 스레드에서
실행되게 했다 (경쟁 조건 없이 위젯을 바로 갱신할 수 있다).
"""

import json
import math
import os
import sys

import rclpy
from nav_msgs.msg import Path
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QFileDialog, QFormLayout, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
    QPushButton, QScrollArea, QSplitter, QVBoxLayout, QWidget,
)
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from std_msgs.msg import String

from pinky_fleet_msgs.msg import FleetCommand, RobotState

from .map_canvas import MapCanvas, MapData, MapLoadError, MODE_GOAL, MODE_INITIALPOSE
from .mission_io import MissionError, load_mission, save_mission

NAV_STATUS_TEXT = {
    RobotState.NAV_IDLE: ('대기', '#9ca3af'),
    RobotState.NAV_ACTIVE: ('주행 중', '#22c55e'),
    RobotState.NAV_SUCCEEDED: ('도착', '#38bdf8'),
    RobotState.NAV_ABORTED: ('실패', '#f87171'),
    RobotState.NAV_CANCELED: ('취소됨', '#fbbf24'),
    RobotState.NAV_HOLD: ('양보 대기', '#fb923c'),
}

# fake_state_pub 노드가 그래프에 있으면 화면의 로봇은 가짜다.
# launch 와 `ros2 run` 양쪽에서 이 이름으로 뜬다.
FAKE_NODE_NAME = 'fake_state_pub'
SIM_CHECK_PERIOD = 20          # _refresh(10Hz) 틱 기준 -> 약 2초마다 확인

COMMAND_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

STYLE = """
QWidget { background: #0f172a; color: #e2e8f0; font-size: 12px; }
QGroupBox {
    border: 1px solid #1e293b; border-radius: 6px;
    margin-top: 10px; padding: 8px;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #94a3b8; }
QPushButton {
    background: #1e293b; border: 1px solid #334155; border-radius: 4px;
    padding: 5px 10px;
}
QPushButton:hover { background: #334155; }
QPushButton:checked { background: #be123c; border-color: #fb7185; }
QPushButton:disabled { color: #475569; border-color: #1e293b; }
QDoubleSpinBox { background: #1e293b; border: 1px solid #334155; padding: 3px; }
"""

SIM_BANNER_STYLE = """
background: #78350f;
color: #fde68a;
border: 1px solid #f59e0b;
border-radius: 4px;
padding: 6px;
font-weight: bold;
"""


class RobotPanel(QGroupBox):
    """로봇 1대의 상태 표시 + 조작 카드."""

    def __init__(self, spec, window):
        super().__init__(f"{spec['name']}  (domain {spec['domain_id']})")
        self._spec = spec
        self._window = window
        self.name = spec['name']

        layout = QVBoxLayout(self)

        swatch = QFrame()
        swatch.setFixedHeight(4)
        swatch.setStyleSheet(f"background: {spec['color']}; border: none;")
        layout.addWidget(swatch)

        grid = QGridLayout()
        grid.setVerticalSpacing(2)
        self._labels = {}
        for row, (key, title) in enumerate((
                ('status', '상태'),
                ('pose', '위치'),
                ('goal', '목표'),
                ('speed', '속도'),
                ('battery', '배터리'),
        )):
            caption = QLabel(title)
            caption.setStyleSheet('color: #64748b;')
            value = QLabel('-')
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            grid.addWidget(caption, row, 0)
            grid.addWidget(value, row, 1)
            self._labels[key] = value
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)

        picker_row = QHBoxLayout()
        self.goal_button = QPushButton('목표 지정')
        self.goal_button.setCheckable(True)
        self.goal_button.clicked.connect(
            lambda checked: window.set_pick_mode(MODE_GOAL, self.name, checked))
        self.init_button = QPushButton('초기위치 지정')
        self.init_button.setCheckable(True)
        self.init_button.clicked.connect(
            lambda checked: window.set_pick_mode(MODE_INITIALPOSE, self.name, checked))
        picker_row.addWidget(self.goal_button)
        picker_row.addWidget(self.init_button)
        layout.addLayout(picker_row)

        action_row = QHBoxLayout()
        start = QPushButton('출발')
        start.clicked.connect(lambda: window.send_goto(self.name))
        stop = QPushButton('정지')
        stop.clicked.connect(lambda: window.send_cancel(self.name))
        action_row.addWidget(start)
        action_row.addWidget(stop)
        layout.addLayout(action_row)

        speed_form = QFormLayout()
        self.linear_spin = QDoubleSpinBox()
        self.linear_spin.setRange(0.01, 1.0)
        self.linear_spin.setSingleStep(0.01)
        self.linear_spin.setDecimals(2)
        self.linear_spin.setValue(spec['max_linear_vel'])
        self.angular_spin = QDoubleSpinBox()
        self.angular_spin.setRange(0.05, 5.0)
        self.angular_spin.setSingleStep(0.05)
        self.angular_spin.setDecimals(2)
        self.angular_spin.setValue(spec['max_angular_vel'])
        speed_form.addRow('최대 직진 (m/s)', self.linear_spin)
        speed_form.addRow('최대 회전 (rad/s)', self.angular_spin)
        layout.addLayout(speed_form)

        apply_speed = QPushButton('속도 적용')
        apply_speed.clicked.connect(lambda: window.send_speed(self.name))
        layout.addWidget(apply_speed)

    def clear_pick_buttons(self):
        self.goal_button.setChecked(False)
        self.init_button.setChecked(False)

    def update_state(self, state, goal):
        if state is None:
            self._labels['status'].setText('<i>수신 없음</i>')
            self._labels['status'].setStyleSheet('color: #f87171;')
            self._labels['pose'].setText('-')
            self._labels['speed'].setText('-')
            self._labels['battery'].setText('-')
        else:
            text, color = NAV_STATUS_TEXT.get(state.nav_status, ('?', '#e2e8f0'))
            if not state.localized:
                text += ' · 위치 미확정'
                color = '#f87171'
            self._labels['status'].setText(text)
            self._labels['status'].setStyleSheet(f'color: {color};')
            self._labels['pose'].setText(
                f'{state.x:+.2f}, {state.y:+.2f}  @ {math.degrees(state.yaw):+.0f}°')
            self._labels['speed'].setText(
                f'{state.linear_velocity:+.2f} m/s · {state.angular_velocity:+.2f} rad/s'
                f'   (제한 {state.max_linear_vel:.2f} / {state.max_angular_vel:.2f})')
            battery = state.battery_percent
            self._labels['battery'].setText(
                '-' if battery != battery else f'{battery:.0f} %')

        self._labels['goal'].setText(
            f"{goal['x']:+.2f}, {goal['y']:+.2f}  @ {math.degrees(goal['yaw']):+.0f}°")


class FleetWindow(QMainWindow):

    def __init__(self, node, mission):
        super().__init__()
        self._node = node
        self._mission = mission
        self._states = {}
        self._paths = {}
        self._coordinator_status = None
        self._sim_mode = None          # None = 아직 판정 전
        self._sim_tick = 0

        self._base_title = 'Pinky Fleet Station'
        self.setWindowTitle(self._base_title)
        self.resize(1280, 820)
        self.setStyleSheet(STYLE)

        self.canvas = MapCanvas()
        self.canvas.poseSelected.connect(self._on_pose_selected)
        self.canvas.hoverMoved.connect(self._on_hover)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.canvas)

        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(8, 8, 8, 8)

        title = QLabel('Pinky Fleet Station')
        title.setFont(QFont('', 15, QFont.Bold))
        side_layout.addWidget(title)

        self._hint = QLabel('맵 위에서 클릭 후 드래그하면 위치와 방향이 함께 지정됩니다.')
        self._hint.setWordWrap(True)
        self._hint.setStyleSheet('color: #64748b;')
        side_layout.addWidget(self._hint)

        self._panels = {}
        for spec in self._mission.by_priority():
            panel = RobotPanel(spec, self)
            self._panels[spec['name']] = panel
            side_layout.addWidget(panel)

        fleet_box = QGroupBox('전체 제어')
        fleet_layout = QVBoxLayout(fleet_box)
        row = QHBoxLayout()
        start_all = QPushButton('동시 출발')
        start_all.clicked.connect(self.send_goto_all)
        stop_all = QPushButton('전체 정지')
        stop_all.clicked.connect(self.send_cancel_all)
        row.addWidget(start_all)
        row.addWidget(stop_all)
        fleet_layout.addLayout(row)

        init_all = QPushButton('초기위치 일괄 전송 (mission.yaml 값)')
        init_all.clicked.connect(self.send_initialpose_all)
        fleet_layout.addWidget(init_all)

        self._coord_label = QLabel('coordinator: 대기 중')
        self._coord_label.setWordWrap(True)
        fleet_layout.addWidget(self._coord_label)
        side_layout.addWidget(fleet_box)

        view_box = QGroupBox('표시')
        view_layout = QVBoxLayout(view_box)
        for text, attr, default in (
                ('목표 마커', 'show_goals', True),
                ('전역 경로 (plan)', 'show_paths', True),
                ('로봇 간 거리', 'show_distance', True),
        ):
            check = QCheckBox(text)
            check.setChecked(default)
            check.toggled.connect(
                lambda checked, a=attr: (setattr(self.canvas, a, checked),
                                         self.canvas.update()))
            view_layout.addWidget(check)
        fit = QPushButton('화면에 맞추기')
        fit.clicked.connect(self.canvas.fit_to_view)
        view_layout.addWidget(fit)
        side_layout.addWidget(view_box)

        map_box = QGroupBox('맵')
        map_layout = QVBoxLayout(map_box)
        self._map_label = QLabel('맵 없음')
        self._map_label.setWordWrap(True)
        map_layout.addWidget(self._map_label)
        map_row = QHBoxLayout()
        open_map = QPushButton('맵 열기')
        open_map.clicked.connect(self.open_map_dialog)
        self._reload_map_button = QPushButton('다시 불러오기')
        self._reload_map_button.setToolTip('같은 파일을 다시 읽는다 (맵을 새로 만든 뒤).')
        self._reload_map_button.clicked.connect(self.reload_map)
        self._reload_map_button.setEnabled(False)
        map_row.addWidget(open_map)
        map_row.addWidget(self._reload_map_button)
        map_layout.addLayout(map_row)
        caution = QLabel('로봇에 올린 맵과 같은 파일이어야 위치가 맞습니다.')
        caution.setWordWrap(True)
        caution.setStyleSheet('color: #64748b;')
        map_layout.addWidget(caution)
        side_layout.addWidget(map_box)

        mission_box = QGroupBox('mission.yaml')
        mission_layout = QHBoxLayout(mission_box)
        load = QPushButton('불러오기')
        load.clicked.connect(self.load_mission_dialog)
        save = QPushButton('저장')
        save.clicked.connect(self.save_mission_dialog)
        mission_layout.addWidget(load)
        mission_layout.addWidget(save)
        side_layout.addWidget(mission_box)

        side_layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(side)
        scroll.setMinimumWidth(360)
        splitter.addWidget(scroll)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        self._sim_banner = QLabel(
            '시뮬레이션 모드 — 화면의 로봇은 fake_state_pub 이 만든 가짜입니다. '
            '실제 핑키는 움직이지 않습니다.')
        self._sim_banner.setAlignment(Qt.AlignCenter)
        self._sim_banner.setWordWrap(True)
        self._sim_banner.setStyleSheet(SIM_BANNER_STYLE)
        self._sim_banner.hide()

        container = QWidget()
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(6, 6, 6, 0)
        container_layout.setSpacing(6)
        container_layout.addWidget(self._sim_banner)
        container_layout.addWidget(splitter, 1)
        self.setCentralWidget(container)

        self.statusBar().showMessage('준비')

        self._pick_mode = None
        self._pick_robot = None

        if not node.has_parameter('sim_mode'):
            node.declare_parameter('sim_mode', False)
        self._force_sim_mode = bool(node.get_parameter('sim_mode').value)

        self._setup_ros()
        self._load_map()

        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._spin_ros)
        self._ros_timer.start(20)

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh)
        self._ui_timer.start(100)

    # ------------------------------------------------------------------ ROS

    def _setup_ros(self):
        self._command_pubs = {}
        self._subscriptions = []
        for spec in self._mission.robots:
            name = spec['name']
            self._command_pubs[name] = self._node.create_publisher(
                FleetCommand, spec['command_topic'], COMMAND_QOS)
            self._subscriptions.append(self._node.create_subscription(
                RobotState, spec['state_topic'],
                lambda msg, n=name: self._states.__setitem__(n, msg), 10))
            plan_topic = spec.get('plan_topic') or f'/{name}/plan'
            self._subscriptions.append(self._node.create_subscription(
                Path, plan_topic, lambda msg, n=name: self._on_path(n, msg), 10))
        self._subscriptions.append(self._node.create_subscription(
            String, '/fleet/coordinator_status', self._on_coordinator_status, 10))

    def _spin_ros(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

    def _on_path(self, name, msg: Path):
        self._paths[name] = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def _on_coordinator_status(self, msg: String):
        try:
            self._coordinator_status = json.loads(msg.data)
        except ValueError:
            self._coordinator_status = None

    def _publish(self, name, command, **fields):
        msg = FleetCommand()
        msg.command = command
        for key, value in fields.items():
            setattr(msg, key, float(value))
        self._command_pubs[name].publish(msg)

    # ------------------------------------------------------------------ 맵

    def _load_map(self, path=None, interactive=False):
        """맵 yaml 을 읽어 캔버스에 건다.

        interactive=False (시작 시, mission.yaml 로드 시) 에는 실패해도 모달을 띄우지
        않는다. 경로가 틀렸다고 창이 뜨기도 전에 막히면 [맵 열기] 를 누를 수가 없다.
        """
        path = path or self._mission.map_yaml_path
        if not path:
            self._set_map_status('맵 없음', 'mission.yaml 에 map.yaml_path 가 없습니다.')
            return False

        try:
            map_data = MapData(path)
        except MapLoadError as exc:
            self._set_map_status('맵 로드 실패', str(exc), tooltip=str(path))
            if interactive:
                QMessageBox.warning(self, '맵 로드 실패', str(exc))
            return False

        self.canvas.set_map(map_data)
        self.canvas.fit_to_view()
        # expanduser / expandvars 가 적용된 실제 경로를 되돌려 받아 저장에 반영한다.
        self._mission.map_yaml_path = map_data.yaml_path
        self._set_map_status(map_data.summary(), f'맵 로드: {map_data.yaml_path}',
                             tooltip=map_data.yaml_path, loaded=True)
        return True

    def _set_map_status(self, label, message, tooltip='', loaded=False):
        self._map_label.setText(label)
        self._map_label.setToolTip(tooltip)
        self._map_label.setStyleSheet('' if loaded else 'color: #f87171;')
        self._reload_map_button.setEnabled(loaded)
        self.statusBar().showMessage(message)

    def open_map_dialog(self):
        start = ''
        for candidate in (self._mission.map_yaml_path, self._mission.path):
            if candidate and os.path.isdir(os.path.dirname(candidate)):
                start = os.path.dirname(candidate)
                break
        path, _ = QFileDialog.getOpenFileName(
            self, '맵 yaml 선택', start or os.path.expanduser('~'),
            '맵 yaml (*.yaml *.yml)')
        if not path:
            return
        if self._load_map(path, interactive=True):
            self.statusBar().showMessage(
                f'맵 로드: {path} — 이 경로를 유지하려면 [저장] 으로 mission.yaml 에 '
                '기록하세요.')

    def reload_map(self):
        self._load_map(self._mission.map_yaml_path, interactive=True)

    # ------------------------------------------------------------- 클릭 모드

    def set_pick_mode(self, mode, robot_name, enabled):
        for name, panel in self._panels.items():
            if name != robot_name or not enabled:
                panel.clear_pick_buttons()
        if not enabled:
            self._pick_mode = None
            self._pick_robot = None
            self.canvas.set_mode(None)
            self._hint.setText('맵 위에서 클릭 후 드래그하면 위치와 방향이 함께 지정됩니다.')
            return

        panel = self._panels[robot_name]
        panel.goal_button.setChecked(mode == MODE_GOAL)
        panel.init_button.setChecked(mode == MODE_INITIALPOSE)
        self._pick_mode = mode
        self._pick_robot = robot_name
        self.canvas.set_mode(mode, robot_name)
        label = '목표' if mode == MODE_GOAL else '초기 위치'
        self._hint.setText(f'{robot_name} 의 {label} 를 맵에서 클릭 후 드래그하세요.')

    def _on_pose_selected(self, mode, robot_name, x, y, yaw):
        robot = self._mission.robot(robot_name)
        if mode == MODE_GOAL:
            robot['goal'] = {'x': x, 'y': y, 'yaw': yaw}
            self.statusBar().showMessage(
                f'{robot_name} 목표 설정: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) '
                f'— [출발] 또는 [동시 출발] 을 누르세요.')
        else:
            robot['initial_pose'] = {'x': x, 'y': y, 'yaw': yaw}
            self._publish(robot_name, FleetCommand.CMD_SET_INITIAL_POSE, x=x, y=y, yaw=yaw)
            self.statusBar().showMessage(
                f'{robot_name} 초기 위치 전송: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        self.set_pick_mode(mode, robot_name, False)

    # ------------------------------------------------------------------ 명령

    def send_goto(self, name):
        goal = self._mission.robot(name)['goal']
        self._publish(name, FleetCommand.CMD_GOTO,
                      x=goal['x'], y=goal['y'], yaw=goal['yaw'])
        self.statusBar().showMessage(f'{name} 출발 명령 전송')

    def send_goto_all(self):
        """미션 3번: 두 로봇에 같은 틱에 목표를 내보낸다."""
        for spec in self._mission.by_priority():
            goal = spec['goal']
            self._publish(spec['name'], FleetCommand.CMD_GOTO,
                          x=goal['x'], y=goal['y'], yaw=goal['yaw'])
        self.statusBar().showMessage('동시 출발 명령 전송')

    def send_cancel(self, name):
        self._publish(name, FleetCommand.CMD_CANCEL)
        self.statusBar().showMessage(f'{name} 정지')

    def send_cancel_all(self):
        for spec in self._mission.robots:
            self._publish(spec['name'], FleetCommand.CMD_CANCEL)
        self.statusBar().showMessage('전체 정지')

    def send_speed(self, name):
        panel = self._panels[name]
        linear = panel.linear_spin.value()
        angular = panel.angular_spin.value()
        robot = self._mission.robot(name)
        robot['max_linear_vel'] = linear
        robot['max_angular_vel'] = angular
        self._publish(name, FleetCommand.CMD_SET_SPEED,
                      max_linear_vel=linear, max_angular_vel=angular)
        self.statusBar().showMessage(
            f'{name} 속도 적용: {linear:.2f} m/s, {angular:.2f} rad/s')

    def send_initialpose_all(self):
        for spec in self._mission.robots:
            pose = spec['initial_pose']
            self._publish(spec['name'], FleetCommand.CMD_SET_INITIAL_POSE,
                          x=pose['x'], y=pose['y'], yaw=pose['yaw'])
        self.statusBar().showMessage('초기 위치 일괄 전송')

    # ------------------------------------------------------------ mission I/O

    def load_mission_dialog(self):
        start = os.path.dirname(self._mission.path or '') or os.getcwd()
        path, _ = QFileDialog.getOpenFileName(
            self, 'mission.yaml 불러오기', start, 'YAML (*.yaml *.yml)')
        if not path:
            return
        try:
            mission = load_mission(path)
        except MissionError as exc:
            QMessageBox.warning(self, '불러오기 실패', str(exc))
            return
        if [r['name'] for r in mission.robots] != [r['name'] for r in self._mission.robots]:
            QMessageBox.warning(
                self, '불러오기 실패',
                '로봇 구성이 다릅니다. 토픽 구독을 다시 만들어야 하므로 GUI 를 재시작하세요.')
            return
        for spec in mission.robots:
            current = self._mission.robot(spec['name'])
            current['goal'] = spec['goal']
            current['initial_pose'] = spec['initial_pose']
            current['max_linear_vel'] = spec['max_linear_vel']
            current['max_angular_vel'] = spec['max_angular_vel']
            panel = self._panels[spec['name']]
            panel.linear_spin.setValue(spec['max_linear_vel'])
            panel.angular_spin.setValue(spec['max_angular_vel'])
        if mission.map_yaml_path != self._mission.map_yaml_path:
            self._mission.map_yaml_path = mission.map_yaml_path
            self._load_map()
        self._mission.path = path
        self.statusBar().showMessage(f'불러옴: {path}')

    def save_mission_dialog(self):
        start = self._mission.path or os.path.join(os.getcwd(), 'mission.yaml')
        path, _ = QFileDialog.getSaveFileName(
            self, 'mission.yaml 저장', start, 'YAML (*.yaml *.yml)')
        if not path:
            return
        try:
            save_mission(self._mission, path)
        except (MissionError, OSError) as exc:
            QMessageBox.warning(self, '저장 실패', str(exc))
            return
        self.statusBar().showMessage(f'저장됨: {path}')

    # ------------------------------------------------------------------ 갱신

    def _check_sim_mode(self):
        """가짜 로봇(fake_state_pub)이 그래프에 있으면 시뮬레이션 모드로 표시한다.

        실기라고 생각하고 fake_fleet.launch.xml 을 띄우는 실수를 화면에서 바로
        알 수 있게 하는 것이 목적이다. 파라미터로 강제할 수도 있다.
        """
        if self._force_sim_mode:
            sim = True
        else:
            try:
                sim = FAKE_NODE_NAME in self._node.get_node_names()
            except Exception:  # noqa: BLE001 - 그래프 조회 실패는 표시만 보류
                return
        if sim == self._sim_mode:
            return
        self._sim_mode = sim
        self._sim_banner.setVisible(sim)
        self.setWindowTitle(
            f'{self._base_title} — [시뮬레이션 모드]' if sim else self._base_title)
        if sim:
            self._node.get_logger().warn(
                '시뮬레이션 모드: fake_state_pub 이 떠 있어 화면의 로봇은 가짜입니다.')

    def _on_hover(self, wx, wy):
        self.statusBar().showMessage(f'커서: {wx:+.2f}, {wy:+.2f} m', 1500)

    def _refresh(self):
        self._sim_tick += 1
        if self._sim_mode is None or self._sim_tick % SIM_CHECK_PERIOD == 0:
            self._check_sim_mode()

        render = {}
        for spec in self._mission.robots:
            name = spec['name']
            state = self._states.get(name)
            goal = spec['goal']
            self._panels[name].update_state(state, goal)
            render[name] = {
                'color': spec['color'],
                'x': None if state is None else state.x,
                'y': None if state is None else state.y,
                'yaw': 0.0 if state is None else state.yaw,
                'localized': True if state is None else state.localized,
                'goal': (goal['x'], goal['y'], goal['yaw']),
                'goal_valid': True,
            }
        self.canvas.robots = render
        self.canvas.paths = self._paths
        self.canvas.update()

        status = self._coordinator_status
        if status is None:
            self._coord_label.setText('coordinator: 수신 없음')
            self._coord_label.setStyleSheet('color: #64748b;')
        else:
            state = status.get('state', '?')
            reason = status.get('reason', '')
            distance = status.get('distance')
            suffix = '' if distance is None else f' · 거리 {distance:.2f} m'
            self._coord_label.setText(f'coordinator [{state}] {reason}{suffix}')
            self._coord_label.setStyleSheet(
                'color: #fb923c;' if state == 'YIELD' else 'color: #22c55e;')

    def closeEvent(self, event):
        self._ros_timer.stop()
        self._ui_timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('fleet_gui')
    node.declare_parameter('mission', '')
    mission_path = node.get_parameter('mission').value

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    if not mission_path:
        mission_path, _ = QFileDialog.getOpenFileName(
            None, 'mission.yaml 선택', os.getcwd(), 'YAML (*.yaml *.yml)')
    if not mission_path:
        QMessageBox.critical(None, '시작 실패', 'mission.yaml 경로가 필요합니다.')
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        mission = load_mission(mission_path)
    except MissionError as exc:
        QMessageBox.critical(None, 'mission.yaml 오류', str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return

    window = FleetWindow(node, mission)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
