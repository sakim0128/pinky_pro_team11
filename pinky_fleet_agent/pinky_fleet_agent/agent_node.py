#!/usr/bin/env python3
"""핑키 1대의 도메인 안에서 도는 경량 에이전트.

관제 PC 와는 토픽 2개(``fleet/state`` / ``fleet/command``)로만 대화한다.
Nav2 액션·파라미터 서비스·TF 는 전부 이 로봇 도메인 안에서만 쓰이므로
domain_bridge 가 토픽만 브리지해도 전체 시스템이 동작한다.
"""

import math
import os
import signal

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from pinky_fleet_msgs.msg import FleetCommand, RobotState

from .link_watch import ARMED, LOST, RESTORED, LinkWatch
from .map_paths import resolve_map_path
from . import param_audit

# RViz 의 2D Pose Estimate 와 동일한 공분산
INITIAL_POSE_COV_XX = 0.25
INITIAL_POSE_COV_YY = 0.25
INITIAL_POSE_COV_YAWYAW = 0.06853891909122467


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PinkyAgent(Node):

    def __init__(self):
        super().__init__('pinky_fleet_agent')

        self.declare_parameter('robot_name', 'pinky1')
        self.declare_parameter('domain_id', 10)
        self.declare_parameter('state_topic', '')     # 비우면 /<robot_name>/state
        self.declare_parameter('command_topic', '')   # 비우면 /<robot_name>/command
        self.declare_parameter('plan_topic', '')      # 비우면 /<robot_name>/plan
        # Nav2 planner_server 가 실제로 발행하는 이름. namespace 를 쓰지 않으므로 /plan 이다.
        self.declare_parameter('plan_in_topic', 'plan')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('map_dir', '/home/pinky/map')
        self.declare_parameter('map_name', '')
        self.declare_parameter('state_rate', 10.0)
        self.declare_parameter('pose_timeout', 2.0)
        # 관제 PC 나 브리지가 죽어 CMD_RESUME 이 영영 오지 않는 경우를 대비한 자동 해제.
        self.declare_parameter('hold_watchdog', 20.0)
        # 데드맨 스위치. 관제 PC 의 하트비트가 이만큼 끊기면 스스로 주행을 멈춘다.
        # 0 이면 비활성. 0.2 m/s 로 달리는 로봇이 3초면 0.6m 를 더 간다.
        # 지켜야 할 관계: heartbeat 주기 x 3 <= command_timeout << hold_watchdog
        self.declare_parameter('command_timeout', 3.0)
        # 연결 복구 직후 이만큼(s) 이동 명령을 무시한다 (RELIABLE QoS 재전송 방어).
        self.declare_parameter('restore_grace', 1.0)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('map_server', '/map_server')
        self.declare_parameter('controller_server', '/controller_server')
        self.declare_parameter('velocity_smoother', '/velocity_smoother')
        self.declare_parameter('follow_path_plugin', 'FollowPath')
        # 기동 직후 Nav2 가 정말 이 파일로 떠 있는지 확인한다. param_audit.py 참고.
        self.declare_parameter('nav2_params_file', '')
        self.declare_parameter('audit_nav2_params', True)
        # Nav2 노드들이 네임스페이스 안에 있으면 (가제보 시뮬) 여기에 적는다.
        self.declare_parameter('audit_ns', '')
        # Nav2 가 configure 를 끝내야 값이 제자리에 들어간다. 그 전에 물으면 기본값이 나온다.
        self.declare_parameter('audit_delay', 15.0)

        self._name = self.get_parameter('robot_name').value
        self._domain_id = int(self.get_parameter('domain_id').value)
        self._state_topic = (
            self.get_parameter('state_topic').value or f'/{self._name}/state')
        self._command_topic = (
            self.get_parameter('command_topic').value or f'/{self._name}/command')
        self._plan_topic = (
            self.get_parameter('plan_topic').value or f'/{self._name}/plan')
        self._pose_timeout = float(self.get_parameter('pose_timeout').value)
        self._hold_watchdog = float(self.get_parameter('hold_watchdog').value)
        self._link = LinkWatch(self.get_parameter('command_timeout').value,
                               self.get_parameter('restore_grace').value)
        self._global_frame = self.get_parameter('global_frame').value
        self._base_frame = self.get_parameter('robot_base_frame').value
        self._max_lin = float(self.get_parameter('max_linear_vel').value)
        self._max_ang = float(self.get_parameter('max_angular_vel').value)
        self._follow_path_plugin = self.get_parameter('follow_path_plugin').value

        cb = ReentrantCallbackGroup()

        # --- 위치: TF(map->base_footprint) 를 주 소스로 쓴다. ---
        # amcl_pose 는 AMCL 이 갱신할 때만 나오지만 TF 는 끊기지 않아 더 안정적이다.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_pose_time = None

        self._linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._battery_percent = float('nan')
        self._map_info = None        # 이 로봇 Nav2 가 실제로 로드한 맵의 규격
        self._map_dir = self.get_parameter('map_dir').value
        self._map_name = self.get_parameter('map_name').value or ''

        self._nav_status = RobotState.NAV_IDLE
        self._goal_valid = False
        self._goal = (0.0, 0.0, 0.0)
        self._goal_handle = None
        self._hold = False           # CMD_STOP 으로 대기 중인가 (RESUME 대상)
        self._hold_since = None      # HOLD 진입 시각 (워치독용)
        self._link_lost = False      # 데드맨이 발동해 주행을 끊었는가
        self._goal_seq = 0           # 취소/재전송 중 낡은 콜백을 무시하기 위한 순번

        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=cb)
        self.create_subscription(
            Float32, 'battery/percent', self._on_battery, 10, callback_group=cb)

        # map_server 는 맵을 latch 해서 한 번만 발행한다. 이미 발행된 것을 받으려면
        # 구독도 TRANSIENT_LOCAL 이어야 한다.
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, map_qos, callback_group=cb)

        command_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            FleetCommand, self._command_topic, self._on_command,
            command_qos, callback_group=cb)

        self._state_pub = self.create_publisher(RobotState, self._state_topic, 10)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        # Nav2 의 전역 경로를 로봇 이름이 붙은 토픽으로 중계한다.
        # planner_server 는 namespace 를 쓰지 않아 /plan 으로 발행하는데, domain_bridge 의
        # topics 는 YAML 매핑이라 키가 겹쳐 2대를 한 파일에서 remap 할 수 없다.
        # 그래서 도메인을 넘기 전에 여기서 이름을 붙인다. frame_id 는 이미 map 이라 손댈 게 없다.
        plan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._plan_pub = self.create_publisher(Path, self._plan_topic, plan_qos)
        self.create_subscription(
            Path, self.get_parameter('plan_in_topic').value, self._on_plan,
            plan_qos, callback_group=cb)

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb)

        map_server = self.get_parameter('map_server').value
        self._load_map_client = self.create_client(
            LoadMap, f'{map_server}/load_map', callback_group=cb)

        controller = self.get_parameter('controller_server').value
        smoother = self.get_parameter('velocity_smoother').value
        self._controller_params = self.create_client(
            SetParameters, f'{controller}/set_parameters', callback_group=cb)
        self._smoother_params = self.create_client(
            SetParameters, f'{smoother}/set_parameters', callback_group=cb)

        rate = float(self.get_parameter('state_rate').value)
        self.create_timer(1.0 / rate, self._publish_state, callback_group=cb)

        self._audit_cb = cb
        self._audit_clients = {}
        self._audit_timer = None
        if self.get_parameter('audit_nav2_params').value:
            self._audit_timer = self.create_timer(
                float(self.get_parameter('audit_delay').value),
                self._audit_nav2_params, callback_group=cb)

        self.get_logger().info(
            f'pinky_fleet_agent 시작: name={self._name} domain_id={self._domain_id} '
            f'state={self._state_topic} command={self._command_topic} '
            f'plan={self._plan_topic} command_timeout={self._link.timeout:.1f}s '
            f'map_dir={self._map_dir} map={self._map_name or "(미지정)"} '
            f'nav2_params={self.get_parameter("nav2_params_file").value or "(설치본)"}')

    # ------------------------------------------------------------------ 구독

    def _on_plan(self, msg: Path):
        """Nav2 의 /plan 을 그대로 /<robot_name>/plan 으로 흘려보낸다 (GUI 경로 오버레이)."""
        self._plan_pub.publish(msg)

    def _on_odom(self, msg: Odometry):
        self._linear_velocity = msg.twist.twist.linear.x
        self._angular_velocity = msg.twist.twist.angular.z

    def _on_battery(self, msg: Float32):
        self._battery_percent = float(msg.data)

    def _on_map(self, msg: OccupancyGrid):
        # info 만 남기고 data 배열(수만~수십만 셀)은 버린다.
        info = msg.info
        if self._map_info is None:
            self.get_logger().info(
                f'맵 확인: {info.width}x{info.height} px, {info.resolution:g} m/px, '
                f'origin=({info.origin.position.x:.3f}, {info.origin.position.y:.3f})')
        self._map_info = info

    def _update_pose_from_tf(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0))
        except Exception:
            return
        self._x = tf.transform.translation.x
        self._y = tf.transform.translation.y
        self._yaw = yaw_from_quaternion(tf.transform.rotation)
        self._last_pose_time = self.get_clock().now()

    def _seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _localized(self):
        if self._last_pose_time is None:
            return False
        age = (self.get_clock().now() - self._last_pose_time).nanoseconds * 1e-9
        return age < self._pose_timeout

    # ------------------------------------------------------------------ 명령

    def _on_command(self, msg: FleetCommand):
        # 어떤 명령이든 관제 PC 가 살아 있다는 증거다. 단 감시를 "시작" 하는 것은
        # 하트비트뿐이다 (하트비트를 모르는 옛 관제 PC 와 섞여도 안전하도록).
        now = self._seconds()
        event = self._link.on_command(
            now, heartbeat=(msg.command == FleetCommand.CMD_HEARTBEAT))
        if event == ARMED:
            self.get_logger().info(
                f'관제 PC 연결 확인. 데드맨 감시 시작 (command_timeout='
                f'{self._link.timeout:.1f}s)')
        elif event == RESTORED:
            self._link_lost = False
            self.get_logger().warn('관제 PC 연결 복구. 목표를 다시 지정하세요.')

        if msg.command == FleetCommand.CMD_HEARTBEAT:
            return                       # 수신 시각 갱신이 전부다

        # 끊겼다 붙는 순간 RELIABLE QoS 가 끊기기 전의 CMD_GOTO 를 재전송한다.
        # 그대로 받으면 로봇이 혼자 출발한다 - 데드맨이 막으려던 바로 그 사고다.
        if msg.command in (FleetCommand.CMD_GOTO, FleetCommand.CMD_SET_INITIAL_POSE):
            if not self._link.accepts_motion_command(now):
                self.get_logger().warn(
                    f'연결 복구 직후 재전송으로 보이는 명령({msg.command})을 무시합니다.')
                return

        if msg.command == FleetCommand.CMD_GOTO:
            self._hold = False
            self._hold_since = None
            self._goal = (msg.x, msg.y, msg.yaw)
            self._goal_valid = True
            self._send_goal()

        elif msg.command == FleetCommand.CMD_STOP:
            self._hold = True
            self._hold_since = self.get_clock().now()
            self._cancel_goal()
            if self._goal_handle is None:
                self._nav_status = RobotState.NAV_HOLD
            self.get_logger().info('CMD_STOP: 주행 중단, RESUME 대기')

        elif msg.command == FleetCommand.CMD_RESUME:
            if not self._hold:
                return
            self._hold = False
            self._hold_since = None
            if self._goal_valid:
                self.get_logger().info('CMD_RESUME: 마지막 목표로 재출발')
                self._send_goal()
            else:
                self._nav_status = RobotState.NAV_IDLE

        elif msg.command == FleetCommand.CMD_CANCEL:
            self._hold = False
            self._hold_since = None
            self._goal_valid = False
            self._cancel_goal()
            self._nav_status = RobotState.NAV_CANCELED

        elif msg.command == FleetCommand.CMD_SET_INITIAL_POSE:
            self._publish_initial_pose(msg.x, msg.y, msg.yaw)

        elif msg.command == FleetCommand.CMD_SET_SPEED:
            self._apply_speed(msg.max_linear_vel, msg.max_angular_vel)

        elif msg.command == FleetCommand.CMD_SET_MAP:
            self._set_map(msg.map_name)

        else:
            self.get_logger().warn(f'알 수 없는 command: {msg.command}')

    # ------------------------------------------------------------------ Nav2

    def _send_goal(self):
        if not self._nav_client.server_is_ready():
            # 논블로킹 확인. Nav2 가 아직 안 떴으면 다음 명령에서 다시 시도한다.
            if not self._nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error('navigate_to_pose 액션 서버를 찾을 수 없음')
                self._nav_status = RobotState.NAV_ABORTED
                return

        x, y, yaw = self._goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qz, qw = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._goal_seq += 1
        seq = self._goal_seq
        self._nav_status = RobotState.NAV_ACTIVE
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f, s=seq: self._on_goal_response(f, s))
        self.get_logger().info(f'목표 전송: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')

    def _on_goal_response(self, future, seq):
        if seq != self._goal_seq:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'goal 전송 실패: {exc}')
            self._nav_status = RobotState.NAV_ABORTED
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 가 goal 을 거절함')
            self._nav_status = RobotState.NAV_ABORTED
            return
        self._goal_handle = handle
        self._nav_status = RobotState.NAV_ACTIVE
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, s=seq: self._on_goal_result(f, s))

    def _on_goal_result(self, future, seq):
        if seq != self._goal_seq:
            return
        self._goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'goal 결과 수신 실패: {exc}')
            self._nav_status = RobotState.NAV_ABORTED
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_status = RobotState.NAV_SUCCEEDED
            self._goal_valid = False
            self.get_logger().info('목표 도달')
        elif status == GoalStatus.STATUS_CANCELED:
            # STOP 으로 취소된 것이면 RESUME 을 기다리는 HOLD 상태다.
            self._nav_status = (
                RobotState.NAV_HOLD if self._hold else RobotState.NAV_CANCELED)
        else:
            self._nav_status = RobotState.NAV_ABORTED
            self.get_logger().warn(f'주행 실패 (status={status})')

    def shutdown_navigation(self):
        """종료 직전에 진행 중인 주행을 취소한다.

        이게 없으면 로봇의 launch 를 Ctrl+C 했을 때 Nav2 액션 goal 이 살아 있는 채로
        프로세스가 죽어, 종료 순서에 따라 로봇이 마지막 속도로 계속 나갈 수 있다.
        """
        if self._goal_handle is None:
            return
        self.get_logger().info('종료: 진행 중인 주행을 취소합니다.')
        self._goal_valid = False
        self._cancel_goal()

    def _cancel_goal(self):
        handle = self._goal_handle
        if handle is None:
            return
        handle.cancel_goal_async()

    # ------------------------------------------------------------------ 맵

    def _set_map(self, map_name):
        """관제 PC 가 보낸 맵 "이름" 으로 이 로봇의 맵을 바꾼다.

        경로가 아니라 이름을 받는 이유는 관제 PC 와 로봇의 맵 디렉터리가 다르기
        때문이다. 이름만 받아 <map_dir>/<name>.yaml 을 찾는다.
        """
        path, error = resolve_map_path(self._map_dir, map_name)
        if error is not None:
            self.get_logger().error(f'맵 교체 실패: {error}')
            return

        name = os.path.splitext(os.path.basename(path))[0]
        if name == self._map_name:
            # 관제 PC 가 같은 맵을 다시 보내도 주행 중인 로봇을 건드리지 않는다.
            self.get_logger().info(f'맵 교체 생략: 이미 {name} 을 쓰고 있습니다.')
            return

        if not self._load_map_client.service_is_ready():
            if not self._load_map_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(
                    'map_server/load_map 서비스를 찾을 수 없습니다 (Nav2 기동 확인).')
                return

        request = LoadMap.Request()
        request.map_url = path
        future = self._load_map_client.call_async(request)
        future.add_done_callback(
            lambda f, n=name, p=path: self._on_load_map_result(f, n, p))
        self.get_logger().info(f'맵 교체 요청: {name} ({path})')

    def _on_load_map_result(self, future, name, path):
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'맵 교체 응답 수신 실패: {exc}')
            return

        if result != LoadMap.Response.RESULT_SUCCESS:
            reason = {
                LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST: '맵 파일이 없음',
                LoadMap.Response.RESULT_INVALID_MAP_DATA: '맵 이미지가 잘못됨',
                LoadMap.Response.RESULT_INVALID_MAP_METADATA: '맵 yaml 이 잘못됨',
            }.get(result, f'알 수 없는 오류 (result={result})')
            self.get_logger().error(f'맵 교체 실패: {reason} — {path}')
            return

        self._map_name = name
        self.get_logger().warn(
            f'맵을 {name} 으로 교체했습니다. AMCL 의 기존 위치 추정은 무효가 되므로 '
            '관제 GUI 에서 초기 위치를 다시 지정하세요.')

    # ------------------------------------------------------ initialpose / 속도

    def _publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qz, qw = quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = INITIAL_POSE_COV_XX
        msg.pose.covariance[7] = INITIAL_POSE_COV_YY
        msg.pose.covariance[35] = INITIAL_POSE_COV_YAWYAW
        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'초기 위치 설정: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')

    def _apply_speed(self, max_linear_vel, max_angular_vel):
        if max_linear_vel > 0.0:
            self._max_lin = float(max_linear_vel)
        if max_angular_vel > 0.0:
            self._max_ang = float(max_angular_vel)

        lin, ang = self._max_lin, self._max_ang

        def double_param(name, value):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            return p

        def double_array_param(name, values):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(v) for v in values])
            return p

        self._call_set_parameters(
            self._controller_params, 'controller_server',
            [double_param(f'{self._follow_path_plugin}.desired_linear_vel', lin)])

        # velocity_smoother 는 [x, y, theta] 3원소 배열을 받는다.
        self._call_set_parameters(
            self._smoother_params, 'velocity_smoother',
            [
                double_array_param('max_velocity', [lin, 0.0, ang]),
                double_array_param('min_velocity', [-lin, 0.0, -ang]),
            ])

        self.get_logger().info(f'속도 제한 적용: linear={lin:.2f} angular={ang:.2f}')

    def _call_set_parameters(self, client, label, parameters):
        if not client.service_is_ready():
            self.get_logger().warn(f'{label} 파라미터 서비스가 준비되지 않음 - 건너뜀')
            return
        request = SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)

        def _done(fut, label=label):
            try:
                results = fut.result().results
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'{label} set_parameters 실패: {exc}')
                return
            for result in results:
                if not result.successful:
                    self.get_logger().warn(f'{label} 파라미터 거부: {result.reason}')

        future.add_done_callback(_done)

    # ------------------------------------------------------- Nav2 파라미터 감사

    def _audit_nav2_params(self):
        """실행 중인 Nav2 가 nav2_params_fleet.yaml 로 떠 있는지 한 번 확인한다.

        실기에서 파라미터가 통째로 안 먹은 적이 있는데, 값을 직접 읽어 보기 전까지는
        아무 증상도 없었다 (관제 화면은 멀쩡했다). 왜 필요한지는 param_audit.py 에 적었다.

        한 번만 돌고 타이머를 끈다. 계속 감시하는 게 아니라 기동 시 점검이다.
        """
        if self._audit_timer is not None:
            self._audit_timer.cancel()
            self._audit_timer = None

        path = (self.get_parameter('nav2_params_file').value
                or param_audit.default_params_path())
        if not path or not os.path.isfile(path):
            self.get_logger().warn(
                f'Nav2 파라미터 감사 생략: 파일을 찾을 수 없다 ({path or "경로 미지정"})')
            return
        try:
            with open(path, encoding='utf-8') as handle:
                data = yaml.safe_load(handle)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Nav2 파라미터 감사 생략: {path} 읽기 실패 ({exc})')
            return

        expected = param_audit.expected_values(data)
        if not expected:
            self.get_logger().warn(f'Nav2 파라미터 감사 생략: {path} 에 확인할 값이 없다')
            return

        # 노드별로 한 번씩만 묻는다.
        wanted = {}
        for node, name in expected:
            wanted.setdefault(node, []).append(name)

        self._audit_actual = {}
        self._audit_expected = expected
        self._audit_pending = set(wanted)
        for node, names in wanted.items():
            self._audit_ask(node, names)

    def _audit_ask(self, node, names):
        client = self._audit_clients.get(node)
        if client is None:
            prefix = (self.get_parameter('audit_ns').value or '').strip('/')
            service = f'/{prefix}/{node}' if prefix else f'/{node}'
            client = self.create_client(
                GetParameters, f'{service}/get_parameters',
                callback_group=self._audit_cb)
            self._audit_clients[node] = client
        if not client.service_is_ready():
            self._audit_done(node)
            return
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, node=node, names=names: self._audit_reply(fut, node, names))

    def _audit_reply(self, future, node, names):
        try:
            values = future.result().values
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f'{node} get_parameters 실패: {exc}')
            self._audit_done(node)
            return
        for name, value in zip(names, values):
            plain = self._parameter_value(value)
            if plain is not None:
                self._audit_actual[(node, name)] = plain
        self._audit_done(node)

    @staticmethod
    def _parameter_value(value):
        """ParameterValue -> 파이썬 값. NOT_SET 이면 None (그 노드에 없는 파라미터)."""
        return {
            ParameterType.PARAMETER_BOOL: lambda v: v.bool_value,
            ParameterType.PARAMETER_INTEGER: lambda v: v.integer_value,
            ParameterType.PARAMETER_DOUBLE: lambda v: v.double_value,
            ParameterType.PARAMETER_STRING: lambda v: v.string_value,
        }.get(value.type, lambda v: None)(value)

    def _audit_done(self, node):
        self._audit_pending.discard(node)
        if self._audit_pending:
            return
        bad = param_audit.compare(self._audit_expected, self._audit_actual)
        unread = param_audit.missing(self._audit_expected, self._audit_actual)
        text = param_audit.report(bad, unread)
        if text is None:
            self.get_logger().info(
                f'Nav2 파라미터 확인: nav2_params_fleet.yaml 과 일치 '
                f'({len(self._audit_expected)}개 대조)')
        elif bad:
            self.get_logger().error(text)
        else:
            self.get_logger().warn(text)

    # ------------------------------------------------------------------ 상태

    def _check_link(self):
        """관제 PC 하트비트가 끊기면 스스로 주행을 멈춘다 (데드맨 스위치).

        관제 PC 가 정상 종료되면 GUI / coordinator 가 CMD_CANCEL 을 보내 주지만,
        Wi-Fi 가 끊기거나 PC 가 멈추거나 브리지가 죽으면 그 명령은 오지 않는다.
        그 경우를 여기서 잡는다.
        """
        now = self._seconds()
        if self._link.poll(now) != LOST:
            return
        self._link_lost = True
        if not (self._goal_valid or self._hold):
            # 멈출 것이 없다. 상태만 표시하고 아무것도 건드리지 않는다.
            self.get_logger().warn('관제 PC 신호가 끊겼습니다 (주행 중은 아님).')
            return
        silence = self._link.silence(now)
        self.get_logger().warn(
            f'관제 PC 명령이 {silence:.1f}s 끊겨 주행을 중단합니다 '
            '(Wi-Fi / 관제 PC / 브리지 확인). 복구되어도 자동 재출발하지 않습니다.')
        self._hold = False
        self._hold_since = None
        self._goal_valid = False
        # 진행 중인 액션 콜백이 뒤늦게 상태를 덮어쓰지 않도록 순번을 넘긴다.
        self._goal_seq += 1
        self._cancel_goal()
        self._goal_handle = None

    def _check_hold_watchdog(self):
        """양보가 너무 오래 이어지면 목표를 버리고 대기 상태로 돌아간다.

        예전에는 여기서 마지막 목표를 **재전송**했다. 그게 위험했다 - 관제 PC 가 죽어
        멈춘 로봇이 한참 뒤 아무도 중재하지 않는 상태에서 혼자 다시 움직였다.
        지금은 취소만 한다. 재출발이 필요한 정상 상황은 이미 두 군데가 덮는다.

        * coordinator 가 살아 있는데 RESUME 을 잊음 -> 관제 PC 쪽 resume_timeout(15s)
        * 관제 PC 가 죽음 -> 데드맨(command_timeout, 기본 3s)이 훨씬 먼저 잡는다

        그래서 이 워치독은 둘 다 실패했을 때만 도는 마지막 그물이고, 그때 해야 할 일은
        재출발이 아니라 정지다.
        """
        if not self._hold or self._hold_since is None or self._hold_watchdog <= 0.0:
            return
        held = (self.get_clock().now() - self._hold_since).nanoseconds * 1e-9
        if held < self._hold_watchdog:
            return
        self.get_logger().warn(
            f'HOLD 가 {held:.0f}s 지속되어 목표를 취소합니다. 재출발은 하지 않습니다 '
            '(관제 PC 또는 브리지 연결을 확인하고 목표를 다시 지정하세요).')
        self._hold = False
        self._hold_since = None
        self._goal_valid = False
        self._goal_seq += 1
        self._cancel_goal()
        self._goal_handle = None
        self._nav_status = RobotState.NAV_IDLE

    def _publish_state(self):
        self._update_pose_from_tf()
        self._check_link()
        self._check_hold_watchdog()

        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame
        msg.name = self._name
        msg.domain_id = self._domain_id
        msg.localized = self._localized()
        msg.x = self._x
        msg.y = self._y
        msg.yaw = self._yaw
        msg.linear_velocity = self._linear_velocity
        msg.angular_velocity = self._angular_velocity
        # 연결이 끊긴 동안은 Nav2 가 뭐라고 하든 그 사실이 먼저다. _nav_status 자체를
        # 덮지 않아서, 연결이 돌아오면 실제 주행 상태가 그대로 다시 보인다.
        msg.nav_status = (
            RobotState.NAV_LINK_LOST if self._link_lost else self._nav_status)
        msg.goal_valid = self._goal_valid
        msg.goal_x, msg.goal_y, msg.goal_yaw = self._goal
        msg.max_linear_vel = self._max_lin
        msg.max_angular_vel = self._max_ang

        info = self._map_info
        msg.map_name = self._map_name
        msg.map_known = info is not None
        if info is not None:
            msg.map_resolution = info.resolution
            msg.map_width = info.width
            msg.map_height = info.height
            msg.map_origin_x = info.origin.position.x
            msg.map_origin_y = info.origin.position.y

        msg.battery_percent = self._battery_percent
        self._state_pub.publish(msg)


def main(args=None):
    # rclpy 기본 SIGINT 핸들러는 컨텍스트를 즉시 내려 버려서, 종료 직전에 주행을
    # 취소할 틈이 없다. 로봇의 launch 를 Ctrl+C 했을 때 Nav2 가 계속 달리는 것을
    # 막으려면 컨텍스트가 살아 있는 동안 취소를 보내야 한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PinkyAgent()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    stopping = []

    def _request_stop(_signum, _frame):
        stopping.append(True)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=0.1)
        if rclpy.ok():
            node.shutdown_navigation()
            for _ in range(6):
                executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
