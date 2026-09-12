#!/usr/bin/env python3
"""핑키 1대의 도메인 안에서 도는 경량 에이전트.

관제 PC 와는 토픽 2개(``fleet/state`` / ``fleet/command``)로만 대화한다.
Nav2 액션·파라미터 서비스·TF 는 전부 이 로봇 도메인 안에서만 쓰이므로
domain_bridge 가 토픽만 브리지해도 전체 시스템이 동작한다.
"""

import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from pinky_fleet_msgs.msg import FleetCommand, RobotState

from .map_paths import resolve_map_path

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
        self.declare_parameter('hold_watchdog', 45.0)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('map_server', '/map_server')
        self.declare_parameter('controller_server', '/controller_server')
        self.declare_parameter('velocity_smoother', '/velocity_smoother')
        self.declare_parameter('follow_path_plugin', 'FollowPath')

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

        self.get_logger().info(
            f'pinky_fleet_agent 시작: name={self._name} domain_id={self._domain_id} '
            f'state={self._state_topic} command={self._command_topic} '
            f'plan={self._plan_topic} '
            f'map_dir={self._map_dir} map={self._map_name or "(미지정)"}')

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

    def _localized(self):
        if self._last_pose_time is None:
            return False
        age = (self.get_clock().now() - self._last_pose_time).nanoseconds * 1e-9
        return age < self._pose_timeout

    # ------------------------------------------------------------------ 명령

    def _on_command(self, msg: FleetCommand):
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

    # ------------------------------------------------------------------ 상태

    def _check_hold_watchdog(self):
        """관제 PC 가 죽어도 로봇이 영원히 서 있지 않도록 스스로 HOLD 를 푼다."""
        if not self._hold or self._hold_since is None or self._hold_watchdog <= 0.0:
            return
        held = (self.get_clock().now() - self._hold_since).nanoseconds * 1e-9
        if held < self._hold_watchdog:
            return
        self.get_logger().warn(
            f'HOLD 가 {held:.0f}s 지속되어 워치독으로 자동 해제합니다 '
            '(관제 PC 또는 브리지 연결을 확인하세요).')
        self._hold = False
        self._hold_since = None
        if self._goal_valid:
            self._send_goal()
        else:
            self._nav_status = RobotState.NAV_IDLE

    def _publish_state(self):
        self._update_pose_from_tf()
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
        msg.nav_status = self._nav_status
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
    rclpy.init(args=args)
    node = PinkyAgent()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
