#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav2 + DriveCommandGate 모드의 로봇 에이전트 (hybrid_robot.launch.xml).

agent_node(Nav2 단일 목표) 와 같은 FleetCommand · RobotState 계약을 쓰고, 여기에 레인 관제
(Route · LaneCommand · LaneStatus)를 Nav2 목표 연쇄로 받는다. 그래서 pinky_lane_station 의
코디네이터가 레인 로봇과 같은 방식으로 이 로봇을 중재할 수 있다. agent_node · lane_agent_node 와
동시에 띄우지 않는다 — 셋 다 /<robot_name>/command 를 받는다.

agent_node 와 다른 점:
  * /cmd_vel 을 직접 내지 않는다. Nav2 출력은 /cmd_vel_mission 으로 remap 되고 drive_command_gate 가
    /cmd_vel 의 유일한 발행자다. 이 노드는 ESTOP·링크 유실에서 /estop true 를 낸다.
  * 데드맨은 hybrid_link_watch.py (아무 명령이나 생존 신호로 받는다 — 그 파일 머리말).
  * 정지·래치 중에는 이 노드를 거치지 않고 Nav2 에 들어온 goal 까지 취소한다.

Subscribes:
  /{name}/command (pinky_fleet_msgs/FleetCommand): GOTO · STOP · RESUME · CANCEL · SET_INITIAL_POSE ·
      SET_SPEED · SET_MAP · HEARTBEAT
  /{name}/lane_command (pinky_lane_msgs/LaneCommand): START · CLEARANCE · STOP · ESTOP · RESUME (D6 route_chain)
  /{name}/route (pinky_lane_msgs/Route, TRANSIENT_LOCAL): 코디네이터가 배정한 경로 (D6)
  odom (nav_msgs/Odometry): Robot odometry
  battery/percent (std_msgs/Float32): Battery level

Publishes:
  /{name}/state (pinky_fleet_msgs/RobotState): 10 Hz telemetry uplink to Relay
  /{name}/lane_status (pinky_lane_msgs/LaneStatus): 10 Hz — 코디네이터의 START ack·도착 판정 채널 (D6)
  /{name}/diag (std_msgs/String, JSON 1 Hz): fix_status · Nav2 lifecycle · /estop · 게이트 · 에이전트 (R-5, 스키마는 diag.py)
  /estop (std_msgs/Bool): **ESTOP·링크 유실에서만** true, LaneCommand RESUME 에서 false.
      대기(clear 0·STOP)는 goal 취소뿐이다 — 게이트 MISSION 소스가 0.5 s 뒤 0 이 된다.
      래치 동안에는 true 를 `estop_refresh` 초(기본 1 s)마다 다시 낸다 — /estop 은 VOLATILE 이라
      늦게 뜨거나 재기동한 DriveCommandGate 는 한 번 낸 true 를 영영 못 받는다(관제 검수 P2).
      false 는 되풀이하지 않는다 — 다른 출처가 건 비상정지를 덮으면 안 된다.
  initialpose (geometry_msgs/PoseWithCovarianceStamped): Initial pose
  /{name}/plan (nav_msgs/Path): Plan forwarding

레인 규약(START ack · clearance 까지 목표 연쇄 · HOLD/ESTOP 구분)의 판단은 `route_chain.py` 가 하고,
이 노드는 ROS 를 이어 붙이기만 한다. 규약 표는 그 파일 머리말에 있다.

Action Client:
  navigate_to_pose (nav2_msgs/action/NavigateToPose): Nav2 autonomous navigation
"""

import functools
import json
import math
import os
import signal
import sys
import threading

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                           qos_profile_action_status_default)
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import LoadMap
    from std_msgs.msg import Bool, Float32, String
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener
    from pinky_fleet_msgs.msg import FleetCommand, RobotState
    from pinky_lane_msgs.msg import LaneCommand, LaneStatus, Route as RouteMsg
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    Node = object

from .hybrid_link_watch import LinkWatch, ARMED, LOST, RESTORED
from .map_paths import resolve_map_path
from . import route_chain as rc
from .route_chain import RouteChain
from . import diag as diag_mod

INITIAL_POSE_COV_XX = 0.25
INITIAL_POSE_COV_YY = 0.25
INITIAL_POSE_COV_YAWYAW = 0.06853891909122467


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _serialized(fn):
    """상태(route_chain·goal 순번·HOLD)를 고치는 진입점을 한 줄로 세운다.

    ⚠️ 콜백 그룹만으로는 안 된다(2026-09-25 직렬 검토): rclpy Jazzy 는 Future 의 done-callback
       (`_on_goal_response`·`_on_goal_result`·파라미터 응답)을 **그룹 없는 task** 로 돌려서,
       MultiThreadedExecutor 에서는 STOP 처리와 동시에 goto 를 낼 수 있다. 그래서 잠금을 쓴다.
       RLock 이라 같은 스레드의 재진입(콜백 안에서 이미 끝난 future 에 add_done_callback)도 된다.
       잠금 안에서는 기다리지 않는다 — 전부 비동기 호출이다.
    """
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        with self._state_lock:
            return fn(self, *args, **kwargs)
    return wrapper


if HAS_RCLPY:
    class PinkyAgent(Node):
        # S2~S4 상태의 클래스 기본값 — __init__ 을 건너뛰고 만든 시험용 에이전트(도커 통합 d11 등)도 같은 코드를 돈다
        _nav_active = frozenset()
        _foreign_cancel_at = None
        _foreign_canceled = frozenset()     # 이미 취소를 보낸 goal id — 새 id 는 기다리지 않고 바로 취소한다
        _release_pending = None
        _release_deadline = None
        _release_future = None              # 해제를 기다리는 **가장 최근** 스탬프 취소 — 옛 응답으로 새 해제를 풀지 않는다
        _map_load = None                    # R-7: 마지막 지도 교체 결과 {'name','result','detail'} (진단에 싣는다)

        def __init__(self):
            super().__init__('pinky_fleet_agent')

            self.declare_parameter('robot_name', 'pinky1')
            self.declare_parameter('domain_id', 10)
            self.declare_parameter('state_topic', '')
            self.declare_parameter('command_topic', '')
            self.declare_parameter('lane_command_topic', '')
            self.declare_parameter('plan_topic', '')
            self.declare_parameter('amcl_pose_topic', '')
            self.declare_parameter('plan_in_topic', 'plan')
            self.declare_parameter('map_topic', 'map')
            self.declare_parameter('map_dir', '/home/pinky/map')
            self.declare_parameter('map_name', '')
            self.declare_parameter('state_rate', 10.0)
            self.declare_parameter('pose_timeout', 2.0)
            self.declare_parameter('hold_watchdog', 20.0)
            self.declare_parameter('command_timeout', 3.0)
            self.declare_parameter('restore_grace', 1.0)
            self.declare_parameter('global_frame', 'map')
            self.declare_parameter('robot_base_frame', 'base_footprint')
            self.declare_parameter('frame_prefix', '')
            self.declare_parameter('max_linear_vel', 0.2)
            self.declare_parameter('max_angular_vel', 1.5)
            self.declare_parameter('controller_server', 'controller_server')
            self.declare_parameter('velocity_smoother', 'velocity_smoother')
            self.declare_parameter('map_server', 'map_server')
            self.declare_parameter('follow_path_plugin', 'FollowPath')
            # D6 레인 규약 (route_chain) — 코디네이터의 Route·LaneCommand 를 Nav2 목표 연쇄로 바꾼다
            self.declare_parameter('route_topic', '')
            self.declare_parameter('lane_status_topic', '')
            self.declare_parameter('chain_reach_tol', rc.REACH_TOL_M)
            self.declare_parameter('chain_resend_min', rc.RESEND_MIN_M)
            self.declare_parameter('chain_near_active', rc.NEAR_ACTIVE_M)
            self.declare_parameter('chain_min_resend', rc.MIN_RESEND_S)
            # D6-1: 도달 판정 반경을 Nav2 의 goal checker 허용 이상으로 맞추려고 그 값을 읽는다
            self.declare_parameter('goal_checker', 'general_goal_checker')
            self.declare_parameter('estop_refresh', 1.0)
            # R-5 진단 업링크 (/pinkyN/diag, JSON 1 Hz) — 스키마 정본은 diag.py
            self.declare_parameter('diag_topic', '')
            self.declare_parameter('diag_rate', 1.0)
            self.declare_parameter('diag_lifecycle_nodes', list(diag_mod.DEFAULT_NAV2_NODES))
            self.declare_parameter('fix_status_topic', '')
            self.declare_parameter('gate_status_topic', '/drive_gate_status')

            self._name = self.get_parameter('robot_name').value
            self._domain_id = int(self.get_parameter('domain_id').value)
            prefix = self.get_parameter('frame_prefix').value or ''
            base_frame = self.get_parameter('robot_base_frame').value

            self._state_topic = self.get_parameter('state_topic').value or f'/{self._name}/state'
            self._command_topic = self.get_parameter('command_topic').value or f'/{self._name}/command'
            self._lane_cmd_topic = self.get_parameter('lane_command_topic').value or f'/{self._name}/lane_command'
            self._plan_topic = self.get_parameter('plan_topic').value or f'/{self._name}/plan'
            self._amcl_pose_topic = (
                self.get_parameter('amcl_pose_topic').value if self.has_parameter('amcl_pose_topic') else ''
            ) or f'/{self._name}/amcl_pose'
            self._route_topic = self.get_parameter('route_topic').value or f'/{self._name}/route'
            self._lane_status_topic = self.get_parameter('lane_status_topic').value or f'/{self._name}/lane_status'
            self._estop_topic = '/estop'
            # ⭐ 진행 인덱스·clearance 판단은 route_chain 이 한다 (B-5: 예전엔 0 에 고정돼 갱신처가 없었다)
            self._chain = RouteChain(
                reach_tol=float(self.get_parameter('chain_reach_tol').value),
                resend_min=float(self.get_parameter('chain_resend_min').value),
                near_active=float(self.get_parameter('chain_near_active').value),
                min_resend_s=float(self.get_parameter('chain_min_resend').value),
            )
            self._deferred_acts = []         # Nav2 가 없을 때의 재시도는 다음 틱으로 미룬다 (재귀 폭주 방지)
            self._goal_tag = None            # ('chain', idx) | ('single', None)
            self._pose_timeout = float(self.get_parameter('pose_timeout').value)
            self._hold_watchdog = float(self.get_parameter('hold_watchdog').value)
            self._link = LinkWatch(self.get_parameter('command_timeout').value,
                                   self.get_parameter('restore_grace').value)
            self._global_frame = self.get_parameter('global_frame').value
            self._base_frame = f"{prefix}{base_frame}" if (prefix and not base_frame.startswith(prefix)) else base_frame
            self._max_lin = float(self.get_parameter('max_linear_vel').value)
            self._max_ang = float(self.get_parameter('max_angular_vel').value)
            self._follow_path_plugin = self.get_parameter('follow_path_plugin').value

            # 관제 검수 P2: 예전엔 ReentrantCallbackGroup + MultiThreadedExecutor 에 잠금이 없어서 STOP 처리와
            # 진행 중인 goto 가 **동시에** 상태기계를 고칠 수 있었다. 이 노드의 콜백은 한 줄로 세운다.
            # 그래서 콜백 안에서 기다리면 안 된다 — Nav2 서버 확인도 막지 않는(server_is_ready) 방식이다.
            cb = MutuallyExclusiveCallbackGroup()
            self._state_lock = threading.RLock()

            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

            self._x = 0.0
            self._y = 0.0
            self._yaw = 0.0
            self._last_pose_time = None
            self._linear_velocity = 0.0
            self._angular_velocity = 0.0
            # ⚠️ 배터리를 받기 전에는 모른다 — 예전엔 95.0 을 내서 재지 않은 값이 그럴듯한 숫자로 보였다
            self._battery_percent = float('nan')
            self._map_info = None
            self._map_dir = self.get_parameter('map_dir').value
            self._map_name = self.get_parameter('map_name').value or ''

            self._nav_status = RobotState.NAV_IDLE
            self._goal_valid = False
            self._goal = (0.0, 0.0, 0.0)
            self._goal_handle = None
            self._hold = False
            self._hold_since = None
            self._link_lost = False
            self._goal_seq = 0

            # Subscriptions
            self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=cb)
            self.create_subscription(Float32, 'battery/percent', self._on_battery, 10, callback_group=cb)

            map_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
            self.create_subscription(
                OccupancyGrid, self.get_parameter('map_topic').value,
                self._on_map, map_qos, callback_group=cb
            )

            cmd_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE
            )
            self.create_subscription(FleetCommand, self._command_topic, self._on_command, cmd_qos, callback_group=cb)
            self.create_subscription(
                LaneCommand, self._lane_cmd_topic, self._on_lane_command, cmd_qos, callback_group=cb)
            # Route 는 코디네이터가 TRANSIENT_LOCAL 로 한 번 낸다 — 늦게 떠도 받도록 같은 QoS 로 구독한다
            route_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
            self.create_subscription(RouteMsg, self._route_topic, self._on_route, route_qos, callback_group=cb)

            # Publishers
            self._state_pub = self.create_publisher(RobotState, self._state_topic, 10)
            # 코디네이터는 LaneStatus 로만 START 를 확인한다 (D6 — 예전엔 이 발행이 없어 5회 재전송 뒤 포기)
            self._lane_status_pub = self.create_publisher(LaneStatus, self._lane_status_topic, cmd_qos)

            # R-5 진단: 재료를 모아 1 Hz 로 /pinkyN/diag 를 낸다. 못 잰 것은 null 로 남긴다.
            self._diag_topic = self.get_parameter('diag_topic').value or f'/{self._name}/diag'
            self._diag_pub = self.create_publisher(String, self._diag_topic, 5)
            self._fix_text, self._fix_time = None, None
            self._gate_text, self._gate_time = None, None
            self._estop_seen = None
            fix_topic = self.get_parameter('fix_status_topic').value or f'/{self._name}/fix_status'
            self.create_subscription(String, fix_topic, self._on_fix_status, 10, callback_group=cb)
            self.create_subscription(String, self.get_parameter('gate_status_topic').value,
                                     self._on_gate_status, 10, callback_group=cb)
            self.create_subscription(Bool, self._estop_topic, self._on_estop_seen, 10, callback_group=cb)
            # 수명주기: 이름은 상대 경로 — 에이전트와 같은 네임스페이스의 Nav2 노드를 묻는다
            self._lc_clients = {n: self.create_client(GetState, f'{n}/get_state', callback_group=cb)
                                for n in self.get_parameter('diag_lifecycle_nodes').value}
            self._lc_states = {}            # 노드 -> (라벨, 받은 시각)
            self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
            self._amcl_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self._amcl_pose_topic, 10)
            self._estop_pub = self.create_publisher(Bool, self._estop_topic, 10)

            self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, 10, callback_group=cb)

            plan_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE
            )
            self._plan_pub = self.create_publisher(Path, self._plan_topic, plan_qos)
            self.create_subscription(
                Path, self.get_parameter('plan_in_topic').value, self._on_plan, plan_qos, callback_group=cb)

            # Nav2 Action Client
            self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
            # D7: HOLD 는 **이 Nav2 서버의 모든 goal** 을 취소한다. 에이전트를 거치지 않고 들어온 goal
            #     (`/robotN/goal_pose` → bt_navigator 우회 경로)도 세우기 위해서다. 빈 goal_info = 전부.
            self._nav_cancel_all = self.create_client(
                CancelGoal, 'navigate_to_pose/_action/cancel_goal', callback_group=cb)
            # S2~S4 (관제 검수 REVIEW_20260925 §3.2): 에이전트를 거치지 않는 목표(`/robotN/goal_pose` → bt_navigator)를
            # 보려고 Nav2 액션 서버의 상태를 듣는다. 정지·래치 중에 살아 있는 goal 은 우리 것이 아니다.
            self._nav_active = set()          # 서버가 ACCEPTED·EXECUTING 이라 보고한 goal id
            self._foreign_cancel_at = None
            self._release_pending = None      # 래치 해제 동작 — 우회 goal 취소가 끝난 뒤에 낸다(S2)
            self._release_deadline = None
            self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self._on_nav_status,
                                     qos_profile_action_status_default, callback_group=cb)

            map_server = self.get_parameter('map_server').value
            self._load_map_client = self.create_client(LoadMap, f'{map_server}/load_map', callback_group=cb)
            controller = self.get_parameter('controller_server').value
            smoother = self.get_parameter('velocity_smoother').value
            self._controller_params = self.create_client(
                SetParameters, f'{controller}/set_parameters', callback_group=cb)
            self._controller_get = self.create_client(GetParameters, f'{controller}/get_parameters', callback_group=cb)
            self._goal_checker_key = f"{self.get_parameter('goal_checker').value}.xy_goal_tolerance"
            self._nav2_tol_asked = False
            self._nav2_tol_asked_at = None
            self._estop_refresh = float(self.get_parameter('estop_refresh').value)
            self._estop_last_pub = None
            self._smoother_params = self.create_client(SetParameters, f'{smoother}/set_parameters', callback_group=cb)

            rate = float(self.get_parameter('state_rate').value)
            self.create_timer(1.0 / rate, self._publish_state, callback_group=cb)
            diag_rate = float(self.get_parameter('diag_rate').value)
            if diag_rate > 0.0:
                self.create_timer(1.0 / diag_rate, self._publish_diag, callback_group=cb)

            self.get_logger().info(
                f'PinkyAgent started: name={self._name} domain_id={self._domain_id} '
                f'state={self._state_topic} command={self._command_topic} '
                f'lane_cmd={self._lane_cmd_topic} map={self._map_name or "(none)"}'
            )

        # ------------------------------------------------------------------
        # R-5 진단 업링크
        # ------------------------------------------------------------------
        LC_STALE_SEC = 3.0      # 이만큼 응답이 없던 수명주기 상태는 unknown 으로 본다

        def _on_fix_status(self, msg):
            self._fix_text, self._fix_time = msg.data, self._seconds()

        def _on_gate_status(self, msg):
            self._gate_text, self._gate_time = msg.data, self._seconds()

        def _on_estop_seen(self, msg):
            self._estop_seen = bool(msg.data)

        def _poll_lifecycle(self):
            for name, cli in self._lc_clients.items():
                if not cli.service_is_ready():
                    continue
                fut = cli.call_async(GetState.Request())
                fut.add_done_callback(lambda f, n=name: self._on_lc_state(n, f))

        @_serialized
        def _on_lc_state(self, name, fut):
            try:
                st = fut.result().current_state
                label = (st.label or diag_mod.LIFECYCLE_LABELS.get(st.id, 'unknown')).lower()
            except Exception:
                return
            prev = self._lc_states.get(name, (None, None))[0]
            self._lc_states[name] = (label, self._seconds())
            if name == 'bt_navigator' and label == 'active' and prev != 'active':
                # L4: Nav2 가 막 켜졌다 — 켜지는 중에 쌓인 실패로 포기한 목표를 다시 시도한다
                self.get_logger().info('bt_navigator active — clearing goal failures')
                self._apply(self._chain.clear_failures())

        def _ask_nav2_tolerance(self):
            """D6-1: controller_server 의 goal checker xy_goal_tolerance 를 읽어 체인 도달 반경에 반영한다."""
            now = self._seconds()
            if self._nav2_tol_asked:
                # 관제 검수 §3.4: 응답이 영영 안 오면 다시 묻지 않았다 — 5 s 넘게 답이 없으면 다시 묻는다(경고는 한 번)
                if self._chain.nav2_xy_tol is not None or now - (self._nav2_tol_asked_at or now) < 5.0:
                    return
                if not getattr(self, '_nav2_tol_timeout_warned', False):
                    self._nav2_tol_timeout_warned = True
                    self.get_logger().warn(f'{self._goal_checker_key}: 5 s 넘게 응답 없음 — 다시 묻는다')
            if not self._controller_get.service_is_ready():
                return
            self._nav2_tol_asked = True
            self._nav2_tol_asked_at = now
            req = GetParameters.Request()
            req.names = [self._goal_checker_key]
            self._controller_get.call_async(req).add_done_callback(self._on_nav2_tolerance)

        @_serialized
        def _on_nav2_tolerance(self, fut):
            try:
                values = list(fut.result().values)
            except Exception as exc:
                values, err = None, exc
            if not values:
                # rclcpp 는 **선언 안 된** 이름에 빈 목록을 준다 — goal checker 는 controller 가 configure 될 때
                # 선언한다. 그래서 다음 진단 틱(1 Hz)에 다시 묻는다. 경고는 한 번만(매초 쌓이면 로그가 묻힌다).
                self._nav2_tol_asked = False
                if not getattr(self, '_nav2_tol_warned', False):
                    self._nav2_tol_warned = True
                    why = 'no value (not declared yet?)' if values is not None else f'error: {err}'
                    self.get_logger().warn(f'{self._goal_checker_key}: {why} — 체인 도달 반경 = 기본값, 계속 다시 묻는다')
                return
            v = values[0]
            if v.type != ParameterType.PARAMETER_DOUBLE:
                # 없는 이름이면 NOT_SET 이 온다 — 모르는 채로 둔다(기본 reach_tol). 추측값을 넣지 않는다.
                self.get_logger().warn(f'{self._goal_checker_key} 가 없다(type={v.type}) — 체인 도달 반경 = 기본값')
                return
            self._chain.set_nav2_xy_tolerance(v.double_value)
            self.get_logger().info(
                f'Nav2 {self._goal_checker_key}={v.double_value:.3f} m → 체인 도달 반경 '
                f'{self._chain.effective_reach_tol:.3f} m')

        @_serialized
        def _publish_diag(self):
            now = self._seconds()
            self._poll_lifecycle()          # 결과는 다음 발행에 실린다 (비동기)
            self._ask_nav2_tolerance()
            states = {}
            for name in self._lc_clients:
                label, t = self._lc_states.get(name, (None, None))
                states[name] = label if (t is not None and now - t <= self.LC_STALE_SEC) else None
            ch = self._chain
            agent = {
                'drive_state': ch.drive_state(),
                'reason': ch.reason,
                'route_seq': ch.route_seq if ch.follower is not None else None,
                'progress_idx': ch.progress_idx if ch.follower is not None else None,
                'clear_until_idx': ch.clear_until_idx if ch.follower is not None else None,
                'active_target': ch.active_target,
                'reached_idx': ch.reached_idx if ch.follower is not None else None,
                'reach_tol': ch.effective_reach_tol,
                'nav2_xy_goal_tolerance': ch.nav2_xy_tol,
                'goals_sent': ch.sends,
                'stopped': ch.stopped,
                'estop_latched': ch.estop,
                'link_lost': ch.link_lost or self._link_lost,
                'single_goal_hold': self._hold,
                'goal_active': self._goal_handle is not None,
                'map_name': self._map_name,
                'map_load': self._map_load,
            }
            tf_age = None if self._last_pose_time is None else now - self._last_pose_time
            d = diag_mod.build(
                self._name, now, self.get_parameter('use_sim_time').value, states,
                fix_text=self._fix_text, fix_time=self._fix_time,
                gate_text=self._gate_text, gate_time=self._gate_time,
                estop=self._estop_seen, tf_age=tf_age, tf_timeout=self._pose_timeout, agent=agent)
            self._diag_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))

        def _on_plan(self, msg: Path):
            self._plan_pub.publish(msg)

        def _on_odom(self, msg: Odometry):
            self._linear_velocity = float(msg.twist.twist.linear.x)
            self._angular_velocity = float(msg.twist.twist.angular.z)

        def _on_battery(self, msg: Float32):
            self._battery_percent = float(msg.data)

        def _on_map(self, msg: OccupancyGrid):
            self._map_info = msg.info

        def _seconds(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _update_pose_from_tf(self):
            try:
                t = self._tf_buffer.lookup_transform(self._global_frame, self._base_frame, rclpy.time.Time())
            except Exception:
                return
            p = t.transform.translation
            self._x = float(p.x)
            self._y = float(p.y)
            self._yaw = yaw_from_quaternion(t.transform.rotation)
            self._last_pose_time = self._seconds()

        def _localized(self):
            if self._last_pose_time is None:
                return False
            return (self._seconds() - self._last_pose_time) <= self._pose_timeout

        def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
            self._amcl_pose_pub.publish(msg)

        # ------------------------------------------------------------------
        # D6 레인 규약: Route · LaneCommand → route_chain → Nav2 목표 연쇄
        # ------------------------------------------------------------------
        @property
        def _progress_idx(self):
            """B-5: 경로 진행 인덱스. route_chain 이 TF 위치를 Route 에 투영해 갱신한다."""
            return self._chain.progress_idx

        @_serialized
        def _on_route(self, msg):
            if msg.robot_name and msg.robot_name != self._name:
                return
            waypoints = [(p.x, p.y) for p in msg.waypoints]
            if len(waypoints) < 2:
                self.get_logger().warn(f'Route seq={msg.route_seq} has {len(waypoints)} waypoints: ignored')
                return
            self.get_logger().info(
                f'Route received: seq={msg.route_seq} waypoints={len(waypoints)} goal_idx={msg.goal_idx}')
            self._apply(self._chain.on_route(msg.route_seq, waypoints, msg.goal_idx))

        @_serialized
        def _on_lane_command(self, msg: LaneCommand):
            now = self._seconds()
            self._link.pulse(now, heartbeat=(msg.command == LaneCommand.CMD_HEARTBEAT))
            if self._link_lost:
                self._link_lost = False
                self.get_logger().info('Link recovered via LaneCommand (latch stays until LaneCommand RESUME).')
            if msg.command == LaneCommand.CMD_SET_SPEED:
                # lane_agent_node 와 같은 뜻 — 속도 상한만 바꾼다. 주행 상태는 건드리지 않는다.
                self._apply_speed(msg.max_linear_vel, msg.max_angular_vel)
                return
            before = self._chain.drive_state()
            was_stopped, was_estop, was_latched = self._chain.stopped, self._chain.estop, self._chain.latched
            acts = self._chain.on_lane_command(msg.command, msg.route_seq, msg.clear_until_idx)
            if msg.command == LaneCommand.CMD_START and was_stopped and not self._chain.stopped:
                # START 가 체인의 STOP 래치를 풀었다 → FleetCommand STOP 이 건 HOLD 도 푼다. 안 풀면 워치독이
                # 20 s 뒤 달리는 체인 goal 을 취소하고, 체인은 모른 채 CRUISE 를 보고하며 멈춘다(직렬 검토 P1).
                self._hold = False
                self._hold_since = None
            if (not was_stopped and self._chain.stopped) or (not was_estop and self._chain.estop):
                # route_chain 은 자기가 보낸 goal 만 안다 — 우회로 들어온 goal 까지 여기서 세운다.
                # S2: ESTOP 전이는 이미 STOP 중이었어도 무조건(예전엔 STOP 뒤 ESTOP 이면 안 냈다).
                self._cancel_goal(all_goals=True)
            if msg.command == LaneCommand.CMD_RESUME:
                # RESUME 은 단일 목표 경로(FleetCommand)의 HOLD 도 푼다
                self._hold = False
                self._hold_since = None
            released = (was_latched and not self._chain.latched) or (was_stopped and not self._chain.stopped)
            if released:
                # S2: 래치를 푸는 순간 `/estop false` 가 게이트를 연다 — 그 전에 **지금까지 수락된** goal 을 전부 취소하고,
                #     취소 응답이 온 뒤에 해제 동작(/estop false·새 goto)을 낸다. 스탬프 취소라 뒤에 보낼 우리 goal 은 안전하다.
                #     Nav2 상태 보고를 기다리지 않고 **늘** 한다 — 해제 직전에 들어온 우회 goal 은 상태가 아직 안 왔을 수 있다.
                fut = self._cancel_all_before(self.get_clock().now().to_msg())
                if fut is not None:
                    # 앞선 해제가 아직 취소를 기다리면 **이어 붙인다** — 덮어쓰면 그쪽의 ('estop', False) 가 사라져
                    # 게이트가 ESTOP 에 남은 채 CRUISE 를 보고한다(직렬 검토). 응답은 이번 취소의 것만 받는다.
                    self._release_pending = (self._release_pending or []) + list(acts)
                    self._release_deadline = now + 1.0
                    self._release_future = fut
                    fut.add_done_callback(self._on_release_cancel_done)
                    self.get_logger().warn('래치 해제 전 Nav2 의 활성 goal %d 개 취소 — 끝나면 해제한다' % len(self._nav_active))
                    acts = []
            self._apply(acts)
            after = self._chain.drive_state()
            if before != after:
                self.get_logger().info(
                    f'lane drive_state {before} -> {after} ({self._chain.reason})')

        @_serialized
        def _on_nav_status(self, msg):
            act = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            self._nav_active = {bytes(st.goal_info.goal_id.uuid) for st in msg.status_list if st.status in act}

        def _nav2_busy(self):
            """Nav2 가 지금 어떤 goal 이든 진행 중이라고 보고했는가 (에이전트를 거친 것이든 아니든)."""
            return bool(self._nav_active)

        def _halted(self):
            return self._chain.stopped or self._chain.latched or self._hold or self._link_lost

        def _cancel_all_before(self, stamp_msg):
            """stamp 까지 수락된 goal 전부 취소(CancelGoal: id 0 + stamp). 서비스가 없으면 None."""
            if not self._nav_cancel_all.service_is_ready():
                return None
            req = CancelGoal.Request()
            req.goal_info.stamp = stamp_msg
            return self._nav_cancel_all.call_async(req)

        @_serialized
        def _on_release_cancel_done(self, fut):
            if fut is not self._release_future:
                return                        # 옛 해제의 취소 응답 — 새 해제의 취소가 아직 처리되지 않았을 수 있다
            self._flush_release('취소 완료')

        def _flush_release(self, why):
            pending, self._release_pending, self._release_deadline = self._release_pending, None, None
            self._release_future = None
            if pending is None:
                return
            if self._chain.latched:
                # 기다리는 사이 다시 ESTOP·링크유실 — 해제 동작(/estop false 포함)을 버린다
                self.get_logger().warn(f'래치 해제 동작 폐기 — 그 사이 다시 래치됨 ({why})')
                return
            self._apply(pending)

        def _cancel_foreign_goals(self):
            """S2·S3: 정지·래치 중 Nav2 에 살아 있는 goal 은 에이전트를 거치지 않은 것이다(우리 것은 정지 때 취소했다).
            1 Hz 상한으로 전부 취소한다 — 10 Hz STOP 은 '이미 정지' 라 cancel-all 을 다시 내지 않으므로 여기서 본다."""
            if not (self._halted() and self._nav2_busy()):
                return
            now = self._seconds()
            active = frozenset(self._nav_active)
            fresh = active - self._foreign_canceled
            # 1 Hz 상한은 **이미 취소를 보낸** goal 의 재취소에만 건다 — 새 id 는 바로(직렬 검토: 시간 상한만 두면
            # 첫 취소 직후 들어온 두 번째 우회 goal 이 최대 0.9 s 달렸다).
            if not fresh and self._foreign_cancel_at is not None and now - self._foreign_cancel_at < 1.0:
                return
            self._foreign_cancel_at = now
            self._foreign_canceled = active
            self.get_logger().warn(f'정지·래치 중 Nav2 활성 goal {len(active)} 개(새 {len(fresh)}) — 에이전트를 거치지 않은 목표로 보고 취소')
            self._cancel_goal(all_goals=True)

        def _apply(self, acts):
            """route_chain 동작 목록을 실행한다. estop 은 route_chain 이 내라고 할 때만 발행한다."""
            for act in acts:
                kind = act[0]
                if kind == 'goto':
                    _, idx, x, y, yaw = act
                    # 🔴 관제 검수 P1: 미뤄 둔 goto 는 다음 틱에 재생된다. 그 사이 STOP·ESTOP·새 경로가 왔으면
                    #    상태기계는 그 목표를 이미 버렸다 — 묻지 않고 보내면 **정지 중에 주행**한다.
                    if not self._chain.goal_is_current(idx, x, y):
                        self.get_logger().warn(f'stale goto {idx} dropped ({self._chain.reason})')
                        continue
                    self._send_goal_to(x, y, yaw, ('chain', idx))
                elif kind == 'cancel':
                    self._deferred_acts = []
                    self._cancel_goal()
                elif kind == 'estop':
                    if act[1]:
                        self._deferred_acts = []
                    self._estop_pub.publish(Bool(data=bool(act[1])))
                    self._estop_last_pub = self._seconds() if act[1] else None
                    self.get_logger().warn(f'/estop {act[1]} ({self._chain.reason})')

        @_serialized
        def _on_command(self, msg: FleetCommand):
            now = self._seconds()
            prev, curr = self._link.pulse(now, heartbeat=(msg.command == FleetCommand.CMD_HEARTBEAT))
            if prev == LOST and curr in (RESTORED, ARMED):
                self._link_lost = False
                self.get_logger().info('LinkWatch recovered to ARMED/RESTORED.')

            if msg.command == FleetCommand.CMD_HEARTBEAT:
                return

            # 연결 복구 직후 RELIABLE QoS 가 재전송한 **이동** 명령만 거른다. 멈추는 명령은 언제나 받는다.
            motion = msg.command in (FleetCommand.CMD_GOTO, FleetCommand.CMD_RESUME,
                                     FleetCommand.CMD_SET_INITIAL_POSE)
            if motion and not self._link.admit_command(now):
                self.get_logger().warn(f'Ignored command {msg.command} during restore_grace.')
                return

            if msg.command == FleetCommand.CMD_GOTO:
                # 🔴 ESTOP·링크 유실 래치 중에는 단일 목표로도 움직이지 않는다 (예전엔 GOTO 가 /estop false 를 냈다)
                if self._chain.latched:
                    self.get_logger().warn('CMD_GOTO ignored: E-STOP/LINK_LOST latched — LaneCommand RESUME first')
                    return
                self._apply(self._chain.abandon())
                self._hold = False
                self._hold_since = None
                self._goal = (float(msg.x), float(msg.y), float(msg.yaw))
                self._goal_valid = True
                self._send_goal()

            elif msg.command == FleetCommand.CMD_STOP:
                # HOLD 이지 ESTOP 이 아니다 — /estop 을 건드리지 않는다 (게이트 MISSION 소스가 0 이 된다)
                was_holding = self._hold
                self._hold = True
                # 워치독은 'HOLD 가 갱신되지 않은 시간' 을 잰다. 예전엔 첫 STOP 부터 재서, 코디네이터가 10 Hz 로
                # STOP 을 거는 STOPPED 상태에서 20 s 마다 풀림→재HOLD(cancel-all 포함)를 되풀이했다(관제 검수 §3).
                self._hold_since = self.get_clock().now()
                if not was_holding:
                    self._cancel_goal(all_goals=True)
                self._apply(self._chain.on_stop('FleetCommand STOP'))
                self._nav_status = RobotState.NAV_HOLD
                if not was_holding:
                    self.get_logger().info('CMD_STOP received: goal cancelled, HOLD (no E-STOP), awaiting RESUME')

            elif msg.command == FleetCommand.CMD_RESUME:
                # STOP 래치만 푼다 — ESTOP·링크 유실은 LaneCommand RESUME 으로만 풀린다
                self._apply(self._chain.on_fleet_resume())
                if not self._hold:
                    return
                self._hold = False
                self._hold_since = None
                if self._goal_valid and not self._chain.latched:
                    self.get_logger().info('CMD_RESUME: resuming to goal')
                    self._send_goal()
                else:
                    self._nav_status = RobotState.NAV_IDLE

            elif msg.command == FleetCommand.CMD_CANCEL:
                self._apply(self._chain.abandon())
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

        def _send_goal(self):
            """단일 목표(FleetCommand GOTO·RESUME) 전송."""
            gx, gy, gyaw = self._goal
            self._send_goal_to(gx, gy, gyaw, ('single', None))

        def _send_goal_to(self, gx, gy, gyaw, tag):
            self._goal_seq += 1
            seq = self._goal_seq
            self._goal_tag = tag
            # 기다리지 않는다(server_is_ready). 콜백이 한 줄로 서므로 여기서 1 s 기다리면 STOP 도 1 s 늦는다.
            if not self._nav_client.server_is_ready():
                self.get_logger().warn('NavigateToPose action server not available')
                if tag[0] == 'chain':
                    # 실패가 아니라 '서버 없음' 이다 — 세지 않는다. 재시도는 다음 틱으로 미루고,
                    # 간격은 route_chain 의 min_resend_s 가 정한다. 재생 직전에 goal_is_current 가 다시 묻는다.
                    self._deferred_acts = self._chain.on_goal_result(tag[1], 'unavailable')
                else:
                    self._nav_status = RobotState.NAV_ABORTED
                return
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = self._global_frame
            goal_msg.pose.pose.position.x = float(gx)
            goal_msg.pose.pose.position.y = float(gy)
            qz, qw = quaternion_from_yaw(float(gyaw))
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            self._nav_status = RobotState.NAV_ACTIVE
            future = self._nav_client.send_goal_async(goal_msg)
            future.add_done_callback(lambda f, s=seq, t=tag: self._on_goal_response(f, s, t))
            self.get_logger().info(
                f'Nav2 Goal sent {tag}: ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f} deg)')

        @_serialized
        def _on_goal_response(self, future, seq, tag=('single', None)):
            handle = future.result()
            if seq != self._goal_seq:
                # 🔴 응답이 오기 전에 취소·교체된 목표다. 수락됐다면 **지금 취소한다** —
                #    예전엔 무시만 해서, HOLD 직후 늦게 수락된 goal 이 그대로 달렸다.
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.get_logger().warn(f'Goal rejected by Nav2 {tag}')
                self._nav_status = RobotState.NAV_ABORTED
                if tag[0] == 'chain':
                    # L4 (관제 검수 REVIEW_20260925 §3.1): 거부는 목표 실패가 아니다 — Nav2 가 켜지는 중(inactive)이면
                    # 거부한다. 실패로 세면 3번 만에 영구 차단됐고 stop/start 로도 안 풀렸다. '없음' 처럼 1 Hz 로 재시도.
                    self._deferred_acts = self._chain.on_goal_result(tag[1], 'unavailable')
                return
            self._goal_handle = handle
            self._nav_status = RobotState.NAV_ACTIVE
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda f, s=seq, t=tag: self._on_goal_result(f, s, t))

        @_serialized
        def _on_goal_result(self, future, seq, tag=('single', None)):
            if seq != self._goal_seq:
                return
            self._goal_handle = None
            try:
                status = future.result().status
            except Exception as exc:
                self.get_logger().error(f'Goal result error: {exc}')
                status = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                outcome = 'succeeded'
            elif status == GoalStatus.STATUS_CANCELED:
                outcome = 'canceled'
            else:
                outcome = 'aborted'

            if tag[0] == 'chain':
                self._apply(self._chain.on_goal_result(tag[1], outcome))
                return

            if outcome == 'succeeded':
                self._nav_status = RobotState.NAV_SUCCEEDED
                self._goal_valid = False
                self.get_logger().info('Goal reached successfully')
            elif outcome == 'canceled':
                self._nav_status = RobotState.NAV_HOLD if self._hold else RobotState.NAV_CANCELED
            else:
                self._nav_status = RobotState.NAV_ABORTED
                self.get_logger().warn(f'Navigation failed (status={status})')

        @_serialized
        def shutdown_navigation(self):
            self._apply(self._chain.abandon())
            if self._goal_handle is None:
                return
            self._goal_valid = False
            self._cancel_goal()

        def _cancel_goal(self, all_goals=False):
            # 응답을 기다리는 goal 도 무효로 만든다 — 늦게 수락되면 _on_goal_response 가 취소한다
            self._goal_seq += 1
            handle = self._goal_handle
            self._goal_handle = None
            if handle is not None:
                handle.cancel_goal_async()
            if all_goals:
                if self._nav_cancel_all.service_is_ready():
                    self._nav_cancel_all.call_async(CancelGoal.Request())
                else:
                    self.get_logger().warn('cancel-all: NavigateToPose cancel service not ready')

        def _set_map(self, map_name):
            """R-7 웹 전환: map_server 에 map_dir/<map_name>.yaml 을 읽힌다(nav2_msgs/LoadMap).

            달리는 중·래치 중에는 거부한다 — 지도가 바뀌면 달리던 목표의 좌표 뜻이 바뀐다. 결과는 로그와 진단
            (`agent.map_load`)에 남고, 성공하면 RobotState.map_name 이 바뀐다(중계가 원점·크기로 대조한다).
            """
            moving = (self._goal_handle is not None or self._nav2_busy()
                      or abs(self._linear_velocity) > 0.01 or abs(self._angular_velocity) > 0.05)
            if moving or self._chain.latched:
                self._map_load = {'name': map_name, 'result': 'REFUSED',
                                  'detail': '달리는 중' if moving else 'ESTOP·링크유실 래치 중'}
                self.get_logger().warn(f"CMD_SET_MAP {map_name!r} refused: {self._map_load['detail']}")
                return
            path, err = resolve_map_path(self._map_dir, map_name)
            if err:
                self._map_load = {'name': map_name, 'result': 'NO_FILE', 'detail': err}
                self.get_logger().error(f'CMD_SET_MAP: {err}')
                return
            if not self._load_map_client.service_is_ready():
                self._map_load = {'name': map_name, 'result': 'NO_SERVICE', 'detail': 'map_server load_map 서비스 없음'}
                self.get_logger().error('CMD_SET_MAP: map_server load_map service not ready')
                return
            # map_paths 는 'map4.yaml' 같은 이름도 받는다 — 보고하는 이름은 확장자를 뗀 파일 이름이다 (agent_node 와 같다)
            map_name = os.path.splitext(os.path.basename(path))[0]
            req = LoadMap.Request()
            req.map_url = path
            self._map_load = {'name': map_name, 'result': 'PENDING', 'detail': path}
            self._load_map_client.call_async(req).add_done_callback(
                lambda f, n=map_name: self._on_map_loaded(n, f))
            self.get_logger().info(f'CMD_SET_MAP: loading {path}')

        @_serialized
        def _on_map_loaded(self, map_name, fut):
            try:
                result = int(fut.result().result)
            except Exception as exc:                       # noqa: BLE001
                self._map_load = {'name': map_name, 'result': 'ERROR', 'detail': str(exc)}
                self.get_logger().error(f'CMD_SET_MAP {map_name}: {exc}')
                return
            if result == LoadMap.Response.RESULT_SUCCESS:
                self._map_name = map_name
                self._map_load = {'name': map_name, 'result': 'OK', 'detail': ''}
                self.get_logger().info(f'CMD_SET_MAP {map_name}: loaded')
            else:
                self._map_load = {'name': map_name, 'result': 'FAILED', 'detail': f'LoadMap result={result}'}
                self.get_logger().error(f'CMD_SET_MAP {map_name}: LoadMap result={result}')

        def _publish_initial_pose(self, x, y, yaw):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._global_frame
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            qz, qw = quaternion_from_yaw(float(yaw))
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.pose.covariance[0] = INITIAL_POSE_COV_XX
            msg.pose.covariance[7] = INITIAL_POSE_COV_YY
            msg.pose.covariance[35] = INITIAL_POSE_COV_YAWYAW
            self._initialpose_pub.publish(msg)
            self.get_logger().info(f'Initial pose set: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')

        def _apply_speed(self, max_linear_vel, max_angular_vel):
            """agent_node._apply_speed 와 같다 — controller 의 desired_linear_vel 과 velocity_smoother 상한."""
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

        def _check_link(self):
            now = self._seconds()
            if self._link.poll(now) != LOST:
                return
            self._link_lost = True
            # S4: 에이전트가 보낸 goal 만 보면 우회 goal 로 달리던 로봇은 링크가 끊겨도 안 선다 —
            #     Nav2 가 진행 중이라 하거나 odom 이 움직이면 움직이는 중이다.
            moving = (self._goal_handle is not None or self._goal_valid or self._hold or self._nav2_busy()
                      or abs(self._linear_velocity) > 0.01 or abs(self._angular_velocity) > 0.05)
            # 래치는 route_chain 이 쥔다 — 단일 목표든 레인 임무든 같은 규칙(LaneCommand RESUME 으로만 해제)
            acts = self._chain.on_link_lost(moving)
            if not acts:
                return
            silence = self._link.silence(now)
            self.get_logger().warn(
                f'Relay link silent for {silence:.1f}s: deadman switch stopping motion (E-STOP latched).')
            self._hold = False
            self._hold_since = None
            self._goal_valid = False
            self._cancel_goal(all_goals=True)
            self._apply(acts)

        def _check_hold_watchdog(self):
            if not self._hold or self._hold_since is None or self._hold_watchdog <= 0.0:
                return
            held = (self.get_clock().now() - self._hold_since).nanoseconds * 1e-9
            if held < self._hold_watchdog:
                return
            self._hold = False
            self._hold_since = None
            self._goal_valid = False
            if self._goal_tag is not None and self._goal_tag[0] == 'chain':
                # 체인 goal 은 route_chain 이 쥔다 — 여기서 취소하면 체인은 모른 채 active_target 을 들고 멈춘다
                self.get_logger().warn(
                    f'HOLD not refreshed for {held:.0f}s: single-goal HOLD released (chain goal untouched)')
                return
            self.get_logger().warn(f'HOLD not refreshed for {held:.0f}s: cancelling goal to avoid infinite stall.')
            self._goal_seq += 1
            self._cancel_goal()
            self._goal_handle = None
            self._nav_status = RobotState.NAV_IDLE

        def _refresh_estop(self):
            """래치 동안 /estop true 를 estop_refresh 초마다 다시 낸다 (늦게 뜬 게이트도 알게)."""
            if self._estop_refresh <= 0.0 or not self._chain.latched:
                return
            now = self._seconds()
            if self._estop_last_pub is not None and now - self._estop_last_pub < self._estop_refresh:
                return
            self._estop_pub.publish(Bool(data=True))
            self._estop_last_pub = now

        # route_chain drive_state → RobotState.nav_status
        _NAV_FROM_DRIVE = {
            rc.DRIVE_CRUISE: 'NAV_ACTIVE',
            rc.DRIVE_WAIT_CLEARANCE: 'NAV_HOLD',
            rc.DRIVE_ARRIVED: 'NAV_SUCCEEDED',
            rc.DRIVE_ESTOP: 'NAV_HOLD',
            rc.DRIVE_LINK_LOST: 'NAV_LINK_LOST',
        }

        @_serialized
        def _publish_state(self):
            self._update_pose_from_tf()
            if self._deferred_acts:
                acts, self._deferred_acts = self._deferred_acts, []
                self._apply(acts)
            if self._localized():
                self._apply(self._chain.on_pose(self._x, self._y))
            self._check_link()
            self._check_hold_watchdog()
            self._refresh_estop()
            self._cancel_foreign_goals()
            if self._release_pending is not None and self._seconds() >= self._release_deadline:
                self._flush_release('취소 응답 1 s 없음')

            chain_active = self._chain.follower is not None and self._chain.started
            drive = self._chain.drive_state()

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
            if self._link_lost or drive == rc.DRIVE_LINK_LOST:
                msg.nav_status = RobotState.NAV_LINK_LOST
            elif chain_active and drive in self._NAV_FROM_DRIVE:
                msg.nav_status = getattr(RobotState, self._NAV_FROM_DRIVE[drive])
            else:
                msg.nav_status = self._nav_status
            target = self._chain.active_target
            if chain_active and target is not None:
                f = self._chain.follower
                tx, ty = f.points[target]
                msg.goal_valid = True
                msg.goal_x, msg.goal_y = tx, ty
                msg.goal_yaw = f.heading_at(f.cum[target])
            else:
                msg.goal_valid = self._goal_valid
                msg.goal_x, msg.goal_y, msg.goal_yaw = self._goal
            msg.max_linear_vel = self._max_lin
            msg.max_angular_vel = self._max_ang

            info = self._map_info
            msg.map_name = self._map_name
            msg.map_known = info is not None
            if info is not None:
                msg.map_resolution = float(info.resolution)
                msg.map_width = int(info.width)
                msg.map_height = int(info.height)
                msg.map_origin_x = float(info.origin.position.x)
                msg.map_origin_y = float(info.origin.position.y)

            msg.battery_percent = float(self._battery_percent)
            self._state_pub.publish(msg)
            self._publish_lane_status()

        def _publish_lane_status(self):
            """코디네이터가 START ack·도착을 읽는 채널. Route 를 받은 뒤부터 낸다."""
            if self._chain.follower is None:
                return
            st = self._chain.status()
            ls = LaneStatus()
            ls.header.stamp = self.get_clock().now().to_msg()
            ls.header.frame_id = self._global_frame
            ls.robot_name = self._name
            ls.drive_state = int(st['drive_state'])
            reason = st['state_reason']
            if self._halted() and self._nav2_busy():
                # R1 (관제 검수 REVIEW_20260925 §3.3): 정지 사유만 보고 '멈췄다' 로 확인하면 Nav2 goal 이 살아 있어도 확인된다.
                # 사유에 표식을 붙인다 — 코디네이터의 정지 확인은 사유가 **정확히** 같을 때만이라 이 동안은 확인되지 않는다.
                # (STOP 에 순번을 싣는 처방은 LaneCommand 에 빈 필드가 없고 팀11 .msg 는 바이트 동일이라 못 한다)
                reason = f'{reason} · Nav2 활성 {len(self._nav_active)}'
            ls.state_reason = reason
            ls.route_seq = int(st['route_seq'])
            ls.route_idx = int(st['route_idx'])
            ls.clear_until_idx = int(st['clear_until_idx'])
            ls.linear_velocity = float(self._linear_velocity)
            ls.angular_velocity = float(self._angular_velocity)
            # ⚠️ Nav2 에이전트는 차선·초음파·라이다 최소거리를 재지 않는다. 0 은 "장애물 0 m" 같은
            #    **다른 주장**이 되므로 못 잰 값은 NaN 으로 둔다.
            nan = float('nan')
            ls.error_x_norm = nan
            ls.path_age = nan
            ls.lidar_min_range = nan
            ls.us_range = nan
            ls.odom_since_state = nan
            self._lane_status_pub.publish(ls)
else:
    class PinkyAgent:
        pass


def main(args=None):
    if not HAS_RCLPY:
        print("rclpy not installed.")
        sys.exit(1)
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
