#!/usr/bin/env python3
"""차선 주행 에이전트 — 로봇 도메인에서 /cmd_vel 을 발행하는 유일한 노드.

관제 PC 에서 Route(경로) · LaneCommand(clearance/명령) · LanePath(카메라 차선) 를 받고,
AMCL 의 TF(map→base_footprint) 로 위치를 잡아 ``LaneDriver`` 가 계산한 (v, ω) 를 20 Hz 로
낸다. 정지 판단은 전부 로봇에서 한다 (LaneDriver / DriveFsm). 관제는 "어디까지 가도 되는지"
와 "무엇이 보이는지" 만 준다.

기존 FleetCommand(``/<name>/command``) 도 받는다 — CMD_SET_INITIAL_POSE, CMD_HEARTBEAT,
CMD_CANCEL/STOP/RESUME 을 그대로 쓴다. RobotState 도 발행해 기존 GUI 가 위치를 그린다.
Nav2 planner/controller 는 띄우지 않는다 (localization_launch 만).
"""

import math
import signal

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from pinky_fleet_msgs.msg import FleetCommand, RobotState
from pinky_lane_msgs.msg import LaneCommand, LanePath, LaneStatus, Route

from .lane_driver import (CMD_CLEARANCE, CMD_ESTOP, CMD_HEARTBEAT, CMD_RESUME, CMD_SET_SPEED,
                          CMD_START, CMD_STOP, DriverParams, LaneDriver)

INITIAL_POSE_COV_XX = 0.25
INITIAL_POSE_COV_YY = 0.25
INITIAL_POSE_COV_YAWYAW = 0.06853891909122467

RELIABLE_10 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
ROUTE_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT_1 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                           reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.VOLATILE)

# LaneCommand.CMD_* ↔ lane_driver.CMD_* 는 값이 같다 (test_lane_msg_constants 가 지킨다)
_CMD_MAP = {LaneCommand.CMD_HEARTBEAT: CMD_HEARTBEAT, LaneCommand.CMD_START: CMD_START,
            LaneCommand.CMD_STOP: CMD_STOP, LaneCommand.CMD_ESTOP: CMD_ESTOP,
            LaneCommand.CMD_RESUME: CMD_RESUME, LaneCommand.CMD_SET_SPEED: CMD_SET_SPEED,
            LaneCommand.CMD_CLEARANCE: CMD_CLEARANCE}


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def declare_driver_params(node):
    """DriverParams 의 숫자 필드를 ROS 파라미터로 노출하고 채워서 돌려준다."""
    p = DriverParams()
    groups = {'control': p.control, 'fsm': p.fsm, 'guard': p.guard, '': p}
    for prefix, obj in groups.items():
        for key, value in vars(obj).items():
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                continue
            name = f'{prefix}.{key}' if prefix else key
            node.declare_parameter(name, value)          # 타입 유지 (int 는 int)
            setattr(obj, key, type(value)(node.get_parameter(name).value))
    return p


class LaneAgent(Node):

    def __init__(self):
        super().__init__('pinky_lane_agent')
        self.declare_parameter('robot_name', 'pinky1')
        self.declare_parameter('domain_id', 10)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('status_rate', 10.0)
        self.declare_parameter('pose_timeout', 1.0)     # TF 가 이만큼 끊기면 정지
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('us_topic', 'us_sensor/range')
        self.declare_parameter('use_ultrasonic', True)
        self.declare_parameter('restore_grace', 1.0)

        self._name = self.get_parameter('robot_name').value
        self._domain_id = int(self.get_parameter('domain_id').value)
        self._global_frame = self.get_parameter('global_frame').value
        self._base_frame = self.get_parameter('robot_base_frame').value
        self._pose_timeout = float(self.get_parameter('pose_timeout').value)

        self._params = declare_driver_params(self)
        self.driver = LaneDriver(self._params)
        self.driver.station_link.restore_grace = float(self.get_parameter('restore_grace').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._pose = None                     # (x, y, yaw)
        self._pose_time = None
        self._lin = self._ang = 0.0
        self._battery = float('nan')
        self._last_out = None
        self._route_msg = None

        n = self._name
        self.create_subscription(Route, f'/{n}/route', self._on_route, ROUTE_QOS)
        self.create_subscription(LaneCommand, f'/{n}/lane_command', self._on_lane_command, RELIABLE_10)
        self.create_subscription(LanePath, f'/{n}/lane_path', self._on_lane_path, BEST_EFFORT_1)
        self.create_subscription(FleetCommand, f'/{n}/command', self._on_fleet_command, RELIABLE_10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, qos_profile_sensor_data)
        if self.get_parameter('use_ultrasonic').value:
            self.create_subscription(Range, self.get_parameter('us_topic').value,
                                     self._on_us, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Float32, 'battery/percent', self._on_battery, 10)

        self._cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._status_pub = self.create_publisher(LaneStatus, f'/{n}/lane_status', RELIABLE_10)
        self._state_pub = self.create_publisher(RobotState, f'/{n}/state', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        self.create_timer(1.0 / float(self.get_parameter('control_rate').value), self._control_tick)
        self.create_timer(1.0 / float(self.get_parameter('status_rate').value), self._status_tick)
        self.get_logger().info(
            f'pinky_lane_agent 시작: name={n} domain={self._domain_id} '
            f'v_max={self._params.control.v_max:.2f} cam_sign={self._params.control.cam_sign:+.0f} '
            f'link_timeout={self._params.link_timeout:.1f}s path_timeout={self._params.path_timeout:.1f}s')

    # ------------------------------------------------------------------ 입력

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_route(self, msg: Route):
        if msg.robot_name and msg.robot_name != self._name:
            return
        if len(msg.waypoints) < 2:
            self.get_logger().warn('waypoint 가 2개 미만인 Route 무시')
            return
        self._route_msg = msg
        self.driver.set_route([(p.x, p.y) for p in msg.waypoints], list(msg.edge_end_idx),
                              list(msg.edge_ids), list(msg.crosswalk_idx), list(msg.junction_idx),
                              int(msg.goal_idx), int(msg.route_seq))
        self.get_logger().info(
            f'Route seq={msg.route_seq}: {" → ".join(msg.node_ids)} ({msg.length:.2f} m, '
            f'{len(msg.waypoints)} wp, 횡단보도 {len(msg.crosswalk_idx)}, 분기 {len(msg.junction_idx)})')

    def _on_lane_command(self, msg: LaneCommand):
        cmd = _CMD_MAP.get(msg.command)
        if cmd is None:
            self.get_logger().warn(f'알 수 없는 LaneCommand {msg.command}')
            return
        before = self.driver.started, self.driver.estop
        self.driver.set_command(cmd, self._now(), route_seq=int(msg.route_seq),
                                clear_until=int(msg.clear_until_idx),
                                max_v=float(msg.max_linear_vel), max_w=float(msg.max_angular_vel))
        if cmd not in (CMD_HEARTBEAT, CMD_CLEARANCE):
            self.get_logger().info(
                f'LaneCommand {cmd}: started {before[0]}→{self.driver.started} '
                f'estop {before[1]}→{self.driver.estop}')

    def _on_lane_path(self, msg: LanePath):
        self.driver.set_lane_path(self._now(), stamp_seconds(msg.source_stamp), int(msg.quality),
                                  float(msg.error_x_norm), bool(msg.crosswalk_detected))

    def _on_fleet_command(self, msg: FleetCommand):
        now = self._now()
        if msg.command == FleetCommand.CMD_HEARTBEAT:
            self.driver.set_command(CMD_HEARTBEAT, now)
        elif msg.command == FleetCommand.CMD_SET_INITIAL_POSE:
            self._publish_initial_pose(msg.x, msg.y, msg.yaw)
        elif msg.command in (FleetCommand.CMD_STOP, FleetCommand.CMD_CANCEL):
            self.driver.set_command(CMD_STOP, now)
        elif msg.command == FleetCommand.CMD_RESUME:
            self.driver.set_command(CMD_RESUME, now)
        elif msg.command == FleetCommand.CMD_SET_SPEED:
            self.driver.set_command(CMD_SET_SPEED, now, max_v=msg.max_linear_vel,
                                    max_w=msg.max_angular_vel)
        # CMD_GOTO / CMD_SET_MAP 은 차선 모드에서 뜻이 없다

    def _on_scan(self, msg: LaserScan):
        self.driver.update_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max)

    def _on_us(self, msg: Range):
        self.driver.update_us(float(msg.range))

    def _on_odom(self, msg: Odometry):
        self._lin = msg.twist.twist.linear.x
        self._ang = msg.twist.twist.angular.z

    def _on_battery(self, msg: Float32):
        self._battery = float(msg.data)

    def _publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = INITIAL_POSE_COV_XX
        msg.pose.covariance[7] = INITIAL_POSE_COV_YY
        msg.pose.covariance[35] = INITIAL_POSE_COV_YAWYAW
        self._initialpose_pub.publish(msg)
        self.get_logger().info(f'초기 위치: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')

    # ------------------------------------------------------------------ 제어

    def _update_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._global_frame, self._base_frame, Time(),
                                                  timeout=Duration(seconds=0.0))
        except Exception:
            return
        self._pose = (tf.transform.translation.x, tf.transform.translation.y,
                      yaw_from_quaternion(tf.transform.rotation))
        self._pose_time = self._now()

    def _localized(self):
        return self._pose_time is not None and (self._now() - self._pose_time) < self._pose_timeout

    def _control_tick(self):
        self._update_pose()
        now = self._now()
        twist = Twist()
        if self._localized():
            out = self.driver.tick(now, *self._pose)
            twist.linear.x = float(out.v)
            twist.angular.z = float(out.omega)
        else:
            out = self.driver.tick(now, 0.0, 0.0, 0.0) if self.driver.follower is None else self._last_out
            if out is not None:
                out.v = out.omega = 0.0
                out.reason = '위치(TF map→base) 없음 — 정지'
        self._last_out = out
        self._cmd_pub.publish(twist)

    def _status_tick(self):
        out = self._last_out
        st = LaneStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self._global_frame
        st.robot_name = self._name
        if out is not None:
            st.drive_state = int(out.state)
            st.state_reason = out.reason
            st.route_seq = int(self.driver.route_seq)
            st.route_idx = int(out.route_idx)
            st.edge_id = out.edge_id
            st.clear_until_idx = int(out.clear_until)
            st.error_x_norm = float(out.error_x)
            st.lane_quality = int(out.quality)
            st.linear_velocity = float(out.v)
            st.angular_velocity = float(out.omega)
            st.path_age = float(out.path_age) if math.isfinite(out.path_age) else -1.0
            st.lidar_min_range = float(out.lidar_min) if math.isfinite(out.lidar_min) else -1.0
            st.us_range = float(out.us_range) if math.isfinite(out.us_range) else -1.0
            st.odom_since_state = float(out.odom_since_state)
        self._status_pub.publish(st)

        rs = RobotState()
        rs.header.stamp = st.header.stamp
        rs.header.frame_id = self._global_frame
        rs.name = self._name
        rs.domain_id = self._domain_id
        rs.localized = self._localized()
        if self._pose is not None:
            rs.x, rs.y, rs.yaw = self._pose
        rs.linear_velocity = self._lin
        rs.angular_velocity = self._ang
        rs.nav_status = self._nav_status(out)
        rs.max_linear_vel = self._params.control.v_max
        rs.max_angular_vel = self._params.control.omega_max
        rs.battery_percent = self._battery
        self._state_pub.publish(rs)

    @staticmethod
    def _nav_status(out):
        if out is None:
            return RobotState.NAV_IDLE
        s = out.state
        if s == LaneStatus.DRIVE_ARRIVED:
            return RobotState.NAV_SUCCEEDED
        if s == LaneStatus.DRIVE_LINK_LOST:
            return RobotState.NAV_LINK_LOST
        if s in (LaneStatus.DRIVE_IDLE,):
            return RobotState.NAV_IDLE
        if s in (LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_WAIT_CLEARANCE):
            return RobotState.NAV_HOLD
        return RobotState.NAV_ACTIVE

    def stop_wheels(self):
        for _ in range(5):
            self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LaneAgent()
    stopping = []

    def _request_stop(_signum, _frame):
        stopping.append(True)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
        if rclpy.ok():
            node.get_logger().info('종료: 바퀴 정지')
            node.stop_wheels()
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
