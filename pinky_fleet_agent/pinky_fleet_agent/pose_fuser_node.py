#!/usr/bin/env python3
"""map→odom TF 발행자 — AMCL 자리 (D7). 관제의 PoseFix(바닥 마커) 와 로봇 odom 을 합친다.

    입력  /odom (nav_msgs/Odometry)            odom→base_footprint. 버퍼에 쌓는다 (10 s)
          /<name>/pose_fix (PoseFix)           stamp = 이미지 시각(로봇 시계) 의 map→base
          initialpose (PoseWithCovarianceStamped)  관제 [출발] 이 보내는 초기 위치 — 게이트 없이 채택
    출력  TF map→odom  (20 Hz, fix_timeout 안이면)
          /<name>/fix_status (String)          "accepted 12 rejected 1 age 2.3s"

fix 가 fix_timeout 동안 없으면 TF 를 끊는다 → lane_agent 의 pose_timeout 이 로봇을 세운다.
마커 없이 맵만 믿고 오래 달리지 않기 위해서다.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from pinky_lane_msgs.msg import PoseFix

from .pose_fuser import PoseFuser, compose, invert

FIX_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.VOLATILE)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PoseFuserNode(Node):

    def __init__(self):
        super().__init__('pose_fuser')
        self.declare_parameter('robot_name', 'pinky1')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('gate_dist', 0.30)
        self.declare_parameter('gate_yaw_deg', 45.0)
        self.declare_parameter('confirm', 2)
        self.declare_parameter('fix_timeout', 15.0)
        self.declare_parameter('max_reproj', 3.0)

        name = self.get_parameter('robot_name').value
        self._map = self.get_parameter('global_frame').value
        self._odom = self.get_parameter('odom_frame').value
        self._base = self.get_parameter('base_frame').value
        self._max_reproj = float(self.get_parameter('max_reproj').value)
        self.fuser = PoseFuser(gate_dist=float(self.get_parameter('gate_dist').value),
                               gate_yaw=math.radians(float(self.get_parameter('gate_yaw_deg').value)),
                               confirm=int(self.get_parameter('confirm').value),
                               fix_timeout=float(self.get_parameter('fix_timeout').value))
        self._tf = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(String, f'/{name}/fix_status', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 50)
        self.create_subscription(PoseFix, f'/{name}/pose_fix', self._on_fix, FIX_QOS)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self._on_initialpose, 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self._publish_tf)
        self.create_timer(1.0, self._publish_status)
        self._was_alive = False
        self.get_logger().info(
            f'pose_fuser 시작: {name} gate={self.fuser.gate_dist:.2f}m/'
            f'{math.degrees(self.fuser.gate_yaw):.0f}° fix_timeout={self.fuser.fix_timeout:.0f}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.fuser.add_odom(stamp_seconds(msg.header.stamp), p.position.x, p.position.y,
                            yaw_from_quaternion(p.orientation))

    def _on_fix(self, msg: PoseFix):
        if msg.reproj_error > self._max_reproj:
            return
        ok, why = self.fuser.on_fix(stamp_seconds(msg.header.stamp), msg.x, msg.y, msg.yaw, self._now())
        if not ok or why != 'ok':
            self.get_logger().info(f'fix id={msg.marker_id} r={msg.marker_range:.2f}: {why}')

    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        """관제가 준 초기 위치. 지금 odom 기준으로 map→odom 을 바로 잡는다 (게이트 없음)."""
        odom = self.fuser.odom_at(self._now()) or (self.fuser._p[-1] if self.fuser._p else None)
        if odom is None:
            self.get_logger().warn('initialpose: odom 이 아직 없음')
            return
        p = msg.pose.pose
        fix = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.fuser.map_odom = compose(fix, invert(odom))
        self.fuser._accept(fix, self._now())
        self.get_logger().info(f'initialpose 채택: ({fix[0]:.2f}, {fix[1]:.2f}, {math.degrees(fix[2]):.0f}°)')

    def _publish_tf(self):
        now = self._now()
        alive = self.fuser.map_odom is not None and self.fuser.alive(now)
        if alive != self._was_alive:
            self.get_logger().warn('map→odom TF 시작' if alive else
                                   f'fix 가 {self.fuser.fix_timeout:.0f}s 없음 — TF 중단 (로봇 정지)')
            self._was_alive = alive
        if not alive:
            return
        x, y, yaw = self.fuser.map_odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map
        t.child_frame_id = self._odom
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf.sendTransform(t)

    def _publish_status(self):
        msg = String()
        age = self.fuser.fix_age(self._now())
        msg.data = (f'accepted {self.fuser.accepted} rejected {self.fuser.rejected} '
                    f'age {"inf" if math.isinf(age) else f"{age:.1f}s"}')
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFuserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
