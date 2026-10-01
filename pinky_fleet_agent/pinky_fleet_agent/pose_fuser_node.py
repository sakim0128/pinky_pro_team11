#!/usr/bin/env python3
"""map→odom TF 발행자 — AMCL 자리. 관제 항공뷰(`overhead_tracker_node`) 의 위치와 로봇 odom 을 합친다.

    입력  /odom (nav_msgs/Odometry)                      odom→base_footprint. 버퍼에 쌓는다 (10 s)
          /<name>/overhead_pose (geometry_msgs/PoseStamped, map)   관제가 ArUco 로 본 로봇 위치.
                                               stamp 는 **관제 시계** 라 쓰지 않고, 수신 시각 − station_latency 의 odom 에 붙인다
          initialpose (PoseWithCovarianceStamped)      관제 [출발] 이 보내는 초기 위치 — 게이트 없이 채택
    출력  TF map→odom  (20 Hz, fix_timeout 안이면)
          /<name>/fix_status (String)                  "accepted 12 rejected 1 age 0.3s"

fix 가 fix_timeout 동안 없으면 TF 를 끊는다 → lane_agent 의 pose_timeout 이 로봇을 세운다.
마커가 안 보이는데 odom 만 믿고 오래 달리지 않기 위해서다.

마커 부착 보정: 관제 노드는 마커 상단 변을 로봇 전방으로 본다. 다르게 붙였으면 yaw_offset(rad),
마커 중심이 base_footprint 에서 전방으로 offset_x(m) 떨어져 있으면 offset_x 로 되돌린다.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .pose_fuser import PoseFuser, compose, invert


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def marker_to_base(x, y, yaw, yaw_offset=0.0, offset_x=0.0):
    """마커 중심 pose → base_footprint pose (ROS-free, 테스트 대상)."""
    yaw_b = math.atan2(math.sin(yaw - yaw_offset), math.cos(yaw - yaw_offset))
    return x - offset_x * math.cos(yaw_b), y - offset_x * math.sin(yaw_b), yaw_b


class PoseFuserNode(Node):

    def __init__(self):
        super().__init__('pose_fuser')
        self.declare_parameter('robot_name', 'pinky1')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_topic', '')                  # 비우면 /<name>/overhead_pose
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('gate_dist', 0.30)
        self.declare_parameter('gate_yaw_deg', 45.0)
        self.declare_parameter('confirm', 2)
        self.declare_parameter('fix_timeout', 3.0)                # 항공뷰는 10 Hz 연속 — 3 s 끊기면 정지
        self.declare_parameter('station_latency', 0.15)           # 캡처→검출→브릿지→여기 (s)
        self.declare_parameter('yaw_offset', 0.0)                 # 마커 +x 가 로봇 전방과 이루는 각 (rad)
        self.declare_parameter('offset_x', 0.0)                   # 마커 중심의 전방 오프셋 (m)
        self.declare_parameter('blend', 1.0)                      # 1.0 = 매 fix 그대로, 0.5 = 절반씩 (떨림 완화)

        name = self.get_parameter('robot_name').value
        self._map = self.get_parameter('global_frame').value
        self._odom = self.get_parameter('odom_frame').value
        self._base = self.get_parameter('base_frame').value
        self._station_latency = float(self.get_parameter('station_latency').value)
        self._yaw_offset = float(self.get_parameter('yaw_offset').value)
        self._offset_x = float(self.get_parameter('offset_x').value)
        self.fuser = PoseFuser(gate_dist=float(self.get_parameter('gate_dist').value),
                               gate_yaw=math.radians(float(self.get_parameter('gate_yaw_deg').value)),
                               confirm=int(self.get_parameter('confirm').value),
                               fix_timeout=float(self.get_parameter('fix_timeout').value),
                               blend=float(self.get_parameter('blend').value))
        topic = self.get_parameter('pose_topic').value or f'/{name}/overhead_pose'
        self._tf = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(String, f'/{name}/fix_status', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 50)
        self.create_subscription(PoseStamped, topic, self._on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self._on_initialpose, 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self._publish_tf)
        self.create_timer(1.0, self._publish_status)
        self._was_alive = False
        self.get_logger().info(
            f'pose_fuser 시작: {name} ← {topic} gate={self.fuser.gate_dist:.2f}m/'
            f'{math.degrees(self.fuser.gate_yaw):.0f}° fix_timeout={self.fuser.fix_timeout:.1f}s '
            f'latency={self._station_latency:.2f}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.fuser.add_odom(stamp_seconds(msg.header.stamp), p.position.x, p.position.y,
                            yaw_from_quaternion(p.orientation))

    def _on_pose(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self._map:
            self.get_logger().warn(f'overhead_pose frame {msg.header.frame_id} ≠ {self._map}',
                                   throttle_duration_sec=5.0)
            return
        now = self._now()
        p = msg.pose
        x, y, yaw = marker_to_base(p.position.x, p.position.y, yaw_from_quaternion(p.orientation),
                                   self._yaw_offset, self._offset_x)
        ok, why = self.fuser.on_fix(now - self._station_latency, x, y, yaw, now)
        if not ok or why != 'ok':
            self.get_logger().info(f'overhead fix ({x:.2f}, {y:.2f}): {why}', throttle_duration_sec=1.0)

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
                                   f'항공뷰 위치가 {self.fuser.fix_timeout:.1f}s 없음 — TF 중단 (로봇 정지)')
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
