"""관제 도메인(기본 20)에 붙는 콘솔 노드 — 자식 프로세스 C.

이 노드는 로봇 도메인(10/11)에 직접 붙지 않는다. 로봇 상태는 전부 domain_bridge 가
관제 도메인으로 중계해 준 토픽으로만 본다. 즉 **관제 평면**의 유일한 ROS 진입점이다.

하는 일
  - 구독 /clicked_point           : RViz "Publish Point" 로 목적지 입력 (2-click)
  - 구독 /<robot>/amcl_pose       : 브리지가 중계한 각 로봇 위치
  - 발행 /fleet/markers           : 맵 위에 로봇 2대 + 출발지 + 목적지 마커
  - 발행 /fleet/state             : 현재 미션 상태 문자열 (RViz 없이 echo 로도 관제 가능)

왜 /tf 를 브리지하지 않는가
  두 로봇 모두 map/odom/base_link 라는 **같은 frame 이름**을 쓴다. domain_bridge 의
  remap 은 토픽 이름만 바꾸고 메시지 안의 frame_id 는 못 바꾸므로 tf 트리가 충돌한다.
  대신 pose 만 받아서 여기서 Marker 로 그린다. Marker 는 frame_id=map, RViz Fixed Frame=map
  이면 TF 없이도 그려진다.
"""

from __future__ import annotations

import queue

from pinky_fleet import protocol as P
from pinky_fleet.mission_config import Pose2D
from pinky_fleet.pose_utils import quaternion_from_yaw_deg, yaw_deg_from_quaternion

# 로봇별 마커 색 (r, g, b)
ROBOT_COLORS = [(0.95, 0.35, 0.60), (0.25, 0.60, 0.95), (0.40, 0.80, 0.40)]
GOAL_COLOR = (1.0, 0.75, 0.10)
HOME_COLOR = (0.60, 0.60, 0.60)


def console_process(cfg_dict, to_console_q, from_console_q):
    """multiprocessing.Process 의 target."""
    import rclpy

    # ★ 관제 도메인. 로봇 도메인(10/11)과 다르므로 이 프로세스도 따로 떠야 한다.
    rclpy.init(args=[], domain_id=int(cfg_dict['control_domain_id']))
    node = build_console_node()(cfg_dict, to_console_q, from_console_q)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.drain_parent_queue() is False:
                break
            node.publish_markers()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def build_console_node():
    """ConsoleNode 클래스를 만들어 돌려준다.

    rclpy.node.Node 를 상속해야 하는데, 모듈 최상단에서 rclpy 를 import 하면
    부모 프로세스(ROS를 쓰지 않는다)와 L0 dry-run 까지 ROS에 묶인다.
    그래서 클래스 정의 자체를 이 함수 안으로 미룬다.
    """
    from rclpy.node import Node

    class ConsoleNode(Node):
        def __init__(self, cfg, to_console_q, from_console_q):
            super().__init__('fleet_console')
            from geometry_msgs.msg import PointStamped
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from std_msgs.msg import String
            from visualization_msgs.msg import MarkerArray

            self.cfg = cfg
            self.to_console_q = to_console_q
            self.from_console_q = from_console_q
            self.map_frame = cfg['map_frame']
            self.robot_names = [r['name'] for r in cfg['robots']]
            self.homes = {r['name']: Pose2D.from_dict(r['home']) for r in cfg['robots']}
            self.poses = {}
            self.goal = None
            self.state_text = P.MissionState.INIT.value
            self.detail = ''
            self.clicks = []
            self.mode = cfg['goal_input']['mode']
            self.fixed_yaw = float(cfg['goal_input']['goal_yaw_deg'])
            self.accept_clicks = False

            self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
            self.state_pub = self.create_publisher(String, '/fleet/state', 10)
            self._String = String

            self.create_subscription(PointStamped, '/clicked_point', self._on_click, 10)
            for name in self.robot_names:
                self.create_subscription(
                    PoseWithCovarianceStamped, f'/{name}/amcl_pose',
                    self._make_pose_cb(name), 10)

            self.get_logger().info(
                f'관제 콘솔 시작 (domain {cfg["control_domain_id"]}). '
                f'구독: /clicked_point, ' + ', '.join(f'/{n}/amcl_pose' for n in self.robot_names))

        # ---- 콜백 --------------------------------------------------------------
        def _make_pose_cb(self, name):
            def cb(m):
                p = m.pose.pose
                self.poses[name] = Pose2D(
                    p.position.x, p.position.y,
                    yaw_deg_from_quaternion(p.orientation.x, p.orientation.y,
                                            p.orientation.z, p.orientation.w))
            return cb

        def _on_click(self, msg):
            if not self.accept_clicks:
                return
            self.clicks.append((msg.point.x, msg.point.y))
            need = 2 if self.mode == 'two_click' else 1
            self.get_logger().info(
                f'clicked_point {len(self.clicks)}/{need}: '
                f'x={msg.point.x:.3f}, y={msg.point.y:.3f}')
            if len(self.clicks) < need:
                return

            from pinky_fleet.pose_utils import yaw_deg_between
            (x, y) = self.clicks[0]
            if self.mode == 'two_click':
                yaw = yaw_deg_between(x, y, self.clicks[1][0], self.clicks[1][1])
            else:
                yaw = self.fixed_yaw
            self.clicks = []
            self.accept_clicks = False
            pose = Pose2D(x, y, yaw)
            self.goal = pose
            self.from_console_q.put({'op': P.CON_GOAL_PICKED, 'pose': pose.as_dict()})

        # ---- 부모 큐 -----------------------------------------------------------
        def drain_parent_queue(self):
            """부모가 보낸 명령 처리. False 를 반환하면 종료."""
            while True:
                try:
                    cmd = self.to_console_q.get_nowait()
                except queue.Empty:
                    return True
                op = cmd.get('op')
                if op == P.CON_SHUTDOWN:
                    return False
                if op == P.CON_SET_STATE:
                    self.state_text = cmd['state']
                    self.detail = cmd.get('detail', '')
                    self.accept_clicks = (self.state_text == P.MissionState.WAIT_GOAL.value)
                    if self.accept_clicks:
                        need = 2 if self.mode == 'two_click' else 1
                        self.get_logger().info(
                            f'목적지 입력 대기 — RViz 의 "Publish Point" 로 {need}번 클릭하세요.'
                            + (' (1: 목적지, 2: 바라볼 방향)' if need == 2 else ''))
                    msg = self._String()
                    msg.data = f'{self.state_text} | {self.detail}'
                    self.state_pub.publish(msg)
                elif op == P.CON_SET_GOAL:
                    self.goal = Pose2D.from_dict(cmd['pose']) if cmd.get('pose') else None

        # ---- 마커 --------------------------------------------------------------
        def publish_markers(self):
            from visualization_msgs.msg import MarkerArray

            arr = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            mid = 0

            for i, name in enumerate(self.robot_names):
                color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
                home = self.homes[name]
                arr.markers.append(self._arrow(mid, stamp, home, HOME_COLOR, 0.35, alpha=0.5))
                mid += 1
                arr.markers.append(self._text(mid, stamp, home, f'{name} home', HOME_COLOR))
                mid += 1

                pose = self.poses.get(name)
                if pose is None:
                    continue
                arr.markers.append(self._arrow(mid, stamp, pose, color, 0.45))
                mid += 1
                arr.markers.append(self._text(mid, stamp, pose, name, color, z=0.35))
                mid += 1

            if self.goal is not None:
                arr.markers.append(self._arrow(mid, stamp, self.goal, GOAL_COLOR, 0.5))
                mid += 1
                arr.markers.append(self._text(mid, stamp, self.goal, 'GOAL', GOAL_COLOR, z=0.35))
                mid += 1

            # 상태 텍스트를 맵 원점 위에 띄운다
            arr.markers.append(self._text(
                mid, stamp, Pose2D(0.0, 0.0, 0.0),
                f'{self.state_text}  {self.detail}', (1.0, 1.0, 1.0), z=1.2, scale=0.28))
            self.marker_pub.publish(arr)

        def _base_marker(self, mid, stamp):
            from visualization_msgs.msg import Marker
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'fleet'
            m.id = mid
            m.action = Marker.ADD
            return m

        def _arrow(self, mid, stamp, pose, color, length, alpha=0.9):
            from visualization_msgs.msg import Marker
            m = self._base_marker(mid, stamp)
            m.type = Marker.ARROW
            q = quaternion_from_yaw_deg(pose.yaw_deg)
            m.pose.position.x = float(pose.x)
            m.pose.position.y = float(pose.y)
            m.pose.position.z = 0.05
            m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
            m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
            m.scale.x, m.scale.y, m.scale.z = float(length), 0.08, 0.08
            m.color.r, m.color.g, m.color.b = color
            m.color.a = alpha
            return m

        def _text(self, mid, stamp, pose, text, color, z=0.25, scale=0.22):
            from visualization_msgs.msg import Marker
            m = self._base_marker(mid, stamp)
            m.type = Marker.TEXT_VIEW_FACING
            m.pose.position.x = float(pose.x)
            m.pose.position.y = float(pose.y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.z = float(scale)
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.95
            m.text = text
            return m

    return ConsoleNode
