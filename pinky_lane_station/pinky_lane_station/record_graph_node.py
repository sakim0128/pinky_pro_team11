#!/usr/bin/env python3
"""텔레옵 주행을 기록해 road_graph.yaml 을 만든다 (D11, 관제 PC).

    ros2 run pinky_lane_station record_graph --ros-args -p robot:=pinky1 -p out:=$HOME/road_graph_rec.yaml

로봇은 위치추정을 켜고 텔레옵으로 차선 중앙을 유지하며 달린다. 이 노드는 /<robot>/state 의 map 좌표를 쌓고,
터미널에서 노드를 찍는다. 위치 소스(AMCL / 마커 / 항공뷰)는 상관없다 — RobotState 만 본다.

  n <이름> [j|c|e|w]   지금 위치를 노드로 (j 분기, c 횡단보도, e 끝점, w 경유; 기본 w). 같은 이름 = 같은 노드
  u                    마지막 노드 취소
  p                    일시정지 / 재개 (후진·재배치 구간)
  l                    현재 노드·거리 표시
  s                    저장 후 종료
  q                    저장하지 않고 종료
"""

import os
import queue
import sys
import threading

import rclpy
from rclpy.node import Node

from pinky_fleet_msgs.msg import RobotState

from .graph_recorder import GraphRecorder, RecorderParams
from .road_graph import RoadGraphError


class RecordGraphNode(Node):

    def __init__(self):
        super().__init__('record_graph')
        self.declare_parameter('robot', 'pinky1')
        self.declare_parameter('out', os.path.expanduser('~/road_graph_rec.yaml'))
        self.declare_parameter('step', 0.05)
        self.declare_parameter('lane_width', 0.20)
        self.declare_parameter('require_localized', True)
        robot = self.get_parameter('robot').value
        self.out = os.path.expanduser(self.get_parameter('out').value)
        self.require_localized = bool(self.get_parameter('require_localized').value)
        self.rec = GraphRecorder(RecorderParams(step=float(self.get_parameter('step').value),
                                                lane_width=float(self.get_parameter('lane_width').value)))
        self.last = None
        self.create_subscription(RobotState, f'/{robot}/state', self._on_state, 10)
        self._cmds = queue.Queue()
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        self.create_timer(0.1, self._poll)
        self.create_timer(5.0, self._progress)
        self.done = False
        self.get_logger().info(f'record_graph: /{robot}/state 기록 → {self.out}. 명령: n <이름> [j|c|e|w], u, p, l, s, q')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: RobotState):
        self.last = msg
        localized = bool(msg.localized) or not self.require_localized
        self.rec.add_pose(self._now(), msg.x, msg.y, msg.yaw, localized)

    def _stdin_loop(self):
        for line in sys.stdin:
            self._cmds.put(line.strip())

    def _poll(self):
        while not self._cmds.empty():
            self._handle(self._cmds.get())

    def _handle(self, line):
        if not line:
            return
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd == 'n' and len(parts) >= 2:
                m = self.rec.mark(parts[1], parts[2] if len(parts) > 2 else 'w')
                self.get_logger().info(f'노드 {m.name} ({m.type}) at ({m.x:.2f}, {m.y:.2f})  마크 {len(self.rec.marks)}개')
                for w in self.rec.warnings[-2:]:
                    if m.name in w:
                        self.get_logger().warn(w)
            elif cmd == 'u':
                name = self.rec.undo()
                self.get_logger().info(f'취소: {name}' if name else '취소할 마크 없음')
            elif cmd == 'p':
                if self.rec.paused:
                    self.rec.resume()
                    self.get_logger().info('재개. 지금 있는 노드를 다시 찍고(n <이름>) 출발하세요')
                else:
                    self.rec.pause()
                    self.get_logger().warn('일시정지 — 이 구간은 엣지가 되지 않습니다')
            elif cmd == 'l':
                self._progress()
            elif cmd == 's':
                self._save()
                self.done = True
            elif cmd == 'q':
                self.get_logger().warn('저장하지 않고 종료')
                self.done = True
            else:
                self.get_logger().warn(f'모르는 명령: {line!r}')
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _progress(self):
        st = self.last
        loc = '없음' if st is None else ('OK' if st.localized else 'localized=False')
        self.get_logger().info(
            f'pose {len(self.rec.poses)}개, 주행 {self.rec.distance():.2f} m, 노드 {list(self.rec.node_coords())}, '
            f'위치 {loc}{" [일시정지]" if self.rec.paused else ""}')

    def _save(self):
        try:
            graph, warnings = self.rec.build()
        except RoadGraphError as exc:
            self.get_logger().error(f'그래프 검증 실패: {exc}')
            return
        for w in warnings:
            self.get_logger().warn(w)
        graph.save(self.out)
        csv_path = os.path.splitext(self.out)[0] + '_trajectory.csv'
        self.rec.write_csv(csv_path)
        self.get_logger().info(f'저장: {self.out} (노드 {len(graph.nodes)}, 엣지 {len(graph.edges)}), 궤적 {csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = RecordGraphNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node.rec.marks:
            node.get_logger().info('Ctrl+C — 저장 시도')
            node._save()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
