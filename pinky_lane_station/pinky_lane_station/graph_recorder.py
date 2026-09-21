"""텔레옵 주행 궤적 + 노드 마킹 → 도로망 그래프 (D11). ROS 에 의존하지 않는다.

    rec = GraphRecorder()
    rec.add_pose(t, x, y, yaw, localized=True)     # 10 Hz
    rec.mark('BL', 'e')                            # 지금 위치를 노드로. 같은 이름 재입력 = 같은 노드(좌표 평균)
    rec.pause(); ...; rec.resume()                 # 후진·재배치 구간은 엣지가 되지 않는다
    graph, warnings = rec.build()                  # road_graph.RoadGraph (validate 통과)

규칙
  * 연속된 두 마크 사이 궤적 = 엣지 waypoints. 이동평균으로 평활 → step 간격 재샘플 → 양 끝을 노드 좌표에 맞춘다
  * 같은 노드 쌍(방향 무관)을 두 번 지나면 두 번째는 버리고 경고
  * 사이에 일시정지가 있었거나 localized=False 인 pose 가 섞이면 그 구간은 버리고 경고
  * 마크 사이 길이가 min_edge_length 미만이면 버린다 (같은 자리에서 두 번 찍은 것)
"""

import csv
import math
from dataclasses import dataclass, field

from .road_graph import Edge, Node, RoadGraph, polyline_length, resample

TYPE_KEYS = {'j': 'junction', 'c': 'crosswalk', 'e': 'endpoint', 'w': 'waypoint',
             'junction': 'junction', 'crosswalk': 'crosswalk', 'endpoint': 'endpoint',
             'waypoint': 'waypoint', '': 'waypoint'}


@dataclass
class RecorderParams:
    step: float = 0.05                 # 엣지 재샘플 간격 (m)
    smooth_window: int = 5             # 이동평균 창 (pose 수, 홀수)
    min_edge_length: float = 0.10      # 이보다 짧은 마크 간격은 엣지가 아니다
    near_node_warn: float = 0.15       # 다른 이름의 기존 노드가 이 거리 안이면 경고 (오타·중복 의심)
    lane_width: float = 0.20


@dataclass
class Pose:
    t: float
    x: float
    y: float
    yaw: float
    localized: bool = True
    paused: bool = False


@dataclass
class Mark:
    name: str
    type: str
    pose_index: int
    x: float
    y: float


@dataclass
class GraphRecorder:
    params: RecorderParams = field(default_factory=RecorderParams)
    poses: list = field(default_factory=list)
    marks: list = field(default_factory=list)
    paused: bool = False
    warnings: list = field(default_factory=list)

    # ------------------------------------------------ 입력

    def add_pose(self, t, x, y, yaw, localized=True):
        self.poses.append(Pose(float(t), float(x), float(y), float(yaw), bool(localized), self.paused))

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def mark(self, name, type_key='w'):
        """지금(마지막 pose) 위치를 노드로 찍는다. 같은 이름이면 같은 노드."""
        if not self.poses:
            raise ValueError('아직 위치를 받지 못했습니다')
        name = str(name).strip()
        if not name:
            raise ValueError('노드 이름이 비었습니다')
        if type_key not in TYPE_KEYS:
            raise ValueError(f'노드 타입은 j/c/e/w 중 하나: {type_key!r}')
        p = self.poses[-1]
        if not p.localized:
            self.warnings.append(f'{name}: localized=False 상태에서 찍음')
        for other, (ox, oy) in self.node_coords().items():
            if other != name and math.hypot(ox - p.x, oy - p.y) < self.params.near_node_warn:
                self.warnings.append(f'{name}: 기존 노드 {other} 와 {math.hypot(ox - p.x, oy - p.y):.2f} m — 같은 노드면 같은 이름을 쓰세요')
        m = Mark(name, TYPE_KEYS[type_key], len(self.poses) - 1, p.x, p.y)
        self.marks.append(m)
        return m

    def undo(self):
        """마지막 마크 취소. 취소된 마크 이름을 돌려준다 (없으면 None)."""
        if not self.marks:
            return None
        return self.marks.pop().name

    # ------------------------------------------------ 조회

    def node_coords(self):
        """이름 → 평균 좌표 (여러 번 찍은 노드는 평균)."""
        acc = {}
        for m in self.marks:
            sx, sy, n = acc.get(m.name, (0.0, 0.0, 0))
            acc[m.name] = (sx + m.x, sy + m.y, n + 1)
        return {k: (sx / n, sy / n) for k, (sx, sy, n) in acc.items()}

    def node_types(self):
        out = {}
        for m in self.marks:
            # 가장 구체적인 타입 우선 (waypoint 는 기본값)
            if m.name not in out or out[m.name] == 'waypoint':
                out[m.name] = m.type
        return out

    def distance(self):
        pts = [(p.x, p.y) for p in self.poses if not p.paused]
        return polyline_length(pts) if len(pts) > 1 else 0.0

    # ------------------------------------------------ 빌드

    def _smooth(self, pts):
        w = max(1, int(self.params.smooth_window) | 1)
        if w <= 1 or len(pts) < w:
            return list(pts)
        half = w // 2
        out = []
        for i in range(len(pts)):
            lo, hi = max(0, i - half), min(len(pts), i + half + 1)
            xs = [p[0] for p in pts[lo:hi]]
            ys = [p[1] for p in pts[lo:hi]]
            out.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        return out

    def build(self):
        """(RoadGraph, warnings). 마크가 2개 미만이면 노드만 있는 그래프."""
        p = self.params
        warnings = list(self.warnings)
        coords = self.node_coords()
        types = self.node_types()
        nodes = [Node(name, x, y, types[name]) for name, (x, y) in coords.items()]
        edges = {}
        seen_pairs = set()
        for a, b in zip(self.marks, self.marks[1:]):
            label = f'{a.name}→{b.name}'
            seg = self.poses[a.pose_index:b.pose_index + 1]
            if any(q.paused for q in seg[1:-1]) or (len(seg) > 1 and seg[1].paused):
                warnings.append(f'{label}: 일시정지 구간 포함 — 버림')
                continue
            if any(not q.localized for q in seg):
                warnings.append(f'{label}: localized=False 포함 — 버림')
                continue
            if a.name == b.name:
                continue                                   # 같은 노드를 다시 찍음 (재시작점)
            pts = [(q.x, q.y) for q in seg]
            if polyline_length(pts) < p.min_edge_length:
                warnings.append(f'{label}: 길이 {polyline_length(pts):.2f} m — 너무 짧아 버림')
                continue
            pair = frozenset((a.name, b.name))
            if pair in seen_pairs:
                warnings.append(f'{label}: 같은 노드 쌍 엣지가 이미 있음 — 두 번째 버림')
                continue
            seen_pairs.add(pair)
            pts = self._smooth(pts)
            pts[0] = coords[a.name]
            pts[-1] = coords[b.name]
            pts = resample(pts, p.step)
            pts[0] = coords[a.name]
            pts[-1] = coords[b.name]
            eid = f'{a.name}_{b.name}'
            edges[eid] = Edge(eid, a.name, b.name, [tuple(q) for q in pts])
        graph = RoadGraph(nodes, list(edges.values()), lane_width=p.lane_width)
        for n in graph.isolated_nodes():
            warnings.append(f'노드 {n}: 연결된 엣지 없음')
        return graph, warnings

    def write_csv(self, path):
        mark_at = {}
        for m in self.marks:
            mark_at.setdefault(m.pose_index, []).append(m.name)
        with open(path, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'x', 'y', 'yaw', 'localized', 'paused', 'mark'])
            for i, q in enumerate(self.poses):
                w.writerow([f'{q.t:.3f}', f'{q.x:.4f}', f'{q.y:.4f}', f'{q.yaw:.4f}',
                            int(q.localized), int(q.paused), '|'.join(mark_at.get(i, []))])
