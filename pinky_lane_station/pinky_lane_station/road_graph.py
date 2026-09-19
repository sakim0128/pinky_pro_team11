"""도로망 그래프 — 노드(끝점·분기·횡단보도) + 엣지(구간 중심선).

ROS 에 의존하지 않는다. 관제 GUI, 그래프 편집기, 예약기, 테스트가 전부 이 모듈을 공유한다.

파일 형식 (road_graph.yaml, 좌표는 map frame, 단위 m):

    frame: map
    lane_width: 0.20
    nodes:
      - {id: BL, x: 0.15, y: 0.20, type: endpoint, label: "왼쪽 아래"}
    edges:
      - id: BL_JS
        from: BL
        to: JS
        oneway: false
        waypoints: [[0.15, 0.20], [0.40, 0.30], [0.60, 0.45]]
    registration:              # 선택. 항공 사진 <-> 맵 정합 (편집기가 채운다)
      photo: ../../docs/course_aerial.jpg
      photo_px: [[x, y], ...]  # 사진 픽셀 4점
      map_xy:   [[x, y], ...]  # 대응 맵 좌표 4점

엣지의 waypoints 는 편집기에서 찍은 점 그대로 저장한다. 0.10 m 등간격 재샘플은
경로를 만들 때 메모리에서만 한다 — 파일은 사람이 읽고 손으로 고칠 수 있어야 한다.
"""

import heapq
import math
import os
from dataclasses import dataclass, field

import yaml

NODE_TYPES = ('endpoint', 'junction', 'crosswalk', 'waypoint')
DEFAULT_LANE_WIDTH = 0.20
DEFAULT_STEP = 0.10
ENDPOINT_TOL = 0.05        # from/to 노드와 waypoints 양 끝의 허용 거리


class RoadGraphError(ValueError):
    pass


@dataclass
class Node:
    id: str
    x: float
    y: float
    type: str = 'waypoint'
    label: str = ''

    def to_dict(self):
        out = {'id': self.id, 'x': round(self.x, 3), 'y': round(self.y, 3), 'type': self.type}
        if self.label:
            out['label'] = self.label
        return out


@dataclass
class Edge:
    id: str
    from_id: str
    to_id: str
    waypoints: list            # [(x, y), ...] from -> to 순서
    oneway: bool = False
    length: float = field(init=False)

    def __post_init__(self):
        self.length = polyline_length(self.waypoints)

    def to_dict(self):
        return {
            'id': self.id, 'from': self.from_id, 'to': self.to_id, 'oneway': self.oneway,
            'waypoints': [[round(x, 3), round(y, 3)] for x, y in self.waypoints],
        }


@dataclass
class Projection:
    edge_id: str
    s: float            # 엣지 시작(from)부터의 호길이
    lateral: float      # 엣지 진행 방향 기준 좌측이 양수
    distance: float     # 점과 중심선의 거리 (= |lateral|)
    x: float            # 중심선 위 투영점
    y: float


@dataclass
class Route:
    """시작 노드 → 목적 노드 경로. pinky_lane_msgs/Route 와 필드가 1:1."""
    waypoints: list                 # 0.10 m 등간격 [(x, y), ...]
    edge_ids: list                  # 통과 엣지 순서
    edge_forward: list              # 각 엣지를 from->to 방향으로 통과하는지
    edge_end_idx: list              # 각 엣지가 끝나는 waypoint 인덱스
    node_ids: list                  # 시작 노드 포함, 통과 노드 순서 (len = len(edge_ids)+1)
    node_idx: list                  # 각 노드의 waypoint 인덱스
    crosswalk_idx: list
    junction_idx: list
    goal_idx: int
    length: float

    @property
    def start_id(self):
        return self.node_ids[0]

    @property
    def goal_id(self):
        return self.node_ids[-1]

    def edge_start_idx(self, k):
        return 0 if k == 0 else self.edge_end_idx[k - 1]

    def cumulative(self):
        """waypoint 별 누적 호길이."""
        out = [0.0]
        for (x0, y0), (x1, y1) in zip(self.waypoints, self.waypoints[1:]):
            out.append(out[-1] + math.hypot(x1 - x0, y1 - y0))
        return out


# ---------------------------------------------------------------- 기하 유틸

def polyline_length(points):
    return sum(math.hypot(x1 - x0, y1 - y0)
               for (x0, y0), (x1, y1) in zip(points, points[1:]))


def resample(points, step=DEFAULT_STEP):
    """폴리라인을 step 간격으로 다시 찍는다. 마지막 점은 항상 포함."""
    if len(points) < 2:
        return list(points)
    out = [tuple(points[0])]
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 1e-9:
            continue
        pos = step - carry
        while pos <= seg:
            t = pos / seg
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
            pos += step
        carry = seg - (pos - step)
    last = tuple(points[-1])
    if math.hypot(last[0] - out[-1][0], last[1] - out[-1][1]) > 1e-6:
        out.append(last)
    return out


def project_to_segment(px, py, ax, ay, bx, by):
    """점 -> 선분 투영. (t in [0,1], 투영점 x, y, 부호 있는 횡오차)"""
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-12:
        return 0.0, ax, ay, math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
    qx, qy = ax + dx * t, ay + dy * t
    seg = math.sqrt(seg2)
    lateral = ((px - ax) * dy - (py - ay) * dx) / seg   # 우측 양수
    return t, qx, qy, -lateral                            # 좌측 양수로 뒤집는다


def project_to_polyline(px, py, points):
    """(호길이 s, 좌측 양수 횡오차, 투영점 x, y, 거리). 가장 가까운 선분 기준."""
    best = None
    s_acc = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        seg = math.hypot(bx - ax, by - ay)
        t, qx, qy, lateral = project_to_segment(px, py, ax, ay, bx, by)
        dist = math.hypot(px - qx, py - qy)
        if best is None or dist < best[4]:
            best = (s_acc + seg * t, lateral, qx, qy, dist)
        s_acc += seg
    if best is None:
        x, y = points[0]
        return 0.0, 0.0, x, y, math.hypot(px - x, py - y)
    return best


# ---------------------------------------------------------------- 그래프

class RoadGraph:
    def __init__(self, nodes=None, edges=None, lane_width=DEFAULT_LANE_WIDTH,
                 frame='map', registration=None):
        self.nodes = {n.id: n for n in (nodes or [])}
        self.edges = {e.id: e for e in (edges or [])}
        self.lane_width = float(lane_width)
        self.frame = frame
        self.registration = registration
        self.validate()

    # ---------------------------------------------------------- 입출력

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise RoadGraphError('road_graph 는 최상위가 mapping 이어야 합니다')
        nodes = []
        for raw in data.get('nodes') or []:
            try:
                nodes.append(Node(str(raw['id']), float(raw['x']), float(raw['y']),
                                  str(raw.get('type', 'waypoint')), str(raw.get('label', ''))))
            except (KeyError, TypeError, ValueError) as exc:
                raise RoadGraphError(f'노드 항목이 잘못되었습니다: {raw!r} ({exc})') from exc
        edges = []
        for raw in data.get('edges') or []:
            try:
                pts = [(float(p[0]), float(p[1])) for p in raw['waypoints']]
                edges.append(Edge(str(raw['id']), str(raw['from']), str(raw['to']), pts,
                                  bool(raw.get('oneway', False))))
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                raise RoadGraphError(f'엣지 항목이 잘못되었습니다: {raw!r} ({exc})') from exc
        return cls(nodes, edges, data.get('lane_width', DEFAULT_LANE_WIDTH),
                   data.get('frame', 'map'), data.get('registration'))

    @classmethod
    def load(cls, path):
        path = os.path.expandvars(os.path.expanduser(str(path)))
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        graph = cls.from_dict(data)
        graph.path = path
        return graph

    def to_dict(self):
        out = {
            'frame': self.frame,
            'lane_width': self.lane_width,
            'nodes': [n.to_dict() for n in self.nodes.values()],
            'edges': [e.to_dict() for e in self.edges.values()],
        }
        if self.registration:
            out['registration'] = self.registration
        return out

    def save(self, path):
        path = os.path.expandvars(os.path.expanduser(str(path)))
        with open(path, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(self.to_dict(), handle, allow_unicode=True,
                           sort_keys=False, default_flow_style=None)
        self.path = path

    # ---------------------------------------------------------- 검증

    def validate(self):
        for node in self.nodes.values():
            if node.type not in NODE_TYPES:
                raise RoadGraphError(f'노드 {node.id}: type 은 {NODE_TYPES} 중 하나여야 합니다')
        for edge in self.edges.values():
            for end, node_id in (('from', edge.from_id), ('to', edge.to_id)):
                if node_id not in self.nodes:
                    raise RoadGraphError(f'엣지 {edge.id}: {end} 노드 {node_id} 가 없습니다')
            if len(edge.waypoints) < 2:
                raise RoadGraphError(f'엣지 {edge.id}: waypoints 가 2개 이상이어야 합니다')
            if edge.from_id == edge.to_id:
                raise RoadGraphError(f'엣지 {edge.id}: from 과 to 가 같습니다')
            a, b = self.nodes[edge.from_id], self.nodes[edge.to_id]
            (x0, y0), (x1, y1) = edge.waypoints[0], edge.waypoints[-1]
            if math.hypot(x0 - a.x, y0 - a.y) > ENDPOINT_TOL:
                raise RoadGraphError(
                    f'엣지 {edge.id}: 첫 waypoint 가 from 노드 {a.id} 에서 {ENDPOINT_TOL} m 이상 떨어져 있습니다')
            if math.hypot(x1 - b.x, y1 - b.y) > ENDPOINT_TOL:
                raise RoadGraphError(
                    f'엣지 {edge.id}: 마지막 waypoint 가 to 노드 {b.id} 에서 {ENDPOINT_TOL} m 이상 떨어져 있습니다')
            if edge.length <= 1e-6:
                raise RoadGraphError(f'엣지 {edge.id}: 길이가 0 입니다')

    def isolated_nodes(self):
        used = set()
        for edge in self.edges.values():
            used.add(edge.from_id)
            used.add(edge.to_id)
        return [n for n in self.nodes if n not in used]

    # ---------------------------------------------------------- 조회

    def neighbors(self, node_id):
        """[(edge_id, 반대편 노드, forward)] — oneway 는 from->to 만."""
        out = []
        for edge in self.edges.values():
            if edge.from_id == node_id:
                out.append((edge.id, edge.to_id, True))
            elif edge.to_id == node_id and not edge.oneway:
                out.append((edge.id, edge.from_id, False))
        return out

    def nearest_node(self, x, y, max_dist=None, types=None):
        best, best_d = None, float('inf')
        for node in self.nodes.values():
            if types and node.type not in types:
                continue
            d = math.hypot(node.x - x, node.y - y)
            if d < best_d:
                best, best_d = node.id, d
        if best is None or (max_dist is not None and best_d > max_dist):
            return None
        return best

    def edge_points(self, edge_id, forward=True):
        pts = self.edges[edge_id].waypoints
        return list(pts) if forward else list(reversed(pts))

    def node_exit_yaw(self, node_id, edge_id=None):
        """노드에서 엣지를 따라 나가는 방향(rad). edge_id 없으면 첫 인접 엣지."""
        options = self.neighbors(node_id)
        if not options:
            return 0.0
        if edge_id is not None:
            options = [o for o in options if o[0] == edge_id] or options
        eid, _, forward = options[0]
        pts = self.edge_points(eid, forward)
        # 노드 바로 다음 점이 너무 가까우면 조금 더 앞 점을 본다
        x0, y0 = pts[0]
        for x1, y1 in pts[1:]:
            if math.hypot(x1 - x0, y1 - y0) > 0.02:
                return math.atan2(y1 - y0, x1 - x0)
        return 0.0

    def project(self, x, y):
        """가장 가까운 엣지에 투영."""
        best = None
        for edge in self.edges.values():
            s, lateral, qx, qy, dist = project_to_polyline(x, y, edge.waypoints)
            if best is None or dist < best.distance:
                best = Projection(edge.id, s, lateral, dist, qx, qy)
        return best

    # ---------------------------------------------------------- 경로

    def shortest_route(self, src, dst, step=DEFAULT_STEP):
        if src not in self.nodes or dst not in self.nodes:
            raise RoadGraphError(f'노드가 없습니다: {src!r} 또는 {dst!r}')
        if src == dst:
            raise RoadGraphError('시작과 목적지가 같습니다')
        dist = {src: 0.0}
        prev = {}
        heap = [(0.0, src)]
        while heap:
            d, node = heapq.heappop(heap)
            if d > dist.get(node, float('inf')):
                continue
            if node == dst:
                break
            for eid, other, forward in self.neighbors(node):
                nd = d + self.edges[eid].length
                if nd < dist.get(other, float('inf')):
                    dist[other] = nd
                    prev[other] = (node, eid, forward)
                    heapq.heappush(heap, (nd, other))
        if dst not in prev:
            raise RoadGraphError(f'{src} 에서 {dst} 로 가는 경로가 없습니다')
        chain = []
        node = dst
        while node != src:
            p, eid, forward = prev[node]
            chain.append((eid, forward, p, node))
            node = p
        chain.reverse()
        return self.build_route([(eid, forward) for eid, forward, _, _ in chain], src, step)

    def build_route(self, edge_seq, start_node, step=DEFAULT_STEP):
        """(edge_id, forward) 순서열 → Route. 이웃 엣지 접합점은 한 번만 넣는다."""
        raw = []
        raw_edge_end = []
        node_ids = [start_node]
        node = start_node
        for eid, forward in edge_seq:
            edge = self.edges[eid]
            expect_from = edge.from_id if forward else edge.to_id
            if expect_from != node:
                raise RoadGraphError(f'엣지 {eid} 가 노드 {node} 에 이어지지 않습니다')
            pts = self.edge_points(eid, forward)
            if raw:
                pts = pts[1:]
            raw.extend(pts)
            raw_edge_end.append(len(raw) - 1)
            node = edge.to_id if forward else edge.from_id
            node_ids.append(node)

        # 재샘플 후 엣지 경계를 누적 길이로 다시 찾는다
        cum_raw = [0.0]
        for (x0, y0), (x1, y1) in zip(raw, raw[1:]):
            cum_raw.append(cum_raw[-1] + math.hypot(x1 - x0, y1 - y0))
        edge_end_len = [cum_raw[i] for i in raw_edge_end]

        wps = resample(raw, step)
        cum = [0.0]
        for (x0, y0), (x1, y1) in zip(wps, wps[1:]):
            cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
        edge_end_idx = [min(range(len(cum)), key=lambda i: abs(cum[i] - L)) for L in edge_end_len]
        edge_end_idx[-1] = len(wps) - 1
        node_idx = [0] + edge_end_idx

        crosswalk_idx = [i for i, n in zip(node_idx, node_ids) if self.nodes[n].type == 'crosswalk']
        junction_idx = [i for i, n in zip(node_idx, node_ids) if self.nodes[n].type == 'junction']
        return Route(
            waypoints=wps,
            edge_ids=[e for e, _ in edge_seq],
            edge_forward=[f for _, f in edge_seq],
            edge_end_idx=edge_end_idx,
            node_ids=node_ids,
            node_idx=node_idx,
            crosswalk_idx=crosswalk_idx,
            junction_idx=junction_idx,
            goal_idx=len(wps) - 1,
            length=cum[-1],
        )

    @staticmethod
    def shared_edges(route_a, route_b):
        """두 경로가 공유하는 엣지. (edge_id, same_direction). 반대 방향이면 마주침 위험."""
        dir_b = dict(zip(route_b.edge_ids, route_b.edge_forward))
        out = []
        for eid, fwd in zip(route_a.edge_ids, route_a.edge_forward):
            if eid in dir_b:
                out.append((eid, fwd == dir_b[eid]))
        return out
