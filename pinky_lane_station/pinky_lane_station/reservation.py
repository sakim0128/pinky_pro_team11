"""구간 예약 — 두 로봇이 같은 엣지·같은 분기에 동시에 들어가지 못하게 한다 (선착순).

ROS 에 의존하지 않는다. coordinator 가 매 틱 로봇 pose 를 넣고 clear_until 을 받아
LaneCommand 로 보낸다.

규칙
  * 엣지는 배타 점유. 엣지 k 를 잡으려면 엣지 k 와 **그 시작 노드** 가 비어 있어야
    한다 (노드 = 분기를 통과할 권리). 출발 노드는 등록 때 잡는다.
  * 로봇이 다음 엣지 시작점의 reserve_ahead 안에 들어오면 요청이 등록된다. 요청 틱이
    빠른 로봇이 먼저 받고, 동률이면 domain_id 가 낮은 쪽.
  * 노드는 그 노드를 release_behind 만큼 지나면 놓는다. 엣지는 그 끝을 release_behind
    만큼 지나면 놓는다. (노드를 먼저 놓아야 교차만 하는 로봇이 오래 기다리지 않는다.)
  * clear_until = 잡고 있는 마지막 엣지의 끝. 단 다음 엣지를 못 잡았으면 끝 노드
    node_stop_margin 앞에서 멈춘다 — 분기 안에서 서지 않기 위해. 마지막 엣지면 목적지.
    아무것도 못 잡았으면 0 (출발점에서 대기).

마주침(같은 엣지 반대 방향)은 배타 점유로 구조적으로 불가능하다. 상호 대기 교착
(서로가 상대가 기다리는 엣지·노드를 잡고 있음)은 감지만 하고 자동으로 풀지 않는다 —
폭 20 cm 도로에서 자동 후진은 위험하다. GUI 가 경고하고 사람이 푼다.
"""

import bisect
from dataclasses import dataclass, field

from .road_graph import project_to_polyline

DEFAULT_RESERVE_AHEAD = 0.40
DEFAULT_RELEASE_BEHIND = 0.25
DEFAULT_NODE_STOP_MARGIN = 0.20


@dataclass
class RobotSlot:
    name: str
    domain_id: int
    route: object                       # road_graph.Route
    cum: list = field(default_factory=list)     # waypoint 별 누적 호길이
    progress_idx: int = 0               # 경로 위 현재 위치 (단조 증가)
    progress_s: float = 0.0
    held: list = field(default_factory=list)    # 연속으로 잡은 엣지 순번 (0 부터)
    finished: bool = False
    waiting_for: str = ''               # 지금 기다리는 엣지 id ('' 이면 없음)

    @property
    def n_edges(self):
        return len(self.route.edge_ids)

    @property
    def next_edge_k(self):
        return (self.held[-1] + 1) if self.held else 0

    def edge_start_s(self, k):
        return 0.0 if k == 0 else self.cum[self.route.edge_end_idx[k - 1]]

    def edge_end_s(self, k):
        return self.cum[self.route.edge_end_idx[k]]

    def idx_at(self, s):
        """누적 호길이 s 이하인 마지막 waypoint 인덱스."""
        return max(0, bisect.bisect_right(self.cum, s + 1e-9) - 1)


class Reservation:
    def __init__(self, graph, reserve_ahead=DEFAULT_RESERVE_AHEAD,
                 release_behind=DEFAULT_RELEASE_BEHIND,
                 node_stop_margin=DEFAULT_NODE_STOP_MARGIN):
        self.graph = graph
        self.reserve_ahead = float(reserve_ahead)
        self.release_behind = float(release_behind)
        self.node_stop_margin = float(node_stop_margin)
        self.robots = {}                # name -> RobotSlot
        self.edge_holder = {}           # edge_id -> robot name
        self.node_holder = {}           # node_id -> robot name
        self.request_tick = {}          # (edge_id, robot) -> tick
        self._tick = 0

    # ---------------------------------------------------------- 등록

    def register(self, name, domain_id, route):
        """경로를 배정한다. 출발 노드가 비어 있으면 잡는다.

        같은 출발점에서 순차 출발하는 두 대는 두 번째가 노드를 못 잡지만, 첫 엣지를
        요청할 때 자연히 앞 로봇이 떠날 때까지 기다리게 된다.
        """
        self.remove(name)
        slot = RobotSlot(name, int(domain_id), route, cum=route.cumulative())
        self.robots[name] = slot
        start = route.node_ids[0]
        if self.node_holder.get(start) is None:
            self.node_holder[start] = name
        return slot

    def remove(self, name):
        if self.robots.pop(name, None) is None:
            return
        for table in (self.edge_holder, self.node_holder):
            for key, holder in list(table.items()):
                if holder == name:
                    del table[key]
        for key in [k for k in self.request_tick if k[1] == name]:
            del self.request_tick[key]

    # ---------------------------------------------------------- 갱신

    def update_pose(self, name, x, y):
        """pose 를 경로 위 진행도로 바꾼다. 뒤로 가는 튐은 무시(단조 증가). 중심선 거리 반환."""
        slot = self.robots[name]
        s, _, _, _, dist = project_to_polyline(x, y, slot.route.waypoints)
        if s >= slot.progress_s:
            slot.progress_s = s
            slot.progress_idx = slot.idx_at(s)
        return dist

    def step(self, tick=None):
        """한 틱. 해제 → 요청 등록 → 선착순 배정. {robot: clear_until_idx} 반환."""
        self._tick = self._tick + 1 if tick is None else int(tick)
        for slot in self.robots.values():
            self._release_passed(slot)
        # 요청 등록을 먼저 전부 한다 — 같은 틱에 들어온 요청은 domain_id 로 갈린다
        for slot in self.robots.values():
            slot.waiting_for = ''
            k = slot.next_edge_k
            if slot.finished or k >= slot.n_edges:
                continue
            if slot.edge_start_s(k) - slot.progress_s <= self.reserve_ahead:
                self.request_tick.setdefault((slot.route.edge_ids[k], slot.name), self._tick)
        order = sorted(self.robots.values(),
                       key=lambda s: (self._earliest_request(s), s.domain_id))
        for slot in order:
            self._try_acquire(slot)
        return {name: self.clear_until(name) for name in self.robots}

    def clear_until(self, name):
        slot = self.robots[name]
        if not slot.held:
            return 0
        k = slot.held[-1]
        if k == slot.n_edges - 1:
            return slot.route.goal_idx
        stop_s = max(slot.edge_start_s(k), slot.edge_end_s(k) - self.node_stop_margin)
        return slot.idx_at(stop_s)

    def mark_arrived(self, name):
        """도착. 엣지는 전부 놓고 목적지 노드만 계속 잡는다 (다른 로봇이 못 들어온다)."""
        slot = self.robots[name]
        slot.finished = True
        goal = slot.route.node_ids[-1]
        for eid, holder in list(self.edge_holder.items()):
            if holder == name:
                del self.edge_holder[eid]
        for nid, holder in list(self.node_holder.items()):
            if holder == name and nid != goal:
                del self.node_holder[nid]
        self.node_holder[goal] = name
        slot.held.clear()

    # ---------------------------------------------------------- 내부

    def _earliest_request(self, slot):
        k = slot.next_edge_k
        if slot.finished or k >= slot.n_edges:
            return float('inf')
        return self.request_tick.get((slot.route.edge_ids[k], slot.name), float('inf'))

    def _try_acquire(self, slot):
        # reserve_ahead 안에 연속으로 들어오는 엣지는 한 틱에 여러 개 잡는다
        while True:
            k = slot.next_edge_k
            if slot.finished or k >= slot.n_edges:
                return
            eid = slot.route.edge_ids[k]
            if (eid, slot.name) not in self.request_tick:
                return
            start_node = slot.route.node_ids[k]
            if (self.edge_holder.get(eid) not in (None, slot.name)
                    or self.node_holder.get(start_node) not in (None, slot.name)):
                slot.waiting_for = eid
                return
            self.edge_holder[eid] = slot.name
            self.node_holder[start_node] = slot.name
            slot.held.append(k)
            del self.request_tick[(eid, slot.name)]
            k2 = k + 1
            if k2 >= slot.n_edges or slot.edge_start_s(k2) - slot.progress_s > self.reserve_ahead:
                return
            self.request_tick.setdefault((slot.route.edge_ids[k2], slot.name), self._tick)

    def _release_passed(self, slot):
        # 노드: 지나간 지 release_behind 이상이면 놓는다 (아직 잡고 있는 뒤쪽 엣지의 시작이어도)
        later_starts = set()
        for k in slot.held:
            nid = slot.route.node_ids[k]
            if slot.progress_s - slot.edge_start_s(k) >= self.release_behind:
                if self.node_holder.get(nid) == slot.name and nid not in later_starts:
                    del self.node_holder[nid]
            else:
                later_starts.add(nid)
        # 엣지: 끝을 release_behind 이상 지나면 놓는다. 마지막 남은 엣지는 도착 전엔 유지
        while len(slot.held) > 1:
            k = slot.held[0]
            if slot.progress_s - slot.edge_end_s(k) < self.release_behind:
                break
            slot.held.pop(0)
            eid = slot.route.edge_ids[k]
            if self.edge_holder.get(eid) == slot.name:
                del self.edge_holder[eid]

    # ---------------------------------------------------------- 조회

    def mutual_wait(self):
        """서로가 상대가 기다리는 엣지 또는 그 시작 노드를 잡고 있는 쌍."""
        out = []
        names = list(self.robots)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if self._blocks(b, a) and self._blocks(a, b):
                    out.append((a, b))
        return out

    def _blocks(self, holder, waiter):
        slot = self.robots[waiter]
        if not slot.waiting_for:
            return False
        k = slot.next_edge_k
        start_node = slot.route.node_ids[k]
        return (self.edge_holder.get(slot.waiting_for) == holder
                or self.node_holder.get(start_node) == holder)

    def status(self, name):
        slot = self.robots[name]
        blocked_by = ''
        if slot.waiting_for:
            k = slot.next_edge_k
            blocked_by = (self.edge_holder.get(slot.waiting_for)
                          or self.node_holder.get(slot.route.node_ids[k]) or '')
        return {
            'progress_idx': slot.progress_idx,
            'clear_until': self.clear_until(name),
            'held_edges': [slot.route.edge_ids[k] for k in slot.held],
            'waiting_for': slot.waiting_for,
            'blocked_by': blocked_by,
            'finished': slot.finished,
        }
