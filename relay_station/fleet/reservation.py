#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""구간 예약 — 두 로봇이 같은 엣지·같은 분기에 동시에 들어가지 못하게 한다 (선착순).

Team11 reservation.py 기반 재사용 및 확장.
규칙:
  * 엣지는 배타 점유. 엣지 k 를 잡으려면 엣지 k 와 그 시작 노드가 비어 있어야 한다.
  * 로봇이 **요청 기준점**의 reserve_ahead 안에 들어오면 다음 엣지 요청 등록. 기준점은 잡은 엣지가 있으면
    코디네이터가 명령한 정지 지점(clear_until), 없으면 다음 엣지 시작점이다(D6-2, 아래 `_request_anchor`).
  * 노드는 그 노드를 release_behind 만큼 지나면 놓는다. 엣지는 그 끝을 release_behind 만큼 지나면 놓는다.
  * clear_until = 잡고 있는 마지막 엣지의 끝. 다음 엣지를 못 잡았으면 끝 노드 node_stop_margin 앞에서 멈춘다.
  * 상호 대기 교착(mutual_wait) 감지.
  * 진행도는 둘이다(REVIEW_20260926 H, 아래 `update_pose`): **잡을 때는 가장 앞(lead_s), 놓을 때는 확인된 것(progress_s
    — 최근 확인 창의 가장 뒤, 속도 상한)**. 위치가 튀어도 로봇이 아직 있을 수 있는 엣지는 놓지 않는다.
    잡기는 에이전트가 보고한 진행(LaneStatus.route_idx)도 따른다(통합 검토 RES-F2, 아래 `note_reported_idx`).
    도착(한꺼번에 놓기)은 확인 창의 과반이 목표 근처일 때만 받는다(통합 검토 FLEET-R1, 아래 `goal_gap`).
"""

import bisect
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from pinky_lane_station.road_graph import project_to_polyline

DEFAULT_RESERVE_AHEAD = 0.40
DEFAULT_RELEASE_BEHIND = 0.25
DEFAULT_NODE_STOP_MARGIN = 0.20
# REVIEW_20260926 H — 튀는 위치 추정이 놓기(해제)를 앞당기지 못하게 하는 값들. 코디네이터는 update_pose 에 시각을 넘기지
# 않고 10 Hz 틱마다 한 번 부른다(fleet_coordinator._loop_tick) — 그래서 시간은 호출 수 × UPDATE_PERIOD 로 잰다.
DEFAULT_MAX_SPEED = 0.30          # m/s — 놓기 진행도가 오를 수 있는 가장 빠른 속도. 팀11 max_linear_vel 0.15 의 2배,
                                  #        플릿 Nav2 velocity_smoother 0.25 보다 위(실제 주행을 늦추지 않게)
DEFAULT_UPDATE_PERIOD = 0.10      # s — update_pose 호출 간격(코디네이터 tick_rate 10 Hz)
DEFAULT_CONFIRM_UPDATES = 20      # 회 — 놓기 전에 위치가 연달아 그 너머여야 하는 횟수(10 Hz 로 2 s: 1 Hz fix 두 번)


@dataclass
class RobotSlot:
    name: str
    domain_id: int
    route: object                       # road_graph.Route
    cum: list = field(default_factory=list)     # waypoint 별 누적 호길이
    progress_idx: int = 0               # progress_s 의 waypoint (내려갈 수도 있다). 코디네이터의 stale 정지(허가 ≤ 이것)도 쓴다 —
                                        # 확인이 늦어 실제보다 뒤여도 에이전트는 허가 ≤ 도달이면 서기만 한다(뒤로 안 간다)
    progress_s: float = 0.0             # 놓기용 — 확인된 진행도(REVIEW_20260926 H). 해제·progress_idx 가 이것을 본다
    held: list = field(default_factory=list)    # 연속으로 잡은 엣지 순번 (0 부터)
    finished: bool = False
    waiting_for: str = ''               # 지금 기다리는 엣지 id ('' 이면 없음)
    lead_s: float = 0.0                 # 잡기용 — 가장 앞선 추정(예전 progress_s 와 같다). 다음 엣지 요청이 이것을 본다
    recent: deque = field(default_factory=deque)    # 최근 투영 호길이(놓기 확인 창)
    behind: list = field(default_factory=list)      # lead_s 보다 regress_tol 넘게 뒤였던 연속 투영
    reported_s: float = 0.0             # 잡기용 — 에이전트가 **스스로 믿는** 진행(LaneStatus.route_idx 의 호길이, 통합 검토 RES-F2).
                                        # 놓기(progress_s)에는 절대 안 쓴다

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
                 node_stop_margin=DEFAULT_NODE_STOP_MARGIN,
                 max_speed=DEFAULT_MAX_SPEED, update_period=DEFAULT_UPDATE_PERIOD,
                 confirm_updates=DEFAULT_CONFIRM_UPDATES, regress_tol=None):
        self.graph = graph
        self.reserve_ahead = float(reserve_ahead)
        self.release_behind = float(release_behind)
        self.node_stop_margin = float(node_stop_margin)
        # REVIEW_20260926 H: 놓기 진행도는 한 번에 max_step 까지만 오른다(물리적으로 갈 수 있는 거리).
        # 여유(margin)를 더하지 않는다 — 10 Hz 로 불리므로 호출마다 더한 여유는 초당 10배로 쌓인다. 여유는 max_speed 에 있다.
        self.max_step = float(max_speed) * float(update_period)
        self.confirm_updates = max(1, int(confirm_updates))
        # 뒤로 재투영 문턱 — 도로 폭보다 크게 뒤면 투영 잡음이 아니다(REVIEW_20260926 H 처방 "> lane_width")
        self.regress_tol = float(graph.lane_width if regress_tol is None else regress_tol)
        self.robots = {}                # name -> RobotSlot
        self.edge_holder = {}           # edge_id -> robot name
        self.node_holder = {}           # node_id -> robot name
        self.request_tick = {}          # (edge_id, robot) -> tick
        self._tick = 0

    def register(self, name, domain_id, route):
        """경로를 배정한다. 출발 노드가 비어 있으면 잡는다."""
        self.remove(name)
        slot = RobotSlot(name, int(domain_id), route, cum=route.cumulative(),
                         recent=deque(maxlen=self.confirm_updates))
        self.robots[name] = slot
        start = route.node_ids[0]
        if self.node_holder.get(start) is None:
            self.node_holder[start] = name
        return slot

    def assign_conflict(self, name, goal_node, start_node=None):
        """(부딪히는 로봇, 이유) 또는 None — 코디네이터가 경로를 배정하기 **전에** 본다(L3). ROS 없이 돈다.

        도착한 로봇은 목표 노드를 **계속** 잡으므로(mark_arrived) 그 자리를 누가 지나가야 하면 영영 못 간다 — 그런 배정은
        여기서 거절한다. L3 (관제 검수 §3.1): 같은 목표 노드. 직렬 검토(L3 후속): 목표가 다른 로봇의 경로 **위**이거나,
        새 경로가 다른 로봇의 목표·도착해 선 자리를 **지나면** 한쪽이 서는 순간 다른 쪽이 경고도 없이 영구 대기했다.
        2026-09-29(map5 임시 미션 pinky1 BL→TR · pinky2 BR→BL): register 는 출발 노드도 잡는다(BL = pinky1 출발 = pinky2 목표).
        그 잡음은 로봇이 떠나면 풀리므로(release_behind) 거절 사유가 아니다 — **도착해 선(finished) 로봇**이 잡은 노드만 본다.
        아직 출발 전이면 pinky2 는 J 앞에서 pinky1 이 BL·BL_J 를 놓을 때까지 기다린다(blocked_by 로 화면에 보인다).
        """
        for other, slot in self.robots.items():
            if other == name:
                continue
            if slot.route.node_ids[-1] == goal_node:
                return other, f'{other} 가 이미 목표 노드 {goal_node} 로 간다(도착한 로봇은 목표 노드를 계속 잡는다)'
            if not slot.finished and goal_node in slot.route.node_ids[1:-1]:
                return other, (f'목표 {goal_node} 가 {other} 의 경로 위다 — 여기 도착해 서면 {other} 가 지나가지 못한다')
        holder = self.node_holder.get(goal_node)
        if holder not in (None, name) and getattr(self.robots.get(holder), 'finished', False):
            return holder, f'{holder} 가 목표 노드 {goal_node} 에 도착해 서 있다'
        if start_node:
            try:
                path = self.graph.shortest_route(start_node, goal_node, step=0.10).node_ids
            except Exception:                                   # noqa: BLE001 — 경로가 없으면 assign_route 가 말한다
                return None
            # 도착한 로봇이 잡는 노드는 제 목표뿐이다(mark_arrived) — 목표만 보면 도착해 선 자리도 걸린다
            for nid in path[1:-1]:
                for other, slot in self.robots.items():
                    if other != name and slot.route.node_ids[-1] == nid:
                        return other, (f'경로가 {other} 의 목표 {nid} 를 지난다 — {other} 가 도착해 서면 지나가지 못한다')
        return None

    def remove(self, name):
        if self.robots.pop(name, None) is None:
            return
        for table in (self.edge_holder, self.node_holder):
            for key, holder in list(table.items()):
                if holder == name:
                    del table[key]
        for key in [k for k in self.request_tick if k[1] == name]:
            del self.request_tick[key]

    def update_pose(self, name, x, y):
        """pose 를 경로 위 진행도로 바꾼다. 중심선 거리 반환. 코디네이터 틱(10 Hz)마다 한 번 불린다.

        🔴 REVIEW_20260926 H (리그 실측): 예전엔 진행도 하나를 단조 증가("뒤로 가는 튐 무시")로 뒀다. 정적 1 Hz fix 와
           odom 전파 사이에서 보고 포즈가 BL 주변을 헤매자 헤맨 **최대치**가 누적돼, 물리적으로 BL 에 선 pinky2 가
           BL·BL_JS 를 놓고 RM_RE 까지 잡았다 — 로봇이 아직 있는 엣지를 다른 로봇에 내줄 수 있다.
        그래서 진행도를 둘로 나눈다. 위치가 불확실하면 로봇은 그 범위 **어디에나** 있을 수 있다:
          * lead_s(잡기) = 가장 앞선 추정 — 예전과 같다. 더 잡는 것은 안전 쪽이다(배타 점유).
            (c) 단, regress_tol 넘게 뒤인 투영이 confirm_updates 번 **연달아** 오면 그 사이 가장 앞선 곳으로 되돌린다
                — 잡은 엣지는 그대로 쥔다(요청이 늦어질 뿐). 놓은 엣지는 held 에서 빠졌으므로 되찾지 않는다.
          * progress_s(놓기) = 확인된 진행도 = 최근 confirm_updates 번 투영의 **가장 뒤**. 해제와 progress_idx 가 이것을 본다.
            (b) 창의 투영이 **모두** 넘은 곳까지만 오른다 — 한 번이라도 되당겨지면(fix) 놓지 않는다.
            (a) 한 번에 max_step(= max_speed × 호출 간격)까지만 오른다 — 확인 창보다 오래 이어진 튐도 물리 속도로만 번진다.
            내려가는 것은 바로 따른다(적대 검토 RES-1) — 오래 틀렸던 포즈가 바로잡히면 부푼 값이 남지 않는다.
        """
        if name not in self.robots:
            return float('inf')
        slot = self.robots[name]
        s, _, _, _, dist = project_to_polyline(x, y, slot.route.waypoints)
        if s < slot.lead_s - self.regress_tol:
            slot.behind.append(s)
            if len(slot.behind) >= self.confirm_updates:
                # 이 N 번이 곧 확인 창이라 되돌린 값은 progress_s(창의 가장 뒤) 보다 뒤일 수 없다
                slot.lead_s = max(slot.behind)
                slot.behind.clear()
        else:
            slot.behind.clear()
            slot.lead_s = max(slot.lead_s, s)
        slot.recent.append(s)
        if len(slot.recent) >= self.confirm_updates:
            # 적대 검토 RES-1: 예전엔 오를 때만 따라갔다(단조). 그러면 창보다 오래 간 틀린 포즈(① 전환 뒤 ②③ 동안 옛
            # 좌표계로 보고, 재배정 뒤 지난 목표를 보고)가 놓기 진행을 **영구히** 부풀려, 바로잡힌 뒤 START 로 잡은 엣지를
            # 한 틱 만에 놓았다 — BL 에 선 pinky1 이 pinky2 가 쥔 BL_JS 를 지나는 허가를 받았다. 이제 내려가는 것은 바로 따른다.
            # 이미 놓은 엣지는 held 에서 빠져 되찾지 않으므로, 낮추면 아직 쥔 엣지의 놓기가 늦어질 뿐이다(막는 쪽).
            slot.progress_s = min(min(slot.recent), slot.progress_s + self.max_step)
            slot.progress_idx = slot.idx_at(slot.progress_s)
        return dist

    def note_reported_idx(self, name, idx):
        """에이전트가 보고한 진행 waypoint(현재 경로의 LaneStatus.route_idx) — **잡기(요청)에만** 쓴다.

        🔴 통합 검토 RES-F2: 기다리던 로봇의 포즈가 한 틱(0.1 s) 0.6 m 앞으로 튀면 에이전트의 RouteFollower 는 그 자리로
           올라가 내려오지 않는다(단조). 코디네이터의 lead_s 는 (c) 로 2 s 뒤 되돌아간다. 허가가 오면 에이전트는 '허가 지점에
           닿았다' 며 서고(`clearance 대기 (진행 6, 허가 7)`), 코디네이터는 `기준점 − lead > reserve_ahead` 라 다음 엣지를
           영영 요청하지 않았다 → STALL_NO_REQUEST 영구. 둘이 다른 진행을 믿으면 교착이다.
        그래서 잡기는 에이전트가 믿는 진행도 따른다: 에이전트가 허가 지점 근처라 여겨 서면 코디네이터도 반드시 다음 엣지를
        요청한다(에이전트는 `허가 − 진행 ≤ 도달 반경` 에서 선다 · 요청은 `허가 − 진행 ≤ reserve_ahead`). 더 잡는 것은 안전
        쪽이다(배타 점유). **놓기는 그대로 확인된 포즈만** 본다 — 에이전트 진행은 튄 포즈로 부푼 값일 수 있다.
        가장 최근 보고를 그대로 쓴다(에이전트가 경로를 새로 받아 0 으로 돌아가면 같이 내려간다 — 쥔 엣지는 그대로다).
        """
        slot = self.robots.get(name)
        if slot is None:
            return
        idx = max(0, min(len(slot.cum) - 1, int(idx)))
        slot.reported_s = slot.cum[idx]

    def request_next_now(self, name):
        """거리와 무관하게 다음 엣지 요청을 지금 등록한다 (2026-09-29 교차로 규칙).

        로봇이 정지선을 보고 JUNCTION_STOP 을 보고하면 코디네이터가 부른다 — 정지선이 reserve_ahead 보다 멀어도
        요청이 서고, 선착순(요청 틱, 같은 틱이면 domain_id) 은 그대로다. 이미 요청했으면 그 틱을 지킨다.
        반환: 요청한 엣지 id ('' 이면 없음 — 도착·마지막 엣지 뒤).
        """
        slot = self.robots.get(name)
        if slot is None:
            return ''
        k = slot.next_edge_k
        if slot.finished or k >= slot.n_edges:
            return ''
        eid = slot.route.edge_ids[k]
        self.request_tick.setdefault((eid, name), self._tick)
        return eid

    def _lead(self, slot):
        """다음 엣지 요청에 쓰는 진행도 — 가장 앞선 추정. 시험·도구가 progress_s 를 직접 올려도 따라간다.
        에이전트가 믿는 진행(reported_s)도 넣는다(통합 검토 RES-F2 — 위 note_reported_idx)."""
        return max(slot.lead_s, slot.progress_s, slot.reported_s)

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
            if self._request_anchor(slot, k) - self._lead(slot) <= self.reserve_ahead:
                self.request_tick.setdefault((slot.route.edge_ids[k], slot.name), self._tick)
        order = sorted(self.robots.values(),
                       key=lambda s: (self._earliest_request(s), s.domain_id))
        for slot in order:
            self._try_acquire(slot)
        return {name: self.clear_until(name) for name in self.robots}

    def _request_anchor(self, slot, k):
        """다음 엣지 k 를 요청할 거리를 재는 기준점(호길이).

        🔴 D6-2 (관제 검수 REVIEW_20260925 §2, 리그 실측 `clearance 대기 (진행 2, 허가 4)` 120 s):
           예전엔 다음 엣지 **시작 노드**에서 쟀다. 그런데 로봇은 노드가 아니라 허가 지점에서 선다 —
           허가 지점 = 노드 − node_stop_margin 을 waypoint 로 **내림**, 로봇은 거기서 Nav2 허용만큼 앞.
           그래서 `margin + 내림 + 허용 > reserve_ahead` 이면(0.20 + 0.10 + 0.15 = 0.45 > 0.40) 요청이 영영 안 생겼다.
           이제 잡은 엣지가 있으면 **코디네이터가 명령한 정지 지점**(clear_until)에서 잰다 — 불변식이
           `Nav2 도달 허용 ≤ reserve_ahead` 로 바뀌어 도로망 모양·margin·내림과 무관해진다.
           노드보다 뒤로는 안 간다(min) — 예전보다 늦게 요청하는 일은 없다.
        """
        start = slot.edge_start_s(k)
        if not slot.held:
            return start
        return min(start, slot.cum[self.clear_until(slot.name)])

    def clear_until(self, name):
        if name not in self.robots:
            return 0
        slot = self.robots[name]
        if not slot.held:
            return 0
        k = slot.held[-1]
        if k == slot.n_edges - 1:
            return slot.route.goal_idx
        stop_s = max(slot.edge_start_s(k), slot.edge_end_s(k) - self.node_stop_margin)
        return slot.idx_at(stop_s)

    def goal_gap(self, name):
        """확인 창(최근 confirm_updates 번 투영)의 **과반**이 있는 곳에서 목표까지 남은 호길이(m) — 창이 덜 찼으면 놓기 진행
        (progress_s)으로 잰다(새 경로 직후엔 출발점이다). 모르는 로봇은 0(놓을 것도 없다).

        🔴 통합 검토 FLEET-R1: 도착(`mark_arrived`)은 쥔 엣지·노드를 한꺼번에 놓는 **해제**다. 그런데 에이전트의 도착 보고는
           제 RouteFollower 진행(튄 포즈로 단조로 부푼 값일 수 있다 — RES-F2)으로 낸다. 잡기가 그 진행을 따르자(note_reported_idx)
           목표까지 다 잡아 허가가 목표가 되고, 출발 노드에 선 채 goto 없이 '목표 도착' 을 보고한 로봇의 엣지·노드를 놓아
           다른 로봇이 그 자리로 들어왔다(54bb60b 에서도 달리던 로봇의 0.2 s 튐이면 났다). 그래서 코디네이터는 도착을 이것으로 받는다.
        왜 창의 가장 뒤(progress_s)가 아니라 **중앙값**인가: 코디네이터는 에이전트가 도착을 말한 지 창 하나(2 s)가 지난 뒤에 본다 —
        그 창은 로봇이 **서 있는 동안**의 포즈뿐이라 중앙값이 곧 선 자리다. 한두 틱 튐(< 창의 절반)에 속지 않고, 가장 뒤를 쓰면
        AMCL 식 잡음(검토 모의 hold1hz·iid)에서 목표에 선 로봇의 도착을 늦게 받거나 영영 못 받았다(모의 329 경우: 참 도착 9 잃음,
        늦음 p95 11 s·최대 60 s — 과반은 747 경우에서 잃은 참 도착 0, 늦음은 말한 뒤 창 하나 2 s 남짓).
        한계: 창보다 오래(과반) 틀린 포즈가 목표 근처면 속는다 — 놓기 진행의 RES-N2(5 s 오류)와 같은 한계다.
        """
        slot = self.robots.get(name)
        if slot is None:
            return 0.0
        if len(slot.recent) < self.confirm_updates:
            return max(0.0, slot.cum[slot.route.goal_idx] - slot.progress_s)
        majority = sorted(slot.recent)[(len(slot.recent) - 1) // 2]      # 아래쪽 중앙값 — 창의 과반이 이 자리 이상
        return max(0.0, slot.cum[slot.route.goal_idx] - majority)

    def mark_arrived(self, name):
        """도착. 엣지는 전부 놓고 목적지 노드만 계속 잡는다 (다른 로봇이 못 들어온다).
        코디네이터는 확인 창의 포즈가 목표 근처일 때만 부른다(goal_gap, 통합 검토 FLEET-R1)."""
        if name not in self.robots:
            return
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

    def _earliest_request(self, slot):
        k = slot.next_edge_k
        if slot.finished or k >= slot.n_edges:
            return float('inf')
        return self.request_tick.get((slot.route.edge_ids[k], slot.name), float('inf'))

    def _try_acquire(self, slot):
        while True:
            k = slot.next_edge_k
            if slot.finished or k >= slot.n_edges:
                return
            eid = slot.route.edge_ids[k]
            if (eid, slot.name) not in self.request_tick:
                return
            start_node = slot.route.node_ids[k]
            # L3 (관제 검수 REVIEW_20260925 §3.1): 마지막 엣지는 **끝 노드(목표)** 도 비어 있어야 잡는다 — 도착한 로봇은
            # 목표 노드를 계속 잡는데, 예전엔 둘째 로봇이 마지막 엣지를 잡아 같은 자리로 갔다. 잡으면 목표 노드도 잡는다.
            end_node = slot.route.node_ids[k + 1] if k == slot.n_edges - 1 else None
            if (self.edge_holder.get(eid) not in (None, slot.name)
                    or self.node_holder.get(start_node) not in (None, slot.name)
                    or (end_node is not None and self.node_holder.get(end_node) not in (None, slot.name))):
                slot.waiting_for = eid
                return
            self.edge_holder[eid] = slot.name
            self.node_holder[start_node] = slot.name
            if end_node is not None:
                self.node_holder[end_node] = slot.name
            slot.held.append(k)
            if (eid, slot.name) in self.request_tick:
                del self.request_tick[(eid, slot.name)]
            k2 = k + 1
            # 같은 틱 안의 연쇄 요청은 예전 규칙(노드 기준) 그대로 둔다 — D6-2 의 기준점 요청은 다음 틱의
            # step() 이 낸다. 여기까지 바꾸면 한 틱 일찍 다음 엣지를 기다리게 돼 기존 중재 시험(d10)의 순서가 바뀐다.
            if k2 >= slot.n_edges or slot.edge_start_s(k2) - self._lead(slot) > self.reserve_ahead:
                return
            self.request_tick.setdefault((slot.route.edge_ids[k2], slot.name), self._tick)

    def _release_passed(self, slot):
        # 놓기는 확인된 진행도(progress_s)로만 — lead_s 로 놓으면 H 가 그대로 돌아온다(REVIEW_20260926 H)
        later_starts = set()
        for k in slot.held:
            nid = slot.route.node_ids[k]
            if slot.progress_s - slot.edge_start_s(k) >= self.release_behind:
                if self.node_holder.get(nid) == slot.name and nid not in later_starts:
                    del self.node_holder[nid]
            else:
                later_starts.add(nid)
        while len(slot.held) > 1:
            k = slot.held[0]
            if slot.progress_s - slot.edge_end_s(k) < self.release_behind:
                break
            slot.held.pop(0)
            eid = slot.route.edge_ids[k]
            if self.edge_holder.get(eid) == slot.name:
                del self.edge_holder[eid]

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
        end_node = slot.route.node_ids[k + 1] if k == slot.n_edges - 1 else None
        return (self.edge_holder.get(slot.waiting_for) == holder
                or self.node_holder.get(start_node) == holder
                or (end_node is not None and self.node_holder.get(end_node) == holder))

    def status(self, name):
        if name not in self.robots:
            return {}
        slot = self.robots[name]
        blocked_by = ''
        if slot.waiting_for:
            k = slot.next_edge_k
            end_node = slot.route.node_ids[k + 1] if k == slot.n_edges - 1 else None
            blocked_by = (self.edge_holder.get(slot.waiting_for)
                          or self.node_holder.get(slot.route.node_ids[k])
                          or (self.node_holder.get(end_node) if end_node else None) or '')
        k = slot.next_edge_k
        has_next = not slot.finished and k < slot.n_edges
        return {
            'progress_idx': slot.progress_idx,
            'clear_until': self.clear_until(name),
            'held_edges': [slot.route.edge_ids[k2] for k2 in slot.held],
            'waiting_for': slot.waiting_for,
            'blocked_by': blocked_by,
            'finished': slot.finished,
            # D6-2 교착 신호의 재료: 다음 엣지가 있는데 요청이 없고(requesting False) 막는 이도 없으면 중재가 아니다
            'next_edge': slot.route.edge_ids[k] if has_next else '',
            'requesting': bool(has_next and (slot.route.edge_ids[k], slot.name) in self.request_tick),
        }
