"""구간 예약 단위 테스트 — 시나리오 1·2 를 가짜 로봇으로 돌린다 (ROS 없이)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.reservation import Reservation  # noqa: E402
from pinky_lane_station.road_graph import RoadGraph  # noqa: E402


def course():
    r"""시나리오용 축소 도로망. 길이는 전부 1 m.

        MC                RE
         \e4             /e3
          RM ---e2--- JS ---e1--- BL
    """
    return RoadGraph.from_dict({
        'nodes': [
            {'id': 'BL', 'x': 0, 'y': 0, 'type': 'endpoint'},
            {'id': 'JS', 'x': 1, 'y': 0, 'type': 'junction'},
            {'id': 'RM', 'x': 2, 'y': 0, 'type': 'junction'},
            {'id': 'RE', 'x': 1, 'y': 1, 'type': 'endpoint'},
            {'id': 'MC', 'x': 3, 'y': 0, 'type': 'endpoint'},
        ],
        'edges': [
            {'id': 'e1', 'from': 'BL', 'to': 'JS', 'waypoints': [[0, 0], [1, 0]]},
            {'id': 'e2', 'from': 'JS', 'to': 'RM', 'waypoints': [[1, 0], [2, 0]]},
            {'id': 'e3', 'from': 'JS', 'to': 'RE', 'waypoints': [[1, 0], [1, 1]]},
            {'id': 'e4', 'from': 'RM', 'to': 'MC', 'waypoints': [[2, 0], [3, 0]]},
        ],
    })


class Sim:
    """clear_until 까지만 전진하는 가짜 로봇들. 한 틱에 step 만큼 움직인다."""

    def __init__(self, res, step=0.1):
        self.res = res
        self.step = step
        self.pos = {}       # name -> 경로 호길이
        self.log = []       # (tick, name, s, clear_until)

    def add(self, name, domain_id, route, start_delay=0):
        self.res.register(name, domain_id, route)
        self.pos[name] = 0.0
        self.delay = getattr(self, 'delay', {})
        self.delay[name] = start_delay

    def run(self, ticks):
        for t in range(1, ticks + 1):
            clear = self.res.step(t)
            for name, slot in self.res.robots.items():
                if slot.finished:
                    continue
                cum = slot.cum
                limit_s = cum[clear[name]]
                if t > self.delay[name]:
                    self.pos[name] = min(self.pos[name] + self.step, limit_s)
                x, y = point_at(slot.route, self.pos[name])
                self.res.update_pose(name, x, y)
                self.log.append((t, name, round(self.pos[name], 3), clear[name]))
                if self.pos[name] >= slot.route.length - 1e-9:
                    self.res.mark_arrived(name)
        return self


def point_at(route, s):
    cum = route.cumulative()
    for i in range(len(cum) - 1):
        if cum[i] <= s <= cum[i + 1]:
            t = 0 if cum[i + 1] == cum[i] else (s - cum[i]) / (cum[i + 1] - cum[i])
            (x0, y0), (x1, y1) = route.waypoints[i], route.waypoints[i + 1]
            return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
    return route.waypoints[-1]


def positions_over_time(sim, name):
    return [(t, s) for t, n, s, _ in sim.log if n == name]


# ------------------------------------------------------------ 기본 동작

def test_single_robot_drives_to_goal():
    g = course()
    sim = Sim(Reservation(g))
    sim.add('pinky1', 10, g.shortest_route('BL', 'MC'))
    sim.run(60)
    assert sim.res.robots['pinky1'].finished
    assert sim.pos['pinky1'] == pytest.approx(3.0)
    # 도착 후 목적지 노드는 계속 잡고, 엣지는 전부 놓는다
    assert sim.res.node_holder == {'MC': 'pinky1'}
    assert sim.res.edge_holder == {}


def test_clear_until_stops_short_of_unreserved_junction():
    g = course()
    res = Reservation(g, node_stop_margin=0.2)
    a = res.register('pinky1', 10, g.shortest_route('BL', 'MC'))
    # pinky2 가 e2 의 시작 노드 JS 를 잡고 있다고 가정
    res.register('pinky2', 11, g.shortest_route('JS', 'RE'))
    clear = res.step(1)
    assert res.edge_holder.get('e1') == 'pinky1'
    # e2 를 못 잡았으므로 JS 0.2 m 앞에서 멈춘다
    stop_s = a.cum[clear['pinky1']]
    assert stop_s == pytest.approx(0.8, abs=0.06)


def test_head_on_is_impossible():
    g = course()
    sim = Sim(Reservation(g))
    sim.add('pinky1', 10, g.shortest_route('BL', 'RM'))   # e1, e2 정방향
    sim.add('pinky2', 11, g.shortest_route('RM', 'BL'))   # e2, e1 역방향
    sim.run(80)
    # 두 로봇 위치가 겹친 적이 없어야 한다 (같은 x 좌표에서 0.15 m 이내)
    pos1 = dict(positions_over_time(sim, 'pinky1'))
    pos2 = dict(positions_over_time(sim, 'pinky2'))
    for t in pos1:
        if t in pos2:
            x1 = pos1[t]                 # BL 기준 호길이 = x
            x2 = 2.0 - pos2[t]           # RM 기준 호길이 → x
            assert abs(x1 - x2) > 0.15 or pos1[t] == 0 or pos2[t] == 0


# ------------------------------------------------------------ 시나리오 1

def test_scenario1_sequential_start_from_same_node():
    """왼쪽 아래에서 2대. 하나는 MC, 하나는 RE. e1 을 같은 방향으로 공유한다."""
    g = course()
    ra, rb = g.shortest_route('BL', 'MC'), g.shortest_route('BL', 'RE')
    assert dict(RoadGraph.shared_edges(ra, rb)) == {'e1': True}
    sim = Sim(Reservation(g))
    sim.add('pinky1', 10, ra)
    sim.add('pinky2', 11, rb)
    sim.run(120)
    assert all(r.finished for r in sim.res.robots.values())
    # pinky2 는 pinky1 이 e1 을 놓을 때까지 출발점에 있었다
    p2 = positions_over_time(sim, 'pinky2')
    first_move = next(t for t, s in p2 if s > 0)
    p1_at = dict(positions_over_time(sim, 'pinky1'))[first_move]
    assert p1_at >= 1.0 + sim.res.release_behind - sim.step - 1e-9


def test_same_start_node_goes_to_whoever_registered_first():
    """같은 출발점이면 먼저 등록한(= 앞에 서 있는) 로봇이 출발 노드를 잡고 먼저 간다."""
    g = course()
    res = Reservation(g)
    res.register('pinky2', 11, g.shortest_route('BL', 'RE'))
    res.register('pinky1', 10, g.shortest_route('BL', 'MC'))
    res.step(1)
    assert res.edge_holder['e1'] == 'pinky2'
    assert res.status('pinky1')['waiting_for'] == 'e1'
    assert res.status('pinky1')['blocked_by'] == 'pinky2'


def test_same_tick_request_tie_breaks_by_domain_id():
    """서로 다른 곳에서 같은 틱에 같은 엣지를 요청하면 domain_id 낮은 쪽."""
    g = course()
    res = Reservation(g, reserve_ahead=5.0)
    res.register('pinky2', 11, g.shortest_route('RM', 'RE'))   # e2r, e3
    res.register('pinky1', 10, g.shortest_route('BL', 'RE'))   # e1, e3
    res.step(1)
    assert res.edge_holder['e3'] == 'pinky1'
    assert res.node_holder['JS'] == 'pinky1'
    assert res.status('pinky2')['waiting_for'] == 'e3'


# ------------------------------------------------------------ 시나리오 2

def test_scenario2_first_to_arrive_passes_junction_first():
    """MC 와 RE 에서 출발해 둘 다 BL 로. 교차로 JS(e2 끝 / e3 끝)에서 만난다.

    RE→BL 은 1 m 만 가면 JS 에 닿고, MC→BL 은 2 m 를 가야 한다. RE 쪽이 먼저 도착해
    먼저 통과해야 하고, MC 쪽은 JS 앞에서 멈췄다가 지나간다.
    """
    g = course()
    sim = Sim(Reservation(g))
    sim.add('pinky1', 10, g.shortest_route('MC', 'BL'))   # e4r, e2r, e1r  (domain 낮음)
    sim.add('pinky2', 11, g.shortest_route('RE', 'BL'))   # e3r, e1r
    sim.run(120)
    assert all(r.finished for r in sim.res.robots.values())
    p1 = positions_over_time(sim, 'pinky1')
    p2 = positions_over_time(sim, 'pinky2')
    # pinky2 가 e1 에 들어간 틱 (호길이 > 1.0) 이 pinky1 (호길이 > 2.0) 보다 빠르다
    t2 = next(t for t, s in p2 if s > 1.0 + 1e-9)
    t1 = next(t for t, s in p1 if s > 2.0 + 1e-9)
    assert t2 < t1
    # pinky1 은 JS 앞에서 멈춘 적이 있다 (호길이 1.8 근처에서 여러 틱 정체)
    stalled = [s for t, s in p1 if abs(s - 1.8) < 0.06]
    assert len(stalled) >= 3


def test_scenario2_no_two_robots_inside_junction():
    g = course()
    res = Reservation(g)
    sim = Sim(res)
    sim.add('pinky1', 10, g.shortest_route('MC', 'BL'))
    sim.add('pinky2', 11, g.shortest_route('RE', 'BL'))
    for t in range(1, 120):
        clear = res.step(t)
        for name, slot in res.robots.items():
            if slot.finished:
                continue
            sim.pos[name] = min(sim.pos[name] + sim.step, slot.cum[clear[name]])
            x, y = point_at(slot.route, sim.pos[name])
            res.update_pose(name, x, y)
            if sim.pos[name] >= slot.route.length - 1e-9:
                res.mark_arrived(name)
        # 분기 JS (1,0) 반경 0.15 안에 둘이 동시에 있으면 안 된다
        inside = [n for n, s in res.robots.items()
                  if not s.finished and abs(point_at(s.route, sim.pos[n])[0] - 1.0) < 0.15
                  and abs(point_at(s.route, sim.pos[n])[1]) < 0.15]
        assert len(inside) <= 1, (t, inside)


# ------------------------------------------------------------ 교착 감지

def test_mutual_wait_detected():
    """서로의 첫 엣지가 상대의 둘째 엣지. 둘 다 분기 앞에서 멈추고 상대를 기다린다."""
    g = course()
    sim = Sim(Reservation(g))
    sim.add('pinky1', 10, g.shortest_route('BL', 'RM'))    # e1 → e2
    sim.add('pinky2', 11, g.shortest_route('RM', 'BL'))    # e2 → e1
    sim.run(40)
    assert sim.res.edge_holder == {'e1': 'pinky1', 'e2': 'pinky2'}
    assert sim.res.mutual_wait() == [('pinky1', 'pinky2')]
    assert not any(r.finished for r in sim.res.robots.values())


def test_remove_releases_everything():
    g = course()
    res = Reservation(g)
    res.register('pinky1', 10, g.shortest_route('BL', 'MC'))
    res.step(1)
    res.remove('pinky1')
    assert res.edge_holder == {} and res.node_holder == {} and res.request_tick == {}
