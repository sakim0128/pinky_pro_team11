# -*- coding: utf-8 -*-
"""hybrid_agent_node 의 경로 진행도: E2E-1 · RES-F1 · F2.

rkd1rjs2/robot_mini_project_pinky 의 relay_station/tests/test_review_0926_agent2.py 를 옮겼다. import 와,
월요일 시나리오 경로를 만드는 부분(그쪽 좌표 프로파일 → 이 저장소 pinky_lane_station 의 road_graph.yaml + 같은 RoadGraph)만 바꿨다.

**E2E-1 (P1) · RES-F1** — 월요일 절차(`docs/REPLY_20260924_RELAY_SESSION.md` §14-4)대로 하면 한 로봇이 출발점에 영영 섰다.
① 프로파일 전환에서 새 Route 가 먼저 닿고, 로봇은 ③ 초기 위치 전까지 **옛 좌표계** 위치를 보고한다. 예전엔 START 전에도
그 위치를 단조 추종기에 넣어 진행도가 새 경로 멀리(9~22)까지 올라가 내려오지 않았다. ③ 뒤 출발점에서 START 하면
- 첫 허가(7·9)가 진행도보다 뒤라 `clearance 대기 (진행 N, 허가 M)` N>M 로 영영 섰고(코디네이터는 다음 엣지를 요청하지
  않는다 — STALL_NO_REQUEST), 검토 폐루프(실제 코디네이터 + 실제 RouteChain)에서 옛 위치 24개 중 23개가 그랬다;
- 옛 위치가 목표 근처로 투영되면 **움직이지 않고 DRIVE_ARRIVED** 를 보고했다(코디네이터가 그 로봇의 엣지를 놓는다).
고침: START 전 위치는 진행도에 넣지 않는다. 시작 전→시작 때 진행도를 경로 처음부터 그 순간의 위치로 새로 잡는다.
경로 중간에 선 로봇(코디네이터 재시작 뒤 재배정)은 START 때 제자리를 찾는다 — 그 복구는 그대로다.

**F2** — 도메인 브리지만 재기동하면 TRANSIENT_LOCAL Route 가 **같은 메시지 그대로** 다시 닿는다. 예전엔 그것도 새 경로로
받아 `started=False` — RUNNING 중에 로봇이 말없이 IDLE 로 섰다. 같은 메시지(번호·waypoint·goal·**발행 시각**)면 무시한다.
발행 시각까지 보는 이유: 코디네이터 재시작은 번호를 1 부터 다시 매겨 같은 번호·같은 waypoint 를 **새 발행 시각**으로 준다
(검토 e2e 실측: pinky2 seq=1 → 재시작 → seq=1). 번호·waypoint 만 보고 무시하면 옛 코디네이터의 허가·START 를 쥔 채 남아
ASSIGNED 에서 로봇 재개 한 번에 새 코디네이터가 예약하지 않은 구간으로 달린다.

앞부분은 ROS 없이 진짜 `RouteChain` 을, 뒷부분은 진짜 `PinkyAgent` 메서드(`_on_route`·`_on_lane_command`·10 Hz
`_publish_state`)를 잰다(`test_hybrid_agent_review._agent` 의 가짜 객체에 묶는다).
"""
import os
import sys
import types

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(REPO, "pinky_lane_station"))     # 팀11 RoadGraph (ROS 비의존)

from pinky_fleet_agent import route_chain as rc  # noqa: E402
from pinky_fleet_agent.route_chain import RouteChain  # noqa: E402

from test_hybrid_agent_review import _HAVE  # noqa: E402

needs_ros = pytest.mark.skipif(not _HAVE, reason="ROS 환경이 없다 — source /opt/ros/jazzy/setup.bash + install/setup.bash")

SEQ = 7
# 0.1 m 간격 직선 3 m — waypoint 0..30 (goal 30). 진행 idx = x × 10.
WP31 = [(0.1 * i, 0.0) for i in range(31)]
# 0.66 m 짧은 경로(map4_s2 pinky2 RE→TC 꼴) — goal 7
WP8 = [(0.1 * i, 0.0) for i in range(7)] + [(0.66, 0.0)]
STAMP = (1790000000, 5)


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _gotos(acts):
    return [a[1] for a in acts if a[0] == "goto"]


def _feed(c, clk, x, y, n=20):
    """10 Hz 위치 n 틱 (에이전트 `_publish_state` 는 위치 추정 중일 때 틱마다 on_pose 를 부른다)."""
    acts = []
    for _ in range(n):
        clk.t += 0.1
        acts += c.on_pose(x, y)
    return acts


def _chain(wp=WP31, goal=30, stamp=None):
    clk = Clock()
    c = RouteChain(clock=clk)
    if stamp is None:
        c.on_route(SEQ, wp, goal)                         # 예전 호출 모양 그대로 (발행 시각 모름)
    else:
        c.on_route(SEQ, wp, goal, stamp=stamp)
    return c, clk


# ============================================================ E2E-1 · RES-F1 — 진짜 RouteChain

def test_E2E1_START_전_위치는_진행도를_밀지_않는다():
    c, clk = _chain()
    _feed(c, clk, 2.5, 0.0)                               # ① 뒤 ③ 전: 옛 좌표계 위치가 새 경로 2.5 m 에 투영된다
    assert c.progress_idx == 0 and c.reached_idx == 0
    assert c.status()["route_idx"] == 0
    assert c.drive_state() == rc.DRIVE_IDLE


def test_E2E1_옛_좌표_뒤_출발점에서_START_하면_첫_허가로_떠난다__허가_대기에_영영_서지_않는다():
    """검토 재현: 예전엔 'clearance 대기 (진행 25, 허가 7)' — 코디네이터는 다음 엣지를 요청하지 않아 영영 섰다."""
    c, clk = _chain()
    _feed(c, clk, 2.5, 0.0)                               # ①
    _feed(c, clk, 0.0, 0.0)                               # ③ 출발점
    assert _gotos(c.on_lane_command(rc.CMD_START, SEQ, 0)) == []
    assert c.reached_idx == 0
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 7)) == [7]
    assert c.drive_state() == rc.DRIVE_CRUISE


def test_E2E1_옛_좌표가_목표에_투영돼도_움직이지_않고_도착을_보고하지_않는다():
    """검토 재현(map4_s2 pinky2 RE→TC): 예전엔 첫 허가에 곧바로 DRIVE_ARRIVED — 코디네이터가 엣지를 놓았다."""
    c, clk = _chain(WP8, 7)
    _feed(c, clk, 0.66, 0.0)                              # ① 옛 위치가 목표에 투영된다
    _feed(c, clk, 0.0, 0.0)                               # ③
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 7)) == [7]
    assert not c.arrived and c.drive_state() == rc.DRIVE_CRUISE


def test_E2E1_경로_중간에_선_로봇은_START_때_제자리를_찾는다__코디네이터_재시작_뒤_재배정():
    """복구는 그대로: e2e 실측 '재시작 → 재배정 → START → 둘 다 중간 위치에서 도착'."""
    c, clk = _chain()
    _feed(c, clk, 1.5, 0.0)
    assert c.progress_idx == 0                            # START 전엔 넣지 않는다
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert c.reached_idx == 15
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 25)) == [25]
    assert not c.arrived


def test_E2E1_START_때_투영은_탐색_창에_걸리지_않는다__뒤쪽_waypoint_로_되돌아가는_goto_가_없다():
    """한 번만 투영하면 창(±2 m) 끝 20 에 걸린다 — 그때 허가 25 가 오면 2.8 m 에 선 로봇을 25 로 **뒤로** 보낸다."""
    c, clk = _chain()
    _feed(c, clk, 2.8, 0.0)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert c.progress_idx == 28
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 25)) == []


@pytest.mark.parametrize("length_m, x, clear_to", [
    (7.0, 5.5, 50),      # 검토 제안 그대로: 2회로 자르면 진행 41 → ('goto', 50) 뒤로 (3회면 55 — 이 경우는 통과한다)
    (20.0, 19.0, 188),   # 창 ±2 m 로 11번 되풀이해야 190 — 고정 상한 2~9회는 전부 187 이하에 멈춰 188 로 뒤로 보낸다
])
def test_E2E1_START_때_투영은_긴_경로에서도_끝까지_되풀이한다__고정_횟수로_자르면_뒤로_보낸다(length_m, x, clear_to):
    """통합 검토 AG2-T1: 위의 3 m 경로는 2회면 수렴해 되풀이 횟수를 고정 상한(2·3회)으로 잘라도 잡지 못했다.

    지금 team11_map4 의 가장 긴 최단 경로는 3.08 m 라 실제 영향은 없지만, 창이 한 번에 ±2 m 씩만 따라가므로
    경로가 길어지면(맵 교체·경유 추가) 상한에 걸린 진행도가 허가보다 뒤에 서서 **뒤쪽 waypoint 로 goto** 가 나간다.
    진행도가 더 늘지 않을 때까지(경로 waypoint 수만큼까지) 되풀이해야 한다.
    """
    n = int(round(length_m * 10))
    wp = [(0.1 * i, 0.0) for i in range(n + 1)]
    c, clk = _chain(wp, n)
    _feed(c, clk, x, 0.0)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert c.progress_idx == int(round(x * 10)) and c.reached_idx == int(round(x * 10))
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, clear_to)) == []
    assert not c.arrived


def test_E2E1_오래된_위치로는_START_때_진행도를_잡지_않는다():
    """에이전트는 위치 추정 중일 때만 on_pose 를 부른다 — 1 s 넘게 끊긴 뒤의 옛 위치(① 의 옛 좌표일 수 있다)는 쓰지 않는다."""
    c, clk = _chain()
    _feed(c, clk, 2.5, 0.0)
    clk.t += rc.POSE_FRESH_S + 4.0
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert c.progress_idx == 0
    _feed(c, clk, 0.3, 0.0, n=1)                          # 시작 뒤 첫 위치부터 평소처럼 따라간다
    assert c.progress_idx == 3


def test_E2E1_단일_목표로_딴_데_다녀온_뒤_START_는_옛_진행도와_Nav2_도달_확정을_버린다():
    c, clk = _chain()
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 20)) == [20]
    _feed(c, clk, 2.0, 0.0)
    c.on_goal_result(20, "succeeded")
    assert c.reached_idx == 20
    c.abandon()                                           # FleetCommand GOTO — 단일 목표가 레인 임무를 대신했다
    _feed(c, clk, 0.5, 0.0)                               # 그 목표로 0.5 m 까지 되돌아왔다
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert c.reached_idx == 5
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 15)) == [15]    # 예전엔 '진행 20, 허가 15' 로 섰다


def test_E2E1_START_뒤에는_위치를_틱마다_따라간다():
    c, clk = _chain()
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    _feed(c, clk, 1.0, 0.0, n=1)
    assert c.progress_idx == 10


# ============================================================ F2 — 진짜 RouteChain

def _running(stamp=STAMP):
    c, clk = _chain(stamp=stamp)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 10)) == [10]
    return c, clk


def test_F2_같은_메시지의_재전달은_무시한다__달리던_임무가_이어진다():
    c, _ = _running()
    assert c.on_route(SEQ, list(WP31), 30, stamp=STAMP) == []            # 브리지 재기동 — 같은 메시지
    assert c.started and c.clear_until_idx == 10 and c.active_target == 10
    assert c.drive_state() == rc.DRIVE_CRUISE


def test_F2_같은_번호·같은_경로라도_발행_시각이_다르면_새_경로다__코디네이터_재시작():
    c, _ = _running()
    acts = c.on_route(SEQ, list(WP31), 30, stamp=(STAMP[0] + 60, 0))
    assert ("cancel",) in acts
    assert not c.started and c.clear_until_idx == 0 and c.drive_state() == rc.DRIVE_IDLE
    assert _gotos(c.on_lane_command(rc.CMD_RESUME, SEQ, 0)) == []        # ASSIGNED 의 로봇 재개 — 옛 허가로 안 달린다


def test_F2_같은_번호·같은_시각이라도_waypoint_가_다르면_새_경로다():
    c, _ = _running()
    moved = [(x, 0.05) for x, _ in WP31]
    c.on_route(SEQ, moved, 30, stamp=STAMP)
    assert not c.started and c.follower.points[0] == (0.0, 0.05)


def test_F2_같은_번호·같은_시각이라도_goal_이_다르면_새_경로다():
    c, _ = _running()
    c.on_route(SEQ, list(WP31), 20, stamp=STAMP)
    assert not c.started and c.goal_idx == 20


def test_F2_같은_시각·같은_경로라도_번호가_다르면_새_경로다():
    c, _ = _running()
    c.on_route(SEQ + 1, list(WP31), 30, stamp=STAMP)
    assert not c.started and c.route_seq == SEQ + 1


def test_F2_발행_시각을_모르면_같은_메시지로_보지_않는다():
    c, _ = _running(stamp=None)
    c.on_route(SEQ, list(WP31), 30)
    assert not c.started and c.clear_until_idx == 0


# ============================================================ 진짜 PinkyAgent 메서드

if _HAVE:
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import Point
    from pinky_lane_msgs.msg import LaneStatus, Route
    from pinky_lane_station.road_graph import RoadGraph
    from pinky_fleet_agent import hybrid_agent_node as agent_node
    from test_hybrid_agent_review import _agent, _hb, _lane, _ticking  # noqa: E402

ROAD_GRAPH = os.path.join(REPO, "pinky_lane_station", "config", "road_graph.yaml")


def _route_msg(waypoints, goal_idx, seq=SEQ, stamp=STAMP):
    m = Route()
    m.robot_name = "pinky1"
    m.route_seq = seq
    m.header.stamp = TimeMsg(sec=stamp[0], nanosec=stamp[1])
    m.waypoints = [Point(x=float(x), y=float(y), z=0.0) for x, y in waypoints]
    m.goal_idx = int(goal_idx)
    return m


def _wired_agent():
    a, clk = _agent()
    tick = _ticking(a)
    a._on_route = types.MethodType(agent_node.PinkyAgent._on_route, a)
    return a, clk, tick


def _ticks(a, clk, tick, x, y, n=20):
    """진짜 10 Hz `_publish_state` — 위치 추정 중(TF 신선)이고 하트비트가 산다."""
    for _ in range(n):
        clk.t += 0.1
        a._x, a._y = float(x), float(y)
        a._last_pose_time = clk.t
        a._odom_at = clk.t
        a._on_command(_hb())
        tick()


def _last_lane_status(a):
    return a._lane_status_pub.publish.call_args[0][0]


def _mission_route(start, goal):
    g = RoadGraph.load(ROAD_GRAPH)
    return g.shortest_route(start, goal, step=0.10)


# (시작 노드, 목표 노드, ① 때의 옛 좌표계 위치, 코디네이터의 첫 허가) — 원 저장소의 검토 폐루프·e2e 에서 멈춘 조합.
#   경로는 pinky_lane_station/config/lane_mission.yaml 의 시나리오 1(BL→RE)·2(MC→BL, RE→TC).
#   옛 좌표 START_A (0.30,0.30) · START_B (0.30,0.95) · 시나리오 2 pinky2 의 옛 위치 (-1.0,-0.2).
MONDAY = [
    ("BL", "RE", (0.30, 0.95), 7),      # 시나리오 1 pinky2: 예전 진행 16 → '진행 16, 허가 7' 로 영영
    ("MC", "BL", (0.30, 0.30), 9),      # 시나리오 2 pinky1: e2e 실측 '진행 14, 허가 9'
    ("RE", "TC", (-1.0, -0.2), 7),      # 시나리오 2 pinky2: 예전 진행 7 = 목표 → 움직이지 않고 도착
]


@needs_ros
@pytest.mark.parametrize("start,goal,legacy,first_clear", MONDAY)
def test_E2E1_에이전트_월요일_순서__옛_좌표_뒤_출발점에서_START_면_첫_허가로_떠나고_도착이라_하지_않는다(
        start, goal, legacy, first_clear):
    route = _mission_route(start, goal)
    a, clk, tick = _wired_agent()
    a._on_route(_route_msg(route.waypoints, route.goal_idx))          # ① 새 Route
    _ticks(a, clk, tick, *legacy)                                      # ③ 전: 옛 좌표계 위치
    assert _last_lane_status(a).route_idx == 0
    _ticks(a, clk, tick, *route.waypoints[0])                          # ③ 출발점
    a._on_lane_command(_lane(rc.CMD_START))
    _ticks(a, clk, tick, *route.waypoints[0], n=1)
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, clear=first_clear))
    calls = a._send_goal_to.call_args_list
    assert len(calls) == 1, "첫 허가를 받고도 떠나지 않았다 (진행도가 옛 좌표로 밀려 있다)"
    x, y, _yaw, tag = calls[0][0]
    assert tag == ("chain", first_clear) and (x, y) == tuple(route.waypoints[first_clear])
    _ticks(a, clk, tick, *route.waypoints[0], n=1)
    st = _last_lane_status(a)
    assert st.drive_state == LaneStatus.DRIVE_CRUISE and st.route_idx <= 1


@needs_ros
def test_F2_에이전트_브리지_재기동이_같은_Route_를_다시_줘도_달리던_로봇이_IDLE_로_서지_않는다():
    a, clk, tick = _wired_agent()
    msg = _route_msg(WP31, 30)
    a._on_route(msg)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, clear=10))
    assert a._send_goal_to.call_count == 1
    a._cancel_goal.reset_mock()
    a._on_route(_route_msg(WP31, 30))                                  # 같은 메시지(발행 시각까지) 재전달
    a._cancel_goal.assert_not_called()
    assert a._chain.started and a._chain.active_target == 10
    _ticks(a, clk, tick, 0.2, 0.0, n=1)
    assert _last_lane_status(a).drive_state == LaneStatus.DRIVE_CRUISE


@needs_ros
def test_F2_에이전트_코디네이터_재시작의_같은_번호_Route_는_새_경로__로봇_재개로_옛_허가를_달리지_않는다():
    """e2e 실측: 게이트웨이 재시작 뒤 pinky2 는 다시 seq=1 · 같은 waypoint 를 받는다(발행 시각만 다르다)."""
    a, clk, tick = _wired_agent()
    a._on_route(_route_msg(WP31, 30))
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, clear=10))
    assert a._send_goal_to.call_count == 1
    a._on_route(_route_msg(WP31, 30, stamp=(STAMP[0] + 60, 0)))       # 새 코디네이터의 첫 배정
    assert not a._chain.started and a._chain.clear_until_idx == 0
    a._on_lane_command(_lane(rc.CMD_RESUME))                           # ASSIGNED 의 로봇 재개(resume_robot)
    _ticks(a, clk, tick, 0.2, 0.0, n=3)
    assert a._send_goal_to.call_count == 1, "새 코디네이터가 예약하지 않은 구간으로 옛 허가를 달렸다"
    assert _last_lane_status(a).drive_state == LaneStatus.DRIVE_IDLE


@needs_ros
def test_F2_에이전트_발행_시각이_비어_있으면_같은_메시지로_보지_않는다():
    a, clk, tick = _wired_agent()
    a._on_route(_route_msg(WP31, 30, stamp=(0, 0)))
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, clear=10))
    a._on_route(_route_msg(WP31, 30, stamp=(0, 0)))
    assert not a._chain.started and a._chain.clear_until_idx == 0


@needs_ros
def test_F2_에이전트_발행_시각은_나노초까지_같아야_같은_메시지다():
    a, clk, tick = _wired_agent()
    a._on_route(_route_msg(WP31, 30))
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_route(_route_msg(WP31, 30, stamp=(STAMP[0], STAMP[1] + 1)))
    assert not a._chain.started
