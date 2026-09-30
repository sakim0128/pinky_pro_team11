# -*- coding: utf-8 -*-
"""관제 검수 REVIEW_20260925 §3.1 L3~L7 — 주행이 끝까지 가게 (코디네이터·예약·체인, ROS 노드 없이).

L3 도착한 로봇의 목표 노드 · L4 Nav2 거부를 실패로 세지 않음 · L5 STOP 사유는 START 응답이 아님 ·
L6 재배정은 START 재무장 · L7 한 번도 시작 안 한 플릿의 ESTOP→재개.
"""
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pinky_fleet_msgs.msg import FleetCommand  # noqa: E402
from pinky_lane_msgs.msg import LaneCommand, LaneStatus  # noqa: E402
from relay_station.fleet.reservation import Reservation  # noqa: E402
from relay_station.fleet.road_graph import RoadGraph  # noqa: E402
from pinky_fleet_agent import route_chain as rc  # noqa: E402
from pinky_fleet_agent.route_chain import RouteChain  # noqa: E402
from test_d7_robot_stop import GRAPH, _coord, _lane_status, _reset, _sent  # noqa: E402


# ---- L3 · 도착한 로봇의 목표 노드 --------------------------------------------------------------

def _res_two_to_goal():
    g = RoadGraph.load(GRAPH)
    r = Reservation(g, reserve_ahead=0.40, release_behind=0.25, node_stop_margin=0.20)
    ra = g.shortest_route("START_A", "GOAL_C", step=0.10)
    rb = g.shortest_route("START_B", "GOAL_C", step=0.10)
    r.register("pinky1", 10, ra)
    r.register("pinky2", 11, rb)
    return r, ra, rb


def test_L3_마지막_엣지를_잡으면_목표_노드도_잡는다():
    r, ra, _ = _res_two_to_goal()
    s = r.robots["pinky1"]
    s.progress_s = s.edge_start_s(ra.edge_ids.index(ra.edge_ids[-1])) - 0.1     # 마지막 엣지 앞
    for _ in range(3):
        r.step()
    assert r.edge_holder.get(ra.edge_ids[-1]) == "pinky1"
    assert r.node_holder.get("GOAL_C") == "pinky1"


def test_L3_도착한_로봇이_잡은_목표로는_둘째_로봇이_마지막_엣지를_못_잡는다():
    r, ra, rb = _res_two_to_goal()
    r.mark_arrived("pinky1")                                   # pinky1 도착 — 목표 노드를 계속 잡는다
    s2 = r.robots["pinky2"]
    s2.held[:] = list(range(len(rb.edge_ids) - 1))            # 마지막 엣지 직전까지 잡은 셈
    for k in s2.held:
        r.edge_holder[rb.edge_ids[k]] = "pinky2"
    s2.progress_s = s2.edge_start_s(len(rb.edge_ids) - 1) - 0.05
    r.step()
    assert r.edge_holder.get(rb.edge_ids[-1]) != "pinky2", "도착한 로봇이 선 목표로 둘째 로봇이 들어간다"
    assert s2.waiting_for == rb.edge_ids[-1] and r.status("pinky2")["blocked_by"] == "pinky1"


def test_L3_같은_목표_노드_배정은_거부한다():
    c = _coord(state="IDLE")
    c.robots["pinky1"].route = c.robots["pinky2"].route = None
    c.reservation = Reservation(c.graph)
    c.route_pubs = {}
    assert c.assign_route("pinky1", "START_A", "GOAL_C") is True
    assert c.assign_conflict("pinky2", "GOAL_C") == "pinky1"
    assert c.assign_route("pinky2", "START_B", "GOAL_C") is False
    assert c.robots["pinky2"].route is None
    assert c.assign_route("pinky1", "START_A", "GOAL_C") is True     # 자기 자신 재배정은 된다


# ---- L4 · Nav2 거부는 실패가 아니다, START 는 포기를 푼다 ------------------------------------------

WP5 = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]


def test_L4_START_는_실패로_포기한_목표를_다시_시도하게_한다():
    clk = [0.0]
    c = RouteChain(clock=lambda: clk[0])
    c.on_route(7, WP5, 4)
    c.on_lane_command(rc.CMD_START, 7, 0)
    c.on_lane_command(rc.CMD_CLEARANCE, 7, 3)
    for _ in range(3):
        clk[0] += 2.0
        c.on_goal_result(3, "aborted")
        c.on_pose(0.0, 0.0)
    assert c._blocked_target == 3
    clk[0] += 2.0
    acts = c.on_lane_command(rc.CMD_START, 7, 3)
    assert [a[1] for a in acts if a[0] == "goto"] == [3]


def test_L4_bt_navigator_가_active_가_되면_포기를_지운다():
    clk = [0.0]
    c = RouteChain(clock=lambda: clk[0])
    c.on_route(7, WP5, 4)
    c.on_lane_command(rc.CMD_START, 7, 0)
    c.on_lane_command(rc.CMD_CLEARANCE, 7, 3)
    for _ in range(3):
        clk[0] += 2.0
        c.on_goal_result(3, "aborted")
    clk[0] += 2.0
    assert [a[1] for a in c.clear_failures() if a[0] == "goto"] == [3]


# ---- L5 · STOP 사유는 START 응답이 아니다 ---------------------------------------------------------

def test_L5_Nav2_로봇의_STOP_사유_대기는_START_응답이_아니다():
    c = _coord()
    ctx = c.robots["pinky1"]
    c._arm_start(ctx)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 100.0, "FleetCommand STOP", seq=ctx.route_seq)
    assert ctx.start_acknowledged is False
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 100.1, "clearance 대기 (진행 2, 허가 4)",
                 seq=ctx.route_seq)
    assert ctx.start_acknowledged is True


def test_L5_RUNNING_인데_STOP_사유로_서면_START_재무장():
    c = _coord()                                               # RUNNING, start_acknowledged True
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 100.0, "FleetCommand STOP",
                 seq=c.robots["pinky1"].route_seq)
    assert c.robots["pinky1"].start_acknowledged is False
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_START


def test_L5_세워_둔_로봇은_재무장하지_않는다():
    c = _coord()
    c.stop_robot("pinky1")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 100.0, "LaneCommand STOP")
    assert c.robots["pinky1"].start_acknowledged is True      # 그대로 — HOLD 는 코디네이터가 건 것


# ---- L6 · 재배정은 START 재무장 -------------------------------------------------------------------

def test_L6_비상정지_중_재배정해도_재개하면_START_가_나간다():
    c = _coord()
    c.route_pubs = {}
    c.reservation = Reservation(c.graph)
    c.robots["pinky2"].route = None
    c.estop_fleet()
    assert c.assign_route("pinky1", "START_A", "GOAL_C")
    c.resume_fleet()
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_START


# ---- L7 · 한 번도 시작 안 한 플릿 ------------------------------------------------------------------

def test_L7_시작_안_한_플릿은_ESTOP_재개로_출발하지_않고_거짓_포기_로그도_없다():
    c = _coord(state="ASSIGNED")
    for ctx in c.robots.values():
        ctx.start_acknowledged = False
        ctx.start_retry_count = 0
        ctx.start_armed = False
    c.estop_fleet()
    c.resume_fleet()
    assert c.mission_state == "ASSIGNED"
    _reset(c)
    c._publish_lane_commands()
    assert all(m.command != LaneCommand.CMD_START for p in c.lane_cmd_pubs.values() for m in _sent(p))
    assert not any("START not acknowledged" in str(a) for a in c.get_logger().error.call_args_list)


def test_L7_달리던_플릿은_재개로_RUNNING_이고_미응답_로봇은_START_재무장():
    c = _coord()
    c.robots["pinky2"].start_acknowledged = False
    c.robots["pinky2"].start_retry_count = 0
    c.estop_fleet()
    c.resume_fleet()
    assert c.mission_state == "RUNNING"
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky2"])[-1].command == LaneCommand.CMD_START
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_CLEARANCE


def test_L3_게이트웨이는_같은_목표_배정을_409(monkeypatch):
    from unittest.mock import MagicMock
    from test_d7_robot_stop import _post
    import gateway_web_server as g
    node, coord = MagicMock(), MagicMock()
    coord.assign_conflict.return_value = "pinky1"
    coord.estop_latched = False
    monkeypatch.setattr(g, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(g, "GLOBAL_FLEET_COORDINATOR", coord)
    code, body = _post(g, "/api/fleet/assign", ip="127.0.0.1",
                       body=b'{"robot": "pinky2", "start": "START_B", "goal": "GOAL_C"}')
    assert code == 409 and body["reason"] == "ASSIGN_CONFLICT"
    node.send_fleet_control.assert_not_called()



def test_L5_옛_경로_번호의_STOP_보고로는_재무장하지_않는다():
    c = _coord()
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 100.0, "FleetCommand STOP",
                 seq=c.robots["pinky1"].route_seq - 1)
    assert c.robots["pinky1"].start_acknowledged is True


def test_L7_START_를_무장한_적_없는_로봇에는_거짓_포기_로그를_내지_않는다():
    c = _coord()
    ctx = c.robots["pinky2"]
    ctx.start_acknowledged, ctx.start_retry_count, ctx.start_armed = False, 0, False
    for _ in range(3):
        c._t += 2.0
        c._publish_lane_commands()
    assert not ctx.start_gave_up
    assert not any("START not acknowledged" in str(a) for a in c.get_logger().error.call_args_list)
    c._arm_start(ctx)                                          # 무장한 뒤 5회 넘게 응답이 없으면 그때는 포기라고 말한다
    for _ in range(8):
        c._t += 1.1
        c._publish_lane_commands()
    assert ctx.start_gave_up


# ---- 직렬 검토(L3~L7 후속) · 재개는 직전 상태로 — 시작 안 한 플릿은 출발하지 않는다 -------------------------------

def _assigned_fleet():
    """IDLE 코디네이터에 **진짜 assign_route** 로 두 로봇을 배정한다(예전 시험은 무장 값을 손으로 지워 이 결함을 놓쳤다)."""
    c = _coord(state="IDLE")
    c.route_pubs = {}
    c.reservation = Reservation(c.graph)
    for ctx in c.robots.values():
        ctx.route, ctx.route_seq, ctx.start_acknowledged = None, 0, False
    assert c.assign_route("pinky1", "START_A", "GOAL_C")
    assert c.assign_route("pinky2", "START_B", "JUNCTION_1") is False     # 경로 위 목표 — 아래 L3 후속 시험
    assert c.mission_state == "ASSIGNED"
    return c


def _starts_over(c, secs=8):
    _reset(c)
    out = []
    for _ in range(secs):
        c._t += 1.1
        c._publish_lane_commands()
    for n, p in c.lane_cmd_pubs.items():
        out += [n for m in _sent(p) if m.command == LaneCommand.CMD_START]
    return out


def test_L6_달리지_않는_플릿의_배정은_START_를_무장하지_않는다():
    c = _assigned_fleet()
    ctx = c.robots["pinky1"]
    assert ctx.start_retry_count == 0 and ctx.start_armed is False and ctx.start_acknowledged is False


@pytest.mark.parametrize("ops", [
    ("resume",),
    ("stop", "resume"),
    ("stop", "estop", "resume"),
    ("estop", "resume"),
    ("estop", "estop", "resume"),
])
def test_L7_시작_안_한_플릿은_어떤_재개로도_출발하지_않는다(ops):
    c = _assigned_fleet()
    for op in ops:
        getattr(c, op + "_fleet")()
    assert c.mission_state == "ASSIGNED", ops
    assert _starts_over(c) == [], "시작한 적 없는 플릿이 재개로 START 를 냈다 %s" % (ops,)


@pytest.mark.parametrize("ops", [
    ("stop", "resume"),
    ("estop", "resume"),
    ("stop", "estop", "resume"),
])
def test_L7_달리던_플릿은_재개로_다시_달리고_미응답_로봇은_START(ops):
    c = _assigned_fleet()
    assert c.start_fleet() and c.mission_state == "RUNNING"
    for op in ops:
        getattr(c, op + "_fleet")()
    assert c.mission_state == "RUNNING", ops
    assert "pinky1" in _starts_over(c)


def test_L7_직전_상태를_모르는_재개는_출발시키지_않는다():
    c = _assigned_fleet()
    c.mission_state, c._pre_stop_state = "STOPPED", None       # 어떤 길로든 직전 상태를 잃었다
    c.resume_fleet()
    assert c.mission_state == "ASSIGNED"
    assert _starts_over(c) == []
    c.robots["pinky1"].route = None
    c.mission_state, c._pre_estop_state, c.estop_latched = "ESTOP", None, True
    c.resume_fleet()
    assert c.mission_state == "IDLE"


def test_L6_달리는_중_재배정은_START_를_무장한다():
    c = _assigned_fleet()
    c.start_fleet()
    ctx = c.robots["pinky1"]
    ctx.start_acknowledged = True
    assert c.assign_route("pinky1", "START_A", "GOAL_C")
    assert ctx.start_acknowledged is False and ctx.start_retry_count == c.start_retry_max


def test_L6_옛_경로_번호의_주행_보고로는_새_START_를_확인하지_않는다():
    c = _assigned_fleet()
    c.start_fleet()
    ctx = c.robots["pinky1"]
    c.assign_route("pinky1", "START_A", "GOAL_C")                 # RUNNING 재배정 → seq+1, 무장
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, c._t, "clearance 대기", seq=ctx.route_seq - 1)
    assert ctx.start_acknowledged is False
    assert "pinky1" in _starts_over(c, secs=1)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, c._t, "clearance 대기", seq=ctx.route_seq)
    assert ctx.start_acknowledged is True


# ---- 직렬 검토(L3~L7 후속) · S6 복원은 실제 상태를 직전 상태로 -------------------------------------------------

def test_S6_달리던_중_재시작_복원은_재개로_RUNNING_에_돌아간다():
    c = _coord()                                               # RUNNING
    _lane_status(c, "pinky2", LaneStatus.DRIVE_ESTOP, 100.0, "estop", seq=c.robots["pinky2"].route_seq)
    assert c.mission_state == "ESTOP" and c.robots["pinky2"].held
    c.resume_fleet()
    assert c.mission_state == "RUNNING"
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_CLEARANCE


def test_S6_배정만_한_채_재시작_복원은_재개로_출발하지_않는다():
    c = _assigned_fleet()
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, c._t, "estop", seq=c.robots["pinky1"].route_seq)
    c.resume_fleet()
    assert c.mission_state == "ASSIGNED"
    assert _starts_over(c) == []


# ---- 직렬 검토(L3~L7 후속) · 도착한 로봇이 설 자리를 지나는 배정 ----------------------------------------------

def _idle_empty():
    c = _coord(state="IDLE")
    c.route_pubs = {}
    c.reservation = Reservation(c.graph)
    for ctx in c.robots.values():
        ctx.route, ctx.route_seq = None, 0
    return c


def test_L3_경로가_다른_로봇의_목표를_지나면_거부한다():
    c = _idle_empty()
    assert c.assign_route("pinky1", "START_A", "JUNCTION_1")
    assert c.assign_conflict("pinky2", "GOAL_C", "START_B") == "pinky1"
    assert "JUNCTION_1" in c.assign_conflict_why("pinky2", "GOAL_C", "START_B")
    assert c.assign_route("pinky2", "START_B", "GOAL_C") is False


def test_L3_목표가_다른_로봇의_경로_위면_거부한다():
    c = _idle_empty()
    assert c.assign_route("pinky2", "START_B", "GOAL_C")
    assert c.assign_conflict("pinky1", "JUNCTION_1") == "pinky2"
    assert c.assign_route("pinky1", "START_A", "JUNCTION_1") is False


def test_L3_다른_로봇이_이미_도착했으면_지나간_경로_위_목표는_된다():
    c = _idle_empty()
    assert c.assign_route("pinky2", "START_B", "GOAL_C")
    c.reservation.mark_arrived("pinky2")
    assert c.assign_route("pinky1", "START_A", "JUNCTION_1")


def test_L3_도착해_선_자리를_지나는_경로는_거부한다():
    c = _idle_empty()
    assert c.assign_route("pinky1", "START_A", "JUNCTION_1")
    c.reservation.mark_arrived("pinky1")
    assert c.assign_conflict("pinky2", "GOAL_C", "START_B") == "pinky1"


def test_L3_배포_미션은_서로_부딪히지_않는다():
    from relay_station.fleet import profiles as P
    _, profs = P.load_profiles()
    for name in ("map4", "map4_s2"):
        c = _coord(state="IDLE")
        c.graph = profs[name].graph
        c.route_pubs = {}
        c.reservation = Reservation(c.graph)
        c.robots = {k: v for k, v in c.robots.items()}
        for r in profs[name].mission["robots"]:
            assert c.assign_route(r["name"], r["start"], r["goal"]), (name, r["name"])


def test_L3_도착한_로봇에_막힌_대기는_PARKED_BLOCK_경고():
    c = _coord()                                               # RUNNING
    g = c.graph
    c.reservation = Reservation(g, reserve_ahead=0.40, release_behind=0.25, node_stop_margin=0.20)
    ra = g.shortest_route("START_A", "JUNCTION_1", step=0.10)
    rb = g.shortest_route("START_B", "GOAL_C", step=0.10)
    c.reservation.register("pinky1", 10, ra)                   # 예전 배정(또는 손으로 만든 미션)으로 이미 얽힌 상태
    c.reservation.register("pinky2", 11, rb)
    c.reservation.mark_arrived("pinky1")
    c.robots["pinky1"].arrived = True
    s2 = c.reservation.robots["pinky2"]
    s2.held[:] = [0]
    c.reservation.edge_holder[rb.edge_ids[0]] = "pinky2"
    s2.progress_s = s2.edge_start_s(1) - 0.05
    c.reservation.step()
    assert c.reservation.status("pinky2")["blocked_by"] == "pinky1"
    assert c._check_stalls(100.0) == []
    (msg,) = c._check_stalls(100.0 + c.STALL_WARN_SEC + 0.1)
    assert msg.startswith("PARKED_BLOCK: pinky2") and "pinky1" in msg


def test_L3_게이트웨이_409_는_코디네이터의_이유를_말한다(monkeypatch):
    from unittest.mock import MagicMock
    from test_d7_robot_stop import _post
    import gateway_web_server as g
    node = MagicMock()
    c = _idle_empty()
    c.estop_latched = False
    assert c.assign_route("pinky1", "START_A", "JUNCTION_1")
    monkeypatch.setattr(g, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(g, "GLOBAL_FLEET_COORDINATOR", c)
    code, body = _post(g, "/api/fleet/assign", ip="127.0.0.1",
                       body=b'{"robot": "pinky2", "start": "START_B", "goal": "GOAL_C"}')
    assert code == 409 and body["reason"] == "ASSIGN_CONFLICT" and body["other"] == "pinky1"
    assert "JUNCTION_1" in body["message"]
    node.send_fleet_control.assert_not_called()
