# -*- coding: utf-8 -*-
"""D6 · 코디네이터 ↔ Nav2 에이전트 규약 — 양쪽을 **실제 메시지로** 이어 붙여 잰다.

지시: `docs/REQ_20260924_RELAY_SESSION.md` D6 (⑤ 단위 테스트: START→ack→CLEARANCE 전이).
관제 팜 실측(09-24): START 5회 재전송 후 포기 · RUNNING+clear 0 에 10 Hz FleetCommand STOP →
에이전트가 매번 hold+/estop · 코디네이터가 GOTO 를 안 보냄 → Nav2 로봇이 출발하지 못했다.

⭐ 이 파일은 코디네이터와 에이전트 로직(`route_chain`)을 **ROS 전송 없이** 맞물린다. 전송(브리지)은
   관제 리그의 수락 시험이 잰다 — 여기서는 "두 쪽이 같은 말을 하는가" 만 잰다.
"""
import os
import sys
import types
from unittest.mock import MagicMock

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

import pytest
import yaml

from builtin_interfaces.msg import Time
from pinky_fleet_msgs.msg import FleetCommand
from pinky_lane_msgs.msg import LaneCommand, LaneStatus

from relay_station.fleet.fleet_coordinator import (
    DRIVE_MODE_LANE, DRIVE_MODE_NAV2, FleetRobotContext, RelayFleetCoordinator)
from relay_station.fleet.reservation import Reservation
from relay_station.fleet.road_graph import RoadGraph
from relay_station.fleet.route_comparator import RouteComparator
from pinky_fleet_agent.route_chain import RouteChain

GRAPH = os.path.join(REPO, "relay_station", "fleet", "config", "road_graph.yaml")
MISSION = os.path.join(REPO, "relay_station", "fleet", "config", "lane_mission.yaml")


def _coord(mode, state="RUNNING", now=100.0):
    c = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    c.graph = RoadGraph.load(GRAPH)
    c.reservation = Reservation(c.graph)
    c.comparator = RouteComparator()
    c.mission_state = state
    c.start_retry_max = 5
    c.start_retry_interval = 1.0
    c.last_warning = ""
    c.get_logger = MagicMock()
    c._t = now
    c._now = lambda: c._t
    c.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: Time()))
    ctx = FleetRobotContext("pinky1", 10, "START_A", "GOAL_C", drive_mode=mode)
    ctx.route = c.graph.shortest_route("START_A", "GOAL_C", step=0.10)
    ctx.route_seq = 3
    c.robots = {"pinky1": ctx}
    c.lane_cmd_pubs = {"pinky1": MagicMock()}
    c.fleet_cmd_pubs = {"pinky1": MagicMock()}
    return c, ctx


def _sent(pub):
    return [call[0][0] for call in pub.publish.call_args_list]


def _last_fleet(c):
    return _sent(c.fleet_cmd_pubs["pinky1"])[-1].command


# ---- FleetCommand: Nav2 모드는 하트비트, 레인 모드는 그대로 ------------------

def test_Nav2_로봇은_RUNNING_clear0_에서_STOP_을_연발하지_않는다():
    c, ctx = _coord(DRIVE_MODE_NAV2)
    ctx.start_acknowledged = True
    ctx.clear_until_idx = 0
    c._publish_lane_commands()
    assert _last_fleet(c) == FleetCommand.CMD_HEARTBEAT


def test_Nav2_로봇은_ESTOP_에서_CANCEL_대신_하트비트__비상정지는_LaneCommand_가_건다():
    """CANCEL 은 에이전트가 레인 임무를 버리게 해서 RESUME 뒤에도 출발하지 못하게 만든다."""
    c, ctx = _coord(DRIVE_MODE_NAV2, state="ESTOP")
    c._publish_lane_commands()
    assert _last_fleet(c) == FleetCommand.CMD_HEARTBEAT
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_ESTOP


def test_Nav2_로봇은_STOPPED_에서_STOP():
    c, _ = _coord(DRIVE_MODE_NAV2, state="STOPPED")
    c._publish_lane_commands()
    assert _last_fleet(c) == FleetCommand.CMD_STOP


@pytest.mark.parametrize("state,clear,expect", [
    ("RUNNING", 0, FleetCommand.CMD_STOP),
    ("RUNNING", 5, FleetCommand.CMD_RESUME),
    ("ESTOP", 0, FleetCommand.CMD_CANCEL),
    ("STOPPED", 0, FleetCommand.CMD_STOP),
    ("IDLE", 0, FleetCommand.CMD_HEARTBEAT),
])
def test_레인_모드는_예전_그대로다(state, clear, expect):
    c, ctx = _coord(DRIVE_MODE_LANE, state=state)
    ctx.start_acknowledged = True
    ctx.clear_until_idx = clear
    c._publish_lane_commands()
    assert _last_fleet(c) == expect


# ---- START 재전송이 끝나면 조용히 멈추지 않고 드러낸다 -----------------------

def test_START_가_끝내_확인되지_않으면_gave_up_이_상태에_드러난다():
    c, ctx = _coord(DRIVE_MODE_NAV2, now=10.0)
    c.start_fleet()
    for t in (11.1, 12.2, 13.3, 14.4, 15.5, 16.6):
        c._t = t
        c._publish_lane_commands()
    assert ctx.start_retry_count == 0 and ctx.start_gave_up
    st = c.get_fleet_status_dict()["robots"]["pinky1"]
    assert st["drive_mode"] == "nav2"
    assert st["start"] == {"acknowledged": False, "retries_left": 0, "gave_up": True}


def test_다시_START_하면_gave_up_이_풀린다():
    c, ctx = _coord(DRIVE_MODE_NAV2)
    ctx.start_gave_up = True
    c.start_fleet()
    assert not ctx.start_gave_up and ctx.start_retry_count == 5


# ---- ⭐ 규약: START → ack → CLEARANCE → 목표 (두 쪽을 이어 붙인다) --------------

def _agent_from_route(c, ctx):
    """에이전트가 Route 를 받는 방식 그대로 (agent_node._on_route)."""
    msg = c._to_route_msg("pinky1", ctx.route_seq, ctx.route)
    chain = RouteChain()
    chain.on_route(msg.route_seq, [(p.x, p.y) for p in msg.waypoints], msg.goal_idx)
    return chain, msg


def _lane_status_from(chain):
    """에이전트가 LaneStatus 를 채우는 방식 그대로 (agent_node._publish_lane_status)."""
    st = chain.status()
    ls = LaneStatus()
    ls.robot_name = "pinky1"
    ls.drive_state = int(st["drive_state"])
    ls.route_seq = int(st["route_seq"])
    ls.route_idx = int(st["route_idx"])
    ls.clear_until_idx = int(st["clear_until_idx"])
    return ls


def test_규약_START_ack_CLEARANCE_목표까지():
    c, ctx = _coord(DRIVE_MODE_NAV2, now=10.0)
    chain, route_msg = _agent_from_route(c, ctx)

    # ① 코디네이터 START → 에이전트
    c.start_fleet()
    (start,) = _sent(c.lane_cmd_pubs["pinky1"])
    assert start.command == LaneCommand.CMD_START and start.route_seq == ctx.route_seq
    chain.on_lane_command(start.command, start.route_seq, start.clear_until_idx)

    # ② 에이전트 LaneStatus → 코디네이터 ack (예전엔 LaneStatus 가 없어 여기서 막혔다)
    c._cb_lane_status("pinky1", _lane_status_from(chain))
    assert ctx.start_acknowledged

    # ③ 다음 틱은 CLEARANCE (START 재전송이 아니다)
    mid = len(route_msg.waypoints) // 2             # 경로 길이는 road_graph 가 정한다 — 숫자를 박지 않는다
    assert 0 < mid < route_msg.goal_idx
    ctx.clear_until_idx = mid
    c.lane_cmd_pubs["pinky1"].reset_mock()
    c._t = 12.0
    c._publish_lane_commands()
    (clr,) = _sent(c.lane_cmd_pubs["pinky1"])
    assert clr.command == LaneCommand.CMD_CLEARANCE and clr.clear_until_idx == mid

    # ④ 에이전트는 그 waypoint 로 NavigateToPose (예전엔 GOTO 가 어디서도 안 나갔다)
    acts = chain.on_lane_command(clr.command, clr.route_seq, clr.clear_until_idx)
    gotos = [a for a in acts if a[0] == "goto"]
    assert len(gotos) == 1
    _, idx, x, y, _yaw = gotos[0]
    wp = route_msg.waypoints[mid]
    assert idx == mid and (x, y) == pytest.approx((wp.x, wp.y))
    assert "estop" not in [a[0] for a in acts]

    # ⑤ 코디네이터 FleetCommand 는 하트비트 — 에이전트를 HOLD 로 되돌리지 않는다
    assert _last_fleet(c) == FleetCommand.CMD_HEARTBEAT


def test_규약_fleet_stop_resume__HOLD_이지_estop_이_아니다():
    c, ctx = _coord(DRIVE_MODE_NAV2)
    chain, _ = _agent_from_route(c, ctx)
    chain.on_lane_command(LaneCommand.CMD_START, ctx.route_seq, 0)
    chain.on_lane_command(LaneCommand.CMD_CLEARANCE, ctx.route_seq, 20)

    c.stop_fleet()
    (stop,) = _sent(c.lane_cmd_pubs["pinky1"])
    acts = chain.on_lane_command(stop.command, stop.route_seq, stop.clear_until_idx)
    assert [a[0] for a in acts] == ["cancel"]

    c.lane_cmd_pubs["pinky1"].reset_mock()
    c.resume_fleet()
    (res,) = _sent(c.lane_cmd_pubs["pinky1"])
    acts = chain.on_lane_command(res.command, res.route_seq, res.clear_until_idx)
    assert [a[0] for a in acts] == ["goto"]


def test_규약_fleet_estop_은_route_seq_없이도_에이전트를_세운다():
    c, ctx = _coord(DRIVE_MODE_NAV2)
    chain, _ = _agent_from_route(c, ctx)
    chain.on_lane_command(LaneCommand.CMD_START, ctx.route_seq, 0)
    chain.on_lane_command(LaneCommand.CMD_CLEARANCE, ctx.route_seq, 20)
    c.estop_fleet()
    (es,) = _sent(c.lane_cmd_pubs["pinky1"])
    assert es.route_seq == 0                       # 코디네이터는 번호를 안 채운다
    acts = chain.on_lane_command(es.command, es.route_seq, es.clear_until_idx)
    assert ("estop", True) in acts


# ---- 설정 ------------------------------------------------------------------

def test_우리_로봇은_nav2_모드로_설정돼_있다():
    robots = yaml.safe_load(open(MISSION, encoding="utf-8"))["robots"]
    assert {r["name"]: r.get("drive_mode") for r in robots} == {"pinky1": "nav2", "pinky2": "nav2"}


def test_모르는_drive_mode_는_기동을_거부한다():
    with pytest.raises(ValueError, match="drive_mode"):
        FleetRobotContext("pinky1", 10, drive_mode="nav3")
