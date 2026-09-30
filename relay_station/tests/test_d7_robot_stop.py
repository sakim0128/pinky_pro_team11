# -*- coding: utf-8 -*-
"""D7 · `/api/robot1/stop` 이 Nav2 로봇을 실제로 세운다.

관제 팜 실측(09-24): `/api/robot1/stop`(`/api/stop`·`/api/nav/stop`)은 `/robot1/mission_cmd` "stop" 만 냈다.
Nav2 에이전트·DriveCommandGate 는 그 토픽을 안 들어서 주행 중 정지가 **무효**였다. 게다가 D8 구독자 1 은
**브리지**라 "정지 명령 전달 (수신자 1)" 이라는 거짓 성공을 냈다.

처방 (a): 플릿 경로로 통일 — 해당 로봇에 LaneCommand STOP + FleetCommand STOP 을 10 Hz 로 보낸다.
그리고 **멈추는 명령은 어디서든** 받는다(태블릿 화면의 일시정지가 403 이면 버튼이 없는 것과 같다).
"""
import io
import json
import os
import sys
import types
from unittest.mock import MagicMock

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GATEWAY_DIR = os.path.join(REPO, "relay_station", "gateway_web")
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "relay_station"))
sys.path.insert(0, GATEWAY_DIR)

import pytest

from builtin_interfaces.msg import Time
from pinky_fleet_msgs.msg import FleetCommand, RobotState
from pinky_lane_msgs.msg import LaneCommand, LaneStatus
from std_msgs.msg import String

from relay_station.fleet.fleet_coordinator import (
    DRIVE_MODE_LANE, DRIVE_MODE_NAV2, FleetRobotContext, RelayFleetCoordinator)
from relay_station.fleet.reservation import Reservation
from relay_station.fleet.road_graph import RoadGraph
from relay_station.fleet.route_comparator import RouteComparator
from pinky_fleet_agent.route_chain import RouteChain

GRAPH = os.path.join(REPO, "relay_station", "fleet", "config", "road_graph.yaml")


def _coord(state="RUNNING", now=100.0):
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
    c.robots = {}
    c.lane_cmd_pubs, c.fleet_cmd_pubs = {}, {}
    for name, dom, start in (("pinky1", 10, "START_A"), ("pinky2", 11, "START_B")):
        ctx = FleetRobotContext(name, dom, start, "GOAL_C", drive_mode=DRIVE_MODE_NAV2)
        ctx.route = c.graph.shortest_route(start, "GOAL_C", step=0.10)
        ctx.route_seq = 3
        ctx.start_acknowledged = True
        ctx.clear_until_idx = 5
        c.robots[name] = ctx
        c.lane_cmd_pubs[name] = MagicMock()
        c.fleet_cmd_pubs[name] = MagicMock()
    return c


def _sent(pub):
    return [call[0][0] for call in pub.publish.call_args_list]


def _reset(c):
    for p in list(c.lane_cmd_pubs.values()) + list(c.fleet_cmd_pubs.values()):
        p.reset_mock()


# ---- 코디네이터: 로봇별 정지 -----------------------------------------------

def test_stop_robot_은_기다리지_않고_바로_STOP_을_보낸다():
    c = _coord()
    assert c.stop_robot("pinky1")
    (lane,) = _sent(c.lane_cmd_pubs["pinky1"])
    (fleet,) = _sent(c.fleet_cmd_pubs["pinky1"])
    assert lane.command == LaneCommand.CMD_STOP and lane.route_seq == 3
    assert fleet.command == FleetCommand.CMD_STOP
    assert _sent(c.lane_cmd_pubs["pinky2"]) == []          # 다른 로봇은 안 건드린다


def test_정지된_로봇만_10Hz_STOP__다른_로봇은_허가를_계속_받는다():
    c = _coord()
    c.stop_robot("pinky1")
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_STOP
    assert _sent(c.fleet_cmd_pubs["pinky1"])[-1].command == FleetCommand.CMD_STOP
    assert _sent(c.lane_cmd_pubs["pinky2"])[-1].command == LaneCommand.CMD_CLEARANCE
    assert c.mission_state == "RUNNING"                      # 플릿 상태는 그대로


def test_경로가_없는_로봇도_STOP_을_받는다():
    c = _coord(state="IDLE")
    c.robots["pinky1"].route = None
    c.stop_robot("pinky1")
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_STOP


def test_ESTOP_이_로봇별_정지보다_앞선다():
    c = _coord(state="ESTOP")
    c.stop_robot("pinky1")
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_ESTOP


def test_resume_robot_은_RESUME_을_보내고_다음_틱부터_허가():
    c = _coord()
    c.stop_robot("pinky1")
    _reset(c)
    assert c.resume_robot("pinky1")
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_RESUME
    assert _sent(c.fleet_cmd_pubs["pinky1"])[-1].command == FleetCommand.CMD_RESUME
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_CLEARANCE


def test_S7_플릿_재개는_로봇별_정지를_유지한다():
    """관제 검수 REVIEW_20260925 S7 (예전 D7 시험 '플릿 재시작은 로봇별 정지도 푼다' 를 뒤집는다):
    ESTOP 의 유일한 출구가 held 까지 지우면 따로 세워 둔 로봇이 ESTOP 해제 순간 달린다."""
    c = _coord(state="STOPPED")
    c.stop_robot("pinky2")
    _reset(c)
    c.resume_fleet()
    assert c.robots["pinky2"].held
    assert all(m.command != LaneCommand.CMD_RESUME for m in _sent(c.lane_cmd_pubs["pinky2"]))
    assert [m.command for m in _sent(c.lane_cmd_pubs["pinky1"])] == [LaneCommand.CMD_RESUME]
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky2"])[-1].command == LaneCommand.CMD_STOP


def test_S7_ESTOP_해제_뒤_세워_둔_로봇은_로봇_재개로만_출발한다():
    c = _coord()
    c.stop_robot("pinky1")
    c.estop_fleet()
    c.resume_fleet()
    assert c.robots["pinky1"].held
    _reset(c)
    assert c.resume_robot("pinky1") is True                # 래치가 풀렸으니 이제는 된다
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_RESUME


def test_S5_경로_없는_로봇도_비상정지_동안_10Hz_ESTOP():
    c = _coord()
    c.robots["pinky2"].route = None
    c.estop_fleet()
    _reset(c)
    for _ in range(3):
        c._publish_lane_commands()
    assert [m.command for m in _sent(c.lane_cmd_pubs["pinky2"])] == [LaneCommand.CMD_ESTOP] * 3


def test_S5_경로_없는_로봇은_비상정지가_아니면_LaneCommand_를_안_받는다():
    c = _coord()
    c.robots["pinky2"].route = None
    _reset(c)
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky2"]) == []


def test_S6_재시작_직후_로봇이_ESTOP_을_보고하면_플릿_래치를_되살린다():
    c = _coord(state="IDLE")                                 # 재시작한 코디네이터 — 래치 기억 없음
    assert not c.estop_latched
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, 100.0, "E-STOP")
    assert c.estop_latched and c.mission_state == "ESTOP"
    assert c.resume_robot("pinky1") is False               # 한 대씩 풀리지 않는다
    c.resume_fleet()
    assert not c.estop_latched


def test_S6_직접_해제한_뒤의_ESTOP_보고로는_되살리지_않는다__S7_의_세워_둔_로봇():
    c = _coord()
    c.stop_robot("pinky1")
    c.estop_fleet()
    c.resume_fleet()                                       # 세워 둔 pinky1 은 ESTOP 래치를 쥔 채 남는다(S7)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, 101.0, "E-STOP")
    assert not c.estop_latched and c.mission_state == "RUNNING"


def test_robot1_별칭과_모르는_로봇():
    c = _coord()
    assert c.stop_robot("robot1") and c.robots["pinky1"].held
    assert c.stop_robot("pinky9") is False


def test_제어_토픽_JSON_으로도_정지_재개된다():
    c = _coord()
    c._cb_control(String(data=json.dumps({"cmd": "stop_robot", "robot": "pinky2"})))
    assert c.robots["pinky2"].held
    c._cb_control(String(data=json.dumps({"cmd": "resume_robot", "robot": "pinky2"})))
    assert not c.robots["pinky2"].held


def test_마지막으로_들은_시각__들은_적_없으면_None():
    c = _coord(now=50.0)
    assert c.last_heard_sec("pinky1") is None
    c._cb_robot_state("pinky1", RobotState())
    c._t = 51.5
    assert c.last_heard_sec("pinky1") == pytest.approx(1.5)
    st = c.get_fleet_status_dict()["robots"]["pinky1"]
    assert st["held"] is False and st["last_heard_sec"] == pytest.approx(1.5)


# ---- 관제 검수 P1 · 플릿 비상정지 중 로봇 재개는 거부 ------------------------------------

def test_P1_플릿_ESTOP_중_resume_robot_은_거부하고_RESUME_을_안_보낸다():
    """LaneCommand RESUME 은 에이전트의 **모든** 래치를 푼다(`/estop false`) — 한 로봇만 비상정지가 풀린다."""
    c = _coord(state="ESTOP")
    c.stop_robot("pinky1")
    _reset(c)
    assert c.resume_robot("pinky1") is False
    assert _sent(c.lane_cmd_pubs["pinky1"]) == [] and _sent(c.fleet_cmd_pubs["pinky1"]) == []
    assert c.robots["pinky1"].held                          # 상태도 안 바꾼다
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_ESTOP


def test_P1_D8_제어_토픽으로_와도_플릿_ESTOP_중_재개는_거부():
    c = _coord(state="ESTOP")
    c._cb_control(String(data=json.dumps({"cmd": "resume_robot", "robot": "pinky2"})))
    assert all(m.command != LaneCommand.CMD_RESUME for m in _sent(c.lane_cmd_pubs["pinky2"]))


def test_P1_규약_재개가_ESTOP_래치를_풀_수_있는_유일한_길은_resume_fleet():
    c = _coord(state="ESTOP")
    chain = RouteChain()
    ctx = c.robots["pinky1"]
    msg = c._to_route_msg("pinky1", ctx.route_seq, ctx.route)
    chain.on_route(msg.route_seq, [(p.x, p.y) for p in msg.waypoints], msg.goal_idx)
    chain.on_lane_command(LaneCommand.CMD_START, ctx.route_seq, 0)
    chain.on_lane_command(LaneCommand.CMD_ESTOP, 0, 0)
    c.resume_robot("pinky1")
    for m in _sent(c.lane_cmd_pubs["pinky1"]):
        chain.on_lane_command(m.command, m.route_seq, m.clear_until_idx)
    assert chain.estop                                       # 로봇 재개로는 안 풀렸다
    c.resume_fleet()
    for m in _sent(c.lane_cmd_pubs["pinky1"]):
        chain.on_lane_command(m.command, m.route_seq, m.clear_until_idx)
    assert not chain.estop


# ---- 관제 검수 P3 · 로봇별 정지 중인 로봇에는 START 를 보내지 않는다 ----------------------

def test_P3_start_fleet_은_held_로봇에_START_를_안_보낸다():
    c = _coord(state="STOPPED")
    c.stop_robot("pinky1")
    _reset(c)
    c.start_fleet()
    assert [m.command for m in _sent(c.lane_cmd_pubs["pinky1"])] == []
    assert [m.command for m in _sent(c.lane_cmd_pubs["pinky2"])] == [LaneCommand.CMD_START]
    c._publish_lane_commands()                               # 다음 틱: held 는 계속 STOP
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_STOP


# ---- 관제 검수 P2 · 정지 확인 (로봇의 정지 **뒤** 보고) ---------------------------------

def _lane_status(c, name, state, t, reason="", v=0.0, w=0.0, seq=0):
    m = LaneStatus()
    m.drive_state = state
    m.route_seq = seq
    m.state_reason = reason
    m.linear_velocity = v
    m.angular_velocity = w
    c._t = t
    c._cb_lane_status(name, m)


def test_P2_정지_확인은_정지_뒤_보고만_센다():
    c = _coord(now=10.0)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 9.0, "LaneCommand STOP")   # 정지 **전** 보고
    assert c.hold_confirmation("pinky1", since=10.0) is None
    _lane_status(c, "pinky1", LaneStatus.DRIVE_CRUISE, 10.2, "진행 → 8", v=0.12)
    assert c.hold_confirmation("pinky1", since=10.0) is False
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 10.3, "LaneCommand STOP")
    assert c.hold_confirmation("pinky1", since=10.0) is True
    assert c.hold_confirmation("pinky9", since=10.0) is None


@pytest.mark.parametrize("state,reason", [
    (LaneStatus.DRIVE_IDLE, "경로 수신 — START 대기"),         # 단일 목표로 달려도 체인은 IDLE 이다
    (LaneStatus.DRIVE_WAIT_CLEARANCE, "clearance 대기 (진행 3, 허가 5)"),   # STOP 이 닿기 전의 대기
    (LaneStatus.DRIVE_ARRIVED, "목표 도착"),
])
def test_검토P2_멈춘_상태만으로는_확인이_아니다__STOP_처리_증거가_있어야(state, reason):
    """직렬 검토: IDLE·대기·도착은 STOP 을 받기 전에도 나온다 — 그것만으로 success 를 내면 거짓 확인이다."""
    c = _coord(now=10.0)
    _lane_status(c, "pinky1", state, 10.1, reason)
    assert c.hold_confirmation("pinky1", since=10.0) is None


def test_검토P2_STOP_을_처리했어도_아직_속도가_있으면_달리는_중():
    c = _coord(now=10.0)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 10.1, "FleetCommand STOP", v=0.15)   # 우회 goal 감속 전
    assert c.hold_confirmation("pinky1", since=10.0) is False
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 10.6, "FleetCommand STOP", v=0.0)
    assert c.hold_confirmation("pinky1", since=10.0) is True


def test_검토2_제자리_회전도_달리는_중이다():
    c = _coord(now=10.0)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 10.1, "FleetCommand STOP", v=0.0, w=0.8)
    assert c.hold_confirmation("pinky1", since=10.0) is False


@pytest.mark.parametrize("state,expect", [
    (LaneStatus.DRIVE_IDLE, True),                 # 팀11: STOP → started=False → IDLE '대기'
    (LaneStatus.DRIVE_WAIT_CLEARANCE, None),       # 시작된 채 대기 — 허가가 오면 움직인다
    (LaneStatus.DRIVE_ARRIVED, None),
])
def test_검토2_팀11_레인_로봇은_IDLE_이_정지_증거다(state, expect):
    """직렬 검토 2회차: 팀11 lane_agent 는 STOP 사유 문구를 안 남겨서 레인 로봇은 영영 확인이 안 됐다."""
    c = _coord(now=10.0)
    c.robots["pinky1"].drive_mode = DRIVE_MODE_LANE
    _lane_status(c, "pinky1", state, 10.1, "대기")
    assert c.hold_confirmation("pinky1", since=10.0) is expect


def test_검토2_Nav2_로봇의_IDLE_은_여전히_증거가_아니다():
    c = _coord(now=10.0)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 10.1, "대기")
    assert c.hold_confirmation("pinky1", since=10.0) is None


def test_검토P2_ESTOP_링크유실_래치는_그_자체가_증거():
    c = _coord(now=10.0)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, 10.1, "E-STOP")
    assert c.hold_confirmation("pinky1", since=10.0) is True


# ---- 직렬 검토 P1 후속 · 플릿 비상정지 래치는 resume_fleet 만 푼다 --------------------------------

@pytest.mark.parametrize("then", ["stop", "start"])
def test_검토P1_ESTOP_뒤_stop_start_가_래치를_지우지_않는다__로봇_재개는_계속_거부(then):
    """estop → fleet stop/start 가 mission_state 를 덮어써서 로봇별 재개가 ESTOP 을 풀던 우회로."""
    c = _coord()
    c.estop_fleet()
    assert (c.stop_fleet() if then == "stop" else c.start_fleet()) is False
    assert c.mission_state == "ESTOP" and c.estop_latched
    _reset(c)
    assert c.resume_robot("pinky1") is False
    assert all(m.command != LaneCommand.CMD_RESUME for m in _sent(c.lane_cmd_pubs["pinky1"]))
    c._publish_lane_commands()
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_ESTOP


def test_검토P1_resume_fleet_만_ESTOP_래치를_푼다():
    c = _coord()
    c.estop_fleet()
    c.resume_fleet()
    assert not c.estop_latched and c.mission_state == "RUNNING"
    assert c.stop_fleet() is True and c.start_fleet() is True
    assert c.get_fleet_status_dict()["estop_latched"] is False


def test_규약_로봇별_STOP_은_에이전트에서_HOLD__estop_아님():
    c = _coord()
    ctx = c.robots["pinky1"]
    msg = c._to_route_msg("pinky1", ctx.route_seq, ctx.route)
    chain = RouteChain()
    chain.on_route(msg.route_seq, [(p.x, p.y) for p in msg.waypoints], msg.goal_idx)
    chain.on_lane_command(LaneCommand.CMD_START, ctx.route_seq, 0)
    chain.on_lane_command(LaneCommand.CMD_CLEARANCE, ctx.route_seq, 8)
    c.stop_robot("pinky1")
    (stop,) = _sent(c.lane_cmd_pubs["pinky1"])
    acts = chain.on_lane_command(stop.command, stop.route_seq, stop.clear_until_idx)
    assert [a[0] for a in acts] == ["cancel"]


# ---- 게이트웨이 HTTP: 가짜 요청으로 do_POST 를 직접 부른다 -----------------

@pytest.fixture
def gw(monkeypatch):
    import gateway_web_server as g
    node = MagicMock()
    node.send_mission.return_value = 1        # 도메인 8 구독자 1 = 브리지
    coord = MagicMock()
    coord.last_heard_sec.return_value = 0.3
    coord.now.return_value = 100.0
    coord.hold_confirmation.return_value = True          # 기본: 로봇이 정지 뒤 멈춤을 보고
    coord.mission_state = "RUNNING"
    coord.estop_latched = False
    coord.assign_conflict.return_value = None
    # 코디네이터가 제어 명령을 처리한 것처럼(R4 — 게이트웨이는 처리 결과로 답한다). 시험이 ok 를 바꿀 수 있다.
    coord.control_seq, coord.last_control, coord.apply_ok = 0, None, True

    def _processed(m):
        coord.control_seq += 1
        coord.last_control = {"seq": coord.control_seq, "cmd": m["cmd"], "ok": coord.apply_ok,
                              "mission_state": coord.mission_state}
    node.send_fleet_control.side_effect = _processed
    monkeypatch.setattr(g, "CONTROL_APPLY_WAIT_SEC", 0.1)
    monkeypatch.setattr(g, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(g, "GLOBAL_FLEET_COORDINATOR", coord)
    monkeypatch.setattr(g, "STOP_CONFIRM_WAIT_SEC", 0.05)
    monkeypatch.setattr(g, "STOP_CONFIRM_POLL_SEC", 0.01)
    return g, node, coord


def _post(g, path, ip="192.0.2.50", body=b"{}"):
    h = g.GatewayRequestHandler.__new__(g.GatewayRequestHandler)
    h.path = path
    h.headers = {"Content-Length": str(len(body))}
    h.rfile = io.BytesIO(body)
    h.client_address = (ip, 50000)
    out = {}

    def _send_json(body_bytes, code=200):
        out["code"] = code
        out["body"] = json.loads(body_bytes.decode("utf-8"))
    h._send_json = _send_json
    h.do_POST()
    return out["code"], out["body"]


def _dispatched(node):
    return [call[0][0] for call in node.send_fleet_control.call_args_list]


@pytest.mark.parametrize("path,robot", [
    ("/api/robot1/stop", "pinky1"), ("/api/stop", "pinky1"),
    ("/api/nav/stop", "pinky1"), ("/api/robot2/stop", "pinky2"),
])
def test_정지_API_는_플릿_경로로_보내고_원격에서도_받는다(gw, path, robot):
    g, node, _ = gw
    code, body = _post(g, path)                       # 로컬이 아닌 주소
    assert code == 200 and body["success"] is True and body["confirmed"] is True
    assert _dispatched(node) == [{"cmd": "stop_robot", "robot": robot}]
    assert body["dispatched_via"] == "fleet" and body["semantics"] == "HOLD"
    assert body["robot_heard_recently"] is True


def test_정지_응답이_브리지_구독자를_로봇_수신으로_포장하지_않는다(gw):
    g, _, _ = gw
    _, body = _post(g, "/api/robot1/stop")
    legacy = body["legacy_mission_cmd"]
    assert legacy["d8_subscribers"] == 1
    assert legacy["means"] == "DOMAIN_8_SUBSCRIBER_EXISTS_INCLUDING_BRIDGE"
    assert "수신자" not in body["message"]


def test_로봇_소식이_끊겼으면_보냈지만_모른다고_말한다(gw):
    g, _, coord = gw
    coord.hold_confirmation.return_value = None           # 정지 뒤 보고 없음
    coord.last_heard_sec.return_value = 9.0
    code, body = _post(g, "/api/robot1/stop")
    assert code == 202 and body["success"] is False and body["dispatched"] is True
    assert body["reason"] == "NO_STOP_ACK" and body["confirmed"] is None
    assert body["robot_heard_recently"] is False and "모른다" in body["message"]
    coord.last_heard_sec.return_value = None
    _, body = _post(g, "/api/robot1/stop")
    assert body["robot_heard_recently"] is None and body["robot_last_heard_sec"] is None


def test_P2_수신자가_없으면_success_가_아니다__코디네이터_없음_503(gw, monkeypatch):
    """관제 검수 P2: 예전엔 받을 곳이 없어도 success:true 200 이었다."""
    g, node, _ = gw
    monkeypatch.setattr(g, "GLOBAL_FLEET_COORDINATOR", None)
    code, body = _post(g, "/api/robot1/stop")
    assert code == 503 and body["success"] is False and body["reason"] == "NO_FLEET_COORDINATOR"


def test_P2_로봇이_아직_주행_중이라고_보고하면_202(gw):
    g, _, coord = gw
    coord.hold_confirmation.return_value = False
    code, body = _post(g, "/api/robot1/stop")
    assert code == 202 and body["success"] is False and body["confirmed"] is False
    assert body["reason"] == "ROBOT_STILL_CRUISING"


def test_P2_확인_기준_시각은_보내기_전에_잡는다(gw):
    """보낸 뒤에 잡으면 그 사이 온 '멈춤' 보고를 놓치고, 보내기 전 보고를 세면 거짓 확인이 된다."""
    g, node, coord = gw
    order = []
    coord.now.side_effect = lambda: order.append("now") or 100.0
    node.send_fleet_control.side_effect = lambda _m: order.append("send")
    _post(g, "/api/robot1/stop")
    assert order[:2] == ["now", "send"]
    assert coord.hold_confirmation.call_args[0] == ("pinky1", 100.0)


def test_P1_플릿_비상정지_중_로봇_재개는_409_이고_보내지_않는다(gw):
    g, node, coord = gw
    coord.mission_state = "ESTOP"
    code, body = _post(g, "/api/robot1/resume", ip="127.0.0.1")
    assert code == 409 and body["reason"] == "FLEET_ESTOP" and _dispatched(node) == []


def test_ROS_노드가_없으면_503(gw, monkeypatch):
    g, _, _ = gw
    monkeypatch.setattr(g, "GLOBAL_ROBOT_SUB_NODE", None)
    code, body = _post(g, "/api/robot1/stop")
    assert code == 503 and body["reason"] == "NO_ROS_NODE"


def test_재개는_로컬에서만(gw):
    g, node, _ = gw
    code, _ = _post(g, "/api/robot1/resume")
    assert code == 403 and _dispatched(node) == []
    code, body = _post(g, "/api/robot1/resume", ip="127.0.0.1")
    assert code == 200 and _dispatched(node) == [{"cmd": "resume_robot", "robot": "pinky1"}]


@pytest.mark.parametrize("cmd,remote_ok", [
    ("stop", True), ("estop", True), ("start", False), ("resume", False), ("assign", False),
])
def test_플릿_API__멈추는_명령만_원격에서_받는다(gw, cmd, remote_ok):
    g, node, _ = gw
    code, _ = _post(g, "/api/fleet/" + cmd)
    if remote_ok:
        assert code == 200 and _dispatched(node)[-1]["cmd"] == cmd
    else:
        assert code == 403 and _dispatched(node) == []



def test_검토P3_정지_응답은_보고가_늦게_와도_기다렸다가_확인한다(gw, monkeypatch):
    """보내자마자 한 번만 보면 보고는 아직 없다(None) — 기다리는 고리가 없으면 영영 확인을 못 한다."""
    g, _, coord = gw
    monkeypatch.setattr(g, "STOP_CONFIRM_WAIT_SEC", 1.0)
    coord.hold_confirmation.side_effect = [None, None, True]
    code, body = _post(g, "/api/robot1/stop")
    assert code == 200 and body["confirmed"] is True
    assert coord.hold_confirmation.call_count == 3


def test_검토P1_플릿_비상정지_래치_중이면_start_와_로봇_재개가_409(gw):
    g, node, coord = gw
    coord.mission_state = "STOPPED"                        # mission_state 가 덮여도 래치는 남는다
    coord.estop_latched = True
    code, body = _post(g, "/api/fleet/start", ip="127.0.0.1")
    assert code == 409 and body["reason"] == "FLEET_ESTOP" and _dispatched(node) == []
    code, body = _post(g, "/api/robot1/resume", ip="127.0.0.1")
    assert code == 409 and _dispatched(node) == []


# ---- 관제 검수 REVIEW_20260925 S1 · 에이전트를 거치지 않는 목표·미션 경로 ----------------------------------










def test_검토S_경로_없는_Nav2_로봇은_start_에서_FleetCommand_RESUME_으로_STOP_이_풀린다():
    c = _coord(state="STOPPED")
    c.robots["pinky2"].route = None
    _reset(c)
    c.start_fleet()
    assert [m.command for m in _sent(c.fleet_cmd_pubs["pinky2"])] == [FleetCommand.CMD_RESUME]
    assert all(m.command != FleetCommand.CMD_RESUME for m in _sent(c.fleet_cmd_pubs["pinky1"]))   # 경로 있는 로봇은 START


def test_검토S_경로_없어도_세워_둔_로봇은_start_로_안_풀린다():
    c = _coord(state="STOPPED")
    c.robots["pinky2"].route = None
    c.stop_robot("pinky2")
    _reset(c)
    c.start_fleet()
    assert all(m.command != FleetCommand.CMD_RESUME for m in _sent(c.fleet_cmd_pubs["pinky2"]))


def test_검토S_재시작_뒤_ESTOP_을_보고한_로봇은_HOLD_로_되살아나_플릿_재개로_출발하지_않는다():
    c = _coord(state="IDLE")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, 100.0, "E-STOP")
    assert c.robots["pinky1"].held
    _reset(c)
    c.resume_fleet()
    assert all(m.command != LaneCommand.CMD_RESUME for m in _sent(c.lane_cmd_pubs["pinky1"]))
    assert [m.command for m in _sent(c.lane_cmd_pubs["pinky2"])] == [LaneCommand.CMD_RESUME]



# ---- 관제 검수 REVIEW_20260925 §3.3 R4·R5 ------------------------------------------------------

@pytest.mark.parametrize("ok,code,reason", [(True, 200, None), (False, 409, "NOT_APPLIED")])
def test_R4_시작·재개는_코디네이터가_처리한_결과로_답한다(gw, ok, code, reason):
    g, node, coord = gw
    coord.apply_ok = ok
    for cmd in ("start", "resume"):
        c, body = _post(g, "/api/fleet/" + cmd, ip="127.0.0.1")
        assert c == code and body.get("reason") == reason and body["applied"] is ok


def test_R4_처리_결과를_못_받으면_202_모른다(gw):
    g, node, coord = gw
    node.send_fleet_control.side_effect = None               # 코디네이터가 처리하지 않음
    c, body = _post(g, "/api/fleet/start", ip="127.0.0.1")
    assert c == 202 and body["applied"] is None and body["success"] is False


def test_R5_플릿_정지_중_로봇_재개는_HOLD_만_풀고_RESUME_을_안_보낸다():
    c = _coord(state="STOPPED")
    c.stop_robot("pinky1")
    _reset(c)
    assert c.resume_robot("pinky1") is True
    assert not c.robots["pinky1"].held
    assert _sent(c.lane_cmd_pubs["pinky1"]) == [] and _sent(c.fleet_cmd_pubs["pinky1"]) == []
    c._publish_lane_commands()                               # 플릿 STOP 은 계속
    assert _sent(c.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_STOP


def test_R5_게이트웨이는_플릿이_서_있다고_말한다(gw):
    g, node, coord = gw
    coord.mission_state = "STOPPED"
    c, body = _post(g, "/api/robot1/resume", ip="127.0.0.1")
    assert c == 200 and "서 있다" in body["message"]


def test_R1_Nav2_활성_표식이_붙은_STOP_사유로는_확인하지_않는다():
    c = _coord(now=10.0)
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 10.1, "FleetCommand STOP · Nav2 활성 1")
    assert c.hold_confirmation("pinky1", since=10.0) is None
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 10.2, "FleetCommand STOP")
    assert c.hold_confirmation("pinky1", since=10.0) is True
