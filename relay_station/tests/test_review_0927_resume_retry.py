# -*- coding: utf-8 -*-
"""로봇 재개 재전송 — 최종 통합 트리의 실제 프로세스 종단 모의(월요일 절차)가 찾은 것.

게이트웨이를 주행 중 재기동하면 로봇은 데드맨으로 링크유실 래치가 걸리고, 새 코디네이터는 그 로봇을 '링크유실 래치 —
로봇 재개 필요' 로 세운다. 새 코디네이터와 에이전트가 DDS 로 서로를 찾는 데 약 3 s 가 걸리는데, 그 사이 누른 로봇 재개
(LaneCommand·FleetCommand RESUME, volatile, 한 발)는 받는 곳 없이 사라졌다. 게이트웨이는 '적용' 으로 답했고, 유예 3 s 뒤
코디네이터가 로봇을 다시 세워 60 s 동안 아무도 움직이지 않았다. 정지·비상정지는 10 Hz 반복, START 는 재시도가 있다 —
RESUME 만 한 발이었다.
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pinky_fleet_msgs.msg import FleetCommand  # noqa: E402
from pinky_lane_msgs.msg import LaneCommand, LaneStatus  # noqa: E402
from relay_station.fleet import fleet_coordinator as FC  # noqa: E402
from test_d7_robot_stop import _coord, _lane_status, _reset, _sent  # noqa: E402

LOST = LaneStatus.DRIVE_LINK_LOST


def _held_lost(state="ASSIGNED"):
    c = _coord(state=state)
    _lane_status(c, "pinky1", LOST, 100.0, "관제 링크 유실", seq=c.robots["pinky1"].route_seq)
    assert c.robots["pinky1"].held and c.robots["pinky1"].held_reason == FC.HELD_LINK_LOST
    return c


def _tick(c, t):
    c._t = t
    _reset(c)
    c._publish_lane_commands()
    return [m.command for m in _sent(c.lane_cmd_pubs["pinky1"])], [m.command for m in _sent(c.fleet_cmd_pubs["pinky1"])]


def test_재개가_안_닿아_로봇이_아직_링크유실을_말하면_1초마다_다시_보낸다__풀리면_멈춘다():
    c = _held_lost()
    seq = c.robots["pinky1"].route_seq
    assert c.resume_robot("pinky1")
    lane, fleet = _tick(c, 100.5)                               # 1 s 안 — 아직 안 보낸다
    assert LaneCommand.CMD_RESUME not in lane
    _lane_status(c, "pinky1", LOST, 100.9, "관제 링크 유실", seq=seq)   # 로봇은 아직 래치(재개를 못 받았다)
    lane, fleet = _tick(c, 101.1)
    assert LaneCommand.CMD_RESUME in lane and FleetCommand.CMD_RESUME in fleet
    _lane_status(c, "pinky1", LOST, 103.5, "관제 링크 유실", seq=seq)   # 유예(3 s)를 넘겨도 재전송이 유예를 늘린다
    assert not c.robots["pinky1"].held
    lane, _ = _tick(c, 103.6)
    assert LaneCommand.CMD_RESUME in lane
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 103.8, "RESUME", seq=seq)   # 닿았다 — 풀렸다고 말한다
    lane, fleet = _tick(c, 105.0)
    assert LaneCommand.CMD_RESUME not in lane and FleetCommand.CMD_RESUME not in fleet
    assert c.robots["pinky1"].resume_retry_until is None


def test_창이_지나도_래치면_더_보내지_않고_유예_뒤_다시_세운다():
    c = _held_lost()
    seq = c.robots["pinky1"].route_seq
    c.resume_robot("pinky1")
    t = 100.0
    while t < 100.0 + c.RESUME_RETRY_SEC - 0.1:
        t += 0.5
        _lane_status(c, "pinky1", LOST, t, "관제 링크 유실", seq=seq)
        _tick(c, t)
    last = c.robots["pinky1"].resume_last_sent
    t = 100.0 + c.RESUME_RETRY_SEC + 0.5
    lane, _ = _tick(c, t)                                       # 창 밖 — 더 안 보낸다
    assert LaneCommand.CMD_RESUME not in lane and c.robots["pinky1"].resume_retry_until is None
    _lane_status(c, "pinky1", LOST, last + c.LINK_LOST_RESUME_GRACE_SEC + 0.1, "관제 링크 유실", seq=seq)
    assert c.robots["pinky1"].held and c.robots["pinky1"].held_reason == FC.HELD_LINK_LOST    # 안 닿았다 — 다시 세운다


def test_재전송_중_운영자가_다시_세우거나_플릿_비상정지면_RESUME_을_더_안_낸다():
    c = _held_lost()
    seq = c.robots["pinky1"].route_seq
    c.resume_robot("pinky1")
    c.stop_robot("pinky1")
    _lane_status(c, "pinky1", LOST, 101.0, "관제 링크 유실", seq=seq)
    lane, fleet = _tick(c, 101.5)
    assert LaneCommand.CMD_RESUME not in lane and FleetCommand.CMD_RESUME not in fleet
    c = _held_lost()
    c.resume_robot("pinky1")
    c.estop_fleet()
    lane, fleet = _tick(c, 101.5)
    assert LaneCommand.CMD_RESUME not in lane and FleetCommand.CMD_RESUME not in fleet


def test_플릿_정지_중_재전송도_허가_0_다음_RESUME_다음_STOP_이다__F1_순서():
    c = _held_lost(state="STOPPED")
    seq = c.robots["pinky1"].route_seq
    c.resume_robot("pinky1")
    _lane_status(c, "pinky1", LOST, 100.9, "관제 링크 유실", seq=seq)
    c._t = 101.2
    _reset(c)
    c._retry_resumes(101.2)
    msgs = _sent(c.lane_cmd_pubs["pinky1"])
    cmds = [m.command for m in msgs]
    assert cmds[:3] == [LaneCommand.CMD_CLEARANCE, LaneCommand.CMD_RESUME, LaneCommand.CMD_STOP]
    assert msgs[0].clear_until_idx == 0
    assert FleetCommand.CMD_RESUME not in [m.command for m in _sent(c.fleet_cmd_pubs["pinky1"])]


def test_재기동_직후_아직_보고가_없으면_풀린_것으로_보지_않고_계속_보낸다():
    """최종 종단 모의(2회차): 새 코디네이터가 아직 아무 보고도 못 받은 채 누른 재개 — 보고 없음을 '래치 아님' 으로 보고 첫
    틱에 재전송을 멈췄다. 로봇 보고가 들어오자 유예가 지나 다시 세웠다(두 로봇 다, ②③ 409, START 무의미)."""
    c = _coord(state="ASSIGNED")
    ctx = c.robots["pinky1"]
    ctx.held, ctx.held_reason = True, FC.HELD_LINK_LOST          # 재기동 뒤 파일로 되살린 링크유실 래치
    ctx.lane_status, ctx.lane_status_time = None, 0.0             # 새 코디네이터 — 아직 보고 없음
    c._t = 100.0
    assert c.resume_robot("pinky1")
    for t in (101.1, 102.2):
        lane, fleet = _tick(c, t)
        assert LaneCommand.CMD_RESUME in lane and FleetCommand.CMD_RESUME in fleet, t
    _lane_status(c, "pinky1", LOST, 102.5, "관제 링크 유실", seq=ctx.route_seq)   # 탐색이 끝나 들어온 첫 보고 — 아직 래치
    lane, _ = _tick(c, 103.3)
    assert LaneCommand.CMD_RESUME in lane and not ctx.held
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 103.5, "RESUME", seq=ctx.route_seq)   # 닿았다
    lane, _ = _tick(c, 104.6)
    assert LaneCommand.CMD_RESUME not in lane and ctx.resume_retry_until is None and not ctx.held


def test_재개_전에_받은_옛_보고는_풀렸다는_증거가_아니다():
    c = _coord(state="ASSIGNED")
    ctx = c.robots["pinky1"]
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 99.0, "경로 수신 — START 대기", seq=ctx.route_seq)   # 재기동 전의 옛 보고
    ctx.held, ctx.held_reason = True, FC.HELD_LINK_LOST
    c._t = 100.0
    c.resume_robot("pinky1")
    lane, _ = _tick(c, 101.1)
    assert LaneCommand.CMD_RESUME in lane                          # 옛 IDLE 로 재전송을 멈추지 않는다
