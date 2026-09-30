# -*- coding: utf-8 -*-
"""hybrid_agent_node 해제 문·남의 goal·데드맨 규칙 (A-1~A-13) — 에이전트의 **진짜 메서드**를 잰다.

rkd1rjs2/robot_mini_project_pinky 의 relay_station/tests/test_review_0926_agent.py 를 옮겼다(import 만 바꿨다).
제3자 검수 REVIEW_20260926_THIRD_PARTY_S1_S7 §3 (A-1~A-13) · §5.2 생존 변이가 출처다. ROS 환경이 없으면 skip.

DDS 없이 돈다(`test_hybrid_agent_review._agent` 의 가짜 객체에 진짜 메서드를 묶는다). DDS 위 탐침은
`test_hybrid_agent_loopback.py` 의 `--scenario-gate0926`·`--scenario-foreign0926` 이 옮겨 잰다.

- A-1~A-4·A-12 · 해제 문 하나: RESUME·START·Fleet RESUME·링크 회복 전부 '스탬프 취소 → 확인 뒤 연다'.
  시간 초과·ERROR_REJECTED·rclpy 식 거부(ERROR_NONE + 빈 목록)·서비스 없음이면 래치를 쥔 채 1 Hz 로 다시 취소한다.
- A-5 · 스탬프 뒤 ~ 열기 전에 수락된 goal 도 취소한다. 우리 goal id 를 기억하고, 플릿 통제 중이면 남의 goal 만 취소한다.
- A-6 · navigate_through_poses · follow_waypoints 도 같은 규칙.
- A-7 · odom 이 1 s 없으면 데드맨에는 움직임. `_link_lost` 항 하나로만 걸리는 경우(M04).
- A-8 · 정지·래치 중 GOTO 는 거부(받고 곧바로 취소하지 않는다).
- A-9 · Nav2 상태는 '모름' 으로 시작하고, 모르면 바쁨. 신선함은 발행자 생존으로 잰다(상태는 바뀔 때만 온다).
- §5.2 M01 · M07 · M12 · M17 생존 변이를 죽이는 시험.
- 재검(R-agent-2·3) · 문이 선 채 다시 정지했다 풀리면 문을 새로 세운다 · 문 대기 중 링크 유실 재래치 ·
  정지 중 상태를 모르면 눈먼 cancel-all.
"""
import types
from unittest.mock import MagicMock

import pytest

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_hybrid_agent_review import _HAVE  # noqa: E402

pytestmark = pytest.mark.skipif(not _HAVE, reason="ROS 환경이 없다 — source /opt/ros/jazzy/setup.bash + install/setup.bash")

if _HAVE:
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from pinky_fleet_msgs.msg import FleetCommand
    from pinky_lane_msgs.msg import LaneStatus
    from pinky_fleet_agent import hybrid_agent_node as agent_node
    from pinky_fleet_agent import route_chain as rc
    from test_hybrid_agent_review import (SEQ, WP5, _agent, _answer, _bypass, _hb, _lane, _ntp,  # noqa: E402
                                          _published_estops, _stop, _ticking, _cancel_client)

NTP, NTPS, FW = "navigate_to_pose", "navigate_through_poses", "follow_waypoints"
REASON = "해제 보류 — Nav2 취소 미확인"


def _fleet(cmd, x=0.0):
    m = FleetCommand()
    m.command = cmd
    m.x = float(x)
    return m


def _gate_futs(a, action=NTP):
    """해제 문이 낸(응답을 기다리는) 스탬프 취소 요청."""
    return [f for f in a._futs if f.cbs and f.action == action]


def _estop_then_resume(a):
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    a._estop_pub.reset_mock()
    a._on_lane_command(_lane(rc.CMD_RESUME))


def _real_cancel_goal(a):
    a._cancel_goal = types.MethodType(agent_node.PinkyAgent._cancel_goal, a)


def _diag(a):
    """진짜 `_publish_diag` 가 싣는 agent 묶음."""
    import json
    a._lc_clients = {}
    a._controller_get = MagicMock()
    a._controller_get.service_is_ready.return_value = False
    a._nav2_tol_asked = True
    a._nav2_tol_asked_at = a._seconds()
    a._goal_checker_key = "general_goal_checker.xy_goal_tolerance"
    a._fix_text = a._fix_time = a._gate_text = a._gate_time = a._estop_seen = None
    a._last_pose_time, a._pose_timeout = None, 2.0
    a._name, a._map_name, a._map_load = "pinky1", "my_map", None
    a._diag_pub = MagicMock()
    a.get_parameter = lambda _n: types.SimpleNamespace(value=False)
    for name in ("_publish_diag", "_poll_lifecycle"):
        setattr(a, name, types.MethodType(getattr(agent_node.PinkyAgent, name), a))
    a._publish_diag()
    return json.loads(a._diag_pub.publish.call_args[0][0].data)["agent"]


# ---- A-1 · 시간이 지났다고 열지 않는다 (§5.2 M07 · M17) ----------------------------------------------------

def test_A1_취소_확인이_없으면_래치를_쥔_채_사유와_진단으로_말하고_estop_true_를_계속_낸다():
    a, clk = _agent()
    tick = _ticking(a)
    _estop_then_resume(a)
    for _ in range(35):                                            # 3.5 s — 응답 없음 (하트비트는 산다)
        clk.t += 0.1
        a._on_command(_hb())
        tick()
    assert not a._link_lost
    assert False not in _published_estops(a)
    assert _published_estops(a).count(True) >= 3, "문이 /estop false 를 쥔 동안은 래치다 — true 를 다시 내야 한다"
    last = a._lane_status_pub.publish.call_args[0][0]
    assert last.state_reason == REASON
    assert last.drive_state != LaneStatus.DRIVE_CRUISE, "goto 를 쥐고 있는데 달리는 중이라 보고했다"
    d = _diag(a)["release_hold"]
    assert d and d["unconfirmed"] == [NTP] and d["cancel_tries"] == 4 and d["reason"] == REASON
    a._send_goal_to.assert_not_called()


def test_A1_M07_응답을_기다리는_시간은_한_틱이_아니라_1초다():
    a, clk = _agent()
    tick = _ticking(a)
    _estop_then_resume(a)
    for _ in range(5):                                             # 0.5 s
        clk.t += 0.1
        tick()
    assert len(_gate_futs(a)) == 1, "응답을 한 틱도 안 기다리고 다시 취소했다"


# ---- A-2 · 응답 내용을 읽는다 --------------------------------------------------------------------------

def test_A2_ERROR_REJECTED_면_열지_않는다():
    a, _ = _agent()
    _estop_then_resume(a)
    _answer(a, rc=1, canceling=[])
    assert a._release_pending is not None and _published_estops(a) == []


def test_A2_rclpy_식_거부__ERROR_NONE_인데_활성_goal_이_goals_canceling_에_없으면_열지_않는다():
    """부록 B 탐침의 REJECT=1 서버는 rclpy 다 — 거부해도 return_code 0 · 빈 목록을 준다(실측). 상태로 대조한다."""
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    _bypass(a)                                                     # 래치 중 우회 goal — Nav2 가 활성이라 보고
    a._estop_pub.reset_mock()
    a._on_lane_command(_lane(rc.CMD_RESUME))
    _answer(a, rc=0, canceling=[])
    assert _published_estops(a) == [], "거부된 우회 goal 이 살아 있는데 게이트를 열었다"
    a._nav_goals[NTP] = {}                                         # 그 goal 이 끝났다는 상태 보고
    a._try_open()
    assert _published_estops(a) == [False]


def test_A2_ERROR_UNKNOWN_GOAL_ID_와_ERROR_GOAL_TERMINATED_는_확인이다():
    for code in (2, 3):
        a, _ = _agent()
        _estop_then_resume(a)
        _answer(a, rc=code, canceling=[])
        assert _published_estops(a) == [False], code


def test_A2_응답을_못_읽으면_확인이_아니다():
    a, _ = _agent()
    _estop_then_resume(a)
    (f,) = _gate_futs(a)
    f.result = MagicMock(side_effect=RuntimeError("service gone"))
    f.cbs[0](f)
    assert a._release_pending is not None and _published_estops(a) == []


def test_A2_상태를_모르면_응답_코드를_믿는다__실물_rclcpp_는_거부를_REJECTED_로_준다():
    a, _ = _agent()
    a._nav_goals[NTP] = None
    _estop_then_resume(a)
    _answer(a, rc=0, canceling=[])
    assert _published_estops(a) == [False]


# ---- A-3 · 취소 서비스가 없을 때 --------------------------------------------------------------------------

def test_A3_서비스가_없고_활성_goal_이_보이면_열지_않고_아무것도_못_보낸다():
    a, clk = _agent()
    tick = _ticking(a)
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    _bypass(a)
    _ntp(a).service_is_ready.return_value = False
    a._estop_pub.reset_mock()
    a._on_lane_command(_lane(rc.CMD_RESUME))
    for _ in range(15):
        clk.t += 0.1
        tick()
    assert False not in _published_estops(a) and a._release_pending is not None
    _ntp(a).service_is_ready.return_value = True                  # 서비스가 돌아오면 다음 1 Hz 재시도가 낸다
    for _ in range(10):
        clk.t += 0.1
        tick()
    assert _gate_futs(a), "서비스가 돌아왔는데 다시 취소하지 않았다"
    _answer(a)
    assert _published_estops(a)[-1] is False


def test_A3_서비스가_없고_상태도_모르면_열지_않는다():
    a, clk = _agent()
    tick = _ticking(a)
    a.count_publishers = lambda _t: 0                              # Nav2 가 없다 — 상태 발행자도 없다
    a._nav_goals[NTP] = None
    _ntp(a).service_is_ready.return_value = False
    _estop_then_resume(a)
    for _ in range(30):
        clk.t += 0.1
        a._on_command(_hb())                                       # 링크는 산다 — 링크 유실 래치 때문에 닫힌 것이 아니게
        tick()
    assert not a._halted_core()
    assert False not in _published_estops(a) and a._release_pending is not None


@pytest.mark.parametrize("known", [True, False])
def test_A3_취소를_보낸_뒤_서비스가_사라지면_상태가_안다_활성_없음일_때만_연다(known):
    """답이 올 수 없는 요청을 기다리며 영영 서 있지 않되(보조 액션 서버가 죽은 경우), 모르면 열지 않는다."""
    a, clk = _agent()
    tick = _ticking(a)
    _estop_then_resume(a)
    assert len(_gate_futs(a)) == 1
    _ntp(a).service_is_ready.return_value = False                 # 답하기 전에 서버가 사라졌다
    a._nav_goals[NTP] = {} if known else None
    if not known:
        a.count_publishers = lambda _t: 0                          # 기본 액션 발행자도 없다 — 틱이 '모름' 으로 둔다
    for _ in range(3):
        clk.t += 0.1
        tick()
    assert (_published_estops(a) == [False]) is known, _published_estops(a)
    assert (a._release_pending is None) is known


def test_A3_서비스가_없어도_상태가_안다_활성_없음이면_연다():
    a, _ = _agent()
    _ntp(a).service_is_ready.return_value = False
    _estop_then_resume(a)
    assert _published_estops(a) == [False]
    assert not a._futs


# ---- A-4 · Fleet RESUME 도 같은 문 — 어느 RESUME 이 먼저 와도 -------------------------------------------------

def _held_running(a):
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 4))
    a._on_command(_hb())
    for _ in range(3):
        a._on_command(_stop())                                     # resume_robot 전: 로봇별 HOLD (Lane·Fleet STOP)
        a._on_lane_command(_lane(rc.CMD_STOP, 4))
    a._send_goal_to.reset_mock()


@pytest.mark.parametrize("first", ["fleet", "lane"])
def test_A4_resume_robot_의_두_RESUME_은_순서와_무관하게_스탬프_취소_확인_뒤에_연다(first):
    a, _ = _agent()
    _held_running(a)
    _bypass(a)                                                     # HOLD 중 들어온 우회 goal
    fleet, lane = (lambda: a._on_command(_fleet(FleetCommand.CMD_RESUME))), (lambda: a._on_lane_command(_lane(rc.CMD_RESUME)))
    for step in ((fleet, lane) if first == "fleet" else (lane, fleet)):
        step()
    assert len(_gate_futs(a)) == 1, "문 없이 풀렸거나 문이 둘이다"
    req = _gate_futs(a)[0].req
    assert req.goal_info.stamp.sec or req.goal_info.stamp.nanosec
    a._send_goal_to.assert_not_called()
    _answer(a)
    assert a._send_goal_to.called and a._goal_tag == ("chain", 4)


def test_A4_Fleet_RESUME_의_단일_목표_재개도_문_뒤에_나간다():
    a, _ = _agent()
    a._send_goal = MagicMock()
    a._on_command(_hb())
    a._on_command(_fleet(FleetCommand.CMD_GOTO, 1.0))
    a._send_goal.reset_mock()
    a._on_command(_stop())
    a._on_command(_fleet(FleetCommand.CMD_RESUME))
    a._send_goal.assert_not_called()
    _answer(a)
    a._send_goal.assert_called_once()


def test_A4_문이_열리기_전에_다시_STOP_이면_단일_목표를_안_낸다():
    a, _ = _agent()
    a._send_goal = MagicMock()
    a._on_command(_hb())
    a._on_command(_fleet(FleetCommand.CMD_GOTO, 1.0))
    a._on_command(_stop())
    a._on_command(_fleet(FleetCommand.CMD_RESUME))
    a._on_command(_stop())
    a._send_goal.reset_mock()
    _answer(a)
    a._send_goal.assert_not_called()


# ---- A-12 · 링크 회복도 문을 지난다 -----------------------------------------------------------------------

def test_A12_링크_회복은_문을_세우고_그_사이_우회_goal_을_취소한다():
    a, clk = _agent()
    a._on_command(_hb())
    clk.t += 5.0
    a._odom_at = clk.t                                             # 멈춰 있고 odom 도 신선 — 래치는 안 걸린다(설계)
    a._check_link()
    assert a._link_lost and not a._chain.latched
    a._on_command(_hb())                                           # 회복
    assert a._release_pending is not None and len(_gate_futs(a)) == 1
    _bypass(a)                                                     # 마지막 틱 뒤 ~ 회복 사이에 수락된 우회 goal
    a._cancel_goal.reset_mock()
    a._cancel_foreign_goals()
    assert any(c.kwargs.get("all_goals") for c in a._cancel_goal.call_args_list), "문이 확인되기 전인데 우회 goal 을 뒀다"
    _answer(a)
    assert a._release_pending is None and not a._halted()


def test_A12_LaneCommand_로_회복해도_같은_문():
    a, clk = _agent()
    a._on_command(_hb())
    clk.t += 5.0
    a._odom_at = clk.t
    a._check_link()
    a._on_lane_command(_lane(rc.CMD_HEARTBEAT))
    assert a._release_pending is not None and len(_gate_futs(a)) == 1


# ---- A-5 · 스탬프 뒤 ~ 열기 전 · 우리 goal 과 남의 goal ------------------------------------------------------------

def test_A5_문이_열리기_전_스탬프_뒤에_수락된_goal_도_취소한다():
    a, _ = _agent()
    _estop_then_resume(a)
    a._nav_goals[NTP] = {b"L" * 16: 2 ** 62}                       # 스탬프 **뒤**에 수락된 우회 goal (상태가 먼저 왔다)
    a._cancel_goal.reset_mock()
    a._cancel_foreign_goals()
    assert any(c.kwargs.get("all_goals") for c in a._cancel_goal.call_args_list), \
        "체인은 풀렸지만 문이 안 열렸다 — 그 사이 goal 은 남의 것이다"


def test_A5_문을_열_때_지금_스탬프로_한_번_더_취소하고_그_뒤에_goto():
    a, clk = _agent()
    _estop_then_resume(a)
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 4))                 # 문이 선 동안 온 허가 — goto 는 문 뒤에 쌓인다
    a._send_goal_to.assert_not_called()
    order = []
    a._send_goal_to.side_effect = lambda x, y, yaw, tag: order.append(("goto", tag))
    real_call = _ntp(a).call_async.side_effect
    _ntp(a).call_async.side_effect = lambda req: (order.append(("cancel", req.goal_info.stamp.sec)), real_call(req))[1]
    clk.t += 0.3
    _answer(a)
    assert order[0] == ("cancel", int(clk.t)), order              # 문을 여는 시각의 스탬프 취소가 goto 보다 먼저
    assert ("goto", ("chain", 4)) in order[1:]


def test_A5_플릿_통제_중이면_남의_goal_만_id_로_취소하고_우리_goal_은_둔다():
    a, clk = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    own, other = b"O" * 16, b"X" * 16
    a._own_goals.append(own)
    a._nav_goals[NTP] = {own: 1, other: 2}
    a._cancel_foreign_goals()
    reqs = [c[0][0] for c in _ntp(a).call_async.call_args_list]
    assert [bytes(r.goal_info.goal_id.uuid) for r in reqs] == [other]
    a._cancel_goal.assert_not_called()                             # cancel-all 이면 우리 goal 도 죽는다
    clk.t += 0.1
    a._cancel_foreign_goals()                                      # 같은 goal 의 재취소는 1 Hz
    assert _ntp(a).call_async.call_count == 1
    clk.t += 1.0
    a._cancel_foreign_goals()
    assert _ntp(a).call_async.call_count == 2


def test_A5_체인이_쉬면_사람의_수동_goal_은_둔다():
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)                                 # 경로만 받음 — START 전
    _bypass(a)
    a._cancel_foreign_goals()
    a._cancel_goal.assert_not_called()
    assert not _ntp(a).call_async.called


def test_A5_우리가_낸_goal_의_id_를_Nav2_에_실어_보내고_기억한다():
    a, _ = _agent()
    a._nav_client = MagicMock()
    a._nav_client.server_is_ready.return_value = True
    a._global_frame = "map"
    a._send_goal_to_real(1.0, 0.0, 0.0, ("single", None))
    sent = bytes(a._nav_client.send_goal_async.call_args.kwargs["goal_uuid"].uuid)
    assert sent in a._own_goals and len(sent) == 16


# ---- A-6 · through_poses · waypoints ---------------------------------------------------------------------

def test_A6_정지_전이의_cancel_all_은_세_액션_전부에():
    a, _ = _agent()
    for x in (NTPS, FW):
        a._nav_cancel[x] = _cancel_client(a, x, ready=True)
    _real_cancel_goal(a)
    a._goal_handle = None
    a._on_command(_stop())
    for x in (NTP, NTPS, FW):
        (req,) = [c[0][0] for c in a._nav_cancel[x].call_async.call_args_list]
        assert not req.goal_info.stamp.sec and not any(req.goal_info.goal_id.uuid), x   # 빈 goal_info = 전부


def test_A6_정지_중_waypoint_follower_goal_도_감시_취소한다():
    a, _ = _agent()
    a._on_command(_stop())
    a._cancel_goal.reset_mock()
    _bypass(a, action=FW)
    a._cancel_foreign_goals()
    assert any(c.kwargs.get("all_goals") for c in a._cancel_goal.call_args_list)


def test_A6_해제_문은_서비스가_있는_액션마다_확인을_기다린다():
    a, _ = _agent()
    a._nav_cancel[FW] = _cancel_client(a, FW, ready=True)
    _estop_then_resume(a)
    assert len(_gate_futs(a, FW)) == 1
    _answer(a)                                                     # navigate_to_pose 만 답했다
    assert a._release_pending is not None and _published_estops(a) == []
    _answer(a, action=FW)
    assert _published_estops(a) == [False]


def test_A6_플릿_통제_중_남의_waypoint_goal_은_id_로_취소():
    a, _ = _agent()
    a._nav_cancel[FW] = _cancel_client(a, FW, ready=True)
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._nav_goals[FW] = {b"W" * 16: 1}
    a._cancel_foreign_goals()
    (req,) = [c[0][0] for c in a._nav_cancel[FW].call_async.call_args_list]
    assert bytes(req.goal_info.goal_id.uuid) == b"W" * 16


# ---- A-7 · odom 만료 · §5.2 M04 · M12 --------------------------------------------------------------------

def _link_lost_idle(a, clk, odom_fresh=True, lin=0.0, ang=0.0):
    a._on_command(_hb())
    clk.t += 5.0
    a._linear_velocity, a._angular_velocity = lin, ang
    if odom_fresh:
        a._odom_at = clk.t
    a._check_link()


def test_A7_odom_이_1초_넘게_없으면_데드맨은_움직이는_중으로_본다():
    a, clk = _agent()
    _link_lost_idle(a, clk, odom_fresh=False)                      # 마지막 odom 5 s 전, 속도 0
    assert a._chain.link_lost and True in _published_estops(a)


def test_A7_odom_을_한_번도_못_받았어도_움직이는_중():
    a, clk = _agent()
    a._odom_at = None
    _link_lost_idle(a, clk, odom_fresh=False)
    assert a._chain.link_lost


def test_A7_대조__odom_이_신선하고_멈춰_있으면_래치하지_않는다():
    a, clk = _agent()
    _link_lost_idle(a, clk)
    assert a._link_lost and not a._chain.link_lost and True not in _published_estops(a)


def test_A7_odom_은_1초까지는_신선하다():
    a, clk = _agent()
    a._on_command(_hb())
    clk.t += 5.0
    a._odom_at = clk.t - 0.9
    a._check_link()
    assert not a._chain.link_lost


def test_A7_M04_래치_없는_링크_유실_중_우회_goal_은_link_lost_항_하나로_취소된다():
    """멈춰 있던 로봇은 링크가 끊겨도 래치하지 않는다(route_chain 설계) — 그때 들어온 우회 goal 을 세우는 것은
    `_halted()` 의 `_link_lost` 항뿐이다. 그 항을 지키는 시험이 없었다(§5.2 M04 생존)."""
    a, clk = _agent()
    _link_lost_idle(a, clk)
    assert not a._chain.latched and not a._chain.stopped and not a._hold
    a._cancel_goal.reset_mock()
    _bypass(a)
    a._cancel_foreign_goals()
    assert any(c.kwargs.get("all_goals") for c in a._cancel_goal.call_args_list)


def test_A7_M12_제자리_회전도_움직임이다():
    a, clk = _agent()
    _link_lost_idle(a, clk, ang=0.3)
    assert a._chain.link_lost


# ---- A-8 · 정지·래치 중 GOTO 는 거부 ------------------------------------------------------------------------

def test_A8_STOP_뒤_GOTO_는_받아서_취소하지_않고_거부한다():
    a, _ = _agent()
    a._send_goal = MagicMock()
    a._on_command(_hb())
    a._on_command(_stop())
    a._cancel_goal.reset_mock()
    a._on_command(_fleet(FleetCommand.CMD_GOTO, 4.4))
    a._send_goal.assert_not_called()
    a._cancel_goal.assert_not_called()
    assert a._goal_valid is False and a._chain.stopped
    assert a._refused["command"] == "GOTO" and a._refused["reason"] == "정지·래치 중"
    assert _diag(a)["refused"]["command"] == "GOTO"


def test_A8_해제_문_대기_중_GOTO_도_거부하고_이유가_문이다():
    a, _ = _agent()
    a._send_goal = MagicMock()
    a._on_command(_hb())
    a._on_command(_stop())
    a._on_command(_fleet(FleetCommand.CMD_RESUME))
    a._on_command(_fleet(FleetCommand.CMD_GOTO, 4.4))
    a._send_goal.assert_not_called()
    assert a._refused["reason"] == REASON


def test_A8_대조__정지가_아니면_GOTO_를_보낸다():
    a, _ = _agent()
    a._send_goal = MagicMock()
    a._on_command(_hb())
    a._on_command(_fleet(FleetCommand.CMD_GOTO, 4.4))
    a._send_goal.assert_called_once()
    assert a._refused is None


# ---- A-9 · 모름으로 시작 · 모르면 바쁨 · 발행자 생존 ----------------------------------------------------------

def test_A9_클래스_기본은_모름이다__init_을_건너뛴_시험용_에이전트도():
    assert agent_node.PinkyAgent._nav_goals is None
    a = agent_node.PinkyAgent.__new__(agent_node.PinkyAgent)
    assert agent_node.PinkyAgent._nav2_busy(a) is True


def test_A9_상태를_모르면_데드맨은_움직이는_중이고_정지_확인_사유에_표식이_붙는다():
    a, clk = _agent()
    a._nav_goals[NTP] = None
    _link_lost_idle(a, clk)                                        # odom 신선·0 — Nav2 모름 하나로
    assert a._chain.link_lost
    b, _ = _agent()
    b._lane_status_pub = MagicMock()
    b._global_frame, b._name = "map", "pinky1"
    b._chain.on_route(SEQ, WP5, 4)
    b._on_command(_stop())
    b._nav_goals[NTP] = None
    b._publish_lane_status()
    assert b._lane_status_pub.publish.call_args[0][0].state_reason == "FleetCommand STOP · Nav2 상태 모름"


def test_A9_M01_ACCEPTED_도_활성이다():
    a, _ = _agent()
    m = GoalStatusArray()
    for status, byte in ((GoalStatus.STATUS_ACCEPTED, 1), (GoalStatus.STATUS_EXECUTING, 2),
                         (GoalStatus.STATUS_CANCELING, 3), (GoalStatus.STATUS_SUCCEEDED, 4)):
        st = GoalStatus()
        st.status = status
        st.goal_info.goal_id.uuid = [byte] * 16
        st.goal_info.stamp.sec = 7
        m.status_list.append(st)
    a._on_nav_status(NTP, m)
    assert a._nav_goals[NTP] == {bytes([1]) * 16: 7_000_000_000, bytes([2]) * 16: 7_000_000_000}
    assert a._nav_rx_at[NTP] == a._seconds()


def test_A9_상태_보고가_문을_연다__취소_서비스가_없던_경우():
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    _bypass(a)
    _ntp(a).service_is_ready.return_value = False
    a._estop_pub.reset_mock()
    a._on_lane_command(_lane(rc.CMD_RESUME))
    assert _published_estops(a) == []
    a._on_nav_status(NTP, GoalStatusArray())                       # Nav2: 이제 활성 없음
    assert _published_estops(a) == [False]


def test_A9_발행자가_2초_넘게_보였는데_아무것도_안_왔으면_goal_이_없었다():
    a, clk = _agent()
    a._nav_goals = {x: None for x in agent_node.NAV_ACTIONS}
    a._nav_pub_since = {x: None for x in agent_node.NAV_ACTIONS}
    a._poll_nav_graph()
    clk.t += 1.9
    a._poll_nav_graph()
    assert a._nav_goals[NTP] is None and a._nav2_busy()
    clk.t += 0.2
    a._poll_nav_graph()
    assert a._nav_goals[NTP] == {} and not a._nav2_busy()


def test_A9_틱이_발행자_그래프를_본다__기본_액션은_2초_뒤_안다__보조_액션은_서버가_없으면_goal_없음():
    a, clk = _agent()
    tick = _ticking(a)
    a._nav_goals = {x: None for x in agent_node.NAV_ACTIONS}
    a._nav_pub_since = {x: None for x in agent_node.NAV_ACTIONS}
    a.count_publishers = lambda t: 1 if t.startswith(NTP) else 0  # navigate_to_pose 서버만 떠 있다
    clk.t += 0.1
    a._on_command(_hb())
    tick()
    assert a._nav_goals[NTP] is None and a._nav_goals[NTPS] == {} and a._nav_goals[FW] == {}
    for _ in range(21):                                            # 2.1 s — 발행자가 보였는데 상태가 안 왔다
        clk.t += 0.1
        a._on_command(_hb())
        tick()
    assert a._nav_goals[NTP] == {} and not a._nav2_busy()


def test_A9_발행자가_사라지면_기본_액션은_모름__보조_액션은_goal_없음():
    a, clk = _agent()
    a._nav_goals[NTP] = {b"A" * 16: 0}
    a._nav_goals[FW] = {b"B" * 16: 0}
    a.count_publishers = lambda _t: 0
    clk.t += 5.0
    a._poll_nav_graph()
    assert a._nav_goals[NTP] is None and a._nav_goals[FW] == {} and a._nav_goals[NTPS] == {}
    assert a._nav_pub_since[NTP] is None


def test_A9_방금_받은_상태는_그래프_캐시가_늦어도_지우지_않는다():
    a, clk = _agent()
    a._nav_goals[FW] = {b"B" * 16: 0}
    a._nav_rx_at[FW] = clk.t
    a.count_publishers = lambda _t: 0
    clk.t += 0.5
    a._poll_nav_graph()
    assert a._nav_goals[FW] == {b"B" * 16: 0}, "보조 액션의 활성 goal 을 그래프 지연 때문에 '없음' 으로 지웠다"


def test_A9_진단은_액션별로_아는지와_받은_지_몇_초인지를_싣는다():
    a, clk = _agent()
    a._nav_goals[NTP] = None
    a._nav_goals[FW] = {b"B" * 16: 0}
    a._nav_rx_at[FW] = clk.t - 1.5
    a._odom_at = clk.t - 0.25
    a._nav_pub_since[NTP] = None                                   # 기본 액션: 발행자 없음
    d = _diag(a)
    assert d["nav2_goals"][NTP] == {"known": False, "active": None, "publisher": False, "status_age_s": None}
    assert d["nav2_goals"][FW] == {"known": True, "active": 1, "publisher": True, "status_age_s": 1.5}
    assert d["odom_age_s"] == 0.25 and d["release_hold"] is None


def test_A9_지도_교체는_아는_활성만_막는다__모름은_정지·데드맨_규칙():
    a, _ = _agent()
    a._nav_goals[NTP] = None
    a._goal_handle = None
    a._map_dir, a._map_name, a._map_load = "/nonexistent", "my_map", None
    a._load_map_client = MagicMock()
    a._set_map = types.MethodType(agent_node.PinkyAgent._set_map, a)
    a._set_map("map4")
    assert a._map_load["result"] == "NO_FILE"                      # 거부(REFUSED)가 아니라 파일 검사까지 갔다
    _bypass(a, action=FW)
    a._set_map("map4")
    assert a._map_load["result"] == "REFUSED"


# ---- 문이 선 동안의 보고 · 재래치 -------------------------------------------------------------------------

def test_문_대기_중_CRUISE_는_대기로_보고한다__goto_를_아직_안_냈다():
    a, _ = _agent()
    _estop_then_resume(a)
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 4))
    assert a._chain.drive_state() == rc.DRIVE_CRUISE
    assert a._drive_state() == rc.DRIVE_WAIT_CLEARANCE
    _answer(a)
    assert a._drive_state() == rc.DRIVE_CRUISE


def test_문_대기_중_다시_STOP_이면_정지_사유를_그대로_보고한다__정지_확인이_막히지_않게():
    a, _ = _agent()
    a._lane_status_pub = MagicMock()
    a._global_frame, a._name = "map", "pinky1"
    _estop_then_resume(a)
    a._on_command(_stop())
    a._publish_lane_status()
    assert a._lane_status_pub.publish.call_args[0][0].state_reason == "FleetCommand STOP"


def test_문_대기_중_새_goto_도_문_뒤에_쌓이고_열릴_때_현재_것만_나간다():
    a, _ = _agent()
    _estop_then_resume(a)
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 2))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 4))
    a._send_goal_to.assert_not_called()
    _answer(a)
    assert [c[0][3] for c in a._send_goal_to.call_args_list] == [("chain", 4)]


# ---- 제3자 검수 재검 (R-agent-2 · R-agent-3) ---------------------------------------------------------------

def test_재검_R2_문이_선_채_다시_ESTOP_됐다_풀리면_옛_스탬프의_확인으로_열지_않는다__상태를_모를_때():
    """문 1 은 navigate_to_pose 만 확인된 채 서 있다 → 다시 ESTOP → RESUME. 예전엔 해제 2 가 문 1 에 이어 붙어 새 스탬프
    취소를 안 냈고, 상태를 모르니(대조 불가) ESTOP 전 스탬프의 확인 + follow_waypoints 응답만으로 열렸다(/estop false 두 번)."""
    a, _ = _agent()
    a._nav_cancel[FW] = _cancel_client(a, FW, ready=True)
    a._nav_goals[NTP] = None                                       # 기본 액션 상태 모름 — 취소 서비스는 산다
    _estop_then_resume(a)
    _answer(a, action=NTP)                                         # 문 1: navigate_to_pose 만 확인
    assert a._release_pending is not None
    n1 = len(_gate_futs(a))
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    a._estop_pub.reset_mock()
    a._on_lane_command(_lane(rc.CMD_RESUME))                       # 해제 2
    assert len(_gate_futs(a)) == n1 + 1, "해제 2 의 스탬프로 navigate_to_pose 를 다시 취소하지 않았다"
    _answer(a, action=FW)
    assert _published_estops(a) == [], "ESTOP 전 스탬프의 확인으로 문을 열었다"
    _answer(a, action=NTP)
    assert _published_estops(a) == [False, False]                  # 해제 1·2 가 쌓은 false — 잃지 않는다


def test_재검_R3_문_대기_중_링크_유실로_다시_래치되면_쌓인_estop_false_를_버린다():
    """ESTOP 만이 아니다 — 확인을 기다리는 사이 링크 유실 래치(/estop true)가 걸리면, 뒤늦은 확인이 /estop false 를 내면
    안 된다(`_flush_release` 의 `latched` 가 `estop` 이어도 시험이 몰랐다). 다음 RESUME 이 새 문으로 다시 푼다."""
    a, clk = _agent()
    a._on_command(_hb())
    _estop_then_resume(a)
    clk.t += 5.0
    a._odom_at = clk.t
    a._check_link()                                                # 임무 중 — 링크 유실 래치
    assert a._chain.link_lost and not a._chain.estop and _published_estops(a) == [True]
    _answer(a)                                                     # 문 1 의 늦은 확인
    assert _published_estops(a) == [True], "링크 유실 래치 중에 /estop false 를 냈다"
    a._on_command(_hb())                                           # 링크 회복 — 래치는 LaneCommand RESUME 까지 쥔다
    a._on_lane_command(_lane(rc.CMD_RESUME))
    assert _published_estops(a) == [True]
    _answer(a)
    assert _published_estops(a) == [True, False]


def test_재검_R3_정지_중_Nav2_상태를_모르면_눈먼_cancel_all_을_1Hz_로_낸다():
    """상태 구독이 안 맞아(모름) 아는 활성 goal 이 없어도, 취소 서비스는 살아 있을 수 있다 — 그때 유일한 보루다(A-9)."""
    a, clk = _agent()
    a._on_command(_stop())                                         # HOLD
    a._nav_goals[NTP] = None
    a._cancel_goal.reset_mock()
    for _ in range(25):                                            # 2.5 s
        clk.t += 0.1
        a._cancel_foreign_goals()
    n = sum(1 for c in a._cancel_goal.call_args_list if c.kwargs.get("all_goals"))
    assert n == 3, n                                               # 0 s·1 s·2 s
