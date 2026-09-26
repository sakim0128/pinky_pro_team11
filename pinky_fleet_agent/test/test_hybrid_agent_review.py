# -*- coding: utf-8 -*-
"""hybrid_agent_node.PinkyAgent 의 **실제 메서드**를 잰다 (ROS 환경에서만 돈다 — 없으면 skip).

rkd1rjs2/robot_mini_project_pinky 의 relay_station/tests/test_agent_review_fixes.py 를 옮겼다.
경로만 이 저장소에 맞췄고, 그쪽 코디네이터의 STOP_ACK_REASONS 는 값으로 옮겨 적었다.

DDS 없이 돈다: 노드를 만들지 않고, 메서드가 쓰는 속성만 가진 가짜 객체에 진짜 메서드를 묶어 부른다.
(DDS 위 배선은 `test_hybrid_agent_loopback.py` 가 잰다.)

- P1 · 미뤄 둔 goto 를 재생하기 직전에 다시 묻는다 — 그 사이 STOP 이 왔으면 보내지 않는다.
- P2 · ESTOP 래치 동안 `/estop true` 를 주기적으로 다시 낸다(false 는 되풀이하지 않는다).
- P2 · Nav2 서버 확인이 콜백을 막지 않는다(콜백이 한 줄로 서므로 막으면 STOP 이 늦는다).
- §3 · HOLD 워치독은 '갱신되지 않은 HOLD' 만 푼다 — 10 Hz STOP 을 받는 동안 20 s 마다 풀리지 않는다.
- D6-1 · controller_server 의 xy_goal_tolerance 를 체인 도달 반경에 반영한다.
"""
import os
import sys
import threading
import time
import types
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from rclpy.time import Time
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from pinky_fleet_msgs.msg import FleetCommand, RobotState
    from pinky_lane_msgs.msg import LaneCommand
    from pinky_fleet_agent import hybrid_agent_node as agent_node
    from pinky_fleet_agent import route_chain as rc
    from pinky_fleet_agent.route_chain import RouteChain
    from pinky_fleet_agent.hybrid_link_watch import LinkWatch
    _HAVE = agent_node.HAS_RCLPY
except Exception:                                   # noqa: BLE001
    _HAVE = False

# 코디네이터가 LaneStatus.state_reason 으로 'STOP 을 처리했다' 를 확인하는 문자열
# (rkd1rjs2/robot_mini_project_pinky relay_station/fleet/fleet_coordinator.py STOP_ACK_REASONS)
STOP_ACK_REASONS = ('LaneCommand STOP', 'FleetCommand STOP')

pytestmark = pytest.mark.skipif(not _HAVE, reason="ROS 환경이 없다 — source /opt/ros/jazzy/setup.bash + install/setup.bash")

WP5 = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
SEQ = 7


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _agent(clock=None):
    """PinkyAgent 의 메서드가 쓰는 것만 가진 가짜. 메서드는 진짜를 묶는다."""
    clk = clock or Clock()
    a = types.SimpleNamespace()
    a._chain = RouteChain(clock=clk)
    a._state_lock = threading.RLock()
    a._goal_tag = None
    a._deferred_acts = []
    a._estop_pub = MagicMock()
    # 진짜 _send_goal_to 처럼 마지막 goal 의 꼬리표를 남긴다 (워치독이 체인 goal 인지 본다)
    a._send_goal_to = MagicMock(side_effect=lambda x, y, yaw, tag: setattr(a, "_goal_tag", tag))
    a._cancel_goal = MagicMock()
    a._estop_refresh = 1.0
    a._estop_last_pub = None
    a._seconds = clk
    a._hold = False
    a._hold_since = None
    a._hold_watchdog = 1.0
    a._goal_valid = False
    a._goal_seq = 0
    a._goal_handle = None
    a._link_lost = False
    a._nav_status = RobotState.NAV_IDLE
    a._link = LinkWatch(3.0, 0.0)
    a._nav_active = set()
    a._foreign_cancel_at = None
    a._foreign_canceled = frozenset()
    a._release_pending = None
    a._release_deadline = None
    a._release_future = None
    a._nav_cancel_all = MagicMock()
    a._nav_cancel_all.service_is_ready.return_value = True
    a._nav_cancel_all.call_async.side_effect = lambda _req: MagicMock()     # 취소마다 새 future
    a._linear_velocity = 0.0
    a._angular_velocity = 0.0
    a._lc_states = {}
    a.get_logger = MagicMock
    a.get_clock = lambda: types.SimpleNamespace(now=lambda: Time(nanoseconds=int(clk.t * 1e9)))
    for name in ("_apply", "_refresh_estop", "_on_command", "_on_lane_command", "_check_hold_watchdog",
                 "_on_nav2_tolerance", "_send_goal_to_real", "_nav2_busy", "_halted", "_cancel_all_before",
                 "_flush_release", "_on_release_cancel_done", "_cancel_foreign_goals", "_check_link",
                 "_on_goal_response", "_on_lc_state", "_publish_lane_status", "_ask_nav2_tolerance"):
        real = getattr(agent_node.PinkyAgent, name if name != "_send_goal_to_real" else "_send_goal_to")
        setattr(a, name, types.MethodType(real, a))
    return a, clk


def _started(a):
    c = a._chain
    c.on_route(SEQ, WP5, 4)
    c.on_lane_command(rc.CMD_START, SEQ, 0)


def _published_estops(a):
    return [call[0][0].data for call in a._estop_pub.publish.call_args_list]


# ---- P1 · 미뤄 둔 goto ------------------------------------------------------------------

def test_P1_미뤄_둔_goto_는_그_사이_STOP_이_오면_재생해도_안_보낸다():
    a, clk = _agent()
    _started(a)
    (g,) = [x for x in a._chain.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3) if x[0] == "goto"]
    a._deferred_acts = [g]                                    # Nav2 부재·거부 뒤 미뤄진 재시도
    a._chain.on_stop("FleetCommand STOP")                     # 그 사이 STOP
    acts, a._deferred_acts = a._deferred_acts, []
    a._apply(acts)                                            # _publish_state 가 하는 재생
    a._send_goal_to.assert_not_called()


def test_P1_미뤄_둔_goto_는_그대로면_보낸다():
    a, _ = _agent()
    _started(a)
    (g,) = [x for x in a._chain.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3) if x[0] == "goto"]
    a._apply([g])
    a._send_goal_to.assert_called_once()
    assert a._send_goal_to.call_args[0][3] == ("chain", 3)


def test_P1_HOLD_와_ESTOP_은_미뤄_둔_재시도를_지운다():
    a, _ = _agent()
    _started(a)
    a._deferred_acts = [("goto", 3, 1.5, 0.0, 0.0)]
    a._apply([("cancel",)])
    assert a._deferred_acts == []
    a._deferred_acts = [("goto", 3, 1.5, 0.0, 0.0)]
    a._apply([("estop", True)])
    assert a._deferred_acts == []


# ---- P2 · Nav2 서버 확인이 막지 않는다 ------------------------------------------------------

def test_P2_Nav2_서버가_없으면_기다리지_않고_미루며_실패로_세지_않는다():
    a, _ = _agent()
    _started(a)
    a._nav_client = MagicMock()
    a._nav_client.server_is_ready.return_value = False
    a._goal_tag = None
    a._chain.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)       # active_target = 3
    a._send_goal_to_real(1.5, 0.0, 0.0, ("chain", 3))
    a._nav_client.wait_for_server.assert_not_called()         # 1 s 막지 않는다
    a._nav_client.send_goal_async.assert_not_called()
    assert a._chain._failures == {}                           # 서버 없음 ≠ 목표 실패
    assert a._chain.active_target is None


# ---- P2 · ESTOP 래치 재발행 ----------------------------------------------------------------

def test_P2_ESTOP_래치_동안_estop_true_를_주기적으로_다시_낸다():
    a, clk = _agent()
    _started(a)
    a._apply(a._chain.on_lane_command(rc.CMD_ESTOP, 0, 0))   # 전이: true 1회
    assert _published_estops(a) == [True]
    for _ in range(35):                                       # 3.5 s @ 10 Hz
        clk.t += 0.1
        a._refresh_estop()
    assert _published_estops(a).count(True) == 1 + 3          # 1 s 마다 한 번 — 늦게 뜬 게이트도 안다
    assert False not in _published_estops(a)


def test_P2_래치가_없으면_아무것도_다시_내지_않는다__false_를_되풀이해_남의_비상정지를_덮지_않는다():
    a, clk = _agent()
    _started(a)
    a._apply(a._chain.on_lane_command(rc.CMD_ESTOP, 0, 0))
    a._apply(a._chain.on_lane_command(rc.CMD_RESUME, SEQ, 0))
    n = len(_published_estops(a))
    for _ in range(30):
        clk.t += 0.1
        a._refresh_estop()
    assert len(_published_estops(a)) == n


# ---- §3 · HOLD 워치독 ---------------------------------------------------------------------

def _stop():
    m = FleetCommand()
    m.command = FleetCommand.CMD_STOP
    return m


def test_검수3_STOP_을_10Hz_로_받는_동안에는_워치독이_풀지_않는다():
    """예전엔 첫 STOP 부터 재서, STOPPED 상태에서 20 s(여기선 1 s)마다 풀림→재HOLD+cancel-all 을 되풀이했다."""
    a, clk = _agent()
    for _ in range(50):                                       # 5 s, 워치독 1 s
        clk.t += 0.1
        a._on_command(_stop())
        a._check_hold_watchdog()
    assert a._hold is True
    assert sum(1 for c in a._cancel_goal.call_args_list if c.kwargs.get("all_goals")) == 1   # 처음 전이 한 번만


def test_검수3_STOP_이_끊긴_채_워치독_시간이_지나면_푼다():
    a, clk = _agent()
    clk.t += 0.1
    a._on_command(_stop())
    clk.t += 1.2                                              # 갱신 없이 1.2 s
    a._check_hold_watchdog()
    assert a._hold is False


# ---- D6-1 · Nav2 허용 반경 읽기 -----------------------------------------------------------

def _param_future(ptype, value=None):
    v = ParameterValue(type=ptype)
    if value is not None:
        v.double_value = value
    fut = MagicMock()
    fut.result.return_value = types.SimpleNamespace(values=[v])
    return fut


def test_D6_1_controller_server_의_xy_goal_tolerance_를_체인에_넣는다():
    a, _ = _agent()
    a._goal_checker_key = "general_goal_checker.xy_goal_tolerance"
    a._nav2_tol_asked = True
    a._on_nav2_tolerance(_param_future(ParameterType.PARAMETER_DOUBLE, 0.15))
    assert a._chain.nav2_xy_tol == 0.15
    assert a._chain.effective_reach_tol == pytest.approx(0.15 + rc.REACH_MARGIN_M)


def test_D6_1_그_파라미터가_없으면_추측하지_않고_기본_반경():
    a, _ = _agent()
    a._goal_checker_key = "nope.xy_goal_tolerance"
    a._nav2_tol_asked = True
    a._on_nav2_tolerance(_param_future(ParameterType.PARAMETER_NOT_SET))
    assert a._chain.nav2_xy_tol is None and a._chain.effective_reach_tol == rc.REACH_TOL_M


# ---- P2 · 콜백 그룹 ------------------------------------------------------------------------

def _calls(src):
    """소스의 실제 호출 이름들 (주석·문자열은 세지 않는다)."""
    import ast
    out = []
    for n in ast.walk(ast.parse(src)):
        if isinstance(n, ast.Call):
            f = n.func
            out.append(f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", None))
    return out


def test_P2_에이전트_콜백은_한_줄로_선다__Reentrant_그룹을_쓰지_않는다():
    calls = _calls(open(agent_node.__file__, encoding="utf-8").read())
    assert "ReentrantCallbackGroup" not in calls
    assert calls.count("MutuallyExclusiveCallbackGroup") == 1
    assert "wait_for_server" not in calls                    # 한 줄 콜백 안에서 기다리면 STOP 이 늦는다
    assert "spin_until_future_complete" not in calls and "call" not in calls   # 동기 호출도 같다


# ---- 2026-09-25 직렬 적대 검토에서 확인된 것 ----------------------------------------------------

def _lane(cmd, clear=0, seq=SEQ):
    m = LaneCommand()
    m.command = cmd
    m.route_seq = seq
    m.clear_until_idx = clear
    return m


def _hb():
    m = FleetCommand()
    m.command = FleetCommand.CMD_HEARTBEAT
    return m


def test_검토P1_STOP_뒤_START_로_달리는_체인_goal_을_워치독이_취소하지_않는다():
    """stop_fleet → start_fleet: 코디네이터는 FleetCommand STOP(10 Hz) 뒤 LaneCommand START 만 보낸다(RESUME 없음).
    START 가 체인 STOP 은 풀고 단일 목표 HOLD 는 남기면, 워치독이 달리는 체인 goal 을 취소하고 체인은
    CRUISE 를 보고하며 영영 멈췄다(검토자 재현)."""
    a, clk = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 4))
    for _ in range(20):                                        # STOPPED: 2 s 동안 10 Hz STOP 두 가지
        clk.t += 0.1
        a._on_lane_command(_lane(rc.CMD_STOP, 4))
        a._on_command(_stop())
    assert a._hold and a._chain.stopped
    a._on_lane_command(_lane(rc.CMD_START, 4))                 # start_fleet
    assert not a._chain.stopped and a._hold is False          # 둘 다 풀린다
    assert a._goal_tag == ("chain", 4)
    a._cancel_goal.reset_mock()
    for _ in range(250):                                       # 25 s (워치독 1 s) — 하트비트·허가만
        clk.t += 0.1
        a._on_command(_hb())
        a._on_lane_command(_lane(rc.CMD_CLEARANCE, 4))
        a._check_hold_watchdog()
    a._cancel_goal.assert_not_called()


def test_검토P1_워치독은_체인_goal_을_건드리지_않는다():
    a, clk = _agent()
    a._hold, a._hold_since = True, a.get_clock().now()
    a._goal_tag = ("chain", 4)
    clk.t += 2.0
    a._check_hold_watchdog()
    a._cancel_goal.assert_not_called()
    assert a._hold is False


def test_검토P2_상태를_고치는_진입점은_모두_같은_잠금을_잡는다():
    """콜백 그룹은 Future done-callback(_on_goal_response·_on_goal_result)을 막지 못한다 — 잠금으로 선다."""
    for name in ("_on_route", "_on_lane_command", "_on_command", "_on_goal_response", "_on_goal_result",
                 "_on_nav2_tolerance", "_publish_state", "_publish_diag", "shutdown_navigation"):
        assert hasattr(getattr(agent_node.PinkyAgent, name), "__wrapped__"), name + " 가 잠금 밖이다"


def test_검토P2_잠금을_다른_스레드가_쥐면_done_callback_은_기다린다():
    a, _ = _agent()
    a._goal_checker_key = "general_goal_checker.xy_goal_tolerance"
    done = threading.Event()
    a._state_lock.acquire()
    t = threading.Thread(target=lambda: (a._on_nav2_tolerance(_param_future(ParameterType.PARAMETER_DOUBLE, 0.15)),
                                         done.set()))
    t.start()
    time.sleep(0.2)
    assert not done.is_set() and a._chain.nav2_xy_tol is None
    a._state_lock.release()
    t.join(2.0)
    assert done.is_set() and a._chain.nav2_xy_tol == 0.15


def test_검토P3_선언_안_된_파라미터는_빈_목록이다__다시_묻고_경고는_한_번():
    """rclcpp/rclpy 의 get_parameters 는 선언 안 된 이름에 values=[] 를 준다(goal checker 는 configure 때 선언)."""
    a, _ = _agent()
    a._goal_checker_key = "general_goal_checker.xy_goal_tolerance"
    warn = MagicMock()
    a.get_logger = lambda: types.SimpleNamespace(warn=warn, info=MagicMock())
    for _ in range(5):
        a._nav2_tol_asked = True
        fut = MagicMock()
        fut.result.return_value = types.SimpleNamespace(values=[])
        a._on_nav2_tolerance(fut)
        assert a._nav2_tol_asked is False                      # 다음 틱에 다시 묻는다
    assert warn.call_count == 1
    assert a._chain.nav2_xy_tol is None


def test_검토P2_에이전트가_내는_STOP_사유가_코디네이터의_정지_확인_증거와_같다():
    """정지 확인은 LaneStatus.state_reason 으로 'STOP 을 처리했다' 를 본다 — 두 파일의 문자열이 갈라지면 확인이 영영 안 된다."""
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_command(_stop())
    assert a._chain.status()["state_reason"] in STOP_ACK_REASONS
    b, _ = _agent()
    b._chain.on_route(SEQ, WP5, 4)
    b._on_lane_command(_lane(rc.CMD_STOP))
    assert b._chain.status()["state_reason"] in STOP_ACK_REASONS
    c, _ = _agent()                                            # START 가 푼 사유는 증거가 아니다
    c._chain.on_route(SEQ, WP5, 4)
    c._on_lane_command(_lane(rc.CMD_STOP))
    c._on_lane_command(_lane(rc.CMD_START))
    assert c._chain.status()["state_reason"] not in STOP_ACK_REASONS


# ---- 관제 검수 REVIEW_20260925 §3.2 · 에이전트를 거치지 않는 목표(S2~S4) — 진짜 메서드 ----------------------

def _bypass(a, n=1):
    """Nav2 가 에이전트 것이 아닌 goal n 개를 진행 중이라고 보고한 상태."""
    a._nav_active = {bytes([i]) * 16 for i in range(n)}


def test_S2_ESTOP_전이는_이미_STOP_중이어도_cancel_all():
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_STOP))
    a._cancel_goal.reset_mock()
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    assert any(c.kwargs.get("all_goals") for c in a._cancel_goal.call_args_list), "STOP 뒤 ESTOP 에 cancel-all 이 없다"


def test_S2_래치를_풀_때_우회_goal_이_있으면_취소가_끝난_뒤에_estop_false():
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    a._estop_pub.reset_mock()
    _bypass(a)                                                  # ESTOP 중 들어온 우회 goal
    a._on_lane_command(_lane(rc.CMD_RESUME))
    req = a._nav_cancel_all.call_async.call_args[0][0]
    assert req.goal_info.stamp.sec or req.goal_info.stamp.nanosec   # 스탬프 취소 — 뒤에 보낼 우리 goal 은 안 건드린다
    assert _published_estops(a) == [], "취소 응답 전에 /estop false 를 냈다 — 게이트가 열리며 우회 goal 로 달린다"
    a._on_release_cancel_done(a._release_future)                      # 취소 응답
    assert _published_estops(a) == [False]


def test_S2_취소를_기다리는_사이_다시_ESTOP_이면_해제를_버린다():
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    _bypass(a)
    a._on_lane_command(_lane(rc.CMD_RESUME))
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    a._estop_pub.reset_mock()
    a._on_release_cancel_done(a._release_future)
    assert False not in _published_estops(a)


def test_S2_S3_정지·래치_중_살아_있는_goal_은_1Hz_로_취소한다(tmp_path=None):
    a, clk = _agent()
    a._on_command(_stop())                                     # HOLD
    a._cancel_goal.reset_mock()
    _bypass(a)
    for _ in range(25):                                         # 2.5 s
        clk.t += 0.1
        a._cancel_foreign_goals()
    n = sum(1 for c in a._cancel_goal.call_args_list if c.kwargs.get("all_goals"))
    assert n == 3, n                                            # 0 s·1 s·2 s — 10 Hz STOP 이 못 하던 것


def test_S3_정지가_아니면_남의_goal_을_건드리지_않는다():
    a, clk = _agent()
    _bypass(a)
    for _ in range(20):
        clk.t += 0.1
        a._cancel_foreign_goals()
    a._cancel_goal.assert_not_called()


def test_S4_우회_goal_로_달리다_링크가_끊기면_래치한다():
    """데드맨이 에이전트 goal 만 보면 체인 없이 우회 goal 로 달리던 로봇은 링크가 끊겨도 안 섰다."""
    a, clk = _agent()
    a._on_command(_hb())                                       # 링크 무장
    _bypass(a)
    clk.t += 5.0                                               # command_timeout 3 s 넘게 조용
    a._check_link()
    assert a._chain.link_lost and True in _published_estops(a)
    assert any(c.kwargs.get("all_goals") for c in a._cancel_goal.call_args_list)


def test_S4_odom_이_움직여도_달리는_중이다():
    a, clk = _agent()
    a._on_command(_hb())
    a._linear_velocity = 0.12
    clk.t += 5.0
    a._check_link()
    assert a._chain.link_lost


def test_S2_Nav2_상태_보고가_오기_전이어도_해제는_취소_응답을_기다린다():
    """해제 직전에 들어온 우회 goal 은 상태 보고가 아직 안 왔을 수 있다 — 그래서 늘 스탬프 취소 뒤에 해제한다."""
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    a._estop_pub.reset_mock()
    assert not a._nav_active                                    # 상태 보고 없음
    a._on_lane_command(_lane(rc.CMD_RESUME))
    assert a._nav_cancel_all.call_async.called and _published_estops(a) == []
    a._on_release_cancel_done(a._release_future)
    assert _published_estops(a) == [False]


def test_S2_취소_응답이_1초_없으면_해제한다__Nav2_가_죽은_경우():
    a, clk = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    a._estop_pub.reset_mock()
    a._on_lane_command(_lane(rc.CMD_RESUME))
    assert a._release_pending is not None
    clk.t += 1.1
    if a._release_pending is not None and a._seconds() >= a._release_deadline:   # _publish_state 의 그 두 줄
        a._flush_release("취소 응답 1 s 없음")
    assert _published_estops(a) == [False]


# ---- 안전 묶음 직렬 검토(09-26)가 확인한 것 ------------------------------------------------------

def test_검토S_새_우회_goal_은_1Hz_상한을_기다리지_않고_바로_취소():
    a, clk = _agent()
    a._on_command(_stop())
    _bypass(a)                                                  # goal A
    clk.t += 0.1
    a._cancel_foreign_goals()
    a._cancel_goal.reset_mock()
    a._nav_active = {b"B" * 16}                                 # 0.1 s 뒤 goal B
    clk.t += 0.1
    a._cancel_foreign_goals()
    assert any(c.kwargs.get("all_goals") for c in a._cancel_goal.call_args_list), "B 가 1 s 상한에 걸려 달렸다"
    a._cancel_goal.reset_mock()
    clk.t += 0.1                                                # 같은 B 는 재취소를 1 Hz 로 묶는다
    a._cancel_foreign_goals()
    a._cancel_goal.assert_not_called()


def test_검토S_해제가_연달아_와도_앞의_estop_false_를_잃지_않는다():
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_ESTOP, seq=0))
    a._estop_pub.reset_mock()
    a._on_lane_command(_lane(rc.CMD_RESUME))                    # 해제 1 — /estop false 대기
    f1 = a._release_future
    a._on_lane_command(_lane(rc.CMD_STOP))
    a._on_lane_command(_lane(rc.CMD_START))                     # 해제 2 (STOP 래치) — 예전엔 해제 1 을 덮어썼다
    f2 = a._release_future
    assert f1 is not f2
    a._on_release_cancel_done(f1)                               # 옛 응답 — 새 해제의 취소가 아직 안 끝났다
    assert _published_estops(a) == []
    a._on_release_cancel_done(f2)
    assert _published_estops(a) == [False]


# ---- L4 · Nav2 거부는 실패로 세지 않는다 · bt_navigator 가 켜지면 포기를 지운다 --------------------------

def test_L4_Nav2_거부는_실패_3회에_세지_않는다():
    a, clk = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 3))            # goto 3 (가짜 _send_goal_to 가 꼬리표를 남긴다)
    for _ in range(5):                                         # Nav2 가 켜지는 중 — 매번 거부
        fut = MagicMock()
        fut.result.return_value = types.SimpleNamespace(accepted=False)
        a._on_goal_response(fut, a._goal_seq, ("chain", 3))
        clk.t += 1.1
        a._chain.on_pose(0.0, 0.0)
    assert a._chain._failures == {} and a._chain._blocked_target is None


def test_L4_bt_navigator_가_active_로_바뀌면_포기한_목표를_다시_보낸다():
    a, clk = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 3))
    for _ in range(3):
        clk.t += 2.0
        a._chain.on_goal_result(3, "aborted")
    assert a._chain._blocked_target == 3
    a._send_goal_to.reset_mock()
    clk.t += 2.0

    def _state(label):
        fut = MagicMock()
        fut.result.return_value = types.SimpleNamespace(current_state=types.SimpleNamespace(label=label, id=0))
        return fut
    a._on_lc_state("bt_navigator", _state("inactive"))
    a._send_goal_to.assert_not_called()
    a._on_lc_state("bt_navigator", _state("active"))
    assert a._send_goal_to.called and a._chain._blocked_target is None


# ---- 관제 검수 REVIEW_20260925 §3.3 R1 · §3.4 --------------------------------------------------

def _lane_status_stub(a):
    a._lane_status_pub = MagicMock()
    a._global_frame = "map"
    a._name = "pinky1"
    return lambda: a._lane_status_pub.publish.call_args[0][0]


def test_R1_정지_중_Nav2_goal_이_살아_있으면_사유에_표식이_붙는다():
    a, _ = _agent()
    last = _lane_status_stub(a)
    a._chain.on_route(SEQ, WP5, 4)
    a._on_command(_stop())
    a._publish_lane_status()
    assert last().state_reason == "FleetCommand STOP"
    _bypass(a)
    a._publish_lane_status()
    assert last().state_reason == "FleetCommand STOP · Nav2 활성 1"
    assert last().state_reason not in STOP_ACK_REASONS


def test_검수3_4_허용치_조회가_응답_없으면_5초_뒤_다시_묻고_경고는_한_번():
    a, clk = _agent()
    a._controller_get = MagicMock()
    a._controller_get.service_is_ready.return_value = True
    a._goal_checker_key = "general_goal_checker.xy_goal_tolerance"
    a._nav2_tol_asked, a._nav2_tol_asked_at = False, None
    warn = MagicMock()
    a.get_logger = lambda: types.SimpleNamespace(warn=warn, info=MagicMock())
    a._ask_nav2_tolerance()
    clk.t += 2.0
    a._ask_nav2_tolerance()
    assert a._controller_get.call_async.call_count == 1
    for _ in range(3):
        clk.t += 5.1
        a._ask_nav2_tolerance()
    assert a._controller_get.call_async.call_count == 4 and warn.call_count == 1


def test_검수3_4_Nav2_부재_사유가_재전송_간격_사유에_덮이지_않는다():
    clk = Clock()
    c = RouteChain(clock=clk)
    c.on_route(SEQ, WP5, 4)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    assert c.on_goal_result(3, "unavailable") == []
    assert "Nav2" in c.reason


def test_검수0924_거부_뒤_미룬_goto_는_STOP_이_오면_틱에서_버려진다__진짜_경로():
    """REVIEW_20260924 §4.1 이 요구한 '거부 → STOP → 틱' — 주입이 아니라 _on_goal_response 로 미룬 재시도를 만든다."""
    a, clk = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, 2))            # goto 2 를 보냄
    a._chain.clear_until_idx = 4                               # 응답 전 허가 확장(다음 거부 뒤 goto 4 가 미뤄지게)
    fut = MagicMock()
    fut.result.return_value = types.SimpleNamespace(accepted=False)
    a._on_goal_response(fut, a._goal_seq, ("chain", 2))       # Nav2 거부 → 미룬 재시도
    assert [x[1] for x in a._deferred_acts if x[0] == "goto"] == [4], a._deferred_acts
    a._send_goal_to.reset_mock()
    a._on_command(_stop())                                     # 그 사이 STOP
    acts, a._deferred_acts = a._deferred_acts, []              # _publish_state 의 재생 두 줄
    a._apply(acts)
    a._send_goal_to.assert_not_called()


# ---- 공개 패치 수락 (B-2) · FleetCommand 전 명령이 제자리로 간다 --------------------------------------
# 예전 패치(0a357a8)는 LaneCommand 처리부를 FleetCommand 분기 한가운데에 끼워 넣어 SET_INITIAL_POSE ·
# SET_SPEED · SET_MAP 이 경고 없이 버려졌다. 원본 저장소의 에이전트에는 SET_SPEED 분기와 else 경고가 없었다.

def _fleet(cmd, **kw):
    m = FleetCommand()
    m.command = cmd
    for k, v in kw.items():
        setattr(m, k, v)
    return m


def _with_logger(a):
    logger = MagicMock()
    a.get_logger = lambda: logger
    return logger


def _with_handlers(a):
    a._publish_initial_pose = MagicMock()
    a._apply_speed = MagicMock()
    a._set_map = MagicMock()


def test_B2_FleetCommand_4_5_6_은_각자의_처리로_간다():
    a, _ = _agent()
    _with_handlers(a)
    a._on_command(_fleet(FleetCommand.CMD_SET_INITIAL_POSE, x=1.0, y=2.0, yaw=0.5))
    a._publish_initial_pose.assert_called_once_with(1.0, 2.0, 0.5)
    a._on_command(_fleet(FleetCommand.CMD_SET_SPEED, max_linear_vel=0.1, max_angular_vel=0.8))
    a._apply_speed.assert_called_once_with(0.1, 0.8)
    a._on_command(_fleet(FleetCommand.CMD_SET_MAP, map_name="map4"))
    a._set_map.assert_called_once_with("map4")


def test_B2_알_수_없는_FleetCommand_는_경고한다():
    a, _ = _agent()
    logger = _with_logger(a)
    a._on_command(_fleet(99))
    assert any("99" in str(c) for c in logger.warn.call_args_list), logger.warn.call_args_list


def test_B2_LaneCommand_HEARTBEAT_START_는_경고가_없다():
    a, _ = _agent()
    logger = _with_logger(a)
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_HEARTBEAT))
    a._on_lane_command(_lane(rc.CMD_START))
    logger.warn.assert_not_called()


def test_B2_음수_clearance_는_예외_없이_대기다():
    a, _ = _agent()
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    a._on_lane_command(_lane(rc.CMD_CLEARANCE, -1))
    a._send_goal_to.assert_not_called()
    assert a._chain.drive_state() == rc.DRIVE_WAIT_CLEARANCE


def test_LaneCommand_SET_SPEED_는_속도_상한만_바꾼다():
    a, _ = _agent()
    _with_handlers(a)
    a._chain.on_route(SEQ, WP5, 4)
    a._on_lane_command(_lane(rc.CMD_START))
    before = a._chain.drive_state()
    m = _lane(rc.CMD_SET_SPEED)
    m.max_linear_vel = 0.12
    m.max_angular_vel = 1.0
    a._on_lane_command(m)
    assert a._apply_speed.call_args[0] == pytest.approx((0.12, 1.0))
    assert a._chain.drive_state() == before


def test_복구_직후에도_STOP_은_받고_GOTO_는_거른다():
    """RELIABLE QoS 재전송 방어는 이동 명령에만 건다 — 멈추라는 명령을 버리면 안 된다."""
    a, clk = _agent()
    a._link = LinkWatch(3.0, 1.0)
    a._on_command(_fleet(FleetCommand.CMD_HEARTBEAT))          # 무장
    clk.t += 4.0
    a._link.poll(clk.t)                                        # LOST
    clk.t += 0.1
    a._on_command(_fleet(FleetCommand.CMD_HEARTBEAT))          # 복구 — 1 s 은혜 기간 시작
    a._on_command(_fleet(FleetCommand.CMD_GOTO, x=1.0, y=0.0))
    a._send_goal_to.assert_not_called()
    a._on_command(_stop())
    assert a._hold is True
