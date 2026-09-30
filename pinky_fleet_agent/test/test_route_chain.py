# -*- coding: utf-8 -*-
"""Nav2 로봇용 레인 규약 (`pinky_fleet_agent/route_chain.py`) — ROS 없이 돈다.

rkd1rjs2/robot_mini_project_pinky 의 relay_station/tests/test_route_chain.py 를 옮겼다
(경로만 이 저장소에 맞췄고, 그쪽 사본 해시를 대사하던 시험 하나는 뺐다 — 여기서는 route_follower.py 가 원본이다).
수락 조건: "5개 waypoint 경로에서 위치를 전진시키는 단위 테스트로 barrier 도달에 HOLD,
`clear_until` 확장에 RELEASE(+goal 전송)".

⭐ 안전 규칙을 시험으로 못 박는다:
- 대기(clear 0 · STOP)는 **goal 취소뿐** — `/estop` 을 건드리지 않는다.
- ESTOP · 링크 유실만 `/estop true`, 그리고 **`LaneCommand RESUME` 으로만** 풀린다.
- `FleetCommand RESUME`(레인 모드 코디네이터가 10 Hz 로 보냄)은 ESTOP 을 풀지 못한다.
"""
import os
import re
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
sys.path.insert(0, PKG)

import pytest  # noqa: E402

from pinky_fleet_agent import route_chain as rc  # noqa: E402
from pinky_fleet_agent.route_chain import RouteChain  # noqa: E402

MSG_DIR = os.path.join(REPO, "pinky_lane_msgs", "msg")

# 0.5 m 간격 직선 5점 — 0,1,2,3,4 (goal_idx 4)
WP5 = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
SEQ = 7


def _kinds(acts):
    return [a[0] for a in acts]


def _estops(acts):
    return [a[1] for a in acts if a[0] == "estop"]


def _gotos(acts):
    return [a for a in acts if a[0] == "goto"]


class FakeClock:
    """재전송 간격(D6-1)을 벽시계 없이 재려고 쓴다."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def started_chain(**kw):
    kw.setdefault("clock", FakeClock())
    c = RouteChain(**kw)
    c.on_route(SEQ, WP5, 4)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    return c


# ---- 상수가 .msg 와 같다 (손으로 적은 값을 믿지 않는다) -------------------------

def _msg_constants(name):
    text = open(os.path.join(MSG_DIR, name), encoding="utf-8").read()
    return {m.group(1): int(m.group(2))
            for m in re.finditer(r"^uint8\s+(\w+)\s*=\s*(\d+)", text, re.M)}


def test_LaneCommand_상수가_msg_와_같다():
    c = _msg_constants("LaneCommand.msg")
    for k in ("CMD_HEARTBEAT", "CMD_START", "CMD_STOP", "CMD_ESTOP",
              "CMD_RESUME", "CMD_SET_SPEED", "CMD_CLEARANCE"):
        assert getattr(rc, k) == c[k], k


def test_LaneStatus_상수가_msg_와_같다():
    c = _msg_constants("LaneStatus.msg")
    for k in ("DRIVE_IDLE", "DRIVE_CRUISE", "DRIVE_WAIT_CLEARANCE",
              "DRIVE_ARRIVED", "DRIVE_ESTOP", "DRIVE_LINK_LOST"):
        assert getattr(rc, k) == c[k], k


# ---- START ack -------------------------------------------------------------

def test_경로_없이_온_START_는_무시하고_IDLE_로_남는다():
    c = RouteChain()
    assert c.on_lane_command(rc.CMD_START, SEQ, 0) == []
    assert c.drive_state() == rc.DRIVE_IDLE


def test_다른_경로의_START_는_무시한다():
    c = RouteChain()
    c.on_route(SEQ, WP5, 4)
    c.on_lane_command(rc.CMD_START, SEQ + 1, 0)
    assert c.drive_state() == rc.DRIVE_IDLE


def test_START_를_받으면_IDLE_을_벗어난다__코디네이터의_ack_조건():
    """코디네이터 `_cb_lane_status` 는 IDLE·ESTOP·LINK_LOST 가 아닌 상태를 ack 로 본다."""
    c = started_chain()
    assert c.drive_state() not in (rc.DRIVE_IDLE, rc.DRIVE_ESTOP, rc.DRIVE_LINK_LOST)
    assert c.status()["route_seq"] == SEQ


def test_START_만으로는_움직이지_않는다__허가가_0():
    c = RouteChain()
    c.on_route(SEQ, WP5, 4)
    acts = c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert _gotos(acts) == []
    assert c.drive_state() == rc.DRIVE_WAIT_CLEARANCE


# ---- 허가까지 목표 연쇄 ------------------------------------------------------

def test_허가를_받으면_그_waypoint_로_간다__좌표와_진행방향():
    c = started_chain()
    acts = c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 2)
    (g,) = _gotos(acts)
    _, idx, x, y, yaw = g
    assert (idx, x, y) == (2, 1.0, 0.0)
    assert abs(yaw) < 1e-9                      # +x 방향 경로
    assert c.drive_state() == rc.DRIVE_CRUISE


def test_허가가_목표를_넘어도_목표까지만_간다():
    c = started_chain()
    (g,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 99))
    assert g[1] == 4


def test_B5_barrier_도달에_HOLD__확장에_RELEASE():
    """B-5 수락: 5점 경로에서 위치를 전진시킨다."""
    c = started_chain()
    (g,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3))
    assert g[1] == 3

    # 위치가 전진하면 진행 인덱스가 오른다 (B-5: 예전엔 0 에 고정돼 있었다)
    progress = []
    for x in (0.2, 0.6, 1.1, 1.5):
        c.on_pose(x, 0.0)
        progress.append(c.progress_idx)
    assert progress == sorted(progress) and progress[-1] == 3

    # Nav2 가 barrier 에 닿았다 → HOLD (목표 없음, clearance 대기)
    acts = c.on_goal_result(3, "succeeded")
    assert _gotos(acts) == [] and "estop" not in _kinds(acts)
    assert c.active_target is None
    assert c.drive_state() == rc.DRIVE_WAIT_CLEARANCE

    # 중계가 clear_until 을 넓힌다 → RELEASE + 목표 재전송
    (g2,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 4))
    assert g2[1] == 4
    assert c.drive_state() == rc.DRIVE_CRUISE


def test_목표에_닿으면_ARRIVED():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 4)
    c.on_pose(2.0, 0.0)
    c.on_goal_result(4, "succeeded")
    assert c.drive_state() == rc.DRIVE_ARRIVED


def test_허가가_1칸씩_늘어날_때마다_goal_을_다시_보내지_않는다():
    """10 Hz clearance 마다 선점하면 Nav2 가 재계획만 하다 끝난다."""
    wp = [(0.1 * i, 0.0) for i in range(31)]            # 0.1 m 간격 3 m
    c = RouteChain()
    c.on_route(SEQ, wp, 30)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 10))
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 11)) == []   # +0.1 m
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 14)) == []   # +0.4 m
    (g,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 15))         # +0.5 m
    assert g[1] == 15


def test_달리던_목표가_가까우면_허가가_조금_늘어도_다시_보낸다():
    wp = [(0.1 * i, 0.0) for i in range(31)]
    c = RouteChain()
    c.on_route(SEQ, wp, 30)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 10)
    c.on_pose(0.7, 0.0)                                  # 목표 10 까지 0.3 m
    (g,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 11))
    assert g[1] == 11


# ---- 대기는 HOLD 이지 ESTOP 이 아니다 ----------------------------------------

def test_허가가_0_으로_줄면_goal_만_취소하고_estop_은_안_건드린다():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    acts = c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 0)
    assert _kinds(acts) == ["cancel"]
    assert c.drive_state() == rc.DRIVE_WAIT_CLEARANCE


def test_허가가_앞쪽으로_줄면_새_허가로_다시_겨눈다():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 4)
    (g,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 2))
    assert g[1] == 2


def test_STOP_은_HOLD_래치__estop_없음__RESUME_으로_재개():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    acts = c.on_lane_command(rc.CMD_STOP, SEQ, 0)
    assert _kinds(acts) == ["cancel"]
    # 10 Hz 로 다시 와도 동작이 쌓이지 않는다
    assert c.on_lane_command(rc.CMD_STOP, SEQ, 0) == []
    # STOP 중에는 허가가 와도 안 간다
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 4)) == []
    acts = c.on_lane_command(rc.CMD_RESUME, SEQ, 0)
    assert _estops(acts) == []                   # 래치가 STOP 뿐이면 /estop 을 안 낸다
    assert _gotos(acts)


# ---- ESTOP · 링크 유실은 래치, LaneCommand RESUME 으로만 풀린다 --------------

def test_ESTOP_은_취소와_estop_true():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    acts = c.on_lane_command(rc.CMD_ESTOP, 0, 0)            # route_seq 0 이어도 받는다
    assert _kinds(acts) == ["cancel", "estop"] and _estops(acts) == [True]
    assert c.drive_state() == rc.DRIVE_ESTOP
    assert c.on_lane_command(rc.CMD_ESTOP, 0, 0) == []     # 반복 무해


def test_FleetCommand_RESUME_은_ESTOP_을_풀지_못한다():
    """레인 모드 코디네이터는 RUNNING 에서 FleetCommand RESUME 을 10 Hz 로 보낸다."""
    c = started_chain()
    c.on_lane_command(rc.CMD_ESTOP, 0, 0)
    assert c.on_fleet_resume() == []
    assert c.drive_state() == rc.DRIVE_ESTOP
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)) == []


def test_STOP_과_ESTOP_이_함께_걸려도_FleetCommand_RESUME_은_ESTOP_을_남긴다():
    """앞 시험은 STOP 이 없어 `on_fleet_resume` 의 앞 문턱에서 끝난다 — 래치 선택은 여기서 잰다."""
    c = started_chain()
    c.on_lane_command(rc.CMD_STOP, SEQ, 0)
    c.on_lane_command(rc.CMD_ESTOP, 0, 0)
    acts = c.on_fleet_resume()
    assert _estops(acts) == []                  # /estop false 를 내지 않는다
    assert c.estop and not c.stopped            # STOP 만 풀렸다
    assert c.drive_state() == rc.DRIVE_ESTOP


def test_FleetCommand_RESUME_연발이_Nav2_포기_상태를_지우지_않는다():
    """레인 모드 코디네이터는 RUNNING 에서 FleetCommand RESUME 을 10 Hz 로 보낸다.
    그게 포기 기록을 지우면 Nav2 가 실패하는 목표를 무한히 다시 보낸다."""
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    for _ in range(3):
        c.on_goal_result(3, "aborted")
    for _ in range(10):
        assert c.on_fleet_resume() == []
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)) == []


def test_LaneCommand_RESUME_은_estop_false_와_함께_재개():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    c.on_lane_command(rc.CMD_ESTOP, 0, 0)
    acts = c.on_lane_command(rc.CMD_RESUME, SEQ, 0)
    assert _estops(acts) == [False]
    assert _gotos(acts)


def test_링크_유실은_주행_중에만_래치하고_estop_true():
    idle = RouteChain()
    assert idle.on_link_lost(moving=False) == []            # 할 일 없는 로봇은 안 건드린다
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    acts = c.on_link_lost(moving=True)
    assert _kinds(acts) == ["cancel", "estop"] and _estops(acts) == [True]
    assert c.drive_state() == rc.DRIVE_LINK_LOST
    assert c.on_fleet_resume() == []                        # FleetCommand RESUME 으로는 안 풀린다
    assert _estops(c.on_lane_command(rc.CMD_RESUME, SEQ, 0)) == [False]


def test_래치가_걸리면_단일_목표도_막는다():
    c = started_chain()
    c.on_lane_command(rc.CMD_ESTOP, 0, 0)
    assert c.latched


# ---- 경로 교체 · Nav2 실패 --------------------------------------------------

def test_새_경로가_오면_달리던_goal_을_취소하고_옛_번호_명령은_무시한다():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    acts = c.on_route(SEQ + 1, WP5, 4)
    assert _kinds(acts) == ["cancel"]
    assert c.drive_state() == rc.DRIVE_IDLE
    assert c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3) == []


def test_Nav2_가_같은_목표를_3번_실패하면_허가가_바뀔_때까지_안_보낸다():
    """실패 뒤 재시도는 바로가 아니라 재전송 간격(min_resend_s) 뒤의 위치 틱에서 나간다(D6-1)."""
    clk = FakeClock()
    c = started_chain(clock=clk)
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    for _ in range(2):                                       # 1·2번째 실패 → 간격 뒤 재시도
        assert _gotos(c.on_goal_result(3, "aborted")) == []
        clk.advance(rc.MIN_RESEND_S + 0.1)
        assert _gotos(c.on_pose(0.0, 0.0))
    assert c.on_goal_result(3, "aborted") == []             # 3번째 → 포기
    assert "실패" in c.reason
    clk.advance(10.0)
    assert _gotos(c.on_pose(0.0, 0.0)) == []
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)) == []
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 4))   # 허가가 바뀌면 다시


def test_Nav2_포기_뒤_운영자가_RESUME_하면_같은_목표를_다시_시도한다():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    for _ in range(3):
        c.on_goal_result(3, "aborted")
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)) == []
    (g,) = _gotos(c.on_lane_command(rc.CMD_RESUME, SEQ, 0))
    assert g[1] == 3


def test_이미_버린_목표의_결과는_무시한다():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 0)             # 취소됨
    assert c.on_goal_result(3, "succeeded") == []
    assert c.drive_state() == rc.DRIVE_WAIT_CLEARANCE


# ---- D6-1 · 같은 목표 재전송 고리 (관제 리그 2026-09-24: 813회 전송 / 811회 succeeded) ------------

# 0.10 m 간격 직선 21점 — 실제 Route 간격. 로봇은 허가 지점(idx 10 = 1.0 m) 0.14 m 앞(0.86 m)에 서 있다.
WP21 = [(0.1 * i, 0.0) for i in range(21)]


def _sim_nav2(c, clk, seconds, x, respond, rate_hz=10):
    """가짜 Nav2: goto 가 나오면 즉시 respond 결과를 돌려준다. 10 Hz 위치 틱을 seconds 동안 돈다. 보낸 수를 센다."""
    sent = 0

    def feed(acts):
        nonlocal sent
        for a in _gotos(acts):
            sent += 1
            feed(c.on_goal_result(a[1], respond))
    for _ in range(int(seconds * rate_hz)):
        clk.advance(1.0 / rate_hz)
        feed(c.on_pose(x, 0.0))
    return sent, feed


def _chain21(clk, **kw):
    c = RouteChain(clock=clk, **kw)
    c.on_route(SEQ, WP21, 20)
    c.on_lane_command(rc.CMD_START, SEQ, 0)
    c.on_pose(0.86, 0.0)
    return c


def test_D6_1_Nav2_가_즉시_성공해도_같은_목표를_다시_보내지_않는다__60초():
    """관제 처방 ④: Nav2 가 즉시 succeeded 를 돌려주는 가짜로 60 s — 전송 ≤ 2. (Nav2 허용 반경은 모르는 상태)"""
    clk = FakeClock()
    c = _chain21(clk)
    assert c.nav2_xy_tol is None and c.effective_reach_tol == rc.REACH_TOL_M      # 0.14 > 0.10 — 예전 고리 조건
    sent, feed = _sim_nav2(c, clk, 0, 0.86, "succeeded")
    feed(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 10))
    more, _ = _sim_nav2(c, clk, 60.0, 0.86, "succeeded")
    assert c.sends <= 2, c.sends
    assert c.sends == 1                                      # 성공 = 도달 확정 → 한 번이면 끝
    assert c.drive_state() == rc.DRIVE_WAIT_CLEARANCE and c.reached_idx == 10


def test_D6_1_도달_확정_뒤_허가가_늘면_다음_목표로_간다():
    clk = FakeClock()
    c = _chain21(clk)
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 10)
    assert c.on_goal_result(10, "succeeded") == []
    (g,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 16))
    assert g[1] == 16


def test_D6_1_Nav2_허용_반경을_알면_그_안에서는_목표를_아예_안_보낸다():
    clk = FakeClock()
    c = _chain21(clk)
    c.set_nav2_xy_tolerance(0.15)                           # 팜 nav2_farm_params 의 값
    assert abs(c.effective_reach_tol - (0.15 + rc.REACH_MARGIN_M)) < 1e-12
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 10)) == []
    assert c.sends == 0
    c.set_nav2_xy_tolerance(0.08)                           # 실물 nav2_params_fleet 의 값 — 기본보다 좁으면 기본을 쓴다
    assert c.effective_reach_tol == rc.REACH_TOL_M


def test_D6_1_서버가_없어도_실패로_세지_않고_재시도는_1Hz_를_넘지_않는다():
    """Nav2 가 늦게 뜨는 것은 목표 실패가 아니다 — 3번 만에 포기하면 코디네이터가 같은 허가를 보내는 동안 영영 안 간다."""
    clk = FakeClock()
    c = started_chain(clock=clk)
    _, feed = _sim_nav2(c, clk, 0, 0.0, "unavailable")
    feed(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3))
    _sim_nav2(c, clk, 60.0, 0.0, "unavailable")
    assert 55 <= c.sends <= 61, c.sends                      # 간격이 없으면 600
    assert c._blocked_target is None


def test_D6_1_재전송_간격은_허가_축소_선점을_막지_않는다():
    """달리는 목표가 허가 밖으로 나가면 바로 가까운 목표로 바꿔야 한다 — 간격으로 막으면 허가 밖으로 달린다."""
    clk = FakeClock()
    c = started_chain(clock=clk)
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 2))
    clk.advance(0.1)
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 4))
    clk.advance(0.1)
    (g,) = _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 2))
    assert g[1] == 2 and c.active_target == 2


def test_D6_1_운영자_재개는_재전송_간격으로_늦추지_않는다():
    clk = FakeClock()
    c = started_chain(clock=clk)
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    c.on_stop()
    assert _gotos(c.on_lane_command(rc.CMD_RESUME, SEQ, 0))   # 같은 순간(간격 안)이라도 바로


# ---- 관제 검수 P1 · 미뤄 둔 goto 는 재생 직전에 다시 묻는다 -----------------------------

def _deferred(c):
    """Nav2 거부 뒤 미뤄진 재시도 goto 하나를 만든다 (agent_node._deferred_acts 와 같은 모양)."""
    clk = c._clock
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    clk.advance(rc.MIN_RESEND_S + 0.1)
    (g,) = _gotos(c.on_goal_result(3, "unavailable")) or _gotos(c.on_pose(0.0, 0.0))
    return g


def test_P1_미뤄_둔_goto_는_그대로면_보낸다():
    c = started_chain()
    g = _deferred(c)
    assert c.goal_is_current(g[1], g[2], g[3])


@pytest.mark.parametrize("why", ["lane_stop", "fleet_stop", "estop", "link_lost", "new_route_idle"])
def test_P1_미뤄_둔_goto_는_그_사이_정지가_오면_버린다(why):
    c = started_chain()
    g = _deferred(c)
    if why == "lane_stop":
        c.on_lane_command(rc.CMD_STOP, SEQ, 3)
    elif why == "fleet_stop":
        c.on_stop("FleetCommand STOP")
    elif why == "estop":
        c.on_lane_command(rc.CMD_ESTOP, 0, 0)
    elif why == "link_lost":
        c.on_link_lost(moving=True)
    else:
        c.on_route(SEQ + 1, WP5, 4)                          # 새 경로 — START 전이라 아무것도 보내면 안 된다
    assert not c.goal_is_current(g[1], g[2], g[3])


def test_P1_새_경로의_좌표가_다르면_옛_goto_는_버린다():
    c = started_chain()
    g = _deferred(c)
    c.on_route(SEQ + 1, [(x, y + 1.0) for x, y in WP5], 4)
    c.on_lane_command(rc.CMD_START, SEQ + 1, 0)
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ + 1, 3)
    assert c.active_target == 3
    assert not c.goal_is_current(g[1], g[2], g[3])


# ---- 관제 검수 P3 · STOP 뒤 START 는 STOP 을 푼다 (ESTOP 은 아니다) --------------------------

def test_P3_STOP_뒤_START_는_다시_움직인다():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    c.on_lane_command(rc.CMD_STOP, SEQ, 3)
    assert c.stopped
    c.on_lane_command(rc.CMD_START, SEQ, 3)
    assert not c.stopped
    assert _gotos(c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)) or c.active_target == 3


def test_P3_ESTOP_은_START_로_풀리지_않는다():
    c = started_chain()
    c.on_lane_command(rc.CMD_CLEARANCE, SEQ, 3)
    c.on_lane_command(rc.CMD_ESTOP, 0, 0)
    assert _gotos(c.on_lane_command(rc.CMD_START, SEQ, 3)) == []
    assert c.estop and c.drive_state() == rc.DRIVE_ESTOP
