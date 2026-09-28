# -*- coding: utf-8 -*-
"""통합 적대 검토(2026-09-26 통합본 54bb60b) — 코디네이터·예약의 살아 있음(liveness)과 로봇별 정지·재개의 틈.

RES-F2 (P2) 기다리던 로봇의 포즈가 한 틱(0.1 s) 0.55~0.6 m 앞으로 튀면 영영 못 떠났다. 에이전트의 RouteFollower 는 튄 자리로
           올라가 안 내려오고(단조), 예약의 잡기 진행(lead_s)은 H/RES-1 로 2 s 뒤 되돌아간다. 허가가 오면 에이전트는 '허가 지점에
           닿았다' 며 서고 코디네이터는 다음 엣지를 요청하지 않는다 → `clearance 대기 (진행 6, 허가 7)` · STALL_NO_REQUEST 영구.
           → 잡기(요청)만 에이전트가 보고한 진행(LaneStatus.route_idx)도 따른다. 놓기는 그대로 확인된 포즈만.
RES-F3 (P3) RES-1·H 시험은 로봇을 허가 지점으로 순간이동시키고 '허가를 받았다' 만 봤다 — 실제 RouteChain 을 로봇마다 두고
           둘 다 목표에 닿는지, 교착 경고가 없는지 잰다(아래 `_closed_loop`, 검토 도구 final_review/reservation/sim.py 와 같은 모형).
F1  (P3) 플릿 STOPPED·DONE 중 래치된 로봇의 재개(R5)는 RESUME 뒤 STOP 을 보냈다. RESUME 이 낸 goto 는 에이전트 해제 문 뒤에
           쌓이고 Nav2 취소 확인이 STOP 보다 먼저 닿으면 진짜 목표가 나갔다 → RESUME 앞에 허가 0(CLEARANCE 0).
F2  (P2) RUNNING 중 에이전트가 시작 전(IDLE)으로 돌아가면(에이전트 재시작·브리지 재시작의 같은 번호 Route) START 를 다시 못 받고
           화면엔 아무것도 없었다 → 저절로 출발시키지 않고 경고(ROBOT_IDLE_IN_RUNNING), 로봇 재개가 START 를 다시 무장한다.
F3  (P3) 팀11 레인 로봇의 LINK_LOST 는 저절로 풀리는 상태다 — 세우지 않는다(링크유실 래치 규칙은 Nav2 로봇만).
F4  (P3) · E2E-2 경로 없는 Nav2 에이전트는 LaneStatus 를 안 내 링크유실 래치가 안 보였다 → RobotState NAV_LINK_LOST 도 본다.
FLEET-R1 (P1, 6b1d0d6 재검토) 잡기가 에이전트 진행을 따르자 튄 포즈로 목표까지 올라간 로봇이 목표까지 다 잡고 goto 없이 '도착' 을
           보고했다 → mark_arrived 가 출발 노드에 선 로봇의 엣지를 놓았다(54bb60b 도 달리던 로봇의 0.2 s 튐이면 났다).
           → 도착은 말한 뒤 확인 창 하나(2 s)가 지나고 그 창의 과반이 목표 근처일 때만 받는다. 아니면 쥔 채 ARRIVED_NOT_AT_GOAL.
FLEET-R2 (P3) 폐루프 시험이 믿음(arrived)만 봤다 → 도착을 받은 순간의 실제 거리(gap)와 받은 뒤의 쥐지 않은 엣지도 잰다.

새 이름은 getattr 로 꺼낸다 — 고치기 전 코드(54bb60b)로 돌려도 모음이 깨지지 않고 시험마다 판정이 나오게.
⚠️ 저장 파일은 늘 임시 폴더로(PINKY_RELAY_STATE_DIR) — 실물 중계의 ~/.local/state 를 건드리지 않는다.
"""
import bisect
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pinky_fleet_msgs.msg import FleetCommand, RobotState  # noqa: E402
from pinky_lane_msgs.msg import LaneCommand, LaneStatus  # noqa: E402
from relay_station.fleet import fleet_coordinator as FC  # noqa: E402
from relay_station.fleet import profiles as P  # noqa: E402
from relay_station.fleet import reservation as R  # noqa: E402
from pinky_fleet_agent import route_chain as rc  # noqa: E402
from pinky_fleet_agent.route_chain import RouteChain  # noqa: E402
from test_d6_2_closed_loop import repo_nav2_tolerances  # noqa: E402
from test_d7_robot_stop import _coord, _lane_status, _reset, _sent  # noqa: E402
from test_fleet_profiles import _pcoord  # noqa: E402
import test_hybrid_agent_review as TA  # noqa: E402

DT = 0.1
TOLS, _ = repo_nav2_tolerances()
LL = LaneStatus.DRIVE_LINK_LOST


@pytest.fixture(autouse=True)
def _state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv(P.STATE_ENV, str(tmp_path / "state"))


def _chain_for(c, name="pinky1"):
    ctx = c.robots[name]
    msg = c._to_route_msg(name, ctx.route_seq, ctx.route)
    ch = RouteChain()
    ch.on_route(msg.route_seq, [(p.x, p.y) for p in msg.waypoints], msg.goal_idx)
    return ch


def _report(c, ch, name="pinky1", dt=0.1):
    """체인의 지금 상태를 LaneStatus 로 코디네이터에 — 에이전트의 _publish_lane_status 와 같은 필드."""
    st = ch.status()
    m = LaneStatus()
    m.drive_state = int(st["drive_state"])
    m.state_reason = st["state_reason"]
    m.route_seq = int(st["route_seq"])
    m.route_idx = int(st["route_idx"])
    m.clear_until_idx = int(st["clear_until_idx"])
    c._t += dt
    c._cb_lane_status(name, m)


def _feed(ch, c, name="pinky1", upto=None):
    """코디네이터가 보낸 LaneCommand 를 순서대로 체인에(보낸 것은 비운다 — 두 번 넣지 않게, 넣은 명령은 c.fed 에 쌓는다).
    upto 명령까지만(그것 포함) 넣고 나머지는 돌려준다."""
    acts, sent = [], _sent(c.lane_cmd_pubs[name])
    c.lane_cmd_pubs[name].reset_mock()
    c.__dict__.setdefault("fed", []).extend(m.command for m in sent)
    for i, m in enumerate(sent):
        acts += ch.on_lane_command(m.command, m.route_seq, m.clear_until_idx)
        if upto is not None and m.command == upto:
            return acts, sent[i + 1:]
    return acts, []


def _gotos(acts):
    return [a for a in acts if a[0] == "goto"]


# ==== RES-F2 · RES-F3: 실제 RouteChain 을 로봇마다 둔 폐루프 ==========================================================

def _pos(route, cum, s):
    pts = route.waypoints
    s = max(0.0, min(cum[-1], s))
    i = max(0, min(len(pts) - 2, bisect.bisect_right(cum, s) - 1))
    seg = cum[i + 1] - cum[i]
    t = 0.0 if seg <= 0 else (s - cum[i]) / seg
    return pts[i][0] + t * (pts[i + 1][0] - pts[i][0]), pts[i][1] + t * (pts[i + 1][1] - pts[i][1])


def _closed_loop(pname, tol, speed=0.15, glitch=None, T=90.0, pre=None, reassign=None):
    """진짜 코디네이터(switch_profile·start_fleet·_loop_tick·_cb_lane_status) + 로봇마다 진짜 RouteChain + Nav2 모형, 10 Hz.

    Nav2 모형: 목표 허용 반경(tol) 안에 처음 들어오면 성공, 아니면 speed 로 목표 쪽으로. 로봇은 경로 위 호길이 s 로만 움직인다.
    glitch = (로봇, 틱 또는 틱 모음, 앞으로 m) — 그 틱만 보고 포즈가 경로를 따라 그만큼 앞이다(AMCL 튐).
    pre = (로봇, (x, y), 틱 수) — START 전 그 틱 수 동안 그 자리로 보고한 뒤 3 s 제자리(월요일 §14-4: ① 뒤 옛 좌표계 포즈, ③ 로 바로잡힘).
    reassign = (로봇, 시각 s) — 그 시각 운영자가 같은 출발·목표로 다시 배정한다(새 Route 를 체인에).
    돌려주는 것: 도착 시각(코디네이터가 도착을 **받은** 시각), 그 순간 목표까지 **실제로** 남은 호길이(gap — 통합 검토 FLEET-R2:
    예전엔 믿음(체인·코디네이터의 arrived)만 봐서 움직이지도 않은 로봇이 '도착' 으로 통과했다), 처음 본 경고들, 로봇이 쥐지 않은
    엣지 안에 있던 틱, 두 로봇이 한 엣지 안에 있던 틱(둘 다 로봇이 **실제로** 목표에 닿기 전까지는 도착을 받았어도 잰다),
    체인이 목표에서 먼 채 도착을 말한 적이 있나(claim_far), 그 동안 제 자리 엣지를 놓은 틱, 선 로봇의 사유.
    """
    c = _pcoord()
    c.state_timeout_sec, c.global_seq = 1.0, 0
    assert c.switch_profile(pname)
    bots = {}
    for n, ctx in c.robots.items():
        if ctx.route is None:
            continue
        clk = [c._t]
        ch = RouteChain(clock=lambda clk=clk: clk[0])
        ch.set_nav2_xy_tolerance(tol)
        ch.on_route(ctx.route_seq, list(ctx.route.waypoints), ctx.route.goal_idx)
        bots[n] = {"route": ctx.route, "cum": ctx.route.cumulative(), "s": 0.0, "goal": None, "ch": ch, "clk": clk,
                   "lane": 0, "fleet": 0, "arrive": None}
    out = {"warn": {}, "unheld": [], "both": [], "arrive": {}, "stuck": {}, "gap": {}, "claim_far": set(),
           "released_while_claim": []}
    ticks = () if glitch is None else ((glitch[1],) if isinstance(glitch[1], int) else tuple(glitch[1]))

    def at_goal(n, b):
        return b["cum"][b["route"].goal_idx] - b["s"] <= tol + 0.05

    def apply(b, acts):
        for a in acts:
            if a[0] == "goto":
                b["goal"] = a[1]
            elif a[0] == "cancel":
                b["goal"] = None

    def deliver():
        for n, b in bots.items():
            calls = c.lane_cmd_pubs[n].publish.call_args_list
            for call in calls[b["lane"]:]:
                m = call[0][0]
                apply(b, b["ch"].on_lane_command(m.command, m.route_seq, m.clear_until_idx))
            b["lane"] = len(calls)
            calls = c.fleet_cmd_pubs[n].publish.call_args_list
            for call in calls[b["fleet"]:]:
                m = call[0][0]
                if m.command == FleetCommand.CMD_STOP:
                    apply(b, b["ch"].on_stop("FleetCommand STOP"))
                elif m.command == FleetCommand.CMD_RESUME:
                    apply(b, b["ch"].on_fleet_resume())
            b["fleet"] = len(calls)

    def tick(k):
        c._t += DT
        for n, b in bots.items():
            b["clk"][0] = c._t
            if b["goal"] is not None:
                tgt = b["cum"][b["goal"]]
                if abs(tgt - b["s"]) <= tol:
                    g, b["goal"] = b["goal"], None
                    apply(b, b["ch"].on_goal_result(g, "succeeded"))
                else:
                    b["s"] = min(tgt, b["s"] + speed * DT) if tgt > b["s"] else max(tgt, b["s"] - speed * DT)
        for n, b in bots.items():
            ahead = glitch[2] if (glitch and glitch[0] == n and k in ticks) else 0.0
            x, y = _pos(b["route"], b["cum"], b["s"] + ahead)
            if k < 0 and pre and pre[0] == n and k < -30:
                x, y = pre[1]                           # START 전 옛 좌표계 포즈(③ 전)
            apply(b, b["ch"].on_pose(x, y))
            c._cb_robot_state(n, RobotState(localized=True, x=float(x), y=float(y)))
            _report(c, b["ch"], n, dt=0.0)
            if b["ch"].drive_state() == rc.DRIVE_ARRIVED and not at_goal(n, b):
                out["claim_far"].add(n)
        c._loop_tick()
        deliver()

    for k in range(pre[2] + 30 if pre else 5, 0, -1):       # START 전 — 로봇은 출발 노드에 선 채 보고한다(pre 면 그 앞에 옛 포즈)
        tick(-k)
    assert c.start_fleet()
    deliver()
    for k in range(1, int(T / DT) + 1):
        if reassign and k == int(round(reassign[1] / DT)):
            n = reassign[0]
            ctx, b = c.robots[n], bots[n]
            assert c.assign_route(n, ctx.start_node, ctx.goal_node)
            assert list(ctx.route.waypoints) == list(b["route"].waypoints)     # 같은 출발·목표 — 실제 위치 s 가 그대로 뜻이 있다
            b["route"], b["goal"] = ctx.route, None
            apply(b, b["ch"].on_route(ctx.route_seq, list(ctx.route.waypoints), ctx.route.goal_idx))
        tick(k)
        res = c.reservation
        for n, b in bots.items():
            if b["arrive"] is None and c.robots[n].arrived:
                b["arrive"] = round(k * DT, 1)
                out["gap"][n] = round(b["cum"][b["route"].goal_idx] - b["s"], 3)
        inside = {}
        for n, b in bots.items():
            if (c.robots[n].arrived and at_goal(n, b)) or n not in res.robots:
                continue
            sl = res.robots[n]
            inside[n] = {eid for kk, eid in enumerate(b["route"].edge_ids)
                         if sl.edge_start_s(kk) + 0.05 < b["s"] < sl.edge_end_s(kk) - 0.05}
            out["unheld"] += [(round(k * DT, 1), n, eid) for eid in inside[n] if res.edge_holder.get(eid) != n]
            if b["ch"].drive_state() == rc.DRIVE_ARRIVED and not at_goal(n, b):
                # 목표에서 먼 채 도착을 말하는 동안 — 지금 자리(노드에 서 있어도)의 엣지를 계속 쥐어야 한다(놓기가 이르면 안 된다)
                here = [eid for kk, eid in enumerate(b["route"].edge_ids)
                        if sl.edge_start_s(kk) - 0.05 <= b["s"] < sl.edge_end_s(kk) - 0.05]
                out["released_while_claim"] += [(round(k * DT, 1), n, eid) for eid in here
                                                if res.edge_holder.get(eid) != n]
        names = sorted(inside)
        for i, a in enumerate(names):
            for bn in names[i + 1:]:
                if inside[a] & inside[bn]:
                    out["both"].append((round(k * DT, 1), sorted(inside[a] & inside[bn])))
        for w in filter(None, (c.last_warning or "").split(" | ")):
            out["warn"].setdefault(w.split(":")[0], (round(k * DT, 1), w[:160]))
        if all(b["arrive"] is not None for b in bots.values()):
            break
    for n, b in bots.items():
        out["arrive"][n] = b["arrive"]
        if b["arrive"] is None:
            out["stuck"][n] = (round(b["s"], 3), b["ch"].reason, c.reservation.status(n))
    return out


def _assert_physical(out, tol):
    """통합 검토 FLEET-R2: 코디네이터가 도착을 받은 순간 로봇은 **실제로** 목표에 있다(Nav2 허용 + 0.05 안) — 믿음만 보지 않는다."""
    far = {n: g for n, g in out["gap"].items() if g > tol + 0.05}
    assert far == {}, f"도착을 받았는데 목표에서 먼 로봇(m): {far}"


@pytest.mark.parametrize("tol", [0.08, 0.15])
@pytest.mark.parametrize("ahead", [0.6, 0.8, 1.2])
def test_RES_F2__BL_에서_기다리던_pinky2_의_포즈가_한_틱_앞으로_튀어도_출발해_도착한다(ahead, tol):
    """map4 시나리오 1: pinky2 는 pinky1 이 BL_JS 를 놓을 때까지 BL 에서 허가 0 으로 기다린다. 1.0 s 에 한 틱만 보고 포즈가
    경로를 따라 ahead m 앞이다. 고치기 전(54bb60b): +0.6·0.8·1.2 모두 16.5 s 에 STALL_NO_REQUEST, pinky2 는 끝까지
    `clearance 대기 (진행 6/8/12, 허가 7)` 로 BL 에 섰다(검토 F2 배치). 고친 뒤: 둘 다 도착·경고 없음·안전 불변식 그대로."""
    out = _closed_loop("map4", tol, glitch=("pinky2", 10, ahead))
    assert out["stuck"] == {}, out["stuck"]
    assert all(out["arrive"].values()), out["arrive"]
    _assert_physical(out, tol)
    assert "STALL_NO_REQUEST" not in out["warn"] and out["warn"] == {}, out["warn"]
    assert out["unheld"] == [] and out["both"] == [], (out["unheld"][:4], out["both"][:4])


@pytest.mark.parametrize("tol", TOLS)
@pytest.mark.parametrize("pname", ["map4", "map4_s2"])
def test_RES_F3__실제_RouteChain_폐루프__두_로봇_모두_목표에_닿고_교착_경고가_없다(pname, tol):
    """RES-1·H 시험의 빈틈(로봇을 허가 지점으로 순간이동, '허가를 받았다' 만 확인)을 메운다: 로봇마다 진짜 RouteChain,
    Nav2 모형, 진짜 코디네이터 틱. 레포의 Nav2 허용치마다 두 시나리오 모두 — 둘 다 도착하고, 어떤 경고(STALL·PARKED·
    DEADLOCK·IDLE)도 없고, 로봇은 늘 제가 쥔 엣지 안에만 있고, 두 로봇이 한 엣지에 함께 있던 적이 없다."""
    out = _closed_loop(pname, tol)
    assert out["stuck"] == {} and all(out["arrive"].values()), (out["arrive"], out["stuck"])
    _assert_physical(out, tol)
    assert out["warn"] == {}, out["warn"]
    assert out["unheld"] == [] and out["both"] == [], (out["unheld"][:4], out["both"][:4])


def _map4_slot(name="pinky2"):
    _, profs = P.load_profiles()
    prof = profs["map4"]
    spec = next(r for r in prof.mission["robots"] if r["name"] == name)
    rt = prof.graph.shortest_route(spec["start"], spec["goal"], step=0.10)
    r = R.Reservation(prof.graph, **{k: float(v) for k, v in prof.mission["reservation"].items()})
    r.register(name, spec["domain_id"], rt)
    x0, y0 = rt.waypoints[0]
    for _ in range(R.DEFAULT_CONFIRM_UPDATES + 5):          # 출발 노드에 선 채 확인 창을 채운다
        r.update_pose(name, x0, y0)
        r.step()
    return r, rt, (x0, y0)


def test_RES_F2__에이전트가_보고한_진행은_다음_엣지_요청만_앞당기고__놓기는_확인된_포즈대로다():
    r, rt, (x0, y0) = _map4_slot()
    sl = r.robots["pinky2"]
    held0, clear0 = list(sl.held), r.clear_until("pinky2")
    assert held0 == [0, 1, 2] and clear0 == 7 and len(rt.edge_ids) > 3    # BL_JS·JS_CW2·CW2_BR, 허가 7 (0.7 m)
    for _ in range(5):
        r.update_pose("pinky2", x0, y0)
        r.step()
    assert sl.held == held0                                   # 포즈로는 허가 지점(0.7 m)이 reserve_ahead 밖 — 요청 없음
    note = getattr(r, "note_reported_idx", None)
    assert note is not None, "Reservation.note_reported_idx 가 없다(통합 검토 RES-F2)"
    note("pinky2", clear0 - 1)                                # 에이전트: 허가 지점 한 칸 앞이라 믿고 선다
    for _ in range(3):
        r.update_pose("pinky2", x0, y0)                       # 포즈는 여전히 출발 노드
        r.step()
    assert len(sl.held) > len(held0), r.status("pinky2")      # 다음 엣지를 요청해 잡았다(교착 없음)
    assert r.clear_until("pinky2") > clear0
    # 놓기는 확인된 포즈(출발 노드)만 — 첫 엣지·출발 노드는 계속 쥔다
    assert sl.progress_s == 0.0 and sl.held[0] == 0
    assert r.edge_holder[rt.edge_ids[0]] == "pinky2" and r.node_holder[rt.node_ids[0]] == "pinky2"


def test_RES_F2__경로_끝을_보고해도_놓지_않는다__부푼_보고는_잡기만_늘린다():
    r, rt, (x0, y0) = _map4_slot()
    sl = r.robots["pinky2"]
    r.note_reported_idx("pinky2", 10 ** 6)                    # 범위 밖도 경로 끝으로 자른다(터지지 않는다)
    for _ in range(3 * R.DEFAULT_CONFIRM_UPDATES):
        r.update_pose("pinky2", x0, y0)
        r.step()
    assert sl.reported_s == pytest.approx(sl.cum[-1])
    assert sl.progress_s == 0.0 and sl.progress_idx == 0
    assert sl.held[0] == 0 and r.edge_holder[rt.edge_ids[0]] == "pinky2"
    assert r.node_holder[rt.node_ids[0]] == "pinky2"


def test_RES_F2__코디네이터는_지금_경로_번호의_보고만_예약에_넣는다():
    c = _pcoord()
    assert c.switch_profile("map4")
    ctx = c.robots["pinky2"]
    sl = c.reservation.robots["pinky2"]
    m = LaneStatus()
    m.drive_state, m.route_idx = LaneStatus.DRIVE_WAIT_CLEARANCE, 12
    m.route_seq = ctx.route_seq - 1                          # 옛 경로의 보고 — 새 경로의 진행이 아니다
    c._cb_lane_status("pinky2", m)
    assert getattr(sl, "reported_s", 0.0) == 0.0
    m.route_seq = ctx.route_seq
    c._cb_lane_status("pinky2", m)
    assert getattr(sl, "reported_s", None) == pytest.approx(sl.cum[12])
    assert sl.progress_s == 0.0 and sl.lead_s == 0.0          # 포즈 기반 두 진행은 그대로


# ==== FLEET-R1 · FLEET-R2: 도착(한꺼번에 놓기)은 확인된 포즈로 받는다 =====================================================
#
# FLEET-R1 (P1, 6b1d0d6 재검토): RES-F2 로 잡기가 에이전트의 진행을 따르자, 튄 포즈로 RouteFollower 가 목표까지 올라간 로봇은
# 목표까지 다 잡아 허가가 목표가 됐고, goto 하나 없이 '목표 도착' 을 보고했다 → mark_arrived 가 출발 노드에 선 로봇의 엣지·노드를
# 전부 놓았다(다른 로봇이 그 자리로 들어왔다). 54bb60b 에서도 **달리던** 로봇의 0.2 s 튐이면 같은 일이 났다(아래 pinky1 시험).
# FLEET-R2: 폐루프 시험이 믿음(arrived)만 봐서 그걸 못 잡았다 → 도착을 받은 순간의 실제 거리(gap)를 잰다.

def _node_xy(c, nid):
    n = c.graph.nodes[nid]
    return n.x, n.y


def _running_map4():
    c = _pcoord()
    c.state_timeout_sec, c.global_seq, c.goal_event_timeout = 1.0, 0, 3.0
    assert c.switch_profile("map4")
    assert c.start_fleet()
    return c


def _arrived_msg(c, name, seq=None):
    m = LaneStatus()
    m.drive_state, m.state_reason = LaneStatus.DRIVE_ARRIVED, "목표 도착"
    ctx = c.robots[name]
    m.route_seq = ctx.route_seq if seq is None else seq
    m.route_idx = ctx.route.goal_idx                         # 에이전트는 목표에 닿았다고 믿는다
    return m


def _run(c, poses, n, claims=()):
    """n 틱(10 Hz): 로봇 포즈를 넣고, claims 의 로봇은 도착(LaneStatus)을 보고한 뒤 코디네이터 틱.
    claims 의 항목은 로봇 이름(지금 경로의 도착) 또는 (로봇, LaneStatus)."""
    for _ in range(n):
        c._t += DT
        for name, xy in poses.items():
            c._cb_robot_state(name, RobotState(localized=True, x=float(xy[0]), y=float(xy[1])))
        for item in claims:
            name, m = (item, _arrived_msg(c, item)) if isinstance(item, str) else item
            c._cb_lane_status(name, m)
        c._loop_tick()


def test_FLEET_R1__출발_노드에_선_로봇의_도착_보고는_받지_않는다__엣지·노드를_쥔_채_경고하고__목표에서_이어지면_받는다():
    c = _running_map4()
    ctx, res = c.robots["pinky1"], c.reservation
    bl, tc = _node_xy(c, "BL"), _node_xy(c, "TC")
    assert ctx.route.node_ids[0] == "BL" and ctx.route.node_ids[-1] == "TC"
    _run(c, {"pinky1": bl, "pinky2": bl}, 25)                 # 둘 다 BL — pinky1 이 BL_JS 를 잡는다
    assert res.edge_holder.get("BL_JS") == "pinky1"
    _run(c, {"pinky1": bl, "pinky2": bl}, 60, claims=["pinky1"])     # 6 s 동안 BL 에서 '목표 도착'
    assert not ctx.arrived, "출발 노드에 선 로봇의 도착을 받았다 — 엣지를 전부 놓는다(통합 검토 FLEET-R1)"
    assert res.edge_holder.get("BL_JS") == "pinky1" and res.node_holder.get("BL") == "pinky1"
    assert not res.robots["pinky1"].finished
    assert "ARRIVED_NOT_AT_GOAL: pinky1" in c.last_warning and "재배정" in c.last_warning, c.last_warning
    # 로봇이 정말 목표에 서서 계속 도착을 말하면 받는다(확인 창이 찬 뒤) — 엣지를 놓고 목표 노드만 쥔다. 경고도 사라진다
    _run(c, {"pinky1": tc, "pinky2": bl}, 30, claims=["pinky1"])
    assert ctx.arrived and res.robots["pinky1"].finished
    assert "pinky1" not in res.edge_holder.values() and res.node_holder.get("TC") == "pinky1"
    assert "ARRIVED_NOT_AT_GOAL" not in c.last_warning


def test_FLEET_R1__목표로_두_틱_튄_포즈로는_도착을_받지_않는다():
    c = _running_map4()
    bl, tc = _node_xy(c, "BL"), _node_xy(c, "TC")
    _run(c, {"pinky1": bl, "pinky2": bl}, 25)
    _run(c, {"pinky1": bl, "pinky2": bl}, 5, claims=["pinky1"])
    _run(c, {"pinky1": tc, "pinky2": bl}, 2, claims=["pinky1"])      # AMCL 튐 0.2 s — 바로 그때 도착 보고
    _run(c, {"pinky1": bl, "pinky2": bl}, 30, claims=["pinky1"])
    assert not c.robots["pinky1"].arrived
    assert c.reservation.edge_holder.get("BL_JS") == "pinky1"


def test_FLEET_R1__경고는_지금_경로의_도착_보고가_몇_초_이어질_때만():
    c = _running_map4()
    ctx = c.robots["pinky1"]
    bl = _node_xy(c, "BL")
    _run(c, {"pinky1": bl, "pinky2": bl}, 25)
    old = _arrived_msg(c, "pinky1", seq=ctx.route_seq - 1)    # 옛 경로의 도착 — 지금 경로가 아니다
    _run(c, {"pinky1": bl, "pinky2": bl}, 60, claims=[("pinky1", old)])
    assert not ctx.arrived and "ARRIVED_NOT_AT_GOAL" not in c.last_warning
    _run(c, {"pinky1": bl, "pinky2": bl}, 45, claims=["pinky1"])      # 4.5 s — 아직 참 도착의 확인 창일 수 있다
    assert "ARRIVED_NOT_AT_GOAL" not in c.last_warning
    _run(c, {"pinky1": bl, "pinky2": bl}, 10, claims=["pinky1"])
    assert "ARRIVED_NOT_AT_GOAL: pinky1" in c.last_warning
    w = LaneStatus()
    w.drive_state, w.route_seq, w.route_idx = LaneStatus.DRIVE_WAIT_CLEARANCE, ctx.route_seq, 0
    _run(c, {"pinky1": bl, "pinky2": bl}, 1, claims=[("pinky1", w)])      # 도착을 더는 말하지 않는다 → 경고도 없다
    assert "ARRIVED_NOT_AT_GOAL" not in c.last_warning
    _run(c, {"pinky1": bl, "pinky2": bl}, 45, claims=["pinky1"])      # 다시 말하면 처음부터 잰다
    assert "ARRIVED_NOT_AT_GOAL" not in c.last_warning


def test_FLEET_R1__받는_틱에도_재배정_직후에도_경고가_남지_않는다():
    c = _running_map4()
    ctx = c.robots["pinky1"]
    bl, tc = _node_xy(c, "BL"), _node_xy(c, "TC")
    _run(c, {"pinky1": bl, "pinky2": bl}, 25)
    _run(c, {"pinky1": bl, "pinky2": bl}, 60, claims=["pinky1"])
    assert "ARRIVED_NOT_AT_GOAL: pinky1" in c.last_warning
    for _ in range(40):                                       # 목표에 서서 계속 말한다 — 받는 바로 그 틱부터 경고가 없다
        _run(c, {"pinky1": tc, "pinky2": bl}, 1, claims=["pinky1"])
        if ctx.arrived:
            break
    assert ctx.arrived and "ARRIVED_NOT_AT_GOAL" not in c.last_warning, c.last_warning
    c2 = _running_map4()
    _run(c2, {"pinky1": bl, "pinky2": bl}, 25)
    _run(c2, {"pinky1": bl, "pinky2": bl}, 60, claims=["pinky1"])
    assert "ARRIVED_NOT_AT_GOAL: pinky1" in c2.last_warning
    assert c2.assign_route("pinky1", "BL", "TC")               # 운영자 재배정 — 에이전트의 새 보고가 닿기 전 틱에도
    _run(c2, {"pinky1": bl, "pinky2": bl}, 1)
    assert "ARRIVED_NOT_AT_GOAL" not in c2.last_warning, c2.last_warning


def test_FLEET_R1__옛_경로_번호의_도착은_목표에_서_있어도_받지_않는다():
    c = _running_map4()
    ctx = c.robots["pinky1"]
    tc, bl = _node_xy(c, "TC"), _node_xy(c, "BL")
    old = _arrived_msg(c, "pinky1", seq=ctx.route_seq - 1)
    _run(c, {"pinky1": tc, "pinky2": bl}, 60, claims=[("pinky1", old)])
    assert not ctx.arrived


@pytest.mark.parametrize("tol", [0.08, 0.15, 0.25])
def test_FLEET_R1__잡음_속에_목표에_선_로봇의_도착은_받는다__창의_가장_뒤가_아니라_과반(tol):
    """AMCL 식 잡음: 목표(허용 tol 만큼 앞)에 선 로봇의 포즈가 다섯 틱에 한 번 0.6 m 뒤로 튄다. 창의 가장 뒤(놓기 진행)로 재면
    창마다 튄 틱이 있어 영영 못 받는다(검토 모의 hold1hz·iid 에서 참 도착을 잃고 60 s 까지 늦었다). 과반은 선 자리다."""
    c = _running_map4()
    ctx = c.robots["pinky1"]
    res = c.reservation
    bl = _node_xy(c, "BL")
    cum = ctx.route.cumulative()
    near, back = _pos(ctx.route, cum, cum[-1] - tol), _pos(ctx.route, cum, cum[-1] - tol - 0.6)
    _run(c, {"pinky1": bl, "pinky2": bl}, 25)
    for i in range(80):
        _run(c, {"pinky1": back if i % 5 == 0 else near, "pinky2": bl}, 1, claims=["pinky1"])
    assert ctx.arrived, res.goal_gap("pinky1")


def test_FLEET_R1__바로잡힌_포즈_앞의_옛_포즈로는_도착을_받지_않는다__도착을_말한_뒤의_창만_본다():
    """월요일 §14-4: ① 뒤 옛 좌표계 포즈가 목표 근처였다가 ③ 으로 바로잡히자마자 START → 에이전트(START 전에 올라간 진행)가
    곧바로 도착을 말한다. 그때 확인 창은 아직 옛 포즈가 과반이다 — 말한 뒤 창 하나(2 s)를 기다려 선 자리만 본다."""
    c = _running_map4()
    ctx = c.robots["pinky1"]
    tc, bl = _node_xy(c, "TC"), _node_xy(c, "BL")
    _run(c, {"pinky1": tc, "pinky2": bl}, 30)                 # 옛 포즈가 목표(TC) 근처
    _run(c, {"pinky1": bl, "pinky2": bl}, 60, claims=["pinky1"])     # 바로잡힌 그 틱부터 도착을 말한다
    assert not ctx.arrived
    assert c.reservation.edge_holder.get("BL_JS") == "pinky1"


def test_FLEET_R1__비전_신선도_기준을_앞당겨도_지금보다_뒤에_받은_이벤트는_여전히_버린다():
    """기준 시각(ref)을 도착을 처음 말한 때로 당겨도 예전 규칙(나이 < 0 이면 무효 — 시계가 뒤로 간 경우)은 그대로다."""
    c = _running_map4()
    ctx = c.robots["pinky1"]
    ctx.last_zone_event = {"zone_id": "TC", "event_type": "ENTER", "received_at": c._t + 5.0}
    assert c._is_valid_goal_zone_event(ctx) is False
    assert c._is_valid_goal_zone_event(ctx, ref=c._t - 1.0) is False
    ctx.last_zone_event["received_at"] = c._t - 0.5              # 말한 뒤·받기 전에 온 이벤트는 유효
    assert c._is_valid_goal_zone_event(ctx, ref=c._t - 1.0) is True
    ctx.last_zone_event["received_at"] = c._t - 4.5              # 말하기 3.5 s 전 — 낡았다
    assert c._is_valid_goal_zone_event(ctx, ref=c._t - 1.0) is False


def test_FLEET_R1__늦게_받은_도착도_비전_신선도는_도착을_처음_말한_시각으로_잰다():
    """도착을 확인된 포즈로 받느라 늦게 받는다 — 비전 이벤트(목표 구역 ENTER, 태블릿은 PRESENT 를 되풀이하지 않는다)의
    신선도(goal_event_timeout 3 s)를 받는 순간으로 재면, 예전엔 확인되던 도착이 영영 ARRIVAL_PENDING 이었다."""
    import json
    from std_msgs.msg import String
    c = _running_map4()
    ctx = c.robots["pinky1"]
    bl, tc = _node_xy(c, "BL"), _node_xy(c, "TC")
    _run(c, {"pinky1": bl, "pinky2": bl}, 25)
    c._cb_vision_zone_event(String(data=json.dumps({"robot": "pinky1", "zone_id": "TC", "event": "ENTER"})))
    _run(c, {"pinky1": bl, "pinky2": bl}, 25)                 # 2.5 s 뒤 도착을 말한다(ENTER 는 2.5 s 전)
    _run(c, {"pinky1": tc, "pinky2": bl}, 30, claims=["pinky1"])
    assert ctx.arrived
    assert ctx.arrival_confirmed, "도착을 처음 말한 시각에 신선했던 ENTER 가 받는 순간엔 낡았다고 버려졌다"


def test_FLEET_R1__goal_gap_은_확인_창으로_잰다__튄_한두_틱이나_채워지지_않은_창으로는_줄지_않는다():
    r, rt, (x0, y0) = _map4_slot()
    total = rt.cumulative()[rt.goal_idx]
    assert r.goal_gap("pinky2") == pytest.approx(total)
    gx, gy = rt.waypoints[rt.goal_idx]
    for _ in range(2):
        r.update_pose("pinky2", gx, gy)
    assert r.goal_gap("pinky2") == pytest.approx(total)
    for _ in range(R.DEFAULT_CONFIRM_UPDATES):
        r.update_pose("pinky2", gx, gy)
    assert r.goal_gap("pinky2") == pytest.approx(0.0, abs=1e-6)
    assert r.goal_gap("nobody") == 0.0
    for _ in range(R.DEFAULT_CONFIRM_UPDATES):
        r.update_pose("pinky2", x0, y0)
    for _ in range(R.DEFAULT_CONFIRM_UPDATES // 2):          # 창의 딱 절반만 목표 — 과반이 아니면 목표 근처가 아니다
        r.update_pose("pinky2", gx, gy)
    assert r.goal_gap("pinky2") == pytest.approx(total)
    r.update_pose("pinky2", gx, gy)                            # 11/20 이 목표 — 과반
    assert r.goal_gap("pinky2") == pytest.approx(0.0, abs=1e-6)
    r.register("pinky2", 11, rt)                              # 새 경로 — 창이 빈 채 목표 포즈 몇 틱으로는 줄지 않는다
    for _ in range(3):
        r.update_pose("pinky2", gx, gy)
    assert r.goal_gap("pinky2") == pytest.approx(total)


@pytest.mark.parametrize("tol", [0.08, 0.15])
def test_FLEET_R1__BL_에서_기다리던_pinky2_가_목표까지_0_2s_튀어도_도착을_안_받고_엣지를_쥔다__재배정하면_실제로_간다(tol):
    """검토 closed_loop_probe2: 6b1d0d6 에선 pinky2 가 goto 0번으로 6.5 s 에 '도착'(실제 2.79 m 앞) — 엣지·BL 을 전부 놓았다.
    54bb60b 에선 영구 STALL_NO_REQUEST. 이제: 도착을 받지 않고 제 자리 엣지를 쥔 채 ARRIVED_NOT_AT_GOAL, 운영자가 다시 배정하면
    (40 s) 실제로 달려 목표에 닿는다. 에이전트(RouteFollower 의 달리는 중 단조 증가)는 이 묶음에서 바꾸지 않았다."""
    out = _closed_loop("map4", tol, glitch=("pinky2", (10, 11), 3.0), reassign=("pinky2", 40.0), T=120.0)
    assert "pinky2" in out["claim_far"]                       # 에이전트는 BL 에 선 채 도착을 말했다
    assert out["released_while_claim"] == [], out["released_while_claim"][:4]
    assert set(out["warn"]) == {"ARRIVED_NOT_AT_GOAL"} and "pinky2" in out["warn"]["ARRIVED_NOT_AT_GOAL"][1]
    assert out["warn"]["ARRIVED_NOT_AT_GOAL"][0] < 40.0
    assert out["stuck"] == {} and all(out["arrive"].values()), (out["arrive"], out["stuck"])
    assert out["arrive"]["pinky2"] > 40.0
    _assert_physical(out, tol)
    assert out["unheld"] == [] and out["both"] == [], (out["unheld"][:4], out["both"][:4])


@pytest.mark.parametrize("tol", [0.08, 0.15])
@pytest.mark.parametrize("at", [6.0, 8.0])
def test_FLEET_R1__달리던_pinky1_의_포즈가_목표까지_0_2s_튀어도_도중에_도착을_받아_엣지를_놓지_않는다(at, tol):
    """54bb60b 에서도 났다: 달리던 pinky1 의 포즈가 두 틱 TC 근처로 튀면 lead_s 로 목표까지 다 잡아 허가가 목표가 되고,
    RouteFollower 는 튄 자리로 올라가 지금 목표(중간 waypoint) 성공에서 '목표 도착' — mark_arrived 가 달리던 엣지를 놓았다
    (JW_JS 에 선 채 157 틱). 이제 도착을 받지 않고 쥔 채 경고한다(pinky1 은 운영자 재배정을 기다린다 — 저절로 움직이지 않는다)."""
    k = int(round(at / DT))
    out = _closed_loop("map4", tol, glitch=("pinky1", (k, k + 1), 3.0), T=60.0)
    assert "pinky1" in out["claim_far"]
    assert out["unheld"] == [] and out["released_while_claim"] == [], (out["unheld"][:4], out["released_while_claim"][:4])
    assert out["both"] == []
    _assert_physical(out, tol)
    assert out["arrive"]["pinky1"] is None and set(out["warn"]) == {"ARRIVED_NOT_AT_GOAL"}
    assert out["arrive"]["pinky2"] is not None                 # 다른 로봇은 제 길을 간다


@pytest.mark.parametrize("tol", [0.08, 0.15])
@pytest.mark.parametrize("legacy_ticks", [20, 100])
def test_FLEET_R1__START_전_옛_좌표계_포즈가_목표였어도__도착을_받는_순간엔_실제로_목표에_있다(legacy_ticks, tol):
    """검토 b6(월요일 §14-4: ① 뒤 옛 좌표계 포즈가 RE 근처 2·10 s, ③ 으로 바로잡힌 뒤 3 s 에 START). 6b1d0d6: pinky2 가 BL 에
    선 채 6.5 s 에 '도착'(2.79 m 앞). START 전 단조 증가는 에이전트 묶음이 고친다 — 이 시험은 어느 쪽 에이전트든 성립해야
    한다: 받은 도착은 늘 실제 도착이고, 목표에서 먼 도착 보고는 경고로 보이며, 재배정(40 s)이 실제 도착으로 끝낸다."""
    c0 = _pcoord()
    assert c0.switch_profile("map4")
    out = _closed_loop("map4", tol, pre=("pinky2", _node_xy(c0, "RE"), legacy_ticks), reassign=("pinky2", 40.0),
                       T=120.0)
    assert out["stuck"] == {} and all(out["arrive"].values()), (out["arrive"], out["stuck"])
    _assert_physical(out, tol)
    assert out["released_while_claim"] == [], out["released_while_claim"][:4]
    assert out["unheld"] == [] and out["both"] == [], (out["unheld"][:4], out["both"][:4])
    assert set(out["warn"]) <= {"ARRIVED_NOT_AT_GOAL"}, out["warn"]
    if out["claim_far"]:
        assert "ARRIVED_NOT_AT_GOAL" in out["warn"]


# ==== F1: 플릿 STOPPED·DONE 중 래치 로봇의 재개는 goto 를 만들지 않는다 ===============================================

def _started_latched(c, clear=5):
    ch = _chain_for(c)
    seq = c.robots["pinky1"].route_seq
    ch.on_lane_command(LaneCommand.CMD_START, seq, 0)
    assert _gotos(ch.on_lane_command(LaneCommand.CMD_CLEARANCE, seq, clear))
    ch.on_link_lost(moving=True)
    assert ch.link_lost and ch.active_target is None
    return ch


@pytest.mark.parametrize("state", ["STOPPED", "DONE"])
def test_F1__R5_는_RESUME_앞에_허가_0__RESUME_까지만_처리한_순간에도_goto_가_없다(state):
    """고치기 전: [RESUME, STOP] — RESUME 을 처리한 순간 체인은 옛 허가(5)로 goto 를 냈다. 그 goto 는 해제 문 뒤에 쌓이고,
    Nav2 취소 확인이 STOP 보다 먼저 닿으면 플릿 STOPPED 중에 진짜 목표가 나갔다(검토 s2.py S4-race: goals [(108.0, chain 5)])."""
    c = _coord(state=state)
    ch = _started_latched(c)
    _lane_status(c, "pinky1", LL, 100.0, "관제 링크 유실")
    assert c.robots["pinky1"].held
    _reset(c)
    assert c.resume_robot("pinky1") is True
    sent = _sent(c.lane_cmd_pubs["pinky1"])
    assert [m.command for m in sent] == [LaneCommand.CMD_CLEARANCE, LaneCommand.CMD_RESUME, LaneCommand.CMD_STOP]
    assert sent[0].clear_until_idx == 0 and sent[0].route_seq == c.robots["pinky1"].route_seq
    acts, rest = _feed(ch, c, upto=LaneCommand.CMD_RESUME)
    assert not ch.latched and ("estop", False) in acts        # 래치는 풀렸다
    assert _gotos(acts) == [] and ch.active_target is None    # 그러나 갈 목표는 없다
    for m in rest:
        ch.on_lane_command(m.command, m.route_seq, m.clear_until_idx)
    assert ch.stopped


def test_F1__플릿이_달리던_곳으로_돌아가면_다음_허가로_간다():
    """허가 0 은 머무르지 않는다 — 플릿 재개(RUNNING)의 첫 틱 CLEARANCE 가 진짜 허가를 다시 준다."""
    c = _coord(state="RUNNING")
    ch = _started_latched(c)
    c.stop_fleet()
    _lane_status(c, "pinky1", LL, 100.0, "관제 링크 유실")
    _reset(c)
    c.resume_robot("pinky1")
    _feed(ch, c)
    assert ch.clear_until_idx == 0 and ch.stopped and not ch.latched
    _reset(c)
    c.resume_fleet()
    assert c.mission_state == "RUNNING"
    c._publish_lane_commands()
    acts, _ = _feed(ch, c)
    assert [g[1] for g in _gotos(acts)] == [5]


@pytest.mark.parametrize("state", ["STOPPED", "DONE"])
def test_F1__진짜_에이전트__RESUME_뒤_Nav2_취소_확인이_STOP_보다_먼저_와도_목표를_안_보낸다(state):
    """검토 s2.py S4-race 를 진짜 PinkyAgent 메서드로: 해제 문(취소 확인 대기) → 확인 응답 → 그 뒤에 STOP.
    고치기 전: 응답 순간 문이 열리며 ('chain', 5) 목표가 Nav2 로 나갔다."""
    clk = TA.Clock(100.0)
    a, _ = TA._agent(clk)
    c = _coord(state="RUNNING")
    ctx = c.robots["pinky1"]
    msg = c._to_route_msg("pinky1", ctx.route_seq, ctx.route)
    a._chain.on_route(msg.route_seq, [(p.x, p.y) for p in msg.waypoints], msg.goal_idx)
    a._on_lane_command(TA._lane(LaneCommand.CMD_START, seq=ctx.route_seq))
    a._on_lane_command(TA._lane(LaneCommand.CMD_CLEARANCE, clear=5, seq=ctx.route_seq))
    assert a._send_goal_to.call_args[0][3] == ("chain", 5)
    if state == "STOPPED":
        c.stop_fleet()
    else:
        c.mission_state = "DONE"
    _reset(c)
    c._publish_lane_commands()
    for m in _sent(c.lane_cmd_pubs["pinky1"]):
        a._on_lane_command(m)
    assert a._chain.stopped
    a._apply(a._chain.on_link_lost(moving=True))               # 게이트웨이 재기동 중 데드맨
    assert a._chain.latched
    _lane_status(c, "pinky1", LL, 101.0, "관제 링크 유실", seq=ctx.route_seq)
    assert ctx.held
    _reset(c)
    a._send_goal_to.reset_mock()
    assert c.resume_robot("pinky1") is True
    sent = _sent(c.lane_cmd_pubs["pinky1"])
    k = [m.command for m in sent].index(LaneCommand.CMD_RESUME)
    for m in sent[:k + 1]:
        a._on_lane_command(m)
    TA._answer(a)                                             # 취소 확인이 STOP 보다 먼저 닿았다
    assert not a._chain.latched
    assert [cl for cl in a._send_goal_to.call_args_list if cl[0][3][0] == "chain"] == []
    for m in sent[k + 1:]:
        a._on_lane_command(m)
    assert a._chain.stopped and a._send_goal_to.call_count == 0


# ==== F2: RUNNING 중 시작 전으로 돌아간 Nav2 로봇 ==================================================================

def _running_started(clear=5):
    c = _coord(state="RUNNING")
    ch = _chain_for(c)
    seq = c.robots["pinky1"].route_seq
    ch.on_lane_command(LaneCommand.CMD_START, seq, 0)
    ch.on_lane_command(LaneCommand.CMD_CLEARANCE, seq, clear)
    _report(c, ch)
    assert c.robots["pinky1"].start_acknowledged
    return c, ch


def _idle_warn(c):
    return [w for w in c._check_stalls(c._t) if w.startswith("ROBOT_IDLE_IN_RUNNING")]


def test_F2__같은_번호_경로를_다시_받아_시작_전이_된_로봇은_경고하되_저절로_START_를_안_보낸다():
    """브리지 재시작이 TRANSIENT_LOCAL Route 를 같은 번호로 다시 준다 → on_route 가 started 를 지운다 → IDLE.
    고치기 전: 경고 없음, START 도 영영 없음(검토 s11.py·k2.py). 고친 뒤: 몇 초 뒤 경고, 그래도 START 는 운영자가 누를 때만."""
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ch.on_route(ctx.route_seq, list(ch.follower.points), ch.goal_idx)
    assert ch.drive_state() == rc.DRIVE_IDLE
    warn_sec = getattr(c, "IDLE_IN_RUNNING_WARN_SEC", 3.0)
    _reset(c)
    c.fed = []
    seen = []
    for _ in range(int((warn_sec + 2.0) / DT)):
        _report(c, ch)
        seen = _idle_warn(c) or seen
        c._publish_lane_commands()
        _feed(ch, c)
    assert seen and seen[0].startswith("ROBOT_IDLE_IN_RUNNING: pinky1 — 로봇 재개 필요"), seen
    assert c.fed and LaneCommand.CMD_START not in c.fed                  # 저절로 출발 안 함(허가만 계속 나간다)
    assert ch.drive_state() == rc.DRIVE_IDLE and ctx.start_acknowledged


def test_F2__경고는_몇_초_이어져야_나고_시작_전이_풀리면_사라진다():
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    warn_sec = getattr(c, "IDLE_IN_RUNNING_WARN_SEC", 3.0)
    ch.on_route(ctx.route_seq, list(ch.follower.points), ch.goal_idx)
    _report(c, ch)
    t0 = c._t
    assert _idle_warn(c) == []
    c._t = t0 + warn_sec - 0.2
    assert _idle_warn(c) == []
    c._t = t0 + warn_sec + 0.1
    assert _idle_warn(c)
    ch.on_lane_command(LaneCommand.CMD_START, ctx.route_seq, 0)
    _report(c, ch)
    assert _idle_warn(c) == []


def test_F2__지금_IDLE_인_로봇의_로봇_재개는_START_를_다시_무장한다():
    """에이전트 프로세스 재시작(k2.py): 래치 없이 '경로 수신 — START 대기' 로 IDLE. 경고를 본 운영자가 로봇 재개를 누르면 간다."""
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ch2 = _chain_for(c)                                       # 새 에이전트 — TRANSIENT_LOCAL Route 만 받았다
    _report(c, ch2)
    _reset(c)
    assert c.resume_robot("pinky1") is True
    assert not ctx.start_acknowledged
    c._publish_lane_commands()
    starts = [m for m in _sent(c.lane_cmd_pubs["pinky1"]) if m.command == LaneCommand.CMD_START]
    assert starts and starts[0].route_seq == ctx.route_seq
    _feed(ch2, c)
    assert ch2.started
    _report(c, ch2)
    assert ctx.start_acknowledged


def test_F2__S11__링크유실_래치_중_누른_재개는_래치가_풀린_뒤_IDLE_보고로_START_를_무장한다():
    """검토 s11.py: 달리던 중 브리지만 4 s 재시작 → 데드맨 래치 + 같은 번호 Route 재수신(started 지움) → 링크유실로 세움.
    그 순간 보고는 LINK_LOST 라 시작 전인지 모른다. 고치기 전: 재개가 RESUME 만 보내 로봇은 'RESUME' 사유로 영영 IDLE."""
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ch.on_link_lost(moving=True)
    ch.on_route(ctx.route_seq, list(ch.follower.points), ch.goal_idx)
    _report(c, ch)
    assert ctx.held and ctx.held_reason == FC.HELD_LINK_LOST
    _reset(c)
    assert c.resume_robot("pinky1") is True
    assert ctx.start_acknowledged                              # 아직 모른다(LINK_LOST 보고) — 바로 무장하지 않는다
    _feed(ch, c)
    assert not ch.latched and ch.drive_state() == rc.DRIVE_IDLE
    _report(c, ch)
    assert not ctx.start_acknowledged                          # 풀린 뒤 첫 보고가 '시작 전' — START 재무장
    _reset(c)
    c._publish_lane_commands()
    _feed(ch, c)
    assert ch.started
    c._publish_lane_commands()                                # ack 전엔 CLEARANCE 대신 START 재전송이 나간다 — ack 뒤 허가
    _report(c, ch)
    assert ctx.start_acknowledged
    c._publish_lane_commands()
    acts, _ = _feed(ch, c)
    assert ch.drive_state() == rc.DRIVE_CRUISE and ch.active_target == 5


def test_F2__재개_유예가_지난_뒤의_IDLE_은_START_를_안_받는다__경고만():
    """재개 뒤 로봇이 달리다가(유예 LINK_LOST_RESUME_GRACE_SEC 가 지난 뒤) 에이전트가 재시작해 IDLE 이 되면 — 그건 운영자가
    누른 재개의 결과가 아니다. 저절로 출발시키지 않고 경고만 한다(다시 누르면 간다)."""
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    c.stop_robot("pinky1")
    _feed(ch, c)
    _report(c, ch)
    c.resume_robot("pinky1")
    _feed(ch, c)
    for _ in range(int(c.LINK_LOST_RESUME_GRACE_SEC / DT) + 5):
        _report(c, ch)                                        # 재개가 닿아 다시 달린다
    assert ch.drive_state() == rc.DRIVE_CRUISE
    ch2 = _chain_for(c)                                       # 그 뒤 에이전트 재시작
    _report(c, ch2)
    assert ctx.start_acknowledged                             # 무장 안 함
    assert _idle_warn(c) == []                                # 경고 시계가 여기서 선다
    c._t += getattr(c, "IDLE_IN_RUNNING_WARN_SEC", 3.0) + 0.5
    _report(c, ch2)
    assert ctx.start_acknowledged
    assert _idle_warn(c)                                      # 경고
    c.resume_robot("pinky1")                                  # 운영자가 다시 누르면 무장
    assert not ctx.start_acknowledged


def test_F2__달리던_로봇의_재개는_START_를_다시_무장하지_않는다():
    """운영자가 세웠다 푼 로봇(체인 started, STOP 래치) — 재개 뒤 대기·주행을 보고한다. START 재전송은 필요 없다."""
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    c.stop_robot("pinky1")
    _feed(ch, c)
    _report(c, ch)
    _reset(c)
    c.resume_robot("pinky1")
    _feed(ch, c)
    for _ in range(5):
        _report(c, ch)
    assert ctx.start_acknowledged
    c._publish_lane_commands()
    assert [m.command for m in _sent(c.lane_cmd_pubs["pinky1"])][-1] == LaneCommand.CMD_CLEARANCE


def test_F2__START_를_아직_확인_못_한_IDLE_은_경고하지_않는다__재전송이_맡는다():
    """RUNNING 중 재배정(새 번호) — START 재전송이 돌고, 다 쓰면 start.gave_up 이 따로 보인다. 이 경고는 '확인했던 START 가
    지워진' 경우만이다."""
    c = _pcoord(state="RUNNING")
    assert c.assign_route("pinky1", "START_A", "GOAL_C")
    ctx = c.robots["pinky1"]
    assert not ctx.start_acknowledged and ctx.start_armed
    ch = _chain_for(c)
    _report(c, ch)
    _idle_warn(c)
    c._t += getattr(c, "IDLE_IN_RUNNING_WARN_SEC", 3.0) + 1.0
    _report(c, ch)
    assert _idle_warn(c) == []


def test_F2__재개_한_번에_START_무장도_한_번__곧이어_또_시작_전이_되면_경고로():
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ch2 = _chain_for(c)
    _report(c, ch2)
    c.resume_robot("pinky1")
    assert not ctx.start_acknowledged                          # 이 누름의 무장
    c._publish_lane_commands()
    _feed(ch2, c)
    _report(c, ch2)
    assert ctx.start_acknowledged and ch2.started
    ch3 = _chain_for(c)                                       # 유예 안에 에이전트가 또 재시작
    _report(c, ch3)
    assert ctx.start_acknowledged                              # 운영자가 다시 누르기 전엔 무장 안 함


def test_F2__재개_유예_안에_다시_세우면_무장하지_않는다():
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ch.on_link_lost(moving=True)
    ch.on_route(ctx.route_seq, list(ch.follower.points), ch.goal_idx)
    _report(c, ch)
    c.resume_robot("pinky1")
    _feed(ch, c)
    c.stop_robot("pinky1")                                    # 운영자가 곧바로 다시 세웠다
    _report(c, ch)
    assert ch.drive_state() == rc.DRIVE_IDLE and ctx.held
    assert ctx.start_acknowledged


@pytest.mark.parametrize("setup", ["held", "stopped", "old_seq"])
def test_F2__세워_뒀거나_달리지_않는_플릿이거나_옛_경로_번호면_경고하지_않는다(setup):
    """세운 로봇은 카드가 세운 이유를 보인다 · 경고는 달리는 플릿에서만 · '시작 전' 은 지금 경로 번호의 IDLE 만이다."""
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ctx.held = setup == "held"
    if setup == "stopped":
        c.mission_state = "STOPPED"
    seq = ctx.route_seq - 1 if setup == "old_seq" else ctx.route_seq
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, c._t + 0.1, "경로 수신 — START 대기", seq=seq)
    assert _idle_warn(c) == []
    c._t += getattr(c, "IDLE_IN_RUNNING_WARN_SEC", 3.0) + 1.0
    assert _idle_warn(c) == []
    if setup == "old_seq":
        c.resume_robot("pinky1")
        assert ctx.start_acknowledged


def test_F2__재개_유예_안이라도_플릿이_RUNNING_이_아니면_무장하지_않는다():
    """L6 과 같은 규칙 — 달리지 않는 플릿에 START 를 무장해 두지 않는다(재개 한 번에 출발하지 않게)."""
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ch.on_link_lost(moving=True)
    ch.on_route(ctx.route_seq, list(ch.follower.points), ch.goal_idx)
    _report(c, ch)
    c.resume_robot("pinky1")
    _feed(ch, c)
    c.stop_fleet()                                            # 유예 안에 플릿 정지
    _report(c, ch)
    assert ch.drive_state() == rc.DRIVE_IDLE
    assert ctx.start_acknowledged


def test_F2__도착한_로봇은_재개로_START_를_다시_받지_않는다():
    c, ch = _running_started()
    ctx = c.robots["pinky1"]
    ctx.arrived = True
    ch2 = _chain_for(c)                                       # 도착 뒤 에이전트 재시작 — 경로를 다시 받아 IDLE
    _report(c, ch2)
    c.resume_robot("pinky1")
    _report(c, ch2)
    assert ctx.start_acknowledged


def test_F2__레인_로봇은_이_규칙을_안_탄다():
    """팀11 레인 로봇의 IDLE 사정은 다르다(STOP 이 started 를 지운다) — 이 변경은 Nav2 에이전트의 '시작 전' 만 다룬다."""
    c = _coord(state="RUNNING")
    ctx = c.robots["pinky1"]
    ctx.drive_mode = FC.DRIVE_MODE_LANE
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 100.0, "대기", seq=ctx.route_seq)
    c._t = 110.0
    assert _idle_warn(c) == []
    c.resume_robot("pinky1")
    assert ctx.start_acknowledged


# ==== F3: 팀11 레인 로봇의 LINK_LOST 는 래치가 아니다 ===============================================================

def test_F3__레인_로봇의_LINK_LOST_는_세우지_않는다():
    """팀11 drive_fsm: started 이고 하트비트·LanePath 가 0.9 s 끊기면 LINK_LOST, 돌아오면 저절로 CRUISE. 고치기 전: 세움(STOP →
    팀11 started=False) → 재개는 RESUME 만(팀11 은 estop 만 푼다) → 영영 IDLE(검토 s8.py)."""
    c = _coord(state="RUNNING")
    c.robots["pinky1"].drive_mode = FC.DRIVE_MODE_LANE
    _lane_status(c, "pinky1", LL, 100.0, "LanePath 끊김")
    assert not c.robots["pinky1"].held
    _reset(c)
    c._publish_lane_commands()
    assert LaneCommand.CMD_STOP not in [m.command for m in _sent(c.lane_cmd_pubs["pinky1"])]
    _lane_status(c, "pinky2", LL, 100.1, "관제 링크 유실")      # Nav2 로봇은 그대로 세운다
    assert c.robots["pinky2"].held and c.robots["pinky2"].held_reason == FC.HELD_LINK_LOST


def test_F3__레인_로봇의_LINK_LOST_는_정지_중_재개를_래치_해제로_만들지_않는다():
    c = _coord(state="STOPPED")
    c.robots["pinky1"].drive_mode = FC.DRIVE_MODE_LANE
    c.stop_robot("pinky1")
    _lane_status(c, "pinky1", LL, 100.0, "LanePath 끊김")
    _reset(c)
    c.resume_robot("pinky1")
    assert _sent(c.lane_cmd_pubs["pinky1"]) == [] and _sent(c.fleet_cmd_pubs["pinky1"]) == []   # R5 그대로


# ==== F4 · E2E-2: 경로 없는 Nav2 로봇의 링크유실 래치 ===============================================================

def _state(c, name, nav, t):
    c._t = t
    c._cb_robot_state(name, RobotState(localized=True, nav_status=nav))


def _routeless(state="DONE"):
    c = _coord(state=state)
    c.robots["pinky2"].route = None                           # legacy: 둘 다 GOAL_C 라 pinky2 는 배정 거절(§14-5-3)
    return c


def test_F4__경로_없는_로봇의_NAV_LINK_LOST_가_이어지면_세우고_이유를_보인다():
    """고치기 전(e2e_monday/routeless): 에이전트는 drive=9 link_lost=True 인데 코디네이터는 held=False, lane_status 없음 —
    V2 카드는 초록 '로봇별 정지 없음'."""
    c = _routeless()
    confirm = getattr(c, "NAV_LINK_LOST_CONFIRM_SEC", 3.0)
    t = 100.0
    while t < 100.0 + confirm - 0.15:
        _state(c, "pinky2", RobotState.NAV_LINK_LOST, t)
        t += DT
    assert not c.robots["pinky2"].held                         # 아직 — 재기동 직후 맥박 전의 LinkWatch 상태일 수 있다
    _state(c, "pinky2", RobotState.NAV_LINK_LOST, 100.0 + confirm + 0.05)
    ctx = c.robots["pinky2"]
    assert ctx.held and ctx.held_reason == FC.HELD_LINK_LOST
    assert c.get_fleet_status_dict()["robots"]["pinky2"]["held_reason"] == FC.HELD_LINK_LOST
    assert ctx.lane_status is None


def test_F4__짧은_NAV_LINK_LOST_는_세우지_않는다__맥박이_닿으면_풀리는_LinkWatch_상태():
    """E2E-4: 멈춰 있던 로봇은 재기동에 래치되지 않는다. 그래도 새 코디네이터의 첫 맥박이 닿기 전 보고는 NAV_LINK_LOST 다."""
    c = _routeless(state="ASSIGNED")
    for i in range(20):
        _state(c, "pinky2", RobotState.NAV_LINK_LOST, 100.0 + i * DT)
    _state(c, "pinky2", RobotState.NAV_IDLE, 102.0)
    _state(c, "pinky2", RobotState.NAV_LINK_LOST, 102.1)       # 새로 끊겨도 확인 시간은 처음부터
    _state(c, "pinky2", RobotState.NAV_IDLE, 102.2)
    _state(c, "pinky2", RobotState.NAV_IDLE, 110.0)
    assert not c.robots["pinky2"].held


@pytest.mark.parametrize("state", ["STOPPED", "DONE"])
def test_F4__로봇_재개가_경로_없는_로봇의_링크유실_래치를_푼다(state):
    """고치기 전: 플릿 STOPPED·DONE 중 재개는 robot_latched 가 거짓이라 아무것도 안 보내고 200 — 래치·/estop true 가 남았다."""
    c = _routeless(state=state)
    ch = RouteChain()                                         # 경로를 한 번도 못 받은 에이전트
    ch.on_link_lost(moving=True)
    assert ch.drive_state() == rc.DRIVE_LINK_LOST
    for i in range(40):
        _state(c, "pinky2", RobotState.NAV_LINK_LOST, 100.0 + i * DT)
    _reset(c)
    assert c.resume_robot("pinky2") is True
    cmds = [m.command for m in _sent(c.lane_cmd_pubs["pinky2"])]
    assert LaneCommand.CMD_RESUME in cmds and cmds[-1] == LaneCommand.CMD_STOP
    acts, _ = _feed(ch, c, name="pinky2")
    assert not ch.link_lost and ("estop", False) in acts and _gotos(acts) == []


def test_F4__재개_뒤_유예_안의_옛_NAV_LINK_LOST_로_다시_세우지_않고__안_닿았으면_유예_뒤_다시_세운다():
    c = _routeless(state="ASSIGNED")
    for i in range(40):
        _state(c, "pinky2", RobotState.NAV_LINK_LOST, 100.0 + i * DT)
    assert c.robots["pinky2"].held
    c._t = 104.0
    c.resume_robot("pinky2")
    _state(c, "pinky2", RobotState.NAV_LINK_LOST, 104.1)       # RESUME 전에 떠난 옛 보고
    assert not c.robots["pinky2"].held
    _state(c, "pinky2", RobotState.NAV_LINK_LOST, 104.0 + c.LINK_LOST_RESUME_GRACE_SEC + 0.1)
    assert c.robots["pinky2"].held and c.robots["pinky2"].held_reason == FC.HELD_LINK_LOST


def test_F4__다른_토픽의_옛_보고가_늦게_와도_방금_푼_로봇을_다시_세우지_않는다():
    """RESUME 을 처리한 틱의 LaneStatus(풀림)가 그 전 틱의 RobotState(NAV_LINK_LOST)보다 먼저 닿을 수 있다(토픽이 다르다).
    LaneStatus 하나만 보고 유예를 끝내면 늦게 온 RobotState 가 방금 푼 로봇을 다시 세운다 — 둘 다 풀렸을 때만 끝낸다."""
    c = _coord(state="RUNNING")
    ctx = c.robots["pinky1"]
    for i in range(40):
        _state(c, "pinky1", RobotState.NAV_LINK_LOST, 100.0 + i * DT)
    _lane_status(c, "pinky1", LL, 104.0, "관제 링크 유실", seq=ctx.route_seq)
    assert ctx.held
    c.resume_robot("pinky1")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 104.1, "RESUME", seq=ctx.route_seq)
    _state(c, "pinky1", RobotState.NAV_LINK_LOST, 104.15)      # 늦게 닿은 옛 RobotState
    assert not ctx.held
    _state(c, "pinky1", RobotState.NAV_HOLD, 104.2)            # 둘 다 풀렸다 — 유예 끝
    _state(c, "pinky1", RobotState.NAV_LINK_LOST, 104.3)       # 새 유실은 확인 시간 뒤 다시 세운다
    _state(c, "pinky1", RobotState.NAV_LINK_LOST, 104.3 + getattr(c, "NAV_LINK_LOST_CONFIRM_SEC", 3.0) + 0.05)
    assert ctx.held and ctx.held_reason == FC.HELD_LINK_LOST


@pytest.mark.parametrize("state", ["STOPPED", "DONE"])
def test_F4__운영자가_세운_경로_없는_로봇이_NAV_LINK_LOST_면_정지·완료_중_재개가_RESUME_을_보낸다(state):
    """세운 이유가 '로봇별 정지' 여도 로봇이 지금 링크유실을 말하면 그 래치는 RESUME 으로만 풀린다(REVIEW G 의 LaneStatus 규칙과 같다)."""
    c = _routeless(state=state)
    c.stop_robot("pinky2")
    _state(c, "pinky2", RobotState.NAV_LINK_LOST, 100.0)
    assert c.get_fleet_status_dict()["robots"]["pinky2"]["held_reason"] == FC.HELD_BY_OPERATOR
    _reset(c)
    c.resume_robot("pinky2")
    assert LaneCommand.CMD_RESUME in [m.command for m in _sent(c.lane_cmd_pubs["pinky2"])]


def test_F4__LaneStatus_가_먼저_풀려도_RobotState_가_아직_링크유실이면_유예를_끝내지_않는다():
    """LaneStatus LL 로 바로 세운 로봇 — 그때 RobotState 의 NAV_LINK_LOST 는 확인 시간 전이었다. 재개 뒤 LaneStatus(풀림)가
    먼저 닿고 옛 RobotState 가 늦게 닿아 확인 시간을 채우면, LaneStatus 만으로 유예를 끝냈을 때 방금 푼 로봇을 다시 세운다."""
    c = _coord(state="RUNNING")
    ctx = c.robots["pinky1"]
    _state(c, "pinky1", RobotState.NAV_LINK_LOST, 100.0)
    _lane_status(c, "pinky1", LL, 100.5, "관제 링크 유실", seq=ctx.route_seq)
    assert ctx.held
    c._t = 100.6
    c.resume_robot("pinky1")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_WAIT_CLEARANCE, 100.7, "RESUME", seq=ctx.route_seq)
    _state(c, "pinky1", RobotState.NAV_LINK_LOST, 100.0 + getattr(c, "NAV_LINK_LOST_CONFIRM_SEC", 3.0) + 0.1)
    assert not ctx.held


def test_F4__레인_로봇의_NAV_LINK_LOST_도_세우지_않는다():
    c = _coord(state="RUNNING")
    c.robots["pinky1"].drive_mode = FC.DRIVE_MODE_LANE
    for i in range(50):
        _state(c, "pinky1", RobotState.NAV_LINK_LOST, 100.0 + i * DT)
    assert not c.robots["pinky1"].held
