# -*- coding: utf-8 -*-
"""REVIEW_20260926 후속 — 예약 진행도(H) · 시작 직후 두 엣지(관찰) · 지도 칸 검사(중계 항목).

H (리그 재검 REVIEW_20260926_RIG_RECHECK §2): 정적 1 Hz fix 와 odom 전파 사이에서 보고 포즈가 BL 주변을 헤매자, 단조 증가
진행도가 헤맨 최대치를 누적해 물리적으로 BL 에 선 pinky2 가 BL·BL_JS 를 놓고 RM_RE 까지 잡았다(로봇이 아직 있는 엣지를
내줄 수 있다). → 잡기(lead_s, 가장 앞)와 놓기(progress_s, 확인된 것)를 나눴다. 여기서 재는 것:
  - 재현: 진짜 코디네이터 틱(map4 프로파일, 시나리오 1)에서 헤매는 pinky2 가 BL·BL_JS 를 놓지 않는다
    (고치기 전: odom 모형 4.5 s 에 BL·BL_JS 해제, 끝에 진행 1.60 · 잡은 것 [BR_RM, RM_RE] — 리그 H 와 같다)
  - (b) 확인 창보다 짧은 튐은 몇 번 되풀이돼도 놓지 않는다 · 멈춘 로봇은 창 뒤에 제자리까지 확인된다
  - (a) 창보다 오래 간 튐도 max_speed 로만 번진다
  - (c) 연달아 뒤면 잡기 진행도를 되돌린다 — 놓은 엣지는 되찾지 않는다
  - 정상 주행(잡음 ±3 cm · 0.15/0.25 m/s)은 두 시나리오 모두 도착하고, 무엇을 놓을 때 로봇은 늘 그것을 지났다
  - 적대 검토 RES-1: 놓기 진행은 내려가는 것도 따른다 — 창보다 오래 틀린 포즈(전환 뒤 옛 좌표계, 지난 목표)가 바로잡힌
    뒤 START 해도, 출발 노드에 선 로봇의 허가 구간은 늘 제가 쥔 엣지다(진짜 코디네이터 틱)
지도 칸: 도로망 엣지가 지도의 점유·알 수 없음·지도 밖 칸을 지나면 경고(막지 않는다) — map4 실제 파일 + 합성 작은 지도.
  - 적대 검토 RES-2: 미션 로봇 이름이 글자가 아니어도 목록 읽기가 터지지 않는다

reservation 의 새 이름은 모듈에서 꺼낸다(`R.…`) — 고치기 전 코드로 돌려도 모음이 깨지지 않고 시험마다 판정이 나오게.
⚠️ 저장 파일은 늘 임시 폴더로(PINKY_RELAY_STATE_DIR) — 실물 중계의 ~/.local/state 를 건드리지 않는다.
"""
import math
import os
import random
import re
import sys

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pinky_fleet_msgs.msg import RobotState  # noqa: E402
from relay_station.fleet import profiles as P  # noqa: E402
from relay_station.fleet import reservation as R  # noqa: E402
from relay_station.fleet.road_graph import RoadGraph  # noqa: E402
from test_fleet_profiles import _pcoord, gw  # noqa: E402,F401 — gw 는 게이트웨이 조립 fixture

MAP4_DIR = os.path.join(REPO, "relay_station", "fleet", "config", "profiles", "team11_map4")
MAP4_GRAPH = os.path.join(MAP4_DIR, "road_graph.yaml")
LEGACY_GRAPH = os.path.join(REPO, "relay_station", "fleet", "config", "road_graph.yaml")
NAV2_FLEET = os.path.join(REPO, "pinky_fleet_agent", "params", "nav2_params_fleet.yaml")
STATIC = os.path.join(REPO, "relay_station", "gateway_web", "static")


def _pos(route, cum, s):
    """경로 위 호길이 s 의 (x, y)."""
    import bisect
    pts = route.waypoints
    i = max(0, min(len(pts) - 2, bisect.bisect_right(cum, s) - 1))
    seg = cum[i + 1] - cum[i]
    t = 0.0 if seg <= 0 else (s - cum[i]) / seg
    return (pts[i][0] + t * (pts[i + 1][0] - pts[i][0]), pts[i][1] + t * (pts[i + 1][1] - pts[i][1]))
BL = (0.95, -0.45)


@pytest.fixture(autouse=True)
def _state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv(P.STATE_ENV, str(tmp_path / "state"))


# ---- H 헤매는 포즈 모형 (둘 다 진실은 BL — 로봇은 움직이지 않았다) ---------------------------------

def _odom_wander(route, cum):
    """리그 그대로: Nav2 가 Gazebo 로봇을 경로를 따라 몬다(odom 0.15 m/s, 1.6 m 까지) · 정적 fix 가 1 Hz 로 BL 로 되당긴다."""
    return lambda k: BL if k % 10 == 0 else _pos(route, cum, min(0.015 * k, 1.6))


def _random_wander(route, cum, seed=926):
    """BL 주변 0.8 m 안에서 헤맨다 — 1 Hz fix 가 BL 로 되당기고, 그 사이엔 무작위 점 쪽으로 번진다."""
    rng, tgt = random.Random(seed), [BL]

    def at(k):
        if k % 10 == 0:
            r, th = 0.8 * math.sqrt(rng.random()), rng.uniform(-math.pi, math.pi)
            tgt[0] = (BL[0] + r * math.cos(th), BL[1] + r * math.sin(th))
            return BL
        u = (k % 10) / 9.0
        return BL[0] + (tgt[0][0] - BL[0]) * u, BL[1] + (tgt[0][1] - BL[1]) * u
    return at


@pytest.mark.parametrize("model", ["odom", "wander"])
def test_H_재현__코디네이터_틱에서_BL_에_선_pinky2_가_헤매도_BL·BL_JS_를_놓지_않는다(model):
    """시나리오 1(둘 다 BL 출발): pinky1 은 허가대로 0.15 m/s 로 가고, pinky2 는 BL 에 선 채 보고 포즈만 헤맨다.
    고치기 전(단조 증가 하나): pinky1 이 놓은 BL_JS 를 4.4 s 에 받고 0.1 s 뒤 BL·BL_JS 를 놓았다(진행 0.675/0.89).
    고친 뒤: 6.3 s 에 받고(pinky1 의 놓기가 확인 창만큼 늦다) 끝까지 쥔다 — 진행 0.000."""
    c = _pcoord()
    assert c.switch_profile("map4")
    c.state_timeout_sec, c.global_seq, c.mission_state = 2.0, 0, "RUNNING"
    r1, r2 = c.robots["pinky1"].route, c.robots["pinky2"].route
    cum1, cum2 = r1.cumulative(), r2.cumulative()
    pose2 = (_odom_wander if model == "odom" else _random_wander)(r2, cum2)
    s1, got = 0.0, None
    for k in range(400):                                   # 10 Hz × 40 s
        c._t += 0.1
        s1 = min(max(s1, cum1[c.robots["pinky1"].clear_until_idx]), s1 + 0.015)
        c._cb_robot_state("pinky1", RobotState(localized=True, x=_pos(r1, cum1, s1)[0], y=_pos(r1, cum1, s1)[1]))
        x2, y2 = pose2(k)
        c._cb_robot_state("pinky2", RobotState(localized=True, x=x2, y=y2))
        c._loop_tick()
        res = c.reservation
        if got is None and res.edge_holder.get("BL_JS") == "pinky2":
            got = k                                        # pinky1 이 놓은 뒤 pinky2 가 받았다
        if got is not None:
            assert res.edge_holder.get("BL_JS") == "pinky2" and res.node_holder.get("BL") == "pinky2", \
                (model, k * 0.1, res.status("pinky2"), res.robots["pinky2"].progress_s)
    assert got is not None, "pinky1 이 BL_JS 를 놓은 뒤 pinky2 가 받아야 한다"
    assert s1 == pytest.approx(cum1[-1])                   # pinky1 은 막힘 없이 TC 까지
    assert c.reservation.robots["pinky2"].progress_s < c.reservation.release_behind


def test_H_결과__헤매는_동안_BL_JS_로_들어오려는_로봇은_기다린다():
    """고치기 전 코드는 헤맴으로 BL·BL_JS 를 놓아, JS→BL 로 오는 로봇이 BL 에 선 pinky2 쪽으로 BL_JS 를 받았다."""
    g = RoadGraph.load(MAP4_GRAPH)
    r = R.Reservation(g)
    rt = g.shortest_route("BL", "RE", step=0.10)
    r.register("pinky2", 11, rt)
    wander = _random_wander(rt, rt.cumulative())
    for k in range(300):
        r.update_pose("pinky2", *wander(k))
        r.step()
    r.register("pinky1", 10, g.shortest_route("JS", "BL", step=0.10))
    r.update_pose("pinky1", 0.7, -0.15)
    r.step()
    st = r.status("pinky1")
    assert st["held_edges"] == [] and st["waiting_for"] == "BL_JS" and st["blocked_by"] == "pinky2"


def _slot(**kw):
    g = RoadGraph.load(MAP4_GRAPH)
    r = R.Reservation(g, **kw)
    rt = g.shortest_route("BL", "RE", step=0.10)
    r.register("pinky2", 11, rt)
    return r, rt, rt.cumulative(), r.robots["pinky2"]


def _feed(r, rt, cum, ss, step=False):
    for s in ss:
        r.update_pose("pinky2", *_pos(rt, cum, s))
        if step:
            r.step()                                       # 코디네이터처럼 매 틱 update_pose 다음에 step


def test_H_b__확인_창보다_짧은_튐은_몇_번_되풀이돼도_놓지_않는다():
    r, rt, cum, sl = _slot()
    n = r.confirm_updates
    for _ in range(10):                                    # fix 한 번(BL) + 1.0 m 앞 튐 n−1 번, 열 번
        _feed(r, rt, cum, [0.0] + [1.0] * (n - 1))
        r.step()
        assert r.node_holder.get("BL") == "pinky2" and r.edge_holder.get("BL_JS") == "pinky2"
    assert sl.progress_s == 0.0 and r._lead(sl) == pytest.approx(1.0)      # 잡기는 가장 앞, 놓기는 그대로
    _feed(r, rt, cum, [1.0])                               # 이제 n 번 연달아 — 그때부터 한 번에 max_step 씩
    assert sl.progress_s == pytest.approx(r.max_step)


def test_H_b__멈춰_선_로봇은_확인_창_뒤에_제자리까지_확인된다():
    r, rt, cum, sl = _slot()
    n = r.confirm_updates
    _feed(r, rt, cum, [0.3] * (n - 1))
    assert sl.progress_s == 0.0 and sl.progress_idx == 0 and sl.lead_s == pytest.approx(0.3)
    _feed(r, rt, cum, [0.3])
    assert sl.progress_s == pytest.approx(r.max_step)
    _feed(r, rt, cum, [0.3] * 20)
    assert sl.progress_s == pytest.approx(0.3) and sl.progress_idx == sl.idx_at(sl.progress_s) > 0


def test_H_a__확인_창보다_오래_간_튐도_max_speed_로만_번진다():
    r, rt, cum, sl = _slot()
    n = r.confirm_updates
    _feed(r, rt, cum, [0.05] * 30)
    assert sl.progress_s == pytest.approx(0.05)
    _feed(r, rt, cum, [1.2] * (n + 10))                    # 3 s 튐 — 창(2 s)을 채운 뒤 11 번 × max_step 만
    assert sl.progress_s == pytest.approx(0.05 + 11 * r.max_step)
    _feed(r, rt, cum, [0.05])                              # 되돌아온 첫 투영에 바로 내려온다(적대 검토 RES-1) — 부푼 값이
    assert sl.progress_s == pytest.approx(0.05)            # 남아 나중에 잡은 엣지를 놓게 하지 않는다
    _feed(r, rt, cum, [1.2] * (n - 1))                     # 다시 오르려면 창이 새로 차야 한다
    assert sl.progress_s == pytest.approx(0.05)


def test_H_c__연달아_뒤면_잡기_진행도를_되돌린다():
    r, rt, cum, sl = _slot()
    n = r.confirm_updates
    _feed(r, rt, cum, [1.2] * 5)                           # 짧은 튐 — 잡기만 앞으로
    assert sl.lead_s == pytest.approx(1.2) and sl.progress_s == 0.0
    _feed(r, rt, cum, [0.0] * (n - 1) + [1.1] + [0.0] * (n - 2) + [0.4])  # 1.1 은 문턱(0.2) 안 — 연달아가 끊긴다
    assert sl.lead_s == pytest.approx(1.2)
    _feed(r, rt, cum, [0.0])                               # n 번 연달아 뒤 → 그 n 번 중 가장 앞(0.4)으로 되돌린다
    assert sl.lead_s == pytest.approx(0.4)


def test_H_잡기는_가장_앞선_추정으로__놓기_확인_2_s_를_기다리지_않는다():
    """나누기의 다른 쪽: 요청(step)·같은 틱 연쇄(_try_acquire)는 확인을 기다리지 않는다 — 기다리면 달리는 로봇이
    허가 지점마다 2 s 씩 서고(D6-2 같은 정지), 확인된 진행도는 아직 0 이다."""
    r, rt, cum, sl = _slot()
    _feed(r, rt, cum, [0.0] * 2, step=True)
    assert r.status("pinky2")["held_edges"] == ["BL_JS", "JS_CW2", "CW2_BR"] and r.clear_until("pinky2") == 7
    _feed(r, rt, cum, [0.1, 0.2, 0.3], step=True)          # 허가 지점 0.69456 − 0.3 ≤ 0.40 → 바로 BR_RM
    assert r.status("pinky2")["held_edges"][-1] == "BR_RM" and sl.progress_s == 0.0
    r, rt, cum, sl = _slot()
    r.edge_holder["JS_CW2"] = "other"                      # 연쇄: JS_CW2 가 풀리는 틱에 그 너머도 가장 앞 기준
    _feed(r, rt, cum, [0.0, 0.05, 0.1, 0.15, 0.2, 0.25], step=True)
    assert r.status("pinky2")["held_edges"] == ["BL_JS"] and sl.progress_s == 0.0
    del r.edge_holder["JS_CW2"]
    _feed(r, rt, cum, [0.25], step=True)                   # CW2_BR 시작 0.595 − 0.25 ≤ 0.40
    assert r.status("pinky2")["held_edges"] == ["BL_JS", "JS_CW2", "CW2_BR"]


def test_H_c__되돌림_문턱은_regress_tol_이다__넘게_뒤면_되돌리고_안쪽이면_그대로():
    """통합 검토 RES-3: 문턱을 2배·0.5배로 바꿔도 시험이 초록이었다. 문턱의 양쪽을 고정한다."""
    r, rt, cum, sl = _slot()
    n, tol = r.confirm_updates, r.regress_tol
    _feed(r, rt, cum, [2.0] * 3)
    _feed(r, rt, cum, [2.0 - 0.75 * tol] * (n + 5))        # 문턱 안쪽 — 투영 잡음, 되돌리지 않는다
    assert sl.lead_s == pytest.approx(2.0)
    _feed(r, rt, cum, [2.0 - 1.5 * tol] * n)               # 문턱 넘게 뒤 n 번 연달아 — 되돌린다
    assert sl.lead_s == pytest.approx(2.0 - 1.5 * tol)


def test_H_c__되돌린_뒤_연달아_세기는_새로_시작한다__두_번째_되돌림은_새_투영_기준():
    """통합 검토 RES-3: 되돌린 뒤 모은 투영을 비우지 않으면 두 번째 되돌림이 옛 투영의 최대(첫 되돌림 자리)에 묶였다."""
    r, rt, cum, sl = _slot()
    n = r.confirm_updates
    _feed(r, rt, cum, [2.0] * 3 + [1.0] * n)               # 첫 되돌림 → 1.0
    assert sl.lead_s == pytest.approx(1.0)
    _feed(r, rt, cum, [0.2] * (n - 1))                     # 새로 n−1 번 — 아직 아니다
    assert sl.lead_s == pytest.approx(1.0)
    _feed(r, rt, cum, [0.2])                               # n 번째 — 새 투영(0.2)으로
    assert sl.lead_s == pytest.approx(0.2)


@pytest.mark.parametrize("regress_tol,held_after", [
    (None, ["BL_JS", "JS_CW2", "CW2_BR"]),                 # 되돌림 — 실제 자리(BL)에서 출발 때와 같은 셋
    (99.0, ["BL_JS", "JS_CW2", "CW2_BR", "BR_RM"])])       # 되돌림 없음(예전 잡기) — 튄 자리 1.2 기준으로 BR_RM 까지
def test_H_c__코디네이터처럼_매_틱_step_해도__막혀_있던_사이_되돌린_잡기는_튄_자리로_더_잡지_않는다(regress_tol, held_after):
    """튐 동안 다음 엣지(JS_CW2)가 남의 것이라 요청만 걸려 있었다. 튐이 끝나고 풀리면: 걸린 요청(JS_CW2)은 받되,
    그 너머는 실제 자리 기준으로만 잡는다(REVIEW_20260926 H (c)). 잡기는 배타라 안전하지만 남을 괜히 막는다."""
    r, rt, cum, sl = _slot(regress_tol=regress_tol)
    r.edge_holder["JS_CW2"] = "other"
    _feed(r, rt, cum, [0.0] * 3 + [1.2] * 5, step=True)
    assert r.status("pinky2")["held_edges"] == ["BL_JS"] and r.status("pinky2")["waiting_for"] == "JS_CW2"
    _feed(r, rt, cum, [0.0] * r.confirm_updates, step=True)
    del r.edge_holder["JS_CW2"]
    _feed(r, rt, cum, [0.0] * 5, step=True)
    assert r.status("pinky2")["held_edges"] == held_after
    assert r.node_holder.get("BL") == "pinky2" and sl.progress_s == 0.0


def test_H_c__오래_뒤면_두_진행이_다_내려와도__놓은_엣지는_되찾지_않고_쥔_엣지는_새_창_뒤에야_놓는다():
    """(c)·적대 검토 RES-1: 창 내내 뒤로 보고되면 잡기·놓기 진행이 둘 다 내려온다. 놓은 엣지·노드는 held 에서 빠졌으니
    되찾지 않고 허가도 뒤로 가지 않는다 — 낮아진 놓기 진행은 아직 쥔 엣지의 놓기를 늦출 뿐이다(막는 쪽)."""
    r, rt, cum, sl = _slot()
    n = r.confirm_updates
    _feed(r, rt, cum, [0.0] * 2 + [1.0] * 60, step=True)   # 확인된 진행 1.0 — BL_JS·JS_CW2 와 BL·JS·CW2 를 놓았다
    assert sl.progress_s == pytest.approx(1.0) and r.status("pinky2")["held_edges"] == ["CW2_BR", "BR_RM"]
    clear = r.clear_until("pinky2")
    _feed(r, rt, cum, [0.1] * n, step=True)                # 2 s 내내 0.1
    assert sl.lead_s == pytest.approx(0.1) and sl.progress_s == pytest.approx(0.1) and sl.progress_idx == 1
    assert r.status("pinky2")["held_edges"] == ["CW2_BR", "BR_RM"] and r.clear_until("pinky2") == clear
    assert r.edge_holder == {"CW2_BR": "pinky2", "BR_RM": "pinky2"} and r.node_holder == {"BR": "pinky2"}
    _feed(r, rt, cum, [1.3] * (n - 1), step=True)          # 앞으로 돌아와도 창이 새로 차기 전엔 CW2_BR(끝 0.99)를 쥔다
    assert sl.progress_s == pytest.approx(0.1) and r.edge_holder.get("CW2_BR") == "pinky2"
    _feed(r, rt, cum, [1.3] * 40, step=True)
    assert sl.progress_s == pytest.approx(1.3) and r.status("pinky2")["held_edges"] == ["BR_RM"]


def test_H_기본값__실제_주행을_늦추지_않고_1_Hz_fix_두_번을_덮는다():
    team11 = yaml.safe_load(open(os.path.join(MAP4_DIR, "lane_mission.yaml"), encoding="utf-8"))["defaults"]
    nav2 = yaml.safe_load(open(NAV2_FLEET, encoding="utf-8"))["velocity_smoother"]["ros__parameters"]
    assert R.DEFAULT_MAX_SPEED > max(float(team11["max_linear_vel"]), float(nav2["max_velocity"][0]))
    assert R.DEFAULT_CONFIRM_UPDATES * R.DEFAULT_UPDATE_PERIOD >= 2.0      # pose_fuser 점프 확인(confirm 2) × 1 Hz
    _, profs = P.load_profiles()                           # 호출 간격 = 코디네이터 틱 — 틱을 바꾸면 이 값도 넘겨야 한다
    for prof in profs.values():
        assert R.DEFAULT_UPDATE_PERIOD == pytest.approx(1.0 / float(prof.mission["coordinator"]["tick_rate"])), prof.name
    g = RoadGraph.load(MAP4_GRAPH)
    r = R.Reservation(g)                                   # 코디네이터는 이 셋을 넘기지 않는다 — 기본값이 실물 값이다
    assert r.max_step == pytest.approx(R.DEFAULT_MAX_SPEED * R.DEFAULT_UPDATE_PERIOD)
    assert r.confirm_updates == R.DEFAULT_CONFIRM_UPDATES and r.regress_tol == g.lane_width
    assert R.Reservation(g, confirm_updates=0).confirm_updates == 1       # 0 이면 창이 비어 min() 이 터진다 — 1 로


@pytest.mark.parametrize("speed", [0.15, 0.25])
@pytest.mark.parametrize("pname", ["map4", "map4_s2"])
def test_H_정상_주행은_두_시나리오_모두_도착하고__놓을_때_로봇은_늘_그것을_지났다(pname, speed):
    """허가 지점까지 speed 로 가는 로봇 둘 + 위치 잡음 ±3 cm. 매 틱: 로봇이 받은 엣지·노드는 그 끝(노드)을
    release_behind(− 잡음 여유)만큼 지나기 전엔 여전히 그 로봇 것이다. 그리고 둘 다 도착한다(놓기가 늦어도 막히지 않는다)."""
    noise = 0.03
    _, profs = P.load_profiles()
    prof = profs[pname]
    g = prof.graph
    r = R.Reservation(g, **{k: float(v) for k, v in prof.mission["reservation"].items()})
    rng = random.Random(7)
    bots = {}
    for spec in prof.mission["robots"]:
        rt = g.shortest_route(spec["start"], spec["goal"], step=0.10)
        r.register(spec["name"], spec["domain_id"], rt)
        bots[spec["name"]] = {"rt": rt, "cum": rt.cumulative(), "s": 0.0, "arrived": False, "edges": set(), "nodes": set()}
    clear = {n: 0 for n in bots}
    for tick in range(1200):                               # 120 s
        for n, b in bots.items():
            if not b["arrived"]:
                b["s"] = min(max(b["s"], b["cum"][clear[n]]), b["s"] + speed * 0.1)
            x, y = _pos(b["rt"], b["cum"], b["s"])
            r.update_pose(n, x + rng.uniform(-noise, noise), y + rng.uniform(-noise, noise))
        clear = r.step()
        for n, b in bots.items():
            if b["arrived"]:
                continue
            if b["s"] >= b["cum"][-1] - 1e-9:
                r.mark_arrived(n)
                b["arrived"] = True
                continue
            sl, rt = r.robots[n], b["rt"]
            b["edges"] |= {rt.edge_ids[k] for k in sl.held}
            b["nodes"] |= {nid for nid, h in r.node_holder.items() if h == n}
            for k, eid in enumerate(rt.edge_ids):
                if eid in b["edges"] and b["s"] < sl.edge_end_s(k) + r.release_behind - 2 * noise:
                    assert r.edge_holder.get(eid) == n, (pname, speed, tick, n, eid, b["s"], sl.progress_s)
            for i, nid in enumerate(rt.node_ids):
                if nid in b["nodes"] and b["s"] < b["cum"][rt.node_idx[i]] + r.release_behind - 2 * noise:
                    assert r.node_holder.get(nid) == n, (pname, speed, tick, n, nid, b["s"], sl.progress_s)
        if all(b["arrived"] for b in bots.values()):
            break
    assert all(b["arrived"] for b in bots.values()), {n: (b["s"], r.status(n)) for n, b in bots.items()}


# ---- 적대 검토 RES-1: 창보다 오래 틀린 포즈가 바로잡힌 뒤 START ------------------------------------------
# 예전 놓기 진행은 오를 때만 따라가(단조) 2 s 창보다 오래 간 틀린 포즈가 값을 영구히 부풀렸다 → 바로잡힌 뒤에도 남아,
# START 로 잡은 엣지를 한 틱 뒤 놓았다. 진짜 코디네이터 틱(switch_profile · start_fleet · _loop_tick)으로 잰다.

LEGACY_POSE = {"pinky1": (0.30, 0.30), "pinky2": (0.30, 0.95)}    # legacy START_A·START_B — ① 뒤 ②③ 전까지 옛 좌표계 보고


def _report(c, poses):
    c._t += 0.1
    for n, (x, y) in poses.items():
        c._cb_robot_state(n, RobotState(localized=True, x=x, y=y))
    c._loop_tick()


def _unheld_in_clearance(c, n, s_now):
    """호길이 s_now 에 선 로봇 n 의 허가 구간(지금 자리 ~ clear_until)에 n 이 쥐지 않은 엣지 — 비어야 한다.
    허가가 있으면 로봇이 선 노드(s_now 가 그 노드에서 release_behind 안)도 n 의 것이어야 한다."""
    res, sl = c.reservation, c.reservation.robots[n]
    clear = c.robots[n].clear_until_idx
    if clear <= 0:
        return []
    end = sl.cum[clear]
    bad = [eid for k, eid in enumerate(sl.route.edge_ids)
           if sl.edge_end_s(k) > s_now + 1e-9 and sl.edge_start_s(k) < end - 1e-9 and res.edge_holder.get(eid) != n]
    bad += [nid for i, nid in enumerate(sl.route.node_ids[:-1])
            if 0.0 <= s_now - sl.cum[sl.route.node_idx[i]] < res.release_behind and res.node_holder.get(nid) != n]
    return bad


@pytest.mark.parametrize("fix_ticks", [1, 30])
@pytest.mark.parametrize("wait_s", [1, 5, 30])
@pytest.mark.parametrize("pname", ["map4", "map4_s2"])
def test_RES_1__전환_뒤_옛_좌표로_오래_보고했어도__바로잡힌_뒤_START_는_쥐지_않은_엣지로_허가하지_않는다(pname, wait_s, fix_ticks):
    """월요일 §14-4 순서: legacy 좌표로 보고하던 로봇 → ① map4 전환(경로 배정, 출발 안 함) → ②③ 동안 wait_s 초 옛 좌표
    → ③ 초기 위치로 출발 노드에 바로잡힘(fix_ticks 틱) → START → 3 s 동안 로봇은 아직 출발 노드.
    고치기 전(단조): wait 5·30 s 면 놓기 진행 약 0.9 가 남아 — map4 는 pinky1 이 BL_JS·BL 을 pinky2 가 쥔 채로 허가 14
    (JW_JS 만 쥠), map4_s2 는 pinky1 이 MC_JI·MC 를 아무도 안 쥔 채로 허가 20. 바로잡힌 투영 하나면 내려와야 한다."""
    c = _pcoord()
    c.state_timeout_sec, c.global_seq = 2.0, 0
    for _ in range(10):
        _report(c, LEGACY_POSE)
    assert c.switch_profile(pname) and c.mission_state == "ASSIGNED"
    starts = {n: tuple(c.robots[n].route.waypoints[0]) for n in LEGACY_POSE}
    for _ in range(int(wait_s * 10)):
        _report(c, LEGACY_POSE)
    for _ in range(fix_ticks):
        _report(c, starts)
    assert c.start_fleet() and c.mission_state == "RUNNING"
    cleared = set()
    for k in range(30):
        _report(c, starts)
        for n in starts:
            bad = _unheld_in_clearance(c, n, 0.0)
            assert not bad, (pname, wait_s, fix_ticks, k * 0.1, n, c.robots[n].clear_until_idx, bad, c.reservation.status(n))
            if c.robots[n].clear_until_idx > 0:
                cleared.add(n)
    assert cleared                                         # 헛돌지 않는다 — 누군가는 허가를 받았다


@pytest.mark.parametrize("wrong_s", [2, 10])
def test_RES_1__지난_목표를_오래_보고했어도__바로잡힌_뒤_BL_에_선_pinky2_의_허가는_늘_제_엣지다(wrong_s):
    """map4 시나리오 1 재배정: pinky2 는 실제 BL 인데 보고 포즈가 지난 미션 끝 RE 에 wrong_s 초 남았다 → 3 s 바로잡힘
    → START. pinky1 은 허가대로 0.15 m/s 로 TC 까지, pinky2 는 BL 에 선 채. 고치기 전(단조): 10 s 면 놓기 진행 2.43 이
    남아 pinky2 가 [RM_RE] 만 쥔 채 허가 29 — BL·BL_JS 는 주인 없음(pinky1 쪽 로봇이 받을 수 있다)."""
    c = _pcoord()
    assert c.switch_profile("map4")
    c.state_timeout_sec, c.global_seq = 2.0, 0
    r1 = c.robots["pinky1"].route
    cum1 = r1.cumulative()
    re_ = tuple(c.robots["pinky2"].route.waypoints[-1])
    for _ in range(wrong_s * 10):
        _report(c, {"pinky1": BL, "pinky2": re_})
    for _ in range(30):
        _report(c, {"pinky1": BL, "pinky2": BL})
    assert c.start_fleet()
    s1, got2 = 0.0, False
    for k in range(400):
        s1 = min(max(s1, cum1[c.robots["pinky1"].clear_until_idx]), s1 + 0.015)
        _report(c, {"pinky1": _pos(r1, cum1, s1), "pinky2": BL})
        for n, s_now in (("pinky1", s1), ("pinky2", 0.0)):
            if n == "pinky1" and c.reservation.robots[n].finished:
                continue
            bad = _unheld_in_clearance(c, n, s_now)
            assert not bad, (wrong_s, k * 0.1, n, s_now, c.robots[n].clear_until_idx, bad, c.reservation.status(n))
        got2 = got2 or c.robots["pinky2"].clear_until_idx > 0
    assert s1 == pytest.approx(cum1[-1]) and got2         # pinky1 은 TC 까지, pinky2 는 pinky1 이 떠난 뒤 허가


# ---- 관찰: D6-2 뒤 시작 즉시 두 엣지 (REVIEW_20260926_RIG_RECHECK §1) ---------------------------------

def test_관찰__시작_직후_두_엣지는_D6_2_기준점의_결과다__legacy_첫_정지_지점_0_39993():
    """답신의 수치를 재현한다: START_A_TO_J1 은 0.69813 m → 정지 목표 0.49813 을 waypoint 로 내리면 4 번(0.39993 m).
    요청 기준점 = 그 정지 지점(D6-2) → 0.39993 − 진행 0 ≤ reserve_ahead 0.40 → 둘째 틱에 J1_TO_MID 요청·획득(허가 4 → 11).
    예전 기준점(J1 노드 0.69813)이면 진행 0.298 에서야 요청한다. 0.00007 m 차이의 경계 — 막는 쪽(더 잡는다)이라 안전하다."""
    g = RoadGraph.load(LEGACY_GRAPH)
    r = R.Reservation(g)
    rt = g.shortest_route("START_A", "GOAL_C", step=0.10)
    r.register("pinky1", 10, rt)
    sl = r.robots["pinky1"]
    r.step()
    assert r.status("pinky1")["held_edges"] == ["START_A_TO_J1"] and r.clear_until("pinky1") == 4
    assert sl.edge_end_s(0) == pytest.approx(0.69813, abs=1e-5)
    assert sl.cum[4] == pytest.approx(0.39993, abs=1e-5) and sl.cum[4] - sl.progress_s <= r.reserve_ahead
    assert sl.edge_start_s(1) - sl.progress_s > r.reserve_ahead          # 노드 기준이었다면 아직 요청 없음
    r.step()
    assert r.status("pinky1")["held_edges"] == ["START_A_TO_J1", "J1_TO_MID"] and r.clear_until("pinky1") == 11


def test_관찰__map4_pinky2_는_노드_규칙만으로도_둘__D6_2_기준점으로_셋():
    """map4 시나리오 1 pinky2 BL→RE: BL_JS 0.39478 m < reserve_ahead 라 첫 틱의 연쇄(노드 기준)가 JS_CW2 까지 잡는다.
    JS_CW2(0.2 m)는 margin 0.20 과 같아 정지 지점이 그 시작 0.39478(4 번) → 둘째 틱 D6-2 기준점이 CW2_BR 을 잡는다."""
    g = RoadGraph.load(MAP4_GRAPH)
    r = R.Reservation(g)
    r.register("pinky2", 11, g.shortest_route("BL", "RE", step=0.10))
    sl = r.robots["pinky2"]
    r.step()
    assert r.status("pinky2")["held_edges"] == ["BL_JS", "JS_CW2"] and r.clear_until("pinky2") == 4
    assert sl.cum[4] == pytest.approx(0.39478, abs=1e-5) == sl.edge_start_s(1)
    r.step()
    assert r.status("pinky2")["held_edges"] == ["BL_JS", "JS_CW2", "CW2_BR"] and r.clear_until("pinky2") == 7


# ---- 지도 칸 검사 (중계 항목) ------------------------------------------------------------------------

def _occ(warnings):
    return {re.search(r"엣지 (\S+) 가", w).group(1): w for w in warnings if w.startswith("지도 칸:")}


def _occ_warnings(p):
    return _occ(p.warnings)


def test_지도_칸__map4_는_JW_JS·TR_MC·TR_TC_가_점유_칸을_지난다고_알리고_고를_수는_있다():
    _, profs = P.load_profiles()
    crossing = P.edge_occupancy(os.path.join(MAP4_DIR, "map4.yaml"), profs["map4"].graph)
    assert set(crossing) == {"JW_JS", "TR_MC", "TR_TC"} and all(set(v) == {"occupied"} for v in crossing.values())
    for name, users in (("map4", {"JW_JS": "pinky1 경로"}), ("map4_s2", {"TR_TC": "pinky2 경로"})):
        p = profs[name]
        assert p.valid and p.problems == []                # 경고만 — 고를 수 있다
        occ = _occ_warnings(p)
        assert sorted(occ) == ["JW_JS", "TR_MC", "TR_TC"]
        for eid, w in occ.items():
            assert "점유 칸 위를 지난다" in w and w.endswith(" — " + users.get(eid, "이 미션 경로에는 없다")), w
    assert "(점유 약 0.10 m)" in _occ_warnings(profs["map4"])["JW_JS"]
    assert P.OCC_SAMPLE_STEP <= 0.005                      # 칸(map4 0.05 m)의 1/10 이하 — 칸 모서리를 스치는 엣지도 본다
    assert _occ_warnings(profs["legacy"]) == {} and profs["legacy"].valid    # 지도를 싣지 않는 legacy 는 안 본다


# 합성 지도의 화소 — (빈칸, 점유 p>0.65, 점유 문턱 바로 아래, free 문턱 바로 위). p = 1 − 화소/maxval (negate 0).
_SHADES = {255: (254, 89, 90, 205),                        # p 0.004 · 0.65098 · 0.64706 · 0.19608
           65535: (65000, 22800, 23000, 52650),           # 16 비트 큰 끝 — p 0.008 · 0.652 · 0.649 · 0.197 (바이트를 뒤집어 읽으면 어긋난다)
           100: (99, 34, 36, 80)}                          # maxval 100 — p 0.01 · 0.66 · 0.64 · 0.20 (255 로 나누면 전부 어긋난다)


def _tiny_map(tmp_path, negate, fmt, maxval=255, mode="trinary"):
    """10×6 칸 · 0.1 m · 원점 (0,0). 그림 1 행(= y 0.4~0.5): 5 열 점유 · 7 열 알 수 없음(점유 문턱 바로 아래).
    그림 4 행(= y 0.1~0.2): 2 열 알 수 없음(free 문턱 바로 위). 나머지 빈칸.
    행을 뒤집어 읽으면(0 행 = y 최소) 점유·알 수 없음 칸이 y 0.1~0.2 로 가 C_D 는 알 수 없음 하나만 · E_F 는 깨끗하게 나온다."""
    w, h = 10, 6
    free, occ, unk_hi, unk_lo = _SHADES[maxval]
    px = [free] * (w * h)
    px[1 * w + 5], px[1 * w + 7], px[4 * w + 2] = occ, unk_hi, unk_lo
    if negate:
        px = [maxval - v for v in px]
    if fmt == "P5":
        if maxval < 256:
            raw = bytes(px)
        else:
            raw = b"".join(bytes([v >> 8, v & 0xFF]) for v in px)       # 16 비트는 큰 끝 먼저(PGM 규격)
        (tmp_path / "t.pgm").write_bytes(b"P5\n%d %d\n%d\n" % (w, h, maxval) + raw)
    else:
        rows = "\n".join(" ".join(str(v) for v in px[i * w:(i + 1) * w]) for i in range(h))
        (tmp_path / "t.pgm").write_text("P2\n# synthetic\n%d %d\n%d\n%s\n" % (w, h, maxval, rows), encoding="ascii")
    (tmp_path / "t.yaml").write_text("image: t.pgm\nmode: %s\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
                                     "negate: %d\noccupied_thresh: 0.65\nfree_thresh: 0.196\n" % (mode, negate),
                                     encoding="utf-8")
    (tmp_path / "g.yaml").write_text(yaml.safe_dump({"nodes": [
        {"id": "A", "x": 0.05, "y": 0.05}, {"id": "B", "x": 0.95, "y": 0.05}, {"id": "G", "x": 1.0, "y": 0.05},
        {"id": "C", "x": 0.05, "y": 0.45}, {"id": "D", "x": 0.95, "y": 0.45},
        {"id": "E", "x": 0.25, "y": 0.05}, {"id": "F", "x": 0.25, "y": 0.35}], "edges": [
        {"id": "A_B", "from": "A", "to": "B", "waypoints": [[0.05, 0.05], [0.95, 0.05]]},
        {"id": "B_G", "from": "B", "to": "G", "waypoints": [[0.95, 0.05], [1.0, 0.05]]},
        {"id": "C_D", "from": "C", "to": "D", "waypoints": [[0.05, 0.45], [0.95, 0.45]]},
        {"id": "E_F", "from": "E", "to": "F", "waypoints": [[0.25, 0.05], [0.25, 0.35]]}]}), encoding="utf-8")
    (tmp_path / "m.yaml").write_text("graph: g.yaml\nrobots:\n  - {name: pinky1, start: A, goal: B}\n"
                                     "  - {name: pinky2, start: C, goal: D}\n", encoding="utf-8")
    (tmp_path / "profiles.yaml").write_text("default: t\nprofiles:\n  t:\n    mission: m.yaml\n    map: t.yaml\n",
                                            encoding="utf-8")
    return P.load_profiles(str(tmp_path / "profiles.yaml"))[1]["t"]


@pytest.mark.parametrize("negate,fmt,maxval", [(0, "P5", 255), (1, "P2", 255), (0, "P5", 65535), (1, "P2", 100)])
def test_지도_칸__합성_지도로_nav2_trinary_규칙·행_방향·negate·maxval_을_잰다(tmp_path, negate, fmt, maxval):
    p = _tiny_map(tmp_path, negate, fmt, maxval)
    assert p.valid and p.problems == []
    occ = _occ_warnings(p)
    assert sorted(occ) == ["B_G", "C_D", "E_F"]                       # A_B 는 빈칸만 지난다
    assert "점유·알 수 없음 칸" in occ["C_D"] and "(점유 약 0.10 m · 알 수 없음 약 0.10 m)" in occ["C_D"]
    assert occ["C_D"].endswith(" — pinky2 경로")
    assert "알 수 없음 칸" in occ["E_F"] and "(알 수 없음 약 0.10 m)" in occ["E_F"]
    assert occ["E_F"].endswith(" — 이 미션 경로에는 없다")
    assert "지도 밖 칸" in occ["B_G"]                                  # x = 1.0 은 마지막 칸(0.9~1.0) 너머 — Nav2 도 거절


@pytest.mark.parametrize("fmt", ["P5", "P2"])
def test_지도_칸__화소를_못_읽으면_경고만_하고_고를_수는_있다(tmp_path, fmt):
    _tiny_map(tmp_path, 0, fmt)
    if fmt == "P5":                                        # 헤더는 맞고 화소가 반만
        (tmp_path / "t.pgm").write_bytes(b"P5\n10 6\n255\n" + bytes([254] * 30))
    else:
        (tmp_path / "t.pgm").write_text("P2\n10 6\n255\n" + " ".join(["254"] * 30) + "\n", encoding="ascii")
    p = P.load_profiles(str(tmp_path / "profiles.yaml"))[1]["t"]
    assert p.valid and _occ_warnings(p) == {}
    assert any(w.startswith("지도 점유를 검사하지 못했다") and "모자란다" in w for w in p.warnings), p.warnings


def test_지도_칸__trinary_가_아닌_지도는_검사하지_못했다고_말한다(tmp_path):
    p = _tiny_map(tmp_path, 0, "P5", mode="scale")
    assert p.valid and _occ_warnings(p) == {}
    assert any(w.startswith("지도 점유를 검사하지 못했다") and "trinary" in w for w in p.warnings), p.warnings


def test_지도_칸__깨진_프로파일은_문제만_말한다__지도_밖_좌표를_칸_경고로_겹쳐_말하지_않는다(tmp_path):
    _tiny_map(tmp_path, 0, "P5")
    doc = yaml.safe_load((tmp_path / "g.yaml").read_text(encoding="utf-8"))
    doc["edges"][1]["waypoints"].insert(1, [1.2, 0.05])                # B_G 가운데 점이 지도 사각형 밖
    (tmp_path / "g.yaml").write_text(yaml.safe_dump(doc), encoding="utf-8")
    p = P.load_profiles(str(tmp_path / "profiles.yaml"))[1]["t"]
    assert not p.valid and any("지도 밖 좌표" in x and "B_G" in x for x in p.problems)
    assert _occ_warnings(p) == {}


@pytest.mark.parametrize("name_line,shown", [("", "None"), ("name: 1, ", "1")], ids=["이름_없음", "숫자_이름"])
def test_지도_칸__RES_2__미션_로봇_이름이_글자가_아니어도_목록이_터지지_않고_칸_경고에_그대로_적는다(tmp_path, name_line, shown):
    """적대 검토 RES-2: 칸을 지나는 엣지의 사용자 이름을 ', '.join 하다 None·int 에서 TypeError → load_profiles 가 통째로
    터져 코디네이터가 뜨지 못했다(fleet_coordinator 는 감싸지 않고 부른다). 초안 하나가 플릿 전체를 막으면 안 된다."""
    _tiny_map(tmp_path, 0, "P5")
    (tmp_path / "m.yaml").write_text("graph: g.yaml\nrobots:\n  - {name: pinky1, start: A, goal: B}\n"
                                     "  - {%sstart: C, goal: D}\n" % name_line, encoding="utf-8")
    p = P.load_profiles(str(tmp_path / "profiles.yaml"))[1]["t"]
    assert p.valid and _occ_warnings(p)["C_D"].endswith(" — %s 경로" % shown)


def test_지도_칸__V2_카드가_읽는_available_에_실리고_카드는_cur_warnings_를_그린다():
    c = _pcoord()
    assert c.switch_profile("map4")
    st = c.profile_status()
    cur = next(a for a in st["available"] if a["name"] == st["active"])
    assert cur["valid"] and any(w.startswith("지도 칸: 엣지 JW_JS") and w.endswith("pinky1 경로") for w in cur["warnings"])
    js = open(os.path.join(STATIC, "fleet_control_v2.js"), encoding="utf-8").read()
    assert 'esc(cur.warnings.join("; "))' in js                        # 카드의 "미션 경고" 줄
    assert "a.valid&&(a.warnings||[]).length" in js                    # 목록의 "(경고)" 표시 — 고를 수는 있다


def test_지도_칸__게이트웨이_api_fleet_profiles_가_카드에_경고를_싣는다(gw):
    """카드가 실제로 받는 JSON(GET /api/fleet/profiles) 에서 확인 — 소켓 없이 처리기를 바로 부른다."""
    import io
    import json
    g, c, _ = gw
    assert c.switch_profile("map4")
    h = g.GatewayRequestHandler.__new__(g.GatewayRequestHandler)
    h.path, h.headers, h.rfile, h.client_address = "/api/fleet/profiles", {}, io.BytesIO(b""), ("127.0.0.1", 50000)
    out = {}

    def _send_json(body, code=200):
        out["code"], out["body"] = code, json.loads(body.decode("utf-8"))
    h._send_json = _send_json
    h.do_GET()
    assert out["code"] == 200 and out["body"]["active"] == "map4"
    cur = next(a for a in out["body"]["available"] if a["name"] == "map4")
    assert cur["valid"] and sorted(_occ(cur["warnings"])) == ["JW_JS", "TR_MC", "TR_TC"]
