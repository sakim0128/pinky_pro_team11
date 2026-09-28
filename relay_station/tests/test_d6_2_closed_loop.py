# -*- coding: utf-8 -*-
"""D6-2 · 교차로 허가 지점 앞 영구 정지 — ROS 없이 도는 폐루프 (관제 검수 REVIEW_20260925 §2).

실제 `Reservation`(코디네이터의 허가) + 실제 `RouteChain`(에이전트의 목표 연쇄) + Nav2 모형을 10 Hz 로 돌린다.
Nav2 모형은 "목표 허용 반경 안에 **처음** 들어오면 성공" — 그래서 로봇은 허가 지점보다 허용만큼 **앞에** 선다.

🔴 실측(관제 리그 2026-09-25): 첫 허가 지점 도달 뒤 `clearance 대기 (진행 2, 허가 4)` 로 120 s 변화 없음.
   허가 지점 = 교차로 − node_stop_margin(0.20) 을 waypoint 로 **내림**(최대 0.10 손실), 로봇은 거기서 허용(0.15)만큼 앞.
   다음 구간 요청은 `교차로 − 진행 ≤ reserve_ahead(0.40)` 일 때만 → 0.20 + 내림 + 0.15 > 0.40 이면 요청이 영영 안 생긴다.

허용치는 레포의 Nav2 파라미터 파일에서 **읽는다**(손으로 적지 않는다 — 새 파라미터가 생기면 여기 저절로 든다).
"""
import glob
import os
import sys

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from relay_station.fleet.reservation import Reservation, DEFAULT_RESERVE_AHEAD  # noqa: E402
from relay_station.fleet.road_graph import RoadGraph  # noqa: E402
from pinky_fleet_agent import route_chain as rc  # noqa: E402
from pinky_fleet_agent.route_chain import RouteChain  # noqa: E402

GRAPH = os.path.join(REPO, "relay_station", "fleet", "config", "road_graph.yaml")
MISSION = os.path.join(REPO, "relay_station", "fleet", "config", "lane_mission.yaml")
NAV2_PARAM_GLOBS = ("pinky_fleet_agent/params/*.yaml", "docker/nav2_*params*.yaml")
DT = 0.1                   # 10 Hz — 코디네이터 틱·에이전트 상태 틱과 같다
SPEED = 0.10               # m/s (실물 max_linear_vel 0.2 의 절반)
TIMEOUT_S = 90.0


def _walk(node):
    if isinstance(node, dict):
        for k, v in node.items():
            yield k, v
            yield from _walk(v)
    elif isinstance(node, list):
        for v in node:
            yield from _walk(v)


def repo_nav2_tolerances():
    """(허용치 목록, 못 읽은 파일 목록). 못 읽은 파일을 조용히 건너뛰면 그 파일의 허용치가 시험에서 빠진다
    — 실제로 nav2_params_fleet.yaml(실물 0.08)이 UTF-16 이라 빠졌었다(2026-09-26)."""
    vals, unreadable = set(), []
    for pat in NAV2_PARAM_GLOBS:
        for path in glob.glob(os.path.join(REPO, pat), recursive=True):
            try:
                doc = yaml.safe_load(open(path, encoding="utf-8"))
            except Exception as exc:                  # noqa: BLE001
                unreadable.append("%s: %s" % (os.path.relpath(path, REPO), type(exc).__name__))
                continue
            for k, v in _walk(doc):
                if k == "xy_goal_tolerance" and isinstance(v, (int, float)):
                    vals.add(round(float(v), 3))
    return sorted(vals), unreadable


def mission_robots():
    doc = yaml.safe_load(open(MISSION, encoding="utf-8"))
    res = doc.get("reservation", {})
    robots = []
    for r in doc.get("robots", []):
        if r.get("start") and r.get("goal"):
            robots.append((r["name"], int(r.get("domain_id", 10)), r["start"], r["goal"]))
    return robots, res


def _pos(route, cum, s):
    """경로 위 호길이 s 의 (x, y)."""
    import bisect
    pts = route.waypoints
    i = max(0, min(len(pts) - 2, bisect.bisect_right(cum, s) - 1))
    seg = cum[i + 1] - cum[i]
    t = 0.0 if seg <= 0 else (s - cum[i]) / seg
    return (pts[i][0] + t * (pts[i + 1][0] - pts[i][0]), pts[i][1] + t * (pts[i + 1][1] - pts[i][1]))


def drive(robot, tol, res_cfg, graph):
    """한 로봇을 경로 끝까지 몰아 본다. (도착했나, 걸린 초, 마지막 상태) 반환."""
    name, dom, start, goal = robot
    route = graph.shortest_route(start, goal, step=0.10)
    cum = route.cumulative()
    resv = Reservation(graph, **{k: float(v) for k, v in res_cfg.items()
                                  if k in ("reserve_ahead", "release_behind", "node_stop_margin")})
    resv.register(name, dom, route)
    clock = [0.0]
    chain = RouteChain(clock=lambda: clock[0])
    chain.set_nav2_xy_tolerance(tol)
    chain.on_route(1, list(route.waypoints), route.goal_idx)
    s = 0.0
    goal = None                                    # Nav2 가 달리는 waypoint idx

    def apply(acts):
        nonlocal goal
        for a in acts:
            if a[0] == "goto":
                goal = a[1]
            elif a[0] == "cancel":
                goal = None

    apply(chain.on_lane_command(rc.CMD_START, 1, 0))
    t = 0.0
    while t < TIMEOUT_S:
        t += DT
        clock[0] = t
        # Nav2 모형 — 허용 반경 안에 처음 들어오면 성공, 아니면 목표 쪽으로 전진
        if goal is not None:
            target_s = cum[goal]
            if abs(target_s - s) <= tol:
                g, goal = goal, None
                apply(chain.on_goal_result(g, "succeeded"))
            else:
                s = min(target_s, s + SPEED * DT) if target_s > s else max(target_s, s - SPEED * DT)
        x, y = _pos(route, cum, s)
        apply(chain.on_pose(x, y))
        # 코디네이터 틱
        resv.update_pose(name, x, y)
        clear = resv.step()[name]
        apply(chain.on_lane_command(rc.CMD_CLEARANCE, 1, clear))
        if chain.arrived:
            return True, round(t, 1), resv.status(name)
    return False, None, dict(resv.status(name), progress_s=round(s, 3), reason=chain.reason)


ROBOTS, RES_CFG = mission_robots()
TOLS, UNREADABLE = repo_nav2_tolerances()


def test_레포의_Nav2_허용치를_읽었다():
    assert UNREADABLE == [], UNREADABLE                          # UTF-8 로 못 읽는 파라미터 파일이 없다
    assert TOLS and 0.08 in TOLS, TOLS                           # 실물 fleet 0.08 (팀11 레포에는 팜 파라미터가 없다)
    assert ROBOTS, "lane_mission.yaml 에 경로 있는 로봇이 없다"


@pytest.mark.parametrize("tol", TOLS)
@pytest.mark.parametrize("robot", ROBOTS, ids=[r[0] for r in ROBOTS])
def test_D6_2_어느_허용치로도_교차로_앞에서_서지_않고_도착한다(robot, tol):
    ok, t, st = drive(robot, tol, RES_CFG, RoadGraph.load(GRAPH))
    assert ok, "허용 %.2f m 에서 %s 가 %d s 안에 도착 못함 — %s" % (tol, robot[0], TIMEOUT_S, st)


def test_D6_2_불변식__도달_허용이_reserve_ahead_보다_작으면_된다():
    """처방의 불변식: 요청 거리를 코디네이터가 명령한 정지 지점에서 재면 '도달허용 ≤ reserve_ahead' 만 필요하다."""
    assert max(TOLS) < float(RES_CFG.get("reserve_ahead", DEFAULT_RESERVE_AHEAD))


# ---- 교착 신호 — 요청 없이 허가 지점에 선 로봇 (관제 처방 "정지 신호") ------------------------------

def _stalled_coord(**status_over):
    from test_d7_robot_stop import _coord                      # 같은 폴더의 코디네이터 조립기
    from pinky_lane_msgs.msg import LaneStatus
    c = _coord(now=100.0)
    st = {"progress_idx": 2, "clear_until": 4, "held_edges": ["START_A_TO_J1"], "waiting_for": "",
          "blocked_by": "", "finished": False, "next_edge": "J1_TO_MID", "requesting": False}
    st.update(status_over)
    c.reservation.status = lambda name: dict(st) if name == "pinky1" else {}
    ls = LaneStatus()
    ls.drive_state = LaneStatus.DRIVE_WAIT_CLEARANCE
    c.robots["pinky1"].lane_status = ls
    return c


def test_D6_2_요청도_막는_이도_없이_10초_서면_교착_경고__한_번만_로그():
    c = _stalled_coord()
    assert c._check_stalls(100.0) == [] and c._check_stalls(109.0) == []
    (msg,) = c._check_stalls(110.5)
    assert msg.startswith("STALL_NO_REQUEST: pinky1") and "J1_TO_MID" in msg
    assert c.get_fleet_status_dict()["robots"]["pinky1"]["stall_sec"] is not None
    c._check_stalls(111.0)
    assert sum("STALL_NO_REQUEST" in str(a) for a in c.get_logger().warn.call_args_list) == 1


@pytest.mark.parametrize("over", [{"waiting_for": "J1_TO_MID"},      # 중재 중 — 막는 이가 있다
                                  {"requesting": True},               # 요청은 들어가 있다
                                  {"next_edge": ""},                   # 마지막 엣지
                                  {"held_edges": []}])
def test_D6_2_중재·요청_중·마지막_엣지는_교착이_아니다(over):
    c = _stalled_coord(**over)
    assert c._check_stalls(100.0) == [] and c._check_stalls(200.0) == []
    assert c.robots["pinky1"].stall_since is None


def test_D6_2_실제_예약도_요청_여부를_말한다():
    g = RoadGraph.load(GRAPH)
    robots, res = mission_robots()
    name, dom, start, goal = robots[0]
    r = Reservation(g, **{k: float(v) for k, v in res.items() if k in ("reserve_ahead", "release_behind", "node_stop_margin")})
    r.register(name, dom, g.shortest_route(start, goal, step=0.10))
    r.step()
    st = r.status(name)
    assert st["held_edges"] and st["next_edge"] and isinstance(st["requesting"], bool)
