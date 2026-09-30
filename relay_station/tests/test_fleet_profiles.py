# -*- coding: utf-8 -*-
"""R-7 웹 전환 — 좌표 프로파일(legacy ↔ 팀11 map4) (PLAN_20260925_MAP4_FRAME_UNIFICATION, 2026-09-26).

월요일 실물 전환을 코드 커밋이 아니라 **중계 웹**에서 한다. 여기서 재는 것:
- 프로파일 파일 자체(팀11 원본 바이트 · 모든 좌표가 지도 안 · 미션 노드 ⊂ 도로망 · 로봇 이름 같음)
- 저장·복원(깨진 저장값은 기본으로 — 그 사실을 말한다)
- 코디네이터 전환(달리는 중·비상정지 중 거부, 경로 재배정) · 로봇 지도·초기 위치 명령
- map4 시나리오 1·2 가 D6-2 폐루프에서 도착한다(관제 표: 팀11 도로망도 수정 없이는 0.15 m 에서 막혔다)
- 게이트웨이 엔드포인트 · 에이전트 CMD_SET_MAP · 화면(V2·index.html)의 연결

⚠️ 저장 파일은 늘 임시 폴더로(PINKY_RELAY_STATE_DIR) — 실물 중계의 ~/.local/state 를 건드리지 않는다.
"""
import hashlib
import math
import os
import re
import sys
import types
from unittest.mock import MagicMock

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pinky_fleet_msgs.msg import FleetCommand, RobotState  # noqa: E402
from pinky_lane_msgs.msg import LaneStatus  # noqa: E402
from relay_station.fleet import profiles as P  # noqa: E402
from relay_station.fleet.reservation import Reservation  # noqa: E402
from test_d7_robot_stop import _coord, _reset, _sent  # noqa: E402
from test_d6_2_closed_loop import drive, repo_nav2_tolerances  # noqa: E402

PROF_DIR = os.path.join(REPO, "relay_station", "fleet", "config", "profiles")
MAP4_DIR = os.path.join(PROF_DIR, "team11_map4")
TEAM11_SHA = {  # 팀11 mini_project_2 1f505cb (SOURCE.md)
    "road_graph.yaml": "547dae27762c635d34547d14c40665f62bea28d7d3b2bf3d87d66abc73754d07",
    "map4.yaml": "2d9fada5469544355f71779ae7620ab053d2bed1942e374fdbd28eb035db4774",
    "map4.pgm": "5a3794d2ef6a9936aa6b1fb112142e28e975434bd07a59ca05d143c6749bf1a7",
}


@pytest.fixture(autouse=True)
def _state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv(P.STATE_ENV, str(tmp_path / "state"))


# ---- 프로파일 파일 ------------------------------------------------------------------------

def test_팀11_원본_세_파일은_바이트_그대로다():
    for name, sha in TEAM11_SHA.items():
        got = hashlib.sha256(open(os.path.join(MAP4_DIR, name), "rb").read()).hexdigest()
        assert got == sha, name
        assert sha in open(os.path.join(MAP4_DIR, "SOURCE.md"), encoding="utf-8").read()


def test_모든_프로파일이_읽히고_기본은_legacy():
    default, profs = P.load_profiles()
    assert default == "legacy" and {"legacy", "map4", "map4_s2"} <= set(profs)
    for name, p in profs.items():
        assert p.valid, (name, p.problems)


def test_map4_는_가운데_원점_2_35_x_1_25_이고_도로망_전부가_지도_안이다():
    _, profs = P.load_profiles()
    m = profs["map4"].map_meta
    assert m["origin"] == [-1.175, -0.625] and m["resolution"] == 0.05
    assert (m["width"], m["height"]) == (47, 25)
    assert len(profs["map4"].graph.nodes) == 14 and len(profs["map4"].graph.edges) == 15
    assert profs["map4"].frame == profs["map4_s2"].frame == "team11_map4"


def test_프로파일끼리_로봇_이름이_같다__코디네이터는_토픽을_기동_때_한_번_만든다():
    _, profs = P.load_profiles()
    names = {tuple(sorted(p.robot_names())) for p in profs.values()}
    assert len(names) == 1


def test_지도_밖_좌표가_있으면_깨진_프로파일로_본다(tmp_path):
    """도로망은 그대로, 지도 원점만 옮겨(좌하단 0,0) 음수 좌표 노드가 지도 밖으로 나가게 한다."""
    (tmp_path / "map.yaml").write_text(
        "image: %s\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n" % os.path.join(MAP4_DIR, "map4.pgm"),
        encoding="utf-8")
    (tmp_path / "m.yaml").write_text("graph: %s\nrobots:\n  - {name: pinky1, start: BL, goal: TC}\n"
                                     % os.path.join(MAP4_DIR, "road_graph.yaml"), encoding="utf-8")
    man = tmp_path / "profiles.yaml"
    man.write_text("default: x\nprofiles:\n  x:\n    mission: m.yaml\n    map: map.yaml\n", encoding="utf-8")
    _, profs = P.load_profiles(str(man))
    assert not profs["x"].valid and profs["x"].problems[0].startswith("지도 밖")
    assert "TC" in profs["x"].problems[0] and "BL" not in profs["x"].problems[0].split(":")[1].split(", ")


# ---- 저장·복원 ------------------------------------------------------------------------------

def test_저장값이_없으면_기본():
    d, profs = P.load_profiles()
    assert P.resolve_active(d, profs) == ("legacy", None)


def test_저장한_프로파일로_다시_뜬다():
    d, profs = P.load_profiles()
    P.write_active("map4")
    assert P.resolve_active(d, profs) == ("map4", None)


def test_저장값이_목록에_없거나_깨졌으면_기본으로__그_사실을_말한다():
    d, profs = P.load_profiles()
    P.write_active("nope")
    name, why = P.resolve_active(d, profs)
    assert name == "legacy" and "nope" in why
    P.write_active("map4")
    profs["map4"].problems.append("일부러 깨뜨림")
    name, why = P.resolve_active(d, profs)
    assert name == "legacy" and "깨졌다" in why


# ---- 로봇 지도 대조 --------------------------------------------------------------------------

def _state(name="map4", known=True, ox=-1.175, oy=-0.625, w=47, h=25, res=0.05):
    s = RobotState()
    s.map_name, s.map_known = name, known
    s.map_origin_x, s.map_origin_y, s.map_width, s.map_height, s.map_resolution = ox, oy, w, h, res
    return s


def test_로봇_지도_대조는_이름만이_아니라_원점·크기까지_본다():
    _, profs = P.load_profiles()
    p = profs["map4"]
    assert P.robot_map_check(p, _state())[0] == "MATCH"
    assert P.robot_map_check(p, _state(ox=-1.35))[0] == "MISMATCH"          # 같은 이름의 옛 지도
    assert P.robot_map_check(p, _state(name="my_map"))[0] == "MISMATCH"
    assert P.robot_map_check(p, _state(known=False))[0] == "UNKNOWN"
    assert P.robot_map_check(p, None)[0] == "UNKNOWN"


def test_지도_규격이_없는_legacy_는_이름만_맞아도_MATCH_라_하지_않는다():
    _, profs = P.load_profiles()
    assert P.robot_map_check(profs["legacy"], _state(name="my_map"))[0] == "NAME_MATCH"
    assert P.robot_map_check(profs["legacy"], _state(name="map4"))[0] == "MISMATCH"


def test_화면_코드는_캐시하지_않게_답한다():
    src = open(os.path.join(REPO, "relay_station", "gateway_web", "gateway_web_server.py"), encoding="utf-8").read()
    assert "self.send_header('Cache-Control', 'no-cache')" in src


# ---- 코디네이터 전환 ------------------------------------------------------------------------

def _pcoord(state="IDLE"):
    c = _coord(state=state)
    c.profiles_default, c.profiles = P.load_profiles()
    c.active_profile = "legacy"
    c.route_pubs = {}
    c.reservation = Reservation(c.graph)
    for ctx in c.robots.values():
        ctx.route = None
    return c


def test_전환하면_도로망·미션이_바뀌고_새_경로가_배정되며_저장된다():
    c = _pcoord()
    assert c.switch_profile("map4") is True
    assert c.active_profile == "map4" and len(c.graph.nodes) == 14
    assert c.robots["pinky1"].goal_node == "TC" and c.robots["pinky2"].goal_node == "RE"
    assert c.robots["pinky1"].route.node_ids[0] == "BL" and c.robots["pinky1"].route.node_ids[-1] == "TC"
    assert c.mission_state == "ASSIGNED"
    assert P.read_active() == "map4"
    st = c.get_fleet_status_dict()["profile"]
    assert st["active"] == "map4" and st["frame"] == "team11_map4" and st["robot_map_name"] == "map4"


@pytest.mark.parametrize("how,why", [
    (lambda c: setattr(c, "mission_state", "RUNNING"), "RUNNING"),
    (lambda c: setattr(c, "mission_state", "STOPPED"), "STOPPED"),
    (lambda c: (setattr(c, "estop_latched", True), setattr(c, "mission_state", "ESTOP")), "비상정지"),
    (lambda c: setattr(c.robots["pinky1"], "state", _moving()), "움직이는"),
], ids=["running", "stopped", "estop", "moving"])
def test_달리는_중·비상정지_중에는_전환을_거부한다(how, why):
    c = _pcoord()
    how(c)
    assert why in c.profile_switch_blocker()
    assert c.switch_profile("map4") is False and c.active_profile == "legacy"
    assert P.read_active() is None


def _moving():
    s = RobotState()
    s.linear_velocity = 0.12
    return s


def test_모르는_·깨진_프로파일은_거부():
    c = _pcoord()
    assert c.switch_profile("nope") is False
    c.profiles["map4"].problems.append("깨짐")
    assert c.switch_profile("map4") is False


def test_로봇_지도_전환은_모든_로봇에_CMD_SET_MAP():
    c = _pcoord()
    c.switch_profile("map4")
    _reset(c)
    assert c.send_robot_maps() is True
    for n in ("pinky1", "pinky2"):
        (fc,) = _sent(c.fleet_cmd_pubs[n])
        assert fc.command == FleetCommand.CMD_SET_MAP and fc.map_name == "map4"


def test_초기_위치는_출발_노드와_첫_구간_방향():
    c = _pcoord()
    c.switch_profile("map4")
    _reset(c)
    assert c.send_initial_poses() is True
    (fc,) = _sent(c.fleet_cmd_pubs["pinky1"])
    wp = c.robots["pinky1"].route.waypoints
    assert fc.command == FleetCommand.CMD_SET_INITIAL_POSE
    assert (fc.x, fc.y) == pytest.approx(wp[0]) and (fc.x, fc.y) == pytest.approx((0.95, -0.45))    # BL
    assert fc.yaw == pytest.approx(math.atan2(wp[1][1] - wp[0][1], wp[1][0] - wp[0][0]))


def test_제어_토픽으로도_전환되고_결과가_남는다():
    import json
    from std_msgs.msg import String
    c = _pcoord()
    c._cb_control(String(data=json.dumps({"cmd": "profile", "name": "map4_s2"})))
    assert c.last_control["cmd"] == "profile" and c.last_control["ok"] is True
    assert c.robots["pinky1"].start_node == "MC" and c.robots["pinky2"].goal_node == "TC"


# ---- map4 시나리오가 D6-2 폐루프에서 도착한다 ----------------------------------------------------

def _scenario_robots(prof):
    return [(r["name"], int(r.get("domain_id", 10)), r["start"], r["goal"]) for r in prof.mission["robots"]]


@pytest.mark.parametrize("pname", ["map4", "map4_s2"])
@pytest.mark.parametrize("tol", repo_nav2_tolerances()[0])
def test_map4_시나리오는_어느_Nav2_허용치로도_도착한다(pname, tol):
    _, profs = P.load_profiles()
    prof = profs[pname]
    for robot in _scenario_robots(prof):
        ok, t, st = drive(robot, tol, prof.mission.get("reservation", {}), prof.graph)
        assert ok, (pname, tol, robot, st)


# ---- 에이전트 CMD_SET_MAP (진짜 메서드, ROS 노드 없이) -----------------------------------------

def _agent(tmp_path, moving=False):
    from test_hybrid_agent_review import _agent as base
    a, clk = base()
    from pinky_fleet_agent import hybrid_agent_node as agent_node
    a._map_dir = str(tmp_path)
    a._map_name = "my_map"
    a._map_load = None
    a._load_map_client = MagicMock()
    a._load_map_client.service_is_ready.return_value = True
    a._linear_velocity = 0.12 if moving else 0.0
    for name in ("_set_map", "_on_map_loaded"):
        setattr(a, name, types.MethodType(getattr(agent_node.PinkyAgent, name), a))
    return a


def _map_files(tmp_path):
    (tmp_path / "map4.yaml").write_text(open(os.path.join(MAP4_DIR, "map4.yaml"), encoding="utf-8").read(),
                                        encoding="utf-8")


def test_에이전트_SET_MAP__지도를_읽히고_성공하면_보고하는_지도_이름이_바뀐다(tmp_path):
    from nav2_msgs.srv import LoadMap
    _map_files(tmp_path)
    a = _agent(tmp_path)
    a._set_map("map4")
    req = a._load_map_client.call_async.call_args[0][0]
    assert req.map_url.endswith("map4.yaml") and a._map_load["result"] == "PENDING"
    fut = MagicMock()
    fut.result.return_value = types.SimpleNamespace(result=LoadMap.Response.RESULT_SUCCESS)
    a._on_map_loaded("map4", fut)
    assert a._map_name == "map4" and a._map_load["result"] == "OK"


def test_에이전트_SET_MAP__달리는_중이면_거부하고_파일이_없으면_말한다(tmp_path):
    a = _agent(tmp_path, moving=True)
    a._set_map("map4")
    assert a._map_load["result"] == "REFUSED"
    a._load_map_client.call_async.assert_not_called()
    b = _agent(tmp_path)
    b._set_map("map4")                                          # 파일 없음
    assert b._map_load["result"] == "NO_FILE" and b._map_name == "my_map"


def test_에이전트_SET_MAP__실패하면_이름을_안_바꾼다(tmp_path):
    from nav2_msgs.srv import LoadMap
    _map_files(tmp_path)
    a = _agent(tmp_path)
    a._set_map("map4")
    fut = MagicMock()
    fut.result.return_value = types.SimpleNamespace(result=LoadMap.Response.RESULT_INVALID_MAP_DATA)
    a._on_map_loaded("map4", fut)
    assert a._map_name == "my_map" and a._map_load["result"] == "FAILED"


# ---- 게이트웨이 --------------------------------------------------------------------------------

@pytest.fixture
def gw(monkeypatch):
    import gateway_web_server as g
    from test_d7_robot_stop import _post  # noqa: F401
    c = _pcoord()
    node = MagicMock()

    def _dispatch(m):
        import json
        from std_msgs.msg import String
        c._cb_control(String(data=json.dumps(m)))
    node.send_fleet_control.side_effect = _dispatch
    ctl = MagicMock()
    ctl.map_yaml_path = "/tmp/legacy.yaml"
    ctl.set_map.return_value = "x.pgm"
    monkeypatch.setattr(g, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(g, "GLOBAL_FLEET_COORDINATOR", c)
    monkeypatch.setattr(g, "GLOBAL_CONTROL", ctl)
    monkeypatch.setattr(g, "GLOBAL_DEFAULT_MAP_YAML", "/tmp/legacy.yaml")
    monkeypatch.setattr(g, "CONTROL_APPLY_WAIT_SEC", 0.2)
    return g, c, ctl


def _post(g, path, ip="127.0.0.1", body=b"{}"):
    from test_d7_robot_stop import _post as p
    return p(g, path, ip=ip, body=body)


def test_게이트웨이_전환은_로컬에서만(gw):
    g, c, _ = gw
    code, _ = _post(g, "/api/fleet/profile", ip="192.0.2.50", body=b'{"name": "map4"}')
    assert code == 403 and c.active_profile == "legacy"


def test_게이트웨이_전환_성공이면_화면_지도도_프로파일_지도로(gw):
    g, c, ctl = gw
    code, body = _post(g, "/api/fleet/profile", body=b'{"name": "map4"}')
    assert code == 200 and body["applied"] is True and body["profile"] == "map4"
    assert ctl.set_map.call_args[0][0].endswith("team11_map4/map4.yaml")


def test_게이트웨이_모르는_프로파일_404__달리는_중_409(gw):
    g, c, _ = gw
    assert _post(g, "/api/fleet/profile", body=b'{"name": "nope"}')[0] == 404
    c.mission_state = "RUNNING"
    code, body = _post(g, "/api/fleet/profile", body=b'{"name": "map4"}')
    assert code == 409 and body["reason"] == "NOT_NOW" and "RUNNING" in body["message"]


def test_게이트웨이_로봇_지도·초기_위치(gw):
    g, c, _ = gw
    _post(g, "/api/fleet/profile", body=b'{"name": "map4"}')
    _reset(c)
    assert _post(g, "/api/fleet/robot_maps")[0] == 200
    assert _sent(c.fleet_cmd_pubs["pinky1"])[-1].command == FleetCommand.CMD_SET_MAP
    assert _post(g, "/api/fleet/initial_poses")[0] == 200
    assert _sent(c.fleet_cmd_pubs["pinky1"])[-1].command == FleetCommand.CMD_SET_INITIAL_POSE


# ---- 화면이 프로파일을 따른다 ------------------------------------------------------------------

STATIC = os.path.join(REPO, "relay_station", "gateway_web", "static")






def test_미션이_배정_거절될_로봇을_품으면_경고하고_전환_화면이_경로_없는_로봇을_보인다():
    _, profs = P.load_profiles()
    assert any("같은 목표 GOAL_C" in w and "pinky2" in w for w in profs["legacy"].warnings)
    assert profs["legacy"].valid                               # 막지는 않는다 — 지금 현장 미션이다
    # map4 두 시나리오는 배정 거절(L3) 경고가 없다 — 지도 칸 경고(REVIEW_20260926 중계 항목, JW_JS·TR_MC·TR_TC)는 따로 있다
    assert not [w for w in profs["map4"].warnings + profs["map4_s2"].warnings if "(L3)" in w or "경로를 못 만든다" in w]
    c = _pcoord()
    assert c.switch_profile("legacy")
    st = c.profile_status()
    assert st["unassigned"] == ["pinky2"]
    assert c.switch_profile("map4")
    assert c.profile_status()["unassigned"] == []


def test_실행본_심링크_농장에서도_게이트웨이가_코디네이터를_찾는다(tmp_path):
    """실물 실행본(~/doc/src/field_gateway_relay)은 gateway_web/*.py 의 파일별 심링크다(G-B). 예전엔 abspath 로
    relay 루트를 잡아 `fleet` 을 못 찾고 코디네이터가 조용히 빠졌다 — 실물은 늘 FLEET_COORDINATOR_UNAVAILABLE 이었다."""
    import glob
    import subprocess
    gw = os.path.join(REPO, "relay_station", "gateway_web")
    farm = tmp_path / "field_gateway_relay"
    farm.mkdir()
    for src in glob.glob(os.path.join(gw, "*.py")) + glob.glob(os.path.join(gw, "*.sh")):
        os.symlink(src, farm / os.path.basename(src))
    os.symlink(os.path.join(gw, "static"), farm / "static")
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(p for p in env.get("PYTHONPATH", "").split(os.pathsep)
                                        if p and "relay_station" not in p and os.path.realpath(p) != REPO)
    r = subprocess.run(["python3", "-B", "-c",
                        "import gateway_web_server as g; print('COORD', g.RelayFleetCoordinator is not None, g.FLEET_IMPORT_ERROR)"],
                       cwd=str(farm), env=env, capture_output=True, text=True, timeout=120)
    assert "COORD True None" in r.stdout, r.stdout[-800:] + r.stderr[-1500:]
