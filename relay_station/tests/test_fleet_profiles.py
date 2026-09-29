# -*- coding: utf-8 -*-
"""R-7 웹 전환 — 좌표 프로파일 (2026-09-26) → map5 통일 (2026-09-29): 프로파일은 team11_map5 하나다.

여기서 재는 것:
- 프로파일 파일 자체(정본 도로망·지도를 상대경로로 가리킨다 — 복사본 없음 · 모든 좌표가 지도 안 · 미션 노드 ⊂ 도로망)
- 저장·복원(깨진 저장값은 기본으로 — 그 사실을 말한다)
- 코디네이터 전환(달리는 중·비상정지 중 거부, 경로 재배정) · 로봇 지도(map5)·초기 위치 명령
- 게이트웨이 엔드포인트 · 화면(V2·index.html)의 연결

⚠️ 저장 파일은 늘 임시 폴더로(PINKY_RELAY_STATE_DIR) — 실물 중계의 ~/.local/state 를 건드리지 않는다.
"""
import math
import os
import re
import sys
from unittest.mock import MagicMock

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pinky_fleet_msgs.msg import FleetCommand, RobotState  # noqa: E402
from relay_station.fleet import profiles as P  # noqa: E402
from relay_station.fleet.reservation import Reservation  # noqa: E402
from test_d7_robot_stop import _coord, _reset, _sent  # noqa: E402

PROF_DIR = os.path.join(REPO, "relay_station", "fleet", "config", "profiles")
PROFILE = "team11_map5"
GRAPH = os.path.join(REPO, "pinky_lane_station", "config", "road_graph.yaml")            # 도로망 정본 (map5 임시)
MAP5_YAML = os.path.join(REPO, "pinky_fleet_station", "config", "map5.yaml")            # 지도 정본
MAP5_PGM = os.path.join(REPO, "pinky_fleet_station", "config", "maps", "map5.pgm")


@pytest.fixture(autouse=True)
def _state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv(P.STATE_ENV, str(tmp_path / "state"))


# ---- 프로파일 파일 ------------------------------------------------------------------------

def test_프로파일은_하나이고_정본_파일을_복사하지_않고_가리킨다():
    default, profs = P.load_profiles()
    assert default == PROFILE and set(profs) == {PROFILE}
    p = profs[PROFILE]
    assert p.valid, p.problems
    assert os.path.realpath(p.graph_path) == os.path.realpath(GRAPH)
    assert os.path.realpath(p.map_yaml) == os.path.realpath(MAP5_YAML)
    assert not os.path.exists(os.path.join(PROF_DIR, "team11_map4"))
    assert p.frame == PROFILE and p.robot_map_name == "map5"


def test_map5_는_좌하단_원점_2_36_x_1_28_이고_도로망_전부가_지도_안이다():
    _, profs = P.load_profiles()
    m = profs[PROFILE].map_meta
    assert m["origin"] == [-0.01, -0.01] and m["resolution"] == 0.01
    assert (m["width"], m["height"]) == (236, 128)
    assert len(profs[PROFILE].graph.nodes) == 4 and len(profs[PROFILE].graph.edges) == 3
    assert [(r["name"], r["start"], r["goal"]) for r in profs[PROFILE].mission["robots"]] == [
        ("pinky1", "BL", "TR"), ("pinky2", "BR", "BL")]


def test_프로파일끼리_로봇_이름이_같다__코디네이터는_토픽을_기동_때_한_번_만든다():
    _, profs = P.load_profiles()
    names = {tuple(sorted(p.robot_names())) for p in profs.values()}
    assert names == {("pinky1", "pinky2")}


def test_지도_밖_좌표가_있으면_깨진_프로파일로_본다(tmp_path):
    """도로망은 그대로, 지도 원점만 옮겨(1,1) 왼쪽·아래 노드가 지도 밖으로 나가게 한다 — TR(2.15, 1.08)만 안에 남는다."""
    (tmp_path / "map.yaml").write_text(
        "image: %s\nresolution: 0.01\norigin: [1.0, 1.0, 0.0]\n" % MAP5_PGM, encoding="utf-8")
    (tmp_path / "m.yaml").write_text("graph: %s\nrobots:\n  - {name: pinky1, start: BL, goal: TR}\n" % GRAPH,
                                     encoding="utf-8")
    man = tmp_path / "profiles.yaml"
    man.write_text("default: x\nprofiles:\n  x:\n    mission: m.yaml\n    map: map.yaml\n", encoding="utf-8")
    _, profs = P.load_profiles(str(man))
    assert not profs["x"].valid and profs["x"].problems[0].startswith("지도 밖")
    listed = profs["x"].problems[0].split(":")[1].split(", ")
    assert "BL" in profs["x"].problems[0] and "TR" not in [x.strip() for x in listed]


# ---- 저장·복원 ------------------------------------------------------------------------------

def test_저장값이_없으면_기본():
    d, profs = P.load_profiles()
    assert P.resolve_active(d, profs) == (PROFILE, None)


def test_저장한_프로파일로_다시_뜬다():
    d, profs = P.load_profiles()
    P.write_active(PROFILE)
    assert P.resolve_active(d, profs) == (PROFILE, None)


def test_저장값이_목록에_없거나_깨졌으면_기본으로__그_사실을_말한다():
    d, profs = P.load_profiles()
    P.write_active("nope")
    name, why = P.resolve_active(d, profs)
    assert name == PROFILE and "nope" in why
    P.write_active(PROFILE)
    profs[PROFILE].problems.append("일부러 깨뜨림")
    name, why = P.resolve_active(d, profs)
    assert name == PROFILE and "깨졌다" in why                 # 기본으로 떨어졌다는 사실은 말한다(같은 이름이어도)


# ---- 로봇 지도 대조 --------------------------------------------------------------------------

def _state(name="map5", known=True, ox=-0.01, oy=-0.01, w=236, h=128, res=0.01):
    s = RobotState()
    s.map_name, s.map_known = name, known
    s.map_origin_x, s.map_origin_y, s.map_width, s.map_height, s.map_resolution = ox, oy, w, h, res
    return s


def test_로봇_지도_대조는_이름만이_아니라_원점·크기까지_본다():
    _, profs = P.load_profiles()
    p = profs[PROFILE]
    assert P.robot_map_check(p, _state())[0] == "MATCH"
    assert P.robot_map_check(p, _state(ox=-1.175, oy=-0.625, w=47, h=25, res=0.05))[0] == "MISMATCH"   # map4 규격
    assert P.robot_map_check(p, _state(name="map4"))[0] == "MISMATCH"
    assert P.robot_map_check(p, _state(known=False))[0] == "UNKNOWN"
    assert P.robot_map_check(p, None)[0] == "UNKNOWN"


def test_지도_규격이_없는_프로파일은_이름만_맞아도_MATCH_라_하지_않는다(tmp_path):
    (tmp_path / "m.yaml").write_text("graph: %s\nrobots:\n  - {name: pinky1, start: BL, goal: TR}\n" % GRAPH,
                                     encoding="utf-8")
    man = tmp_path / "profiles.yaml"
    man.write_text("default: x\nprofiles:\n  x:\n    mission: m.yaml\n    map: null\n    robot_map_name: map5\n",
                   encoding="utf-8")
    _, profs = P.load_profiles(str(man))
    assert profs["x"].valid and profs["x"].map_meta is None
    assert P.robot_map_check(profs["x"], _state(name="map5"))[0] == "NAME_MATCH"
    assert P.robot_map_check(profs["x"], _state(name="map4"))[0] == "MISMATCH"


def test_화면_코드는_캐시하지_않게_답한다():
    src = open(os.path.join(REPO, "relay_station", "gateway_web", "gateway_web_server.py"), encoding="utf-8").read()
    assert "self.send_header('Cache-Control', 'no-cache')" in src


# ---- 코디네이터 전환 ------------------------------------------------------------------------

def _pcoord(state="IDLE"):
    c = _coord(state=state)
    c.profiles_default, c.profiles = P.load_profiles()
    c.active_profile = None
    c.route_pubs = {}
    c.reservation = Reservation(c.graph)
    for ctx in c.robots.values():
        ctx.route = None
    return c


def test_전환하면_도로망·미션이_바뀌고_새_경로가_배정되며_저장된다():
    c = _pcoord()
    assert c.switch_profile(PROFILE) is True
    assert c.active_profile == PROFILE and len(c.graph.nodes) == 4
    assert c.robots["pinky1"].goal_node == "TR" and c.robots["pinky2"].goal_node == "BL"
    assert c.robots["pinky1"].route.node_ids == ["BL", "J", "TR"]
    assert c.robots["pinky2"].route.node_ids == ["BR", "J", "BL"]          # BL 을 pinky1 이 출발 노드로 쥐어도 배정된다
    assert c.mission_state == "ASSIGNED"
    assert P.read_active() == PROFILE
    st = c.get_fleet_status_dict()["profile"]
    assert st["active"] == PROFILE and st["frame"] == PROFILE and st["robot_map_name"] == "map5"


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
    assert c.switch_profile(PROFILE) is False and c.active_profile is None
    assert P.read_active() is None


def _moving():
    s = RobotState()
    s.linear_velocity = 0.12
    return s


def test_모르는_·깨진_프로파일은_거부():
    c = _pcoord()
    assert c.switch_profile("nope") is False
    c.profiles[PROFILE].problems.append("깨짐")
    assert c.switch_profile(PROFILE) is False


def test_로봇_지도_전환은_모든_로봇에_CMD_SET_MAP():
    c = _pcoord()
    c.switch_profile(PROFILE)
    _reset(c)
    assert c.send_robot_maps() is True
    for n in ("pinky1", "pinky2"):
        (fc,) = _sent(c.fleet_cmd_pubs[n])
        assert fc.command == FleetCommand.CMD_SET_MAP and fc.map_name == "map5"


def test_초기_위치는_출발_노드와_첫_구간_방향():
    c = _pcoord()
    c.switch_profile(PROFILE)
    _reset(c)
    assert c.send_initial_poses() is True
    (fc,) = _sent(c.fleet_cmd_pubs["pinky1"])
    wp = c.robots["pinky1"].route.waypoints
    assert fc.command == FleetCommand.CMD_SET_INITIAL_POSE
    assert (fc.x, fc.y) == pytest.approx(wp[0]) and (fc.x, fc.y) == pytest.approx((0.20, 0.20))    # BL
    assert fc.yaw == pytest.approx(math.atan2(wp[1][1] - wp[0][1], wp[1][0] - wp[0][0]))
    (fc2,) = _sent(c.fleet_cmd_pubs["pinky2"])
    assert (fc2.x, fc2.y) == pytest.approx((2.15, 0.20))                                          # BR


def test_제어_토픽으로도_전환되고_결과가_남는다():
    import json
    from std_msgs.msg import String
    c = _pcoord()
    c._cb_control(String(data=json.dumps({"cmd": "profile", "name": PROFILE})))
    assert c.last_control["cmd"] == "profile" and c.last_control["ok"] is True
    assert c.robots["pinky1"].start_node == "BL" and c.robots["pinky2"].goal_node == "BL"


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
    ctl.map_yaml_path = "/tmp/default.yaml"
    ctl.set_map.return_value = "x.pgm"
    monkeypatch.setattr(g, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(g, "GLOBAL_FLEET_COORDINATOR", c)
    monkeypatch.setattr(g, "GLOBAL_CONTROL", ctl)
    monkeypatch.setattr(g, "GLOBAL_DEFAULT_MAP_YAML", "/tmp/default.yaml")
    monkeypatch.setattr(g, "CONTROL_APPLY_WAIT_SEC", 0.2)
    return g, c, ctl


def _post(g, path, ip="127.0.0.1", body=b"{}"):
    from test_d7_robot_stop import _post as p
    return p(g, path, ip=ip, body=body)


def test_게이트웨이_전환은_로컬에서만(gw):
    g, c, _ = gw
    code, _ = _post(g, "/api/fleet/profile", ip="192.0.2.50", body=b'{"name": "%s"}' % PROFILE.encode())
    assert code == 403 and c.active_profile is None


def test_게이트웨이_전환_성공이면_화면_지도도_프로파일_지도로(gw):
    g, c, ctl = gw
    code, body = _post(g, "/api/fleet/profile", body=b'{"name": "%s"}' % PROFILE.encode())
    assert code == 200 and body["applied"] is True and body["profile"] == PROFILE
    assert ctl.set_map.call_args[0][0].endswith(os.path.join("pinky_fleet_station", "config", "map5.yaml"))


def test_게이트웨이_모르는_프로파일_404__달리는_중_409(gw):
    g, c, _ = gw
    assert _post(g, "/api/fleet/profile", body=b'{"name": "nope"}')[0] == 404
    c.mission_state = "RUNNING"
    code, body = _post(g, "/api/fleet/profile", body=b'{"name": "%s"}' % PROFILE.encode())
    assert code == 409 and body["reason"] == "NOT_NOW" and "RUNNING" in body["message"]


def test_게이트웨이_로봇_지도·초기_위치(gw):
    g, c, _ = gw
    _post(g, "/api/fleet/profile", body=b'{"name": "%s"}' % PROFILE.encode())
    _reset(c)
    assert _post(g, "/api/fleet/robot_maps")[0] == 200
    assert _sent(c.fleet_cmd_pubs["pinky1"])[-1].command == FleetCommand.CMD_SET_MAP
    assert _post(g, "/api/fleet/initial_poses")[0] == 200
    assert _sent(c.fleet_cmd_pubs["pinky1"])[-1].command == FleetCommand.CMD_SET_INITIAL_POSE


# ---- 화면이 프로파일을 따른다 ------------------------------------------------------------------

STATIC = os.path.join(REPO, "relay_station", "gateway_web", "static")


def test_V2_설정_탭에_전환_카드가_있고_버튼은_확인을_묻는다():
    html = open(os.path.join(STATIC, "fleet_control_v2.html"), encoding="utf-8").read()
    js = open(os.path.join(STATIC, "fleet_control_v2.js"), encoding="utf-8").read()
    for i in ("profile-card", "profile-select", "profile-switch", "profile-robot-maps", "profile-initial-poses",
              "summary-profile"):
        assert 'id="%s"' % i in html, i
    for url in ("/api/fleet/profiles", "/api/fleet/profile", "/api/fleet/robot_maps", "/api/fleet/initial_poses"):
        assert url in js
    assert "window.confirm" in js and "renderProfile" in js


def test_index_html_지도_규격은_서버가_읽은_지도를_따른다():
    html = open(os.path.join(STATIC, "index.html"), encoding="utf-8").read()
    assert "fetch('/api/fleet/profile_map'" in html
    assert re.search(r"let MAP_ORIGIN\s*=", html) and not re.search(r"const MAP_ORIGIN\s*=", html)


def test_미션이_배정_거절될_로봇을_품으면_경고하고_전환_화면이_경로_없는_로봇을_보인다(tmp_path):
    _, profs = P.load_profiles()
    # map5 임시 미션(BL→TR · BR→BL)은 배정 거절(L3) 경고가 없다
    assert not [w for w in profs[PROFILE].warnings if "(L3)" in w or "경로를 못 만든다" in w]
    # 같은 목표를 품은 미션은 경고하되 막지 않는다 — 뒤 로봇은 경로를 못 받고 전환 화면의 unassigned 에 보인다
    (tmp_path / "m.yaml").write_text("graph: %s\nrobots:\n  - {name: pinky1, domain_id: 10, start: BL, goal: TR}\n"
                                     "  - {name: pinky2, domain_id: 11, start: BR, goal: TR}\n" % GRAPH, encoding="utf-8")
    man = tmp_path / "profiles.yaml"
    man.write_text("default: same\nprofiles:\n  same:\n    mission: m.yaml\n    map: %s\n" % MAP5_YAML, encoding="utf-8")
    _, bad = P.load_profiles(str(man))
    assert bad["same"].valid and any("같은 목표 TR" in w and "pinky2" in w for w in bad["same"].warnings)
    c = _pcoord()
    c.profiles.update(bad)
    assert c.switch_profile("same")
    assert c.profile_status()["unassigned"] == ["pinky2"]
    assert c.switch_profile(PROFILE)
    assert c.profile_status()["unassigned"] == []
    js = open(os.path.join(STATIC, "fleet_control_v2.js"), encoding="utf-8").read()
    assert "p.unassigned" in js and "cur.warnings" in js


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
