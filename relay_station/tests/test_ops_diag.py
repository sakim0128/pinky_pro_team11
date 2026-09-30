# -*- coding: utf-8 -*-
"""R-5 · 진단 업링크(`/pinkyN/diag`)와 `/api/ops/overview`·`/api/ops/diagnostics`.

지시: `docs/REQ_20260924_RELAY_SESSION.md` R-5. 수락: 팜에서 `curl /api/ops/diagnostics` 에 pinky1 의
fix_status·nav2·estop 이 실값으로 보이고, 로봇 컨테이너를 내리면 30 s 안 "미수신".

⭐ 이 파일이 지키는 것: **데이터 없는 칸은 "미수신"(회색) — 초록 기본값 금지.** 화면의 색 규칙은
`fleet_control_v2.js` 의 `statusClass` 를 **그 파일에서 읽어** 대사한다(손으로 옮겨 적지 않는다).
"""
import io
import json
import math
import os
import re
import sys
from unittest.mock import MagicMock

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GATEWAY_DIR = os.path.join(REPO, "relay_station", "gateway_web")
sys.path.insert(0, REPO)
sys.path.insert(0, GATEWAY_DIR)

import pytest

import ops_view as ov
from pinky_fleet_agent import diag as dg

JS = os.path.join(GATEWAY_DIR, "static", "fleet_control_v2.js")
LAUNCH = os.path.join(REPO, "pinky_fleet_agent", "launch", "robot.launch.xml")


# ---- 화면 색 규칙을 JS 에서 읽는다 ---------------------------------------------

def _status_class_rules():
    src = open(JS, encoding="utf-8").read()
    body = src[src.index("function statusClass"):]
    body = body[:body.index("\n  }")]
    rules = []
    for line in body.splitlines():
        m = re.search(r'return "(\w+)"', line)
        if m and "includes" in line:
            rules.append((m.group(1), re.findall(r'includes\("([^"]+)"\)', line)))
    return rules


RULES = _status_class_rules()


def css_class(word):
    s = str(word or "").upper()
    for cls, keys in RULES:
        if any(k in s for k in keys):
            return cls
    return "muted"


def test_화면_색_규칙을_JS_에서_읽었다():
    assert [c for c, _ in RULES] == ["good", "warn", "bad"], RULES


# ---- 로봇 쪽 진단 (diag.py) -----------------------------------------------------

def test_fix_status_실제_발행_형식을_읽는다():
    assert dg.parse_fix_status("accepted 3 rejected 1 age 0.4s") == {
        "accepted": 3, "rejected": 1, "fix_age_s": 0.4, "raw": "accepted 3 rejected 1 age 0.4s"}
    assert dg.parse_fix_status("accepted 0 rejected 2 age inf")["fix_age_s"] is None   # inf 는 null
    assert dg.parse_fix_status("엉뚱한 글") is None


def test_게이트_상태_실제_발행_형식을_읽는다():
    assert dg.parse_gate_status("source=MISSION vx=0.10 wz=-0.20") == {
        "source": "MISSION", "vx": 0.10, "wz": -0.20}
    assert dg.parse_gate_status(None) is None


def test_응답_없는_수명주기_노드는_inactive_가_아니라_unknown():
    s = dg.nav2_summary({"controller_server": "active", "planner_server": "inactive", "bt_navigator": None})
    assert s["nodes"]["bt_navigator"] == "unknown"
    assert (s["active"], s["total"]) == (1, 3)
    assert s["not_active"] == ["bt_navigator", "planner_server"]


def test_기본_8노드는_로봇_launch_의_목록_더하기_map_server():
    xml = open(LAUNCH, encoding="utf-8").read()
    m = re.search(r'name="lifecycle_nodes_nav"\s+value="\[([^\]]+)\]"', xml, re.S)
    nav = [x.strip().strip("'\"") for x in m.group(1).split(",")]
    assert set(dg.DEFAULT_NAV2_NODES) == set(nav) | {"map_server"}
    assert len(dg.DEFAULT_NAV2_NODES) == 8                  # 관제가 말한 "lifecycle 8노드"


def test_위치_출처__AMCL_PoseFuser_모름():
    amcl = dg.nav2_summary({"amcl": "active"})
    assert dg.pose_source(amcl, None) == "AMCL"
    assert dg.pose_source(dg.nav2_summary({}), {"accepted": 1}) == "PoseFuser"
    assert dg.pose_source(dg.nav2_summary({}), None) is None


def test_진단은_못_잰_것을_null_로_두고_표준_JSON_이다():
    d = dg.build("pinky1", 100.0, True, {"controller_server": None})
    assert d["schema"] == "pinky_diag/1"
    assert d["fix_status"] is None and d["gate"] is None and d["estop"] is None
    assert d["tf"] == {"fresh": False, "age_s": None}
    json.dumps(d, allow_nan=False)                           # NaN 이 섞이면 여기서 터진다


def test_진단은_상태를_받은_지_몇_초인지_싣는다():
    d = dg.build("pinky1", 100.0, False, {}, fix_text="accepted 1 rejected 0 age 0.2s", fix_time=97.0,
                 gate_text="source=IDLE vx=0.00 wz=0.00", gate_time=99.5, estop=False, tf_age=0.1)
    assert d["fix_status"]["received_age_s"] == pytest.approx(3.0)
    assert d["gate"]["received_age_s"] == pytest.approx(0.5)
    assert d["tf"]["fresh"] is True and d["estop"] is False


# ---- /api/ops/* (ops_view.py) --------------------------------------------------

def _healthy(amcl=False):
    states = {n: "active" for n in dg.DEFAULT_NAV2_NODES}
    if amcl:
        states["amcl"] = "active"
    return dg.build("pinky1", 1000.0, True, states,
                    fix_text="accepted 5 rejected 1 age 0.3s", fix_time=999.9,
                    gate_text="source=MISSION vx=0.10 wz=0.00", gate_time=999.9,
                    estop=False, tf_age=0.1, agent={"drive_state": 1})


FLEET = {"mission_state": "RUNNING", "robots": {"pinky1": {
    "drive_mode": "nav2", "held": False, "last_heard_sec": 0.2, "clear_until_idx": 9,
    "start": {"acknowledged": True, "retries_left": 4, "gave_up": False},
    "arrival_status": "NOT_ARRIVED", "state": {"battery_percent": 77.0}}}}


def test_받은_적_없으면_전부_미수신_회색():
    r = ov.diagnostics(("pinky1",), {}, None, 1000.0)["robots"]["pinky1"]
    assert r["diag"] == "NEVER" and r["overall"] == ov.NO_DATA
    for k in ("pose_fuser", "tf", "nav2", "planner", "controller", "drive_gate", "estop",
              "obstacle", "motor_watchdog", "camera", "lidar", "robot_state", "battery"):
        assert r[k] == ov.NO_DATA, k
        assert css_class(r[k]) == "muted", (k, r[k])


def test_살아_있으면_실값():
    r = ov.diagnostics(("pinky1",), {"pinky1": (_healthy(), 999.5)}, FLEET, 1000.0)["robots"]["pinky1"]
    assert r["diag"] == "FRESH"
    assert r["pose_fuser"].startswith("LIVE 수락 5·거부 1")
    assert r["nav2"] == "OK 8/8 active"
    assert r["estop"] == "OK 해제" and r["drive_gate"] == "OK source=MISSION"
    assert r["battery"] == "77%" and r["robot_state"].startswith("LIVE")
    assert r["pose_source"] == "PoseFuser"
    assert r["fleet"]["start"]["acknowledged"] is True
    assert css_class(r["overall"]) == "good"
    for k in ov.UNREPORTED:                                   # 살아 있어도 안 보내는 칸은 지어내지 않는다
        assert r[k] == ov.NO_DATA, k


def test_수락__끊기면_5초_넘어서부터_전부_미수신__마지막_값을_보여주지_않는다():
    """수락은 "30 s 안" — 문턱은 5 s 다."""
    entry = {"pinky1": (_healthy(), 1000.0)}
    fresh = ov.diagnostics(("pinky1",), entry, FLEET, 1000.0 + ov.STALE_AFTER_SEC)["robots"]["pinky1"]
    assert fresh["diag"] == "FRESH"
    r = ov.diagnostics(("pinky1",), entry, FLEET, 1000.0 + ov.STALE_AFTER_SEC + 0.1)["robots"]["pinky1"]
    assert r["diag"] == "STALE" and r["detail"] is None and r["pose_source"] is None
    for k in ("pose_fuser", "tf", "nav2", "planner", "controller", "drive_gate", "estop"):
        assert r[k].startswith(ov.NO_DATA), (k, r[k])
        assert css_class(r[k]) == "muted"
    assert ov.STALE_AFTER_SEC < 30.0


def test_FAIL_칸_하나가_전체를_FAIL_로__첫_판은_키를_봐서_늘_OK_였다():
    d = _healthy()
    d["nav2"] = dg.nav2_summary({n: None for n in dg.DEFAULT_NAV2_NODES})   # Nav2 가 응답 없음
    r = ov.diagnostics(("pinky1",), {"pinky1": (d, 999.9)}, FLEET, 1000.0)["robots"]["pinky1"]
    assert r["nav2"].startswith("FAIL 0/8")
    assert r["overall"].startswith("FAIL") and css_class(r["overall"]) == "bad"


def test_ESTOP_이_걸리면_WARN():
    d = _healthy()
    d["estop"] = True
    r = ov.diagnostics(("pinky1",), {"pinky1": (d, 999.9)}, FLEET, 1000.0)["robots"]["pinky1"]
    assert css_class(r["estop"]) == "warn" and css_class(r["overall"]) == "warn"


def test_일부만_못_쟀으면_OK_가_빈칸을_이름으로_밝힌다():
    d = _healthy()
    d["estop"] = None
    r = ov.diagnostics(("pinky1",), {"pinky1": (d, 999.9)}, FLEET, 1000.0)["robots"]["pinky1"]
    assert r["overall"] == "OK (미수신: estop)"


def test_상태_토픽이_끊긴_노드는_값이_아니라_나이로_LOST():
    d = dg.build("pinky1", 1000.0, True, {n: "active" for n in dg.DEFAULT_NAV2_NODES},
                 fix_text="accepted 5 rejected 1 age 0.3s", fix_time=990.0, estop=False, tf_age=0.1)
    r = ov.diagnostics(("pinky1",), {"pinky1": (d, 999.9)}, FLEET, 1000.0)["robots"]["pinky1"]
    assert r["pose_fuser"].startswith("LOST") and css_class(r["pose_fuser"]) == "bad"


def test_json_safe_는_NaN_inf_를_null_로():
    assert ov.json_safe({"a": float("nan"), "b": [1.0, float("inf")], "c": {"d": -float("inf")}}) == \
        {"a": None, "b": [1.0, None], "c": {"d": None}}


def test_응답에_실리는_NaN_도_표준_JSON():
    """배터리 NaN 은 "미수신" 으로 바뀌어 응답에 닿지 않는다 — 응답에 그대로 실리는 칸(last_heard_sec)으로 잰다."""
    fleet = json.loads(json.dumps(FLEET))
    fleet["robots"]["pinky1"]["last_heard_sec"] = float("nan")
    body = ov.json_safe(ov.diagnostics(("pinky1",), {}, fleet, 1000.0))
    json.dumps(body, allow_nan=False)
    assert body["robots"]["pinky1"]["fleet"]["last_heard_sec"] is None


def test_NaN_배터리도_표준_JSON():
    fleet = json.loads(json.dumps(FLEET))
    fleet["robots"]["pinky1"]["state"]["battery_percent"] = float("nan")
    body = ov.json_safe(ov.diagnostics(("pinky1",), {}, fleet, 1000.0))
    text = json.dumps(body, allow_nan=False)
    assert json.loads(text)["robots"]["pinky1"]["battery"] == ov.NO_DATA


def test_overview_는_요약과_경고를_준다():
    o = ov.overview(("pinky1", "pinky2"), {"pinky1": (_healthy(amcl=True), 999.9)}, FLEET, 1000.0)
    assert o["mission_state"] == "RUNNING"
    assert o["robots"]["pinky1"]["pose_source"] == "AMCL"
    assert o["robots"]["pinky1"]["drive_state"] == "CRUISE"
    assert o["robots"]["pinky2"]["overall"] == ov.NO_DATA
    assert any(w.startswith("pinky2") for w in o["warnings"])


# ---- 게이트웨이 GET (가짜 요청으로 do_GET 을 직접 부른다) ----------------------

@pytest.fixture
def gw(monkeypatch):
    import gateway_web_server as g
    node = MagicMock()
    node.diag_snapshot.return_value = {}
    coord = MagicMock()
    fleet = json.loads(json.dumps(FLEET))
    fleet["robots"]["pinky1"]["state"]["battery_percent"] = float("nan")
    fleet["robots"]["pinky1"]["last_heard_sec"] = float("nan")     # 응답에 그대로 실리는 칸
    coord.get_fleet_status_dict.return_value = fleet
    monkeypatch.setattr(g, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(g, "GLOBAL_FLEET_COORDINATOR", coord)
    return g, node


def _get(g, path):
    h = g.GatewayRequestHandler.__new__(g.GatewayRequestHandler)
    h.path = path
    h.headers = {}
    h.client_address = ("192.0.2.9", 5000)
    out = {}

    def _send_json(body_bytes, code=200):
        out["code"], out["raw"] = code, body_bytes.decode("utf-8")
    h._send_json = _send_json
    h.do_GET()
    return out["code"], out["raw"]


@pytest.mark.parametrize("path", ["/api/ops/diagnostics", "/api/ops/overview"])
def test_ops_API_는_표준_JSON_이고_로봇이_없으면_미수신(gw, path):
    g, _ = gw
    code, raw = _get(g, path)
    assert code == 200
    assert "NaN" not in raw                       # 브라우저 JSON.parse 가 깨지지 않는다
    body = json.loads(raw)
    assert body["robots"]["pinky1"]["overall"] == ov.NO_DATA
    assert set(body["robots"]) == {"pinky1", "pinky2"}


def test_ops_diagnostics_가_받은_진단을_실값으로_싣는다(gw):
    g, node = gw
    import time
    node.diag_snapshot.return_value = {"pinky1": (_healthy(), time.time())}
    _, raw = _get(g, "/api/ops/diagnostics")
    r = json.loads(raw)["robots"]["pinky1"]
    assert r["nav2"] == "OK 8/8 active" and r["estop"] == "OK 해제"
    assert r["pose_fuser"].startswith("LIVE")
