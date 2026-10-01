# -*- coding: utf-8 -*-
"""제3자 검수 REVIEW_20260926_THIRD_PARTY_S1_S7 (게이트웨이·플릿) + REVIEW_20260926 G + 중계 항목(목표 구역 별칭).

G-2 (관제 P1) 코디네이터가 없으면 플릿 명령(비상정지 포함)이 받는 쪽 0 인 채 200 이었다 → 503 NO_FLEET_COORDINATOR
G-1 코디네이터가 없으면 우회 목표·미션이 열렸다 → 503 · 기동 검사가 플릿 import 를 크게 말한다
G-11 ROS 노드가 없어도 목표 200 → 503
G-3·G-4·G-5 · G 비상정지 래치·플릿 STOPPED·DONE·로봇별 정지가 재시작을 못 넘었다 → 상태 파일(원자적 쓰기, 깨지면 래치)
   + 재기동 뒤 링크유실 래치를 모르던 것 → 로봇별 정지로 세우고 이유를 보인다, 로봇 재개가 푼다
   + 플릿 STOPPED·DONE 중 로봇 재개(R5)가 로봇의 링크유실·ESTOP 래치 해제를 삼키던 것 → RESUME 바로 뒤 STOP
G-6 플릿 RUNNING 중 경로 있는 로봇의 우회 목표 → 409 FLEET_RUNNING
G-10 로봇 이름을 못 찾으면 HOLD 검사를 건너뛰었다 → 503
G-13 변이 M1(resume_fleet 의 권한 잡기 삭제)을 잡는 시험
G-14 HTTP 스레드 읽기와 executor 틱이 한 잠금을 쓴다
중계 항목 목표 구역 별칭('goal'·'goal_c'·'goal_zone')은 legacy 좌표이거나 프로파일이 없을 때만 — map4 는 목표 노드 이름으로

⚠️ 상태 파일은 늘 tmp_path 로(PINKY_RELAY_STATE_DIR) — 실물 중계의 ~/.local/state 를 건드리지 않는다.
⚠️ 실제 코디네이터 생성자 시험은 격리 도메인(97, LOCALHOST)의 하위 프로세스에서만 노드를 만든다.
"""
import io
import json
import os
import subprocess
import sys
import threading
from unittest.mock import MagicMock

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "relay_station", "scripts"))

from pinky_fleet_msgs.msg import FleetCommand, RobotState  # noqa: E402
from pinky_lane_msgs.msg import LaneCommand, LaneStatus  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from relay_station.fleet import fleet_coordinator as FC  # noqa: E402
from relay_station.fleet import profiles as P  # noqa: E402
from relay_station.fleet.reservation import Reservation  # noqa: E402
from test_d7_robot_stop import _coord, _lane_status, _post, _reset, _sent  # noqa: E402

GATEWAY_DIR = os.path.join(REPO, "relay_station", "gateway_web")
IMPORT_ERR = "ModuleNotFoundError: No module named 'pinky_fleet_msgs'"


@pytest.fixture(autouse=True)
def _state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv(P.STATE_ENV, str(tmp_path / "state"))


@pytest.fixture
def g(monkeypatch):
    import gateway_web_server as gws
    node = MagicMock()
    monkeypatch.setattr(gws, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", None)
    monkeypatch.setattr(gws, "FLEET_IMPORT_ERROR", IMPORT_ERR)
    monkeypatch.setattr(gws, "ALLOW_DIRECT_FALLBACK", False)
    monkeypatch.setattr(gws, "CONTROL_APPLY_WAIT_SEC", 0.1)
    monkeypatch.setattr(gws, "STOP_CONFIRM_WAIT_SEC", 0.05)
    monkeypatch.setattr(gws, "STOP_CONFIRM_POLL_SEC", 0.01)
    return gws, node


# ==== G-2 (관제 P1) · 코디네이터가 없으면 플릿 명령은 503 — 받는 쪽 0 인 성공이 없다 ====================================

@pytest.mark.parametrize("cmd", ["estop", "stop", "start", "resume", "assign"])
def test_G2_코디네이터가_없으면_플릿_명령은_503_이고_보내지_않는다(g, cmd):
    gws, node = g
    ip = "192.0.2.50" if cmd in ("estop", "stop") else "127.0.0.1"   # 멈추는 명령은 원격에서도 받는다(D7)
    code, body = _post(gws, "/api/fleet/" + cmd, ip=ip,
                       body=b'{"robot": "pinky1", "start": "BL", "goal": "TR"}')
    assert code == 503 and body["success"] is False and body["reason"] == "NO_FLEET_COORDINATOR"
    assert body["detail"] == IMPORT_ERR and body["dispatched"] is False
    assert body["message"]                                      # V2 는 message 를 실패 창에 띄운다
    node.send_fleet_control.assert_not_called()


def test_G2_ROS_노드가_없어도_플릿_명령은_503_NO_FLEET_COORDINATOR(g, monkeypatch):
    gws, _ = g
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", _coord())
    monkeypatch.setattr(gws, "GLOBAL_ROBOT_SUB_NODE", None)
    code, body = _post(gws, "/api/fleet/estop")
    assert code == 503 and body["reason"] == "NO_FLEET_COORDINATOR" and "NO_ROS_NODE" in body["detail"]


def test_G2_직접_모드는_그대로__코디네이터가_받는다(g, monkeypatch):
    """RELAY_ALLOW_DIRECT_FALLBACK=1(시험·최소 모드)은 받는 쪽이 코디네이터 자신이다 — 503 으로 막지 않는다."""
    gws, _ = g
    c = _coord()
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", c)
    monkeypatch.setattr(gws, "GLOBAL_ROBOT_SUB_NODE", None)
    monkeypatch.setattr(gws, "ALLOW_DIRECT_FALLBACK", True)
    code, body = _post(gws, "/api/fleet/estop")
    assert code == 200 and body["dispatched_via"] == "direct" and c.estop_latched


def test_G2_로봇_재개도_코디네이터가_없으면_503_이고_보내지_않는다(g):
    gws, node = g
    code, body = _post(gws, "/api/pinky1/resume", ip="127.0.0.1")
    assert code == 503 and body["reason"] == "NO_FLEET_COORDINATOR" and body["detail"] == IMPORT_ERR
    node.send_fleet_control.assert_not_called()


def test_G2_로봇_정지의_503_도_이유를_싣는다(g):
    gws, _ = g
    code, body = _post(gws, "/api/pinky1/stop")
    assert code == 503 and body["reason"] == "NO_FLEET_COORDINATOR" and body["detail"] == IMPORT_ERR


def test_G2_화면은_503_비상정지·정지를_실패로_보인다():
    """V2 의 플릿 버튼은 postJson 이 !res.ok 에서 던지고 '명령 전송 실패' 창을 띄운다(초록 없음).
    index.html 의 정지는 ok·success 가 아니면 빨강 '정지 불가'. (정적 검사 — 렌더링 검증 아님)"""
    static = os.path.join(GATEWAY_DIR, "static")
    js = io.open(os.path.join(static, "fleet_control_v2.js"), encoding="utf-8").read()
    post = js[js.index("async function postJson"):js.index("function initTabs")]
    assert "if (!res.ok) throw new Error(body.message" in post
    ctl = js[js.index("function initControls"):js.index("function robotName")]
    assert 'postJson("/api/fleet/" + cmd' in ctl and 'alert("명령 전송 실패: "' in ctl
    html = io.open(os.path.join(static, "index.html"), encoding="utf-8").read()
    stop = html[html.index("async function stopNavigation"):html.index("function renderNavCanvas")]
    assert "if (r.ok && body.success)" in stop and "#f85149" in stop.split("} else if (msgEl) {")[1]


def test_G1_기동_검사가_플릿_코디네이터_import_를_본다__이_레포는_된다():
    import check_gateway_imports as C
    assert C.fleet_import_problem(GATEWAY_DIR) is None


def test_G1_오버레이를_source_하지_않았으면_이유를_말한다():
    """launch_master_gateway.sh 는 레포 install 오버레이를 **있을 때만** source 한다 — 없으면 pinky_*_msgs 가 없다."""
    import check_gateway_imports as C
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(p for p in env.get("PYTHONPATH", "").split(os.pathsep)
                                        if p and "pinky_fleet_msgs" not in p and "pinky_lane_msgs" not in p)
    why = C.fleet_import_problem(GATEWAY_DIR, env=env)
    assert why and "pinky_" in why and "No module named" in why


def test_G1_실행본_심링크_농장에서도_게이트웨이와_같은_realpath_로_찾는다(tmp_path):
    import glob
    import check_gateway_imports as C
    farm = tmp_path / "field_gateway_relay"
    farm.mkdir()
    for src in glob.glob(os.path.join(GATEWAY_DIR, "*.py")):
        os.symlink(src, farm / os.path.basename(src))
    assert C.fleet_import_problem(str(farm)) is None
    (tmp_path / "copy").mkdir()                                  # 사본(심링크 아님)은 relay 루트를 못 찾는다
    io.open(str(tmp_path / "copy" / "gateway_web_server.py"), "w").write("")
    assert "fleet_coordinator.py 가 없다" in C.fleet_import_problem(str(tmp_path / "copy"))


def test_G1_기동_검사는_플릿_문제를_크게_찍고__require_fleet_이면_실패한다(monkeypatch, capsys):
    import check_gateway_imports as C
    monkeypatch.setattr(C, "fleet_import_problem", lambda d, env=None, timeout=30: IMPORT_ERR)
    monkeypatch.setattr(C, "undefined_names", lambda d, require=False: ([], True))
    assert C.main(["--gateway-dir", GATEWAY_DIR, "--quiet"]) == 0     # 게이트웨이는 뜬다 — 알리기만
    out = capsys.readouterr().out
    assert "🔴 플릿 코디네이터를 import 하지 못합니다" in out and IMPORT_ERR in out
    assert C.main(["--gateway-dir", GATEWAY_DIR, "--quiet", "--require-fleet"]) == 1


def _farm(root, copy_server):
    """현장 실행본(~/doc/src/field_gateway_relay)처럼 레포 파일을 심링크로 건 폴더. copy_server 면 본체만 사본."""
    import glob
    import shutil
    farm = root / "field_gateway_relay"
    farm.mkdir(parents=True)
    for src in glob.glob(os.path.join(GATEWAY_DIR, "*.py")):
        dst = farm / os.path.basename(src)
        if copy_server and dst.name == "gateway_web_server.py":
            shutil.copy(src, str(dst))
        else:
            os.symlink(src, str(dst))
    return farm


def test_GW_R2_기동_검사는_게이트웨이를_띄우는_실행본_폴더로_코디네이터를_찾는다(tmp_path, monkeypatch, capsys):
    """제3자 재검 GW-R2: 런처는 실행본 농장에서 게이트웨이를 띄우는데 검사는 레포 폴더를 봤다 — 본체가 사본이 되면
    (§14-3 의 G-B 회귀) 게이트웨이는 fleet 을 못 찾는데 검사는 'import 정상' 이라 했다."""
    import check_gateway_imports as C
    monkeypatch.setattr(C, "undefined_names", lambda d, require=False: ([], True))
    farm = _farm(tmp_path / "copy", copy_server=True)
    (farm / "robot1_mission_navigator.py").write_text("def (:\n")   # 농장의 다른 파일은 소스 검사 대상이 아니다
    assert C.main(["--gateway-dir", GATEWAY_DIR, "--exec-dir", str(farm)]) == 0
    out = capsys.readouterr().out
    assert "🔴 플릿 코디네이터를 import 하지 못합니다" in out and "fleet_coordinator.py 가 없다" in out
    assert C.main(["--gateway-dir", GATEWAY_DIR, "--exec-dir", str(farm), "--require-fleet"]) == 1
    capsys.readouterr()
    ok = _farm(tmp_path / "links", copy_server=False)
    assert C.main(["--gateway-dir", GATEWAY_DIR, "--exec-dir", str(ok), "--require-fleet"]) == 0
    assert "플릿 코디네이터 import 정상" in capsys.readouterr().out
    assert C.main(["--gateway-dir", str(farm), "--require-fleet"]) == 1       # 안 주면 예전처럼 --gateway-dir 로 본다
    assert "fleet_coordinator.py 가 없다" in capsys.readouterr().out


def test_GW_R2_런처는_게이트웨이를_띄우는_폴더를_검사에_넘긴다():
    src = io.open(os.path.join(REPO, "relay_station", "launch_master_gateway.sh"), encoding="utf-8").read()
    calls = [l for l in src.splitlines() if 'python3 "$_IMPORT_CHECK"' in l]
    assert len(calls) == 1 and '--exec-dir "$SCRIPT_DIR"' in calls[0], calls
    head = src[:src.index("python3 gateway_web_server.py")]
    assert head[head.rfind("\ncd "):].startswith('\ncd "$SCRIPT_DIR"'), "게이트웨이를 띄우는 폴더가 검사에 넘긴 폴더와 다르다"


# ==== G-13 · 변이 M1 — resume_fleet 이 비상정지 권한을 잡지 않으면 ======================================================

def test_G13_S6_복원_뒤_플릿_재개는_권한을_잡아_세워_둔_로봇의_ESTOP_보고로_다시_래치하지_않는다():
    """E5 그대로: 재시작 → 로봇 ESTOP 보고로 S6 복원(+HOLD) → 플릿 재개 → 세워 둔 로봇은 ESTOP 을 계속 보고한다.
    resume_fleet 이 권한을 안 잡으면(M1) 그 보고가 다시 플릿 래치를 건다 — 재개가 안 먹는 고리."""
    c = _coord(state="ASSIGNED")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, 100.0, "E-STOP")
    assert c.estop_latched and c.robots["pinky1"].held
    c.resume_fleet()
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, 100.2, "E-STOP")
    assert not c.estop_latched and c.mission_state == "ASSIGNED"
    assert c.robots["pinky1"].held                            # S7: 세워 둔 로봇은 그대로


# ==== G-14 · HTTP 스레드의 읽기와 executor 틱이 한 잠금을 쓴다 ============================================================

_LOCKED_CALLS = {
    "assign_conflict": lambda c: c.assign_conflict("pinky2", "BL", "BR"),
    "assign_conflict_why": lambda c: c.assign_conflict_why("pinky2", "BL", "BR"),
    "get_fleet_status_dict": lambda c: c.get_fleet_status_dict(),
    "profile_status": lambda c: c.profile_status(),
    "profile_switch_blocker": lambda c: c.profile_switch_blocker(),
    "robot_map_check": lambda c: c.robot_map_check("pinky1"),
    "hold_confirmation": lambda c: c.hold_confirmation("pinky1", 0.0),
    "last_heard_sec": lambda c: c.last_heard_sec("pinky1"),
    "loop_tick": lambda c: c._loop_tick(),
    "cb_control": lambda c: c._cb_control(String(data='{"cmd": "stop_robot", "robot": "pinky2"}')),
    "cb_lane_status": lambda c: _lane_status(c, "pinky2", LaneStatus.DRIVE_IDLE, 101.0),
    "assign_route": lambda c: c.assign_route("pinky1", "BL", "TR"),
    # RELAY_ALLOW_DIRECT_FALLBACK 은 HTTP 스레드에서 이것들을 곧바로 부른다
    "start_fleet": lambda c: c.start_fleet(),
    "stop_fleet": lambda c: c.stop_fleet(),
    "estop_fleet": lambda c: c.estop_fleet(),
    "resume_fleet": lambda c: c.resume_fleet(),
    "cb_vision_zone_event": lambda c: c._cb_vision_zone_event(String(data='{"robot_name": "pinky1", "zone_id": "x"}')),
    "cb_robot_state": lambda c: c._cb_robot_state("pinky1", RobotState()),
    "publish_status": lambda c: c._publish_status(),
}


@pytest.mark.parametrize("name", sorted(_LOCKED_CALLS))
def test_G14_HTTP_읽기와_executor_콜백은_코디네이터_잠금을_기다린다(name):
    c = _coord(state="ASSIGNED")
    c.profiles, c.active_profile = {}, None
    c.state_timeout_sec, c.global_seq = 1.0, 0
    c.route_pubs, c.pub_status = {}, MagicMock()
    c._coord_lock = threading.RLock()
    held, release = threading.Event(), threading.Event()

    def _hold():
        with c._coord_lock:
            held.set()
            release.wait(2.0)
    t = threading.Thread(target=_hold, daemon=True)
    t.start()
    assert held.wait(2.0)
    done = threading.Event()
    threading.Thread(target=lambda: (_LOCKED_CALLS[name](c), done.set()), daemon=True).start()
    assert not done.wait(0.3), "%s 가 잠금을 기다리지 않았다 — 틱이 dict 를 바꾸는 사이 순회한다" % name
    release.set()
    assert done.wait(2.0), "%s 가 잠금이 풀린 뒤에도 끝나지 않았다" % name
    t.join(2.0)


class _OwnedProbe(dict):
    """읽힐 때 코디네이터 잠금을 이 스레드가 쥐고 있었는지 적는다."""
    def __init__(self, lock, seen, *a):
        super().__init__(*a)
        self._lock, self._seen = lock, seen

    def get(self, *a):
        self._seen.append(("profiles", self._lock._is_owned()))
        return super().get(*a)


def test_G14_상태·전환_카드는_읽는_내내_잠금을_쥔다():
    """안쪽에서 부르는 잠긴 메서드가 기다려 주는 것만으로는 부족하다 — 예약 dict(틱이 바꾼다)를 읽는 순간과 프로파일을
    읽는 순간에도 잠금을 쥐어야 한 번의 응답이 한 시점의 모습이다."""
    c = _coord(state="RUNNING")
    c._coord_lock = threading.RLock()
    seen = []
    res = c.reservation
    c.reservation = MagicMock(wraps=res)
    c.reservation.edge_holder, c.reservation.node_holder = {}, {}
    c.reservation.status.side_effect = lambda n: (seen.append(("status", c._coord_lock._is_owned())), res.status(n))[1]
    c.get_fleet_status_dict()
    c.profiles, c.active_profile = _OwnedProbe(c._coord_lock, seen), "team11_map5"
    c.profile_status()
    assert {what for what, _ in seen} == {"status", "profiles"} and all(owned for _, owned in seen), seen


# ==== G-3·G-4·G-5 · 멈추는 쪽 제어 상태는 재시작을 넘는다 ================================================================

def _persisting(tmp_path, state="RUNNING"):
    c = _coord(state=state)
    c._control_state_path = str(tmp_path / "state" / FC.CONTROL_STATE_FILE)
    return c


def _restart(tmp_path, state="ASSIGNED"):
    """재시작한 코디네이터 — 미션 파일로 경로를 다시 배정해 ASSIGNED 로 뜬 뒤(__init__ 순서) 상태 파일을 읽는다."""
    c = _coord(state=state)
    for ctx in c.robots.values():
        ctx.start_acknowledged = False
    c._control_state_path = str(tmp_path / "state" / FC.CONTROL_STATE_FILE)
    c._restore_control_state()
    return c


def _file(tmp_path):
    return json.load(io.open(str(tmp_path / "state" / FC.CONTROL_STATE_FILE), encoding="utf-8"))


def test_상태_파일은_좌표_프로파일_상태와_같은_폴더__PINKY_RELAY_STATE_DIR_을_따른다(tmp_path):
    assert FC.control_state_path() == str(tmp_path / "state" / "fleet_control.json")
    assert os.path.dirname(FC.control_state_path()) == os.path.dirname(P.state_path())


def test_G5_비상정지_래치는_로봇_보고_전에도_재시작을_넘는다(tmp_path, g, monkeypatch):
    """G-5: 예전엔 로봇의 첫 DRIVE_ESTOP 보고 전 창에서 로봇 재개가 RESUME 을 냈고 우회 목표가 200 이었다."""
    c = _persisting(tmp_path)
    c.estop_fleet()
    assert _file(tmp_path)["estop_latched"] is True
    c2 = _restart(tmp_path)
    assert c2.estop_latched and c2.mission_state == "ESTOP"
    _reset(c2)
    assert c2.resume_robot("pinky1") is False
    assert _sent(c2.lane_cmd_pubs["pinky1"]) == []
    # (옛 우회 목표 API /api/robot1/goal 의 409 검사는 지웠다 — 그 경로는 없다, 2026-09-29)
    c2._publish_lane_commands()
    assert _sent(c2.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_ESTOP


def test_G5_복원한_래치는_파일이_권한이다__세워_둔_로봇만_HOLD_로_남는다(tmp_path):
    """파일이 래치와 로봇별 정지를 다 안다 — 로봇의 ESTOP 보고로 모든 로봇을 HOLD 로 되살릴 필요가 없다(재개가 먹는다)."""
    c = _persisting(tmp_path)
    c.stop_robot("pinky2")
    c.estop_fleet()
    c2 = _restart(tmp_path)
    for n in ("pinky1", "pinky2"):
        _lane_status(c2, n, LaneStatus.DRIVE_ESTOP, 100.1, "E-STOP")
    assert not c2.robots["pinky1"].held and c2.robots["pinky2"].held
    _reset(c2)
    c2.resume_fleet()
    assert _file(tmp_path)["estop_latched"] is False and _file(tmp_path)["held"] == {"pinky2": FC.HELD_BY_OPERATOR}
    assert [m.command for m in _sent(c2.lane_cmd_pubs["pinky1"])] == [LaneCommand.CMD_RESUME]
    assert _sent(c2.lane_cmd_pubs["pinky2"]) == []                  # S7
    _lane_status(c2, "pinky2", LaneStatus.DRIVE_ESTOP, 100.3, "E-STOP")
    assert not c2.estop_latched


def test_G5_파일이_래치_아님이면_S6_은_그대로_지킨다(tmp_path):
    c = _persisting(tmp_path)
    c.stop_robot("pinky1")
    c2 = _restart(tmp_path)
    assert not c2.estop_latched
    _lane_status(c2, "pinky2", LaneStatus.DRIVE_ESTOP, 100.1, "E-STOP")
    assert c2.estop_latched and c2.robots["pinky2"].held


def test_G4_플릿_정지는_재시작을_넘고_배정이_ASSIGNED_로_덮지_않는다(tmp_path, g, monkeypatch):
    c = _persisting(tmp_path)
    c.stop_fleet()
    c2 = _restart(tmp_path)
    assert c2.mission_state == "STOPPED"
    # (옛 우회 목표 API /api/robot1/goal 의 409 검사는 지웠다 — 그 경로는 없다, 2026-09-29)
    _reset(c2)
    c2._publish_lane_commands()
    assert _sent(c2.lane_cmd_pubs["pinky1"])[-1].command == LaneCommand.CMD_STOP
    assert _sent(c2.fleet_cmd_pubs["pinky1"])[-1].command == FleetCommand.CMD_STOP


def test_G4_RUNNING_은_남기지_않는다__재시작_뒤_재개는_출발시키지_않는다(tmp_path):
    """달리던 플릿을 멈추고 재시작한 뒤 재개하면 ASSIGNED — 예약도 START 확인도 새로 시작이라 운영자가 다시 시작한다."""
    c = _persisting(tmp_path)
    c.stop_fleet()
    assert _file(tmp_path)["pre_stop_state"] == "RUNNING"          # 기록은 사실대로
    c2 = _restart(tmp_path)
    _reset(c2)
    c2.resume_fleet()
    assert c2.mission_state == "ASSIGNED"
    c2._publish_lane_commands()
    for n in ("pinky1", "pinky2"):
        assert all(m.command != LaneCommand.CMD_START for m in _sent(c2.lane_cmd_pubs[n]))
    c3 = _persisting(tmp_path)
    c3.start_fleet()
    assert _file(tmp_path)["mission_state"] is None               # RUNNING 은 파일에 없다
    c4 = _restart(tmp_path)
    assert c4.mission_state == "ASSIGNED"
    c5 = _persisting(tmp_path)
    c5.estop_fleet()
    c6 = _restart(tmp_path)
    c6.resume_fleet()
    assert c6.mission_state == "ASSIGNED"                          # 비상정지 직전 RUNNING 도 되살리지 않는다


def test_G4_시작은_남긴_정지를_START_보다_먼저_지운다(tmp_path):
    """플릿 정지 → 시작: 파일이 STOPPED 로 남으면 재시작 뒤 달리던 플릿이 STOPPED 로 뜬다(시작한 운영자의 뜻과 다르다).
    START 를 내보내기 전에 지운다 — 내보낸 뒤 쓰기 전에 죽으면 STOPPED 로 뜨는 쪽(멈추는 쪽)이다."""
    c = _persisting(tmp_path, state="ASSIGNED")
    c.stop_fleet()
    assert _file(tmp_path)["mission_state"] == "STOPPED"
    seen = []
    c.lane_cmd_pubs["pinky1"].publish.side_effect = lambda m: seen.append((m.command, _file(tmp_path)["mission_state"]))
    c.start_fleet()
    assert (LaneCommand.CMD_START, None) in seen
    assert _restart(tmp_path).mission_state == "ASSIGNED"


def test_G4_DONE_도_재시작을_넘고__전환하면_지운다(tmp_path):
    c = _persisting(tmp_path)
    for ctx in c.robots.values():
        ctx.arrived = ctx.arrival_confirmed = True
    c._check_mission_done()
    assert c.mission_state == "DONE" and _file(tmp_path)["mission_state"] == "DONE"
    c2 = _restart(tmp_path)
    assert c2.mission_state == "DONE" and c2.profile_switch_blocker() is None   # DONE 에서는 전환된다
    c2.profiles_default, c2.profiles = P.load_profiles()
    c2.active_profile, c2.route_pubs = None, {}
    c2.reservation = Reservation(c2.graph)
    assert c2.switch_profile("team11_map5")
    assert _file(tmp_path)["mission_state"] is None
    assert _restart(tmp_path).mission_state == "ASSIGNED"


def test_G4_복원한_정지·래치_중에는_좌표_전환을_거부한다(tmp_path):
    c = _persisting(tmp_path)
    c.stop_fleet()
    assert "STOPPED" in _restart(tmp_path).profile_switch_blocker()
    c.estop_fleet()
    assert "비상정지" in _restart(tmp_path).profile_switch_blocker()


def test_G3_로봇별_정지는_이유와_함께_재시작을_넘고_start_가_그_로봇에_START_를_안_보낸다(tmp_path, g, monkeypatch):
    c = _persisting(tmp_path, state="ASSIGNED")
    c.stop_robot("pinky1")
    assert _file(tmp_path)["held"] == {"pinky1": FC.HELD_BY_OPERATOR}
    c2 = _restart(tmp_path)
    assert c2.robots["pinky1"].held and c2.robots["pinky1"].held_reason == FC.HELD_BY_OPERATOR
    st = c2.get_fleet_status_dict()["robots"]
    assert st["pinky1"]["held_reason"] == FC.HELD_BY_OPERATOR and st["pinky2"]["held_reason"] is None
    _reset(c2)
    c2.start_fleet()
    assert _sent(c2.lane_cmd_pubs["pinky1"]) == []
    assert [m.command for m in _sent(c2.lane_cmd_pubs["pinky2"])] == [LaneCommand.CMD_START]
    # (옛 우회 목표 API /api/robot1/goal 의 409 검사는 지웠다 — 그 경로는 없다, 2026-09-29)
    c2.resume_robot("pinky1")
    assert _file(tmp_path)["held"] == {}
    assert not _restart(tmp_path).robots["pinky1"].held


@pytest.mark.parametrize("act,check", [
    (lambda c: c.stop_robot("pinky1"), lambda d: d["held"] == {"pinky1": FC.HELD_BY_OPERATOR}),
    (lambda c: c.stop_fleet(), lambda d: d["mission_state"] == "STOPPED"),
], ids=["stop_robot", "stop"])
def test_G3_멈추는_명령은_내보내기_전에_남긴다(tmp_path, act, check):
    """내보낸 뒤 쓰기 전에 죽으면 파일은 '안 세웠다' 로 남아 재시작 뒤 start 가 세워 둔 로봇에 START 를 보낸다."""
    c = _persisting(tmp_path)
    seen = []
    c.lane_cmd_pubs["pinky1"].publish.side_effect = lambda _m: seen.append(check(_file(tmp_path)))
    act(c)
    assert seen and all(seen)


def test_GW_N2_비상정지는_디스크를_기다리지_않고_먼저_내보낸_뒤_남긴다(tmp_path):
    """통합 검토 GW-N2: fsync 가 비상정지 앞에 서면 디스크가 바쁠 때 비상정지가 늦는다. 틈은 S6(로봇 ESTOP 보고)이 메운다."""
    c = _persisting(tmp_path)
    at_publish = []
    c.lane_cmd_pubs["pinky1"].publish.side_effect = lambda _m: at_publish.append(
        os.path.exists(tmp_path / "state" / FC.CONTROL_STATE_FILE) and _file(tmp_path)["estop_latched"])
    c.estop_fleet()
    assert at_publish == [False]                     # 내보낼 때는 아직 안 남겼다
    assert _file(tmp_path)["estop_latched"] is True  # 돌아올 때는 남겼다


def test_GW_N2_비상정지는_저장이_실패해도_내보낸다(tmp_path, monkeypatch):
    c = _persisting(tmp_path)
    monkeypatch.setattr(FC.tempfile, "mkstemp", lambda *a, **k: (_ for _ in ()).throw(OSError("disk full")))
    c.estop_fleet()
    assert [m.command for m in (call[0][0] for call in c.lane_cmd_pubs["pinky1"].publish.call_args_list)] == [LaneCommand.CMD_ESTOP]
    assert c.estop_latched and "저장하지 못했다" in (c.control_notes() or "")


# 제3자 재검 GW-R1: 내용이 멀쩡한(래치 아님) 문서 — 인코딩만 UTF-8 이 아니면 받지 않는다(쓰는 쪽은 UTF-8 만 쓴다)
_VALID_NOT_LATCHED = {"version": 1, "estop_latched": False, "held": {"pinky1": "로봇별 정지"}, "mission_state": None,
                      "pre_estop_state": None, "pre_stop_state": None}


@pytest.mark.parametrize("content", [
    "not json", "[]", '{"version": 1}',
    '{"version": 99, "estop_latched": false, "held": {}, "mission_state": null, "pre_estop_state": null, "pre_stop_state": null}',
    '{"version": 1, "estop_latched": "no", "held": {}, "mission_state": null, "pre_estop_state": null, "pre_stop_state": null}',
    '{"version": 1, "estop_latched": false, "held": [], "mission_state": null, "pre_estop_state": null, "pre_stop_state": null}',
    '{"version": 1, "estop_latched": false, "held": {"pinky1": 5}, "mission_state": null, "pre_estop_state": null, "pre_stop_state": null}',
    '{"version": 1, "estop_latched": false, "held": {}, "mission_state": "RUNNING", "pre_estop_state": null, "pre_stop_state": null}',
    # GW-R1: 예전엔 이 셋이 생성자를 죽였다 — 게이트웨이는 코디네이터 없이 뜨고(플릿 비상정지 503) 파일은 그대로 남았다
    b"\xff\xfe{}",
    json.dumps(_VALID_NOT_LATCHED, ensure_ascii=False).encode("cp949"),   # 한글 이유를 손보고 CP949 로 저장
    json.dumps(_VALID_NOT_LATCHED).encode("utf-16"),
    "[" * 200000,                                                          # json.loads 가 RecursionError
], ids=["text", "list", "keys", "version", "latched-type", "held-type", "held-reason-type", "running",
        "not-utf8", "cp949", "utf16", "deep-nesting"])
def test_깨진_상태_파일이면_플릿_비상정지_래치로_뜨고_알린다(tmp_path, content):
    data = content if isinstance(content, bytes) else content.encode("utf-8")
    path = tmp_path / "state" / FC.CONTROL_STATE_FILE
    path.parent.mkdir(parents=True)
    path.write_bytes(data)
    c = _restart(tmp_path)
    assert c.estop_latched and c.mission_state == "ESTOP"
    assert "깨졌다" in c.control_note and "래치" in c.control_note
    assert c.get_fleet_status_dict()["control_note"] == c.control_note
    c.profiles_default, c.profiles = P.load_profiles()
    assert c.control_note in c.profile_status()["note"]           # 전환 카드의 '알림' 칸에 보인다
    assert (tmp_path / "state" / (FC.CONTROL_STATE_FILE + ".corrupt")).read_bytes() == data   # 증거는 그대로 비켜 둔다
    assert _file(tmp_path)["estop_latched"] is True                 # 다음 재시작도 래치로
    _lane_status(c, "pinky1", LaneStatus.DRIVE_ESTOP, 100.1, "E-STOP")
    assert c.robots["pinky1"].held                                  # 세워 둔 로봇을 모르니 ESTOP 보고 로봇은 HOLD
    c.resume_fleet()
    assert c.control_note is None and c.mission_state == "ASSIGNED"


def test_상태_파일을_풀다_어떤_예외가_나도_생성자를_죽이지_않고_래치로_뜬다(tmp_path, monkeypatch):
    """GW-R1: 예외 목록을 좇지 않는다 — 풀기·검증에서 나는 것은 무엇이든(예: 거대한 파일의 MemoryError) 래치다."""
    import types

    def _boom(_s):
        raise MemoryError()
    path = tmp_path / "state" / FC.CONTROL_STATE_FILE
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(_VALID_NOT_LATCHED), encoding="utf-8")
    monkeypatch.setattr(FC, "json", types.SimpleNamespace(loads=_boom, dump=json.dump))
    c = _restart(tmp_path)
    assert c.estop_latched and c.mission_state == "ESTOP" and "깨졌다" in c.control_note


def test_못_읽는_상태_파일도_래치로_뜬다(tmp_path):
    (tmp_path / "state" / FC.CONTROL_STATE_FILE).mkdir(parents=True)       # 파일 자리에 폴더 — IsADirectoryError
    c = _restart(tmp_path)
    assert c.estop_latched and "못 읽었다" in c.control_note


def test_상태_파일이_없으면_처음_뜬_것이다__되살릴_것_없음(tmp_path):
    c = _restart(tmp_path)
    assert not c.estop_latched and c.mission_state == "ASSIGNED" and c.control_note is None
    c.state_timeout_sec, c.global_seq, c.profiles, c.active_profile = 1.0, 0, {}, None
    c._loop_tick()
    _lane_status(c, "pinky1", LaneStatus.DRIVE_IDLE, 100.1)
    assert not (tmp_path / "state" / FC.CONTROL_STATE_FILE).exists()      # 바뀐 것이 없으면 쓰지 않는다(틱·보고에도)
    c.stop_robot("pinky1")
    assert _file(tmp_path)["held"] == {"pinky1": FC.HELD_BY_OPERATOR}


def test_저장은_원자적이다__쓰다_실패하면_예전_파일이_남고_알린다(tmp_path, monkeypatch):
    c = _persisting(tmp_path)
    c.stop_robot("pinky1")
    before = (tmp_path / "state" / FC.CONTROL_STATE_FILE).read_bytes()

    def _boom(_fd):
        raise OSError(28, "No space left on device")
    with monkeypatch.context() as m:
        m.setattr(FC.os, "fsync", _boom)
        c.estop_fleet()
    assert (tmp_path / "state" / FC.CONTROL_STATE_FILE).read_bytes() == before
    assert os.listdir(str(tmp_path / "state")) == [FC.CONTROL_STATE_FILE]   # 반쯤 쓴 임시 파일이 안 남는다
    assert "저장하지 못했다" in c.get_fleet_status_dict()["control_note"]
    assert c.estop_latched                                          # 메모리의 래치는 그대로 선다
    c.state_timeout_sec, c.global_seq, c.profiles, c.active_profile = 1.0, 0, {}, None
    c._loop_tick()                                                   # 다음 틱이 다시 쓴다
    assert _file(tmp_path)["estop_latched"] is True and c.get_fleet_status_dict()["control_note"] is None


def test_제어_상태를_모르는_시험_객체는_아무것도_쓰지_않는다(tmp_path):
    """__init__ 없이 만든 코디네이터(시험)는 저장 경로가 없다 — 실물 상태 폴더(~/.local/state)를 건드리지 않게."""
    c = _coord()
    c.estop_fleet()
    c.stop_robot("pinky1")
    assert not (tmp_path / "state").exists()


# 통합 검토 F1: R5 의 래치 해제는 RESUME 앞에 허가 0(CLEARANCE 0)을 준다 — RESUME 이 goto 를 만들지 않게.
#   예전 시험은 [RESUME, STOP] 을 고정했다. 순서의 뜻(STOP 이 마지막 — 곧바로 다시 선다)은 그대로다.
R5_LATCH_RELEASE = [LaneCommand.CMD_CLEARANCE, LaneCommand.CMD_RESUME, LaneCommand.CMD_STOP]


def test_G_재기동_뒤_LINK_LOST_보고는_로봇별_정지로__이유를_보이고_남긴다(tmp_path):
    c = _persisting(tmp_path, state="ASSIGNED")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_LINK_LOST, 100.0, "관제 링크 유실")
    ctx = c.robots["pinky1"]
    assert ctx.held and ctx.held_reason == FC.HELD_LINK_LOST == "링크유실 래치 — 로봇 재개 필요"
    assert c.get_fleet_status_dict()["robots"]["pinky1"]["held_reason"] == FC.HELD_LINK_LOST
    c.profiles_default, c.profiles = P.load_profiles()
    assert c.profile_status()["held"] == {"pinky1": FC.HELD_LINK_LOST}
    assert _file(tmp_path)["held"] == {"pinky1": FC.HELD_LINK_LOST}
    _reset(c)
    c.start_fleet()
    assert _sent(c.lane_cmd_pubs["pinky1"]) == []                    # START 는 래치된 로봇에 안 먹는다 — 안 보낸다


def test_G_재개가_안_닿아_유예가_지나도_링크유실이면_다시_세운다():
    """링크가 끊긴 채 누른 재개는 로봇에 안 닿는다 — 그 뒤 LINK_LOST 보고를 영영 '옛 보고' 로 보면 래치가 화면에서 숨는다."""
    c = _coord(state="ASSIGNED")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_LINK_LOST, 100.0, "관제 링크 유실")
    c.resume_robot("pinky1")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_LINK_LOST, 100.1 + c.LINK_LOST_RESUME_GRACE_SEC - 0.2, "관제 링크 유실")
    assert not c.robots["pinky1"].held                             # 유예 안 — 옛 보고일 수 있다
    _lane_status(c, "pinky1", LaneStatus.DRIVE_LINK_LOST, 100.0 + c.LINK_LOST_RESUME_GRACE_SEC + 0.1, "관제 링크 유실")
    assert c.robots["pinky1"].held and c.robots["pinky1"].held_reason == FC.HELD_LINK_LOST


@pytest.mark.parametrize("which", ["HELD_LINK_LOST", "HELD_ESTOP_RESTORED"])
def test_G_재기동_뒤_보고_전에도_파일의_래치_이유로_로봇_재개가_RESUME_을_보낸다(tmp_path, which):
    """재기동 직후 로봇 보고가 오기 전 — 파일이 '링크유실·ESTOP 래치로 세웠다' 를 기억한다. 플릿 STOPPED 라도 로봇 재개가
    RESUME(+곧바로 STOP)을 보내야 그 래치가 풀린다."""
    reason = getattr(FC, which)
    c = _persisting(tmp_path, state="STOPPED")
    c.stop_robot("pinky1")
    c.robots["pinky1"].held_reason = reason
    c._save_control_state()
    c2 = _restart(tmp_path)
    assert c2.mission_state == "STOPPED" and c2.robots["pinky1"].held_reason == reason
    assert c2.robots["pinky1"].lane_status is None
    _reset(c2)
    c2.resume_robot("pinky1")
    assert [m.command for m in _sent(c2.lane_cmd_pubs["pinky1"])] == R5_LATCH_RELEASE


def test_G_운영자가_세운_로봇이_링크유실을_보고해도_로봇_재개가_래치를_푼다():
    c = _coord(state="STOPPED")
    c.stop_robot("pinky1")
    _lane_status(c, "pinky1", LaneStatus.DRIVE_LINK_LOST, 100.0, "관제 링크 유실")
    assert c.get_fleet_status_dict()["robots"]["pinky1"]["held_reason"] == FC.HELD_BY_OPERATOR
    _reset(c)
    c.resume_robot("pinky1")
    assert LaneCommand.CMD_RESUME in [m.command for m in _sent(c.lane_cmd_pubs["pinky1"])]


def test_G_R5_는_그대로__링크유실이_아니면_정지·완료_중_재개는_아무것도_안_보낸다():
    c = _coord(state="STOPPED")
    c.stop_robot("pinky1")
    _reset(c)
    c.resume_robot("pinky1")
    assert _sent(c.lane_cmd_pubs["pinky1"]) == [] and _sent(c.fleet_cmd_pubs["pinky1"]) == []


def test_G_S6_복원_HOLD_도_이유가_있다():
    c = _coord(state="ASSIGNED")
    _lane_status(c, "pinky2", LaneStatus.DRIVE_ESTOP, 100.0, "E-STOP")
    assert c.get_fleet_status_dict()["robots"]["pinky2"]["held_reason"] == FC.HELD_ESTOP_RESTORED


def test_G_전환_카드가_세워_둔_로봇과_이유를_보인다():
    js = io.open(os.path.join(GATEWAY_DIR, "static", "fleet_control_v2.js"), encoding="utf-8").read()
    card = js[js.index("function renderProfile"):js.index("async function profileAction")]
    assert "(Object.keys(p.held||{}).length ? `<div class=\"config-row\"><span>로봇별 정지</span>" in card
    assert "Object.entries(p.held).map(([n,why])" in card and "${esc(why)}" in card


# ==== 실제 생성자 — 경로 배정 뒤에 복원한다 (격리 도메인 하위 프로세스) ================================================

_CTOR = r"""
import json, sys
sys.path.insert(0, sys.argv[1])
import rclpy
rclpy.init()
from relay_station.fleet.fleet_coordinator import RelayFleetCoordinator
c = RelayFleetCoordinator()
print("RESULT " + json.dumps({"latched": c.estop_latched, "mission": c.mission_state, "note": c.control_note,
                              "held": {n: x.held_reason for n, x in c.robots.items() if x.held},
                              "routes": sorted(n for n, x in c.robots.items() if x.route is not None)}, ensure_ascii=False))
c.destroy_node()
rclpy.shutdown()
"""


def _construct(state_dir):
    env = dict(os.environ)
    env.pop("CYCLONEDDS_URI", None)
    env.update(ROS_DOMAIN_ID="97", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", PYTHONDONTWRITEBYTECODE="1")
    env[P.STATE_ENV] = str(state_dir)
    p = subprocess.run([sys.executable, "-B", "-c", _CTOR, REPO], capture_output=True, text=True, env=env, timeout=120)
    lines = [l for l in p.stdout.splitlines() if l.startswith("RESULT ")]
    assert lines, (p.returncode, p.stdout[-800:], p.stderr[-1500:])
    return json.loads(lines[-1][len("RESULT "):])


def test_실제_생성자는_경로를_배정한_뒤_제어_상태를_되살린다(tmp_path):
    sd = tmp_path / "ctor"
    sd.mkdir()
    (sd / FC.CONTROL_STATE_FILE).write_text(json.dumps({
        "version": 1, "estop_latched": False, "pre_estop_state": None, "mission_state": "STOPPED",
        "pre_stop_state": "RUNNING", "held": {"pinky1": FC.HELD_LINK_LOST}}), encoding="utf-8")
    r = _construct(sd)
    assert r["mission"] == "STOPPED" and r["latched"] is False and r["note"] is None
    assert r["held"] == {"pinky1": FC.HELD_LINK_LOST} and r["routes"]      # 배정이 STOPPED 를 ASSIGNED 로 덮지 않았다
    (sd / FC.CONTROL_STATE_FILE).write_text("{broken", encoding="utf-8")
    r = _construct(sd)
    assert r["latched"] is True and r["mission"] == "ESTOP" and "깨졌다" in r["note"]


def test_실제_생성자는_UTF_8_이_아닌_상태_파일에도_죽지_않고_래치로_뜬다(tmp_path):
    """제3자 재검 GW-R1: 예전엔 UnicodeDecodeError 가 __init__ 밖으로 나갔다 — 게이트웨이 main 이 그걸 삼켜
    코디네이터 없이 떴고(플릿 비상정지·정지 503, LaneCommand 도 안 나감), 파일이 .corrupt 로 안 밀려나 재시작마다 같았다."""
    sd = tmp_path / "ctor"
    sd.mkdir()
    (sd / FC.CONTROL_STATE_FILE).write_bytes(b"\xff\xfe{}")
    r = _construct(sd)
    assert r["latched"] is True and r["mission"] == "ESTOP" and "깨졌다" in r["note"] and r["routes"]
    assert (sd / (FC.CONTROL_STATE_FILE + ".corrupt")).read_bytes() == b"\xff\xfe{}"
    assert json.loads((sd / FC.CONTROL_STATE_FILE).read_text(encoding="utf-8"))["estop_latched"] is True


# ==== 중계 항목 · 목표 구역 별칭은 프로파일이 없을 때만 =====================================================================

def _zone(c, zone_id, t):
    c._t = t
    c._cb_vision_zone_event(String(data=json.dumps({"robot_name": "pinky1", "camera_id": "GOAL", "zone_id": zone_id,
                                                    "event_type": "ENTER", "timestamp": t})))


def _profiled(name):
    c = _coord(state="ASSIGNED")
    c.profiles_default, c.profiles = P.load_profiles()
    c.active_profile, c.route_pubs = None, {}
    c.reservation = Reservation(c.graph)
    for ctx in c.robots.values():
        ctx.route = None
    c.goal_event_timeout = 3.0
    assert c.switch_profile(name)
    c.robots["pinky1"].arrived = True
    return c


def test_map5_에서는_옛_별칭_구역_이벤트가_TR_도착을_확정하지_않고_TR_이벤트가_확정한다():
    c = _profiled("team11_map5")
    assert c.robots["pinky1"].goal_node == "TR"
    for alias in ("GOAL_C", "goal", "goal_zone"):
        _zone(c, alias, 100.0)
        assert c.robots["pinky1"].arrival_confirmed is False, alias
    _zone(c, "TR", 100.5)
    assert c.robots["pinky1"].arrival_confirmed is True


def test_프로파일이_없으면_예전처럼_별칭을_받는다():
    c = _coord(state="RUNNING")
    c.goal_event_timeout = 3.0
    ctx = c.robots["pinky1"]
    ctx.goal_node = "TC"
    ctx.last_zone_event = {"zone_id": "goal_c", "event_type": "ENTER", "received_at": 100.0}
    assert c._is_valid_goal_zone_event(ctx) is True
