# -*- coding: utf-8 -*-
"""통합 검토(2026-09-26 병합 뒤) — 게이트웨이·V2 화면·런처의 정직함.

OPS-1  18081 원격 보기 경로는 보기 전용 — 제어(움직이는 명령)는 현장 노트북 :8889 에서. 멈추는 명령은 어디서든 받는다
OPS-2  코디네이터가 없거나(FLEET_COORDINATOR_UNAVAILABLE) 게이트웨이에 닿지 않는데 V2 가 초록 'RELAY CONNECTED' 에 얼어 있었다
OPS-3  링크유실·ESTOP 래치로 세워 둔 로봇이 있는데 ② 로봇 지도·③ 초기 위치가 200 이었다(에이전트는 SET_MAP 을 거부한다) → 409
OPS-4  V2 로봇 카드 버튼을 갱신마다 다시 만들어 누르는 사이 갱신이 끼면 클릭이 사라졌다 · 로봇 정지가 확인을 물었다
OPS-5  로봇이 ESTOP·링크유실 래치나 해제 보류를 보고하는 중에도 목표가 200 이었다 → 409 ROBOT_LATCHED · 200 문구는 '발행했다'
OPS-6  로봇 재개의 '(플릿 재개 때 출발)' 이 DONE·달리지 않던 STOPPED 에서 거짓이었다 → 플릿 재개가 실제로 갈 상태로 말한다
OPS-7  control_note(상태 파일 복원 실패 등)가 설정 탭에만 있었다 → 대시보드 알림 띠
OPS-8  V2 플릿 버튼이 202(적용 여부 모름)에 조용했다
OPS-10 미션 API 가 빈·깨진·mission 없는 본문을 미션 '1'(움직이는 경로)로 냈다 → 400 BAD_MISSION
N1     해제 보류(래치는 풀렸지만 Nav2 취소 미확인) 로봇 카드가 초록이었다
E2E-3  SIGTERM 뒤에도 게이트웨이가 얼어붙은 상태를 계속 내보냈다(rclpy 기본 신호 처리는 ROS 만 내린다) → HTTP 를 닫고 끝난다
UI-R1  (OPS-3 후속) 래치를 마지막으로 보고하고 조용해진 로봇의 옛 보고가 ②·③ 을 플릿 전체에서 막았다 → 최근 보고만 센다

⚠️ 상태 파일은 늘 tmp_path 로(PINKY_RELAY_STATE_DIR). 실물 게이트웨이(:8889)·보기 중계(:18081)에는 닿지 않는다 —
   socat·게이트웨이·Chrome 시험은 빈 포트만 쓰고, 실제 게이트웨이 프로세스·Chrome 은 격리 네트워크(unshare -rn)에서만 돈다.
"""
import io
import json
import os
import re
import select
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import types
import http.client
from unittest.mock import MagicMock

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pinky_fleet_msgs.msg import FleetCommand  # noqa: E402
from pinky_lane_msgs.msg import LaneStatus  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from relay_station.fleet import fleet_coordinator as FC  # noqa: E402
from relay_station.fleet import profiles as P  # noqa: E402
from test_d7_robot_stop import _coord, _post  # noqa: E402

GATEWAY_DIR = os.path.join(REPO, "relay_station", "gateway_web")
STATIC = os.path.join(GATEWAY_DIR, "static")
LAUNCHER = os.path.join(REPO, "relay_station", "launch_master_gateway.sh")
RELEASE_HOLD = "해제 보류 — Nav2 취소 미확인 · Nav2 상태 모름"      # 에이전트가 Nav2 상태를 모를 때 싣는 그대로


@pytest.fixture(autouse=True)
def _state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv(P.STATE_ENV, str(tmp_path / "state"))


@pytest.fixture
def g(monkeypatch):
    import gateway_web_server as gws
    node = MagicMock()
    node.send_mission.return_value = 1
    monkeypatch.setattr(gws, "GLOBAL_ROBOT_SUB_NODE", node)
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", None)
    monkeypatch.setattr(gws, "ALLOW_DIRECT_FALLBACK", False)
    monkeypatch.setattr(gws, "CONTROL_APPLY_WAIT_SEC", 0.2)
    return gws, node


def _wired(gws, node, monkeypatch, c):
    """게이트웨이가 내는 제어를 코디네이터가 바로 처리하게 잇는다(D8 토픽 대신)."""
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", c)
    node.send_fleet_control.side_effect = lambda payload: c._cb_control(String(data=json.dumps(payload)))
    return c


def _ls(state, reason=""):
    m = LaneStatus()
    m.drive_state = state
    m.state_reason = reason
    return m


def _isolated_net():
    """격리 네트워크 이름공간(unshare -rn)인가 — 루프백 말고 인터페이스가 없다."""
    try:
        return {n for _, n in socket.if_nameindex()} <= {"lo"}
    except OSError:
        return False


def _http(port, method, path, body=b"{}"):
    assert port not in (8889, 18081)                                # 실물 게이트웨이·보기 중계에는 절대 닿지 않는다
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        conn.request(method, path, body=body if method == "POST" else None,
                     headers={"Content-Type": "application/json"})
        r = conn.getresponse()
        return r.status, r.read()
    finally:
        conn.close()


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# ==== OPS-1 · 18081 원격 보기 경로는 보기 전용 — 제어는 현장 노트북 :8889 =================================================

def _launcher_18081_argv():
    lines = [ln for ln in io.open(LAUNCHER, encoding="utf-8").read().splitlines()
             if ln.strip().startswith("socat") and "TCP4-LISTEN:18081" in ln]
    assert len(lines) == 1, lines
    return shlex.split(lines[0].strip().rstrip("&").strip())


def test_OPS1_18081_보기_경로는_로컬_제어_목록에_없는_루프백_주소에서_게이트웨이에_붙는다(g):
    gws, _ = g
    argv = _launcher_18081_argv()
    target = [a for a in argv if a.startswith("TCP4:127.0.0.1:8889")]
    assert len(target) == 1, argv                                   # 여전히 이 노트북의 게이트웨이(:8889)로 간다
    m = re.search(r",bind=([0-9.]+)(?:,|$)", target[0])
    assert m, "18081 중계가 발신 주소를 정하지 않는다 — 게이트웨이가 로컬(127.0.0.1)로 본다: %s" % target[0]
    src = m.group(1)
    assert src.startswith("127.") and src not in gws.LOCAL_CONTROL_IPS


def test_OPS1_18081_경로로_온_요청은_보기는_되고_움직이는_명령은_403__멈추는_명령은_받는다(g):
    """런처의 socat 줄을 그대로(포트만 빈 포트로) 돌려 실제 게이트웨이 요청 처리기 앞에 세운다."""
    if not shutil.which("socat"):
        pytest.skip("socat 없음")
    gws, _ = g
    srv = gws.ThreadedHTTPServer(("127.0.0.1", 0), gws.GatewayRequestHandler)
    gw_port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    view_port = _free_port()
    argv = [a.replace("TCP4-LISTEN:18081", "TCP4-LISTEN:%d" % view_port)
             .replace("TCP4:127.0.0.1:8889", "TCP4:127.0.0.1:%d" % gw_port) for a in _launcher_18081_argv()]
    assert not any("18081" in a or "8889" in a for a in argv), argv  # 실물 포트가 남으면 돌리지 않는다
    proc = subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        end = time.time() + 5
        while time.time() < end:
            try:
                socket.create_connection(("127.0.0.1", view_port), timeout=0.5).close()
                break
            except OSError:
                time.sleep(0.05)
        assert _http(view_port, "GET", "/api/fleet/status")[0] == 200            # 보기는 된다
        for path in ("/api/robot1/resume", "/api/robot1/goal", "/api/fleet/start", "/api/fleet/profile"):
            assert _http(view_port, "POST", path)[0] == 403, path              # 움직이는 명령은 원격으로 본다
        for path in ("/api/fleet/estop", "/api/robot1/stop"):
            assert _http(view_port, "POST", path)[0] != 403, path              # 멈추는 명령은 어디서든 받는다(D7)
        assert _http(gw_port, "POST", "/api/robot1/resume")[0] != 403          # 이 노트북에서 :8889 로는 된다
    finally:
        proc.terminate()
        proc.wait(5)
        srv.shutdown()
        srv.server_close()


# ==== OPS-5 · 래치·해제 보류를 보고하는 로봇에 우회 목표·미션을 받지 않는다 ================================================





def test_OPS5_래치_판정은_에이전트가_싣는_상태_번호·해제_보류_문구와_같다():
    import gateway_web_server as gws
    from pinky_fleet_agent import hybrid_agent_node as agent_node
    assert set(gws.LATCHED_DRIVE_STATES) == {LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_LINK_LOST}
    assert agent_node.RELEASE_HOLD_REASON.startswith(gws.RELEASE_HOLD_PREFIX)


# ==== OPS-3 · 래치로 세워 둔 로봇이 있으면 ② 로봇 지도·③ 초기 위치를 보내지 않는다 ======================================

@pytest.mark.parametrize("cmd", ["robot_maps", "initial_poses"])
@pytest.mark.parametrize("why", [FC.HELD_LINK_LOST, FC.HELD_ESTOP_RESTORED], ids=["link-lost", "estop-restored"])
def test_OPS3_래치로_세워_둔_로봇이_있으면_지도·초기_위치는_409_NOT_NOW_이고_로봇을_말한다(g, monkeypatch, cmd, why):
    gws, node = g
    c = _coord(state="ASSIGNED")
    c.robots["pinky2"].held, c.robots["pinky2"].held_reason = True, why
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", c)
    assert c.profile_switch_blocker() is None                    # 코디네이터 쪽 차단은 이것을 보지 않는다
    code, body = _post(gws, "/api/fleet/" + cmd, ip="127.0.0.1")
    assert code == 409 and body["reason"] == "NOT_NOW" and body["latched"] == ["pinky2"]
    assert "pinky2" in body["message"] and "로봇 재개 먼저" in body["message"]
    node.send_fleet_control.assert_not_called()


@pytest.mark.parametrize("state", [LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_LINK_LOST], ids=["estop", "link-lost"])
def test_OPS3_스스로_래치를_보고하는_로봇이_있어도_지도_교체를_보내지_않는다(g, monkeypatch, state):
    gws, node = g
    c = _coord(state="ASSIGNED")
    c.robots["pinky1"].lane_status = _ls(state)
    c.robots["pinky1"].lane_status_time = c._t - 0.5              # 방금 한 보고(UI-R1: 옛 보고는 세지 않는다)
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", c)
    code, body = _post(gws, "/api/fleet/robot_maps", ip="127.0.0.1")
    assert code == 409 and body["latched"] == ["pinky1"]
    node.send_fleet_control.assert_not_called()


def test_OPS3_운영자_로봇별_정지는_래치가_아니다__지도_교체를_보낸다(g, monkeypatch):
    """에이전트는 체인 래치(ESTOP·링크유실) 중에만 SET_MAP 을 거부한다 — 운영자 HOLD 로는 막지 않는다."""
    gws, node = g
    c = _coord(state="ASSIGNED")
    c.stop_robot("pinky1")
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", c)
    _post(gws, "/api/fleet/robot_maps", ip="127.0.0.1")
    assert [call[0][0] for call in node.send_fleet_control.call_args_list] == [{"cmd": "robot_maps"}]


# ==== UI-R1 · 래치를 마지막으로 보고하고 조용해진 로봇이 ②·③ 을 플릿 전체에서 막지 않는다 ===================================

_FLEET_CMD = {"robot_maps": FleetCommand.CMD_SET_MAP, "initial_poses": FleetCommand.CMD_SET_INITIAL_POSE}


def _fleet_cmds(c, name):
    return [call[0][0].command for call in c.fleet_cmd_pubs[name].publish.call_args_list]


@pytest.mark.parametrize("cmd", ["robot_maps", "initial_poses"])
@pytest.mark.parametrize("kind", ["estop", "link-lost"])
def test_UIR1_래치를_보고하고_꺼진_로봇이_있어도_로봇_재개_뒤에는_성한_로봇에_지도·초기_위치를_보낸다(g, monkeypatch, kind, cmd):
    """월요일 흐름: 플릿 비상정지(또는 링크유실) → pinky2 전원 끔(더 보고하지 않는다) → 재개 → ② 로봇 지도 · ③ 초기 위치.

    예전엔 pinky2 의 마지막 ESTOP·링크유실 보고를 언제까지나 세어 409 '로봇 재개 먼저' 였고, 로봇 재개는 옛 보고를 못 바꿔
    풀 길이 없었다 — pinky1 은 지도를 못 받았다. 코디네이터 규칙(estop_fleet·_cb_lane_status·resume_*)을 그대로 쓴다.
    """
    gws, node = g
    c = _wired(gws, node, monkeypatch, _coord(state="ASSIGNED"))
    c.profiles = {"map4": types.SimpleNamespace(robot_map_name="map4")}
    c.active_profile = "map4"
    if kind == "estop":
        c.estop_fleet()
        c._t += 1
        c._cb_lane_status("pinky1", _ls(LaneStatus.DRIVE_ESTOP, "E-STOP"))
        c._cb_lane_status("pinky2", _ls(LaneStatus.DRIVE_ESTOP, "E-STOP"))
        c._t += 1
        c.resume_fleet()
        c._t += 1
        c._cb_lane_status("pinky1", _ls(LaneStatus.DRIVE_IDLE, "RESUME"))      # pinky2 는 꺼졌다 — 보고 없음
    else:
        c._t += 1
        c._cb_lane_status("pinky1", _ls(LaneStatus.DRIVE_IDLE, "경로 수신 — START 대기"))
        c._cb_lane_status("pinky2", _ls(LaneStatus.DRIVE_LINK_LOST, "관제 링크 유실"))  # 코디네이터가 세운다 — 그리고 꺼진다
        assert c.robots["pinky2"].held_reason == FC.HELD_LINK_LOST
    code, body = _post(gws, "/api/fleet/" + cmd, ip="127.0.0.1")        # 보고가 방금이거나 세워 둔 채 — 아직 막는다
    assert code == 409 and body["latched"] == ["pinky2"] and "로봇 재개 먼저" in body["message"]
    assert _post(gws, "/api/robot2/resume", ip="127.0.0.1")[0] == 200     # 409 가 말한 처방
    c._t += 10.0                                                         # pinky2 는 10 s 째 조용하다
    for p in c.fleet_cmd_pubs.values():
        p.reset_mock()
    code, body = _post(gws, "/api/fleet/" + cmd, ip="127.0.0.1")
    assert code == 200 and body["success"] is True and body["applied"] is True, body
    assert _FLEET_CMD[cmd] in _fleet_cmds(c, "pinky1")                   # 성한 로봇이 받는다


@pytest.mark.parametrize("state", [LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_LINK_LOST], ids=["estop", "link-lost"])
def test_UIR1_로봇_스스로의_래치_보고는_최근_것만_센다(g, monkeypatch, state):
    gws, node = g
    c = _coord(state="ASSIGNED")
    ctx = c.robots["pinky1"]
    ctx.lane_status = _ls(state)
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", c)
    ctx.lane_status_time = c._t - 0.5                                    # 방금 한 보고 — 센다
    code, body = _post(gws, "/api/fleet/robot_maps", ip="127.0.0.1")
    assert code == 409 and body["latched"] == ["pinky1"]
    node.send_fleet_control.assert_not_called()
    ctx.lane_status_time = c._t - 10.0                                   # 그 뒤로 10 s 째 조용하다 — 세지 않는다
    code, body = _post(gws, "/api/fleet/robot_maps", ip="127.0.0.1")
    assert code != 409
    assert [call[0][0] for call in node.send_fleet_control.call_args_list] == [{"cmd": "robot_maps"}]




# ==== OPS-6 · 로봇 재개는 플릿 재개가 실제로 무엇을 할지로 말한다 ========================================================

@pytest.mark.parametrize("state,pre,routes,goes", [
    ("DONE", None, True, "DONE"),
    ("STOPPED", None, True, "ASSIGNED"),                    # 재시작 뒤 복원한 STOPPED — 정지 직전 상태를 모른다
    ("STOPPED", None, False, "IDLE"),
    ("STOPPED", "ASSIGNED", True, "ASSIGNED"),
    ("STOPPED", "IDLE", True, "IDLE"),
    ("STOPPED", "DONE", True, "DONE"),
    ("STOPPED", "RUNNING", True, "RUNNING"),
])
def test_OPS6_로봇_재개_문구는_플릿_재개가_실제로_갈_상태와_같다(g, monkeypatch, state, pre, routes, goes):
    gws, node = g
    c = _wired(gws, node, monkeypatch, _coord(state=state))
    c._pre_stop_state = pre
    if not routes:
        for ctx in c.robots.values():
            ctx.route = None
    c.stop_robot("pinky1")
    code, body = _post(gws, "/api/robot1/resume", ip="127.0.0.1")
    assert code == 200 and body["mission_state"] == state
    assert body["fleet_resume_goes_to"] == goes
    assert "(플릿 재개 때 출발)" not in body["message"]
    assert ("출발한다" in body["message"]) == (goes == "RUNNING")
    c.resume_fleet()                                                # 말한 대로 되는가 — 코디네이터 규칙과 대사
    assert c.mission_state == goes


# ==== OPS-10 · 미션 API 는 본문이 말한, 내비게이터가 아는 미션만 낸다 ===================================================







# ==== OPS-2·4·7·8·N1 · V2 화면 (정적 검사) ===============================================================================

def _js():
    return io.open(os.path.join(STATIC, "fleet_control_v2.js"), encoding="utf-8").read()


def _fn(js, head):
    body = js[js.index(head):]
    return body[:body.index("\n  }\n")]








# ==== OPS-2·4·7·8·N1 · V2 화면 (실제 Chrome 에서 — 격리 네트워크에서만) ===================================================

_STUB = r"""<script>
window.__alerts = []; window.__posts = []; window.__confirms = []; window.__mode = {};
window.alert = (m) => window.__alerts.push(String(m));
window.confirm = (m) => { window.__confirms.push(String(m)); return true; };
const J = (status, body) => ({status, body});
window.__fleet = () => ({mission_state: window.__mode.mission || "ASSIGNED", estop_latched:false, control_note: window.__mode.note || null,
  profile:{label:"시험 프로파일", frame:"team11_map4"},                                   // 관제 U-2·U-6: 부제·요약 문장의 label
  robots:{pinky1:{held:false, is_stale: !!window.__mode.stale1, state:{x:0.1,y:0.2,yaw:0,localized:true},
                  lane_status: window.__mode.ls1 || {drive_state:0, state_reason:"경로 수신 — START 대기"}},
          pinky2:{held: !!window.__mode.held2, held_reason: window.__mode.held2 ? "운영자" : null, is_stale:false, state:{x:0.3,y:0.2,yaw:0,localized:true}, lane_status:{drive_state:0, state_reason:"IDLE"}}}});
window.fetch = async (url, opts={}) => {
  const m = window.__mode, method = (opts.method || "GET").toUpperCase();
  let r;
  if (method === "POST") { window.__posts.push(url); r = (m.post || (() => J(200, {success:true, applied:true, message:"ok"})))(url); }
  else if (m.hang) return new Promise(() => {});
  else if (m.netdown) r = null;
  else if (url.startsWith("/api/fleet/status")) r = m.nocoord ? J(200, {status:"FLEET_COORDINATOR_UNAVAILABLE", robots:{}, detail:"import 실패"}) : J(200, window.__fleet());
  else if (url.startsWith("/api/status")) r = J(200, m.gateway || {});      // 관제 U-1: view_only 플래그
  else r = J(200, {});
  if (!r) throw new TypeError("Failed to fetch");
  return new Response(JSON.stringify(r.body), {status:r.status, headers:{"Content-Type":"application/json"}});
};
</script>"""


class _Chrome:
    """headless Chrome 을 --remote-debugging-pipe 로 부린다(네트워크 포트 없음). 한 호출은 10 s 안에 끝나야 한다."""

    def __init__(self, exe, profile, url):
        r1, w1 = os.pipe()
        r2, w2 = os.pipe()

        def _fds():
            os.dup2(r1, 3)
            os.dup2(w2, 4)
        self.p = subprocess.Popen([exe, "--headless=new", "--no-sandbox", "--disable-gpu", "--no-first-run",
                                   "--no-default-browser-check", "--disable-background-networking",
                                   "--disable-component-update", "--disable-sync", "--remote-debugging-pipe",
                                   "--user-data-dir=" + profile, "--allow-file-access-from-files",
                                   "--window-size=1400,1000", "about:blank"],
                                  preexec_fn=_fds, pass_fds=(3, 4), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        os.close(r1)
        os.close(w2)
        self.w, self.r, self.buf, self.mid = w1, r2, b"", 0
        tid = [t for t in self.send("Target.getTargets")["targetInfos"] if t["type"] == "page"][0]["targetId"]
        self.sid = self.send("Target.attachToTarget", {"targetId": tid, "flatten": True})["sessionId"]
        self.send("Page.enable", session=True)
        self.send("Page.navigate", {"url": url}, session=True)
        end = time.time() + 15
        while time.time() < end and not self.ev('document.querySelector("#dashboard-robots [data-robot-cmd]") !== null'):
            time.sleep(0.1)

    def send(self, method, params=None, session=False):
        self.mid += 1
        msg = {"id": self.mid, "method": method, "params": params or {}}
        if session:
            msg["sessionId"] = self.sid
        os.write(self.w, json.dumps(msg).encode() + b"\0")
        end = time.time() + 10
        while True:
            while b"\0" not in self.buf:
                left = end - time.time()
                if left <= 0 or not select.select([self.r], [], [], left)[0]:
                    raise TimeoutError(method)
                chunk = os.read(self.r, 65536)
                if not chunk:
                    raise EOFError("Chrome 이 끝났다")
                self.buf += chunk
            raw, self.buf = self.buf.split(b"\0", 1)
            m = json.loads(raw)
            if m.get("id") == self.mid:
                if "error" in m:
                    raise RuntimeError(m["error"])
                return m.get("result")

    def ev(self, expr):
        r = self.send("Runtime.evaluate", {"expression": expr, "returnByValue": True, "awaitPromise": True}, session=True)
        assert "exceptionDetails" not in r, r
        return r["result"].get("value")

    def mode(self, js_mode):
        """가짜 게이트웨이 상태를 바꾸고 한 번 갱신한다(화면의 새로고침 버튼 = refresh)."""
        self.ev("window.__mode = %s; window.__alerts.length = 0; window.__posts.length = 0; "
                "window.__confirms.length = 0; document.getElementById('refresh-events').click(); 0" % js_mode)
        time.sleep(0.4)

    def mouse(self, kind, x, y):
        self.send("Input.dispatchMouseEvent", {"type": kind, "x": x, "y": y, "button": "left", "clickCount": 1}, session=True)

    def close(self):
        self.p.kill()
        self.p.wait(5)
        os.close(self.w)
        os.close(self.r)




_PILL = '[document.getElementById("backend-status").textContent, document.getElementById("backend-status").className]'
_CARDS = '[...document.querySelectorAll("#dashboard-robots .robot-head .status-pill")].map(e => e.className)'


















# ==== E2E-3 · 실제 게이트웨이는 SIGTERM·SIGINT 에 HTTP 를 닫고 곧 끝난다 ================================================

@pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGINT], ids=["SIGTERM", "SIGINT"])
def test_E2E3_실제_게이트웨이는_신호를_받으면_몇_초_안에_HTTP_를_닫고_128_더하기_신호로_끝난다(tmp_path, sig):
    if not _isolated_net():
        pytest.skip("격리 네트워크(unshare -rn)에서만 — 실제 게이트웨이는 기동 때 로봇 ping·카메라 pull 을 한다")
    port = _free_port()
    env = dict(os.environ, HOME=str(tmp_path), PINKY_RELAY_STATE_DIR=str(tmp_path / "state"),
               MCV_CALIBRATION_DIR=str(tmp_path / "calib"), MCV_ARENA_CONFIG=os.path.join(REPO, "relay_station", "configs", "arena.json"),
               ROS_LOG_DIR=str(tmp_path / "roslog"), ROS_DOMAIN_ID="97", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               PYTHONUNBUFFERED="1")
    env.pop("CYCLONEDDS_URI", None)
    log = open(str(tmp_path / "gateway.out"), "w")
    proc = subprocess.Popen([sys.executable, "-B", os.path.join(GATEWAY_DIR, "gateway_web_server.py"), "--port", str(port),
                             "--no-camera", "--map-yaml", os.path.join(STATIC, "my_map.yaml")],
                            cwd=str(tmp_path), env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        end, up = time.time() + 60, False
        while time.time() < end and proc.poll() is None and not up:
            try:
                up = _http(port, "GET", "/api/fleet/status")[0] == 200
            except OSError:
                time.sleep(0.3)
        assert up, "게이트웨이가 뜨지 않았다: " + open(str(tmp_path / "gateway.out")).read()[-2000:]
        t0 = time.time()
        proc.send_signal(sig)
        try:
            proc.wait(6.0)
        except subprocess.TimeoutExpired:
            pytest.fail("신호 뒤 6 s 가 지나도 게이트웨이가 살아 있다(얼어붙은 상태를 계속 내보낸다)")
        out = open(str(tmp_path / "gateway.out")).read()
        assert proc.returncode == 128 + sig, out[-2000:]
        assert time.time() - t0 < 6.0
        assert "[Shutdown]" in out and "ROS spin 스레드가" not in out, out[-2000:]   # 정리 길로 끝났다(강제 종료 아님)
        with pytest.raises(OSError):
            socket.create_connection(("127.0.0.1", port), timeout=1).close()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(5)
        log.close()


# ==== 관제 U-1·U-2·U-5 (REQ_20260927_RELAY_WEBUI_CEO_REVIEW P1) · 실제 Chrome ===========================================

_VIS = '(s => [...document.querySelectorAll(s)].map(e => e.offsetParent !== null))'
_DISP = '(s => [...document.querySelectorAll(s)].map(e => getComputedStyle(e).display))'












