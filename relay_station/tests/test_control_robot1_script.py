# -*- coding: utf-8 -*-
"""`relay_station/scripts/control_robot1.sh` — 게이트웨이의 거절을 우회하지 않는다
(관제 검수 REVIEW_20260925 S1·§3.4 · 제3자 검수 REVIEW_20260926 G-7·G-8·G-12).

⚠️ 라이브 게이트웨이(:8889)에 절대 보내지 않는다: 가짜 HTTP 서버를 임의 포트에 띄우고 `GATEWAY_URL` 로 가리킨다.
   토픽 직접 발행은 가짜 `python3` 가 가로채 표시만 남긴다 — 이 시험은 DDS 에 아무것도 내지 않는다.
   소켓을 여니 `unshare -rn` 안에서 돌린다.

- 409(ESTOP·정지·HOLD 중)는 **거절**이다. 예전 스크립트는 curl 종료코드만 봐서 409 에도 "전송 완료" 를 찍었고,
  거절을 실패로 다루면 토픽 직접 발행으로 넘어가 거절을 우회한다.
- G-7: 게이트웨이에 닿지 못할 때(curl 6·7)도 목표·미션을 직접 발행하지 않는다 — 그 순간은 플릿 래치가 전부 없다.
  직접 발행은 멈추는 쪽(비상정지의 0 속도)만.
- G-8: 비상정지는 0 속도 직접 발행과 정지 API 를 **동시에** — 서로를 기다리지 않고, 둘 다 상한이 있다.
  (§3.4: 0 속도가 정지 API(로봇 보고를 최대 1 s 기다림)를 기다리면 안 된다.)
- G-12: 종료코드와 문구가 결과를 말한다 — 실패·거절에 "완료" 도, 0 도 없다.
"""
import http.server
import json
import os
import shutil
import socket
import stat
import subprocess
import threading
import time

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(REPO, "relay_station", "scripts", "control_robot1.sh")


class _Fake(http.server.BaseHTTPRequestHandler):
    replies = {}
    log = []
    done = []                                                # (답한 시각, 경로)
    bodies = []                                              # (경로, 받은 JSON — 게이트웨이처럼 깨졌으면 {})

    stall = 0.0

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n)
        _Fake.log.append((time.time(), self.path))
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            body = {}                                        # 진짜 게이트웨이도 깨진 본문을 {} 로 본다(→ x·y·yaw 기본 0)
        _Fake.bodies.append((self.path, body))
        if _Fake.stall:
            time.sleep(_Fake.stall)                          # 살아 있지만 느린 게이트웨이
        _Fake.done.append((time.time(), self.path))
        code, body = _Fake.replies.get(self.path, (404, {}))
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *_a):
        pass


@pytest.fixture
def gw():
    _Fake.replies, _Fake.log, _Fake.done, _Fake.bodies, _Fake.stall = {}, [], [], [], 0.0
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _Fake)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield srv, "http://127.0.0.1:%d" % srv.server_address[1]
    srv.shutdown()


def _closed_port_url():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()                                           # 아무도 안 듣는 포트
    return "http://127.0.0.1:%d" % port


def _run(tmp_path, url, *args, py="ok", stdin=None):
    shim = tmp_path / "shim"
    shim.mkdir(exist_ok=True)
    marker = tmp_path / "direct_publish.log"
    pyshim = shim / "python3"
    # 토픽 직접 발행(create_publisher 가 든 python3 -c)만 가로채 부른 시각을 적는다. 나머지(yaw 도→라디안 계산 등)는 진짜로.
    # FAKE_PY: ok = 발행 성공 · slow = 2.5 s 걸려 성공(검수 실측 0.66~2.22 s 의 최악보다 길게, G-8 · 검수 R-script-2)
    #          fail = rclpy 실패(현장 NIC 없는 곳, G-12)
    #          hang = 멈춤 · hang_ignore_term = TERM 도 무시(G-8)
    pyshim.write_text('#!/bin/sh\ncase "$*" in *create_publisher*) echo "$(date +%%s.%%N)" >> "%s"\n'
                      '  case "$FAKE_PY" in\n'
                      '    slow) sleep 2.5; exit 0;;\n'
                      '    fail) echo "RCLError: failed to create domain (가짜)" >&2; exit 1;;\n'
                      '    hang) exec sleep 30;;\n'
                      '    hang_ignore_term) trap "" TERM; exec sleep 30;;\n'
                      '  esac\n'
                      '  exit 0;; esac\n'
                      'exec /usr/bin/python3 "$@"\n' % marker, encoding="utf-8")
    pyshim.chmod(pyshim.stat().st_mode | stat.S_IEXEC)
    env = dict(os.environ, PATH="%s:/usr/bin:/bin" % shim, GATEWAY_URL=url, FAKE_PY=py, TERM="dumb")
    r = subprocess.run([shutil.which("bash"), SCRIPT] + list(args), env=env, input=stdin,
                       capture_output=True, text=True, timeout=60)
    direct = [float(x) for x in marker.read_text().split()] if marker.exists() else []
    return r.returncode, r.stdout + r.stderr, direct


def test_S1_목표가_409_로_거절되면_거절이라_말하고_직접_발행으로_우회하지_않는다(gw, tmp_path):
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (409, {"success": False, "reason": "FLEET_ESTOP", "message": "거부 — 플릿 비상정지 중"})
    rc, out, direct = _run(tmp_path, url, "goal", "0.5", "0.3")
    assert "거절 (HTTP 409)" in out and "FLEET_ESTOP" in out
    assert "전송 완료" not in out
    assert direct == [], "409 를 실패로 보고 /robot1/goal_pose 직접 발행으로 넘어갔다 — 거절 우회"


def test_S1_목표가_200_이면_완료(gw, tmp_path):
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, "goal", "0.5", "0.3")
    assert "전송 완료" in out and direct == []


@pytest.mark.parametrize("args", [("goal", "0.5", "0.3"), ("mission1",), ("mission2",),
                                  ("point1",), ("point2",), ("point3",)])
def test_G7_게이트웨이에_닿지_못하면_움직이는_명령을_직접_발행하지_않고_거절한다(tmp_path, args):
    """제3자 검수 G-7: 예전엔 curl 6·7 이면 /robot1/goal_pose·mission_cmd 로 직접 발행했다.
    그 순간은 게이트웨이(=플릿 코디네이터)가 죽은 때라 비상정지·정지·HOLD 래치가 전부 없다."""
    rc, out, direct = _run(tmp_path, _closed_port_url(), *args)
    assert direct == [], "플릿 래치가 없는 순간에 움직이는 명령을 토픽으로 직접 발행했다: " + out
    assert rc == 1, out
    assert "닿지 못했다" in out and "보내지 않았다" in out
    assert "완료" not in out


@pytest.mark.parametrize("code,expect", [(202, "미션 보냄"), (409, "거절 (HTTP 409)")])
def test_R3_미션은_202_를_성공이라_하지_않고_409_는_우회하지_않는다(gw, tmp_path, code, expect):
    _, url = gw
    _Fake.replies["/api/robot1/mission"] = (code, {"success": False, "message": "m"})
    rc, out, direct = _run(tmp_path, url, "mission1")
    assert expect in out and "전송 성공" not in out and direct == []


def test_비상정지_0_속도_직접_발행은_정지_API_응답을_기다리지_않는다(gw, tmp_path):
    """관제 검수 §3.4 — 정지 API 는 로봇 보고를 최대 1 s 기다린다. 0 속도가 그 뒤로 밀리면 안 된다.

    예전 시험은 '0 속도 python 을 부른 시각 < 정지 API 가 도착한 시각' 이었다. G-8 로 둘을 동시에 보내면서
    그 순서는 몇 ms 차의 경합이 되었다 — 지킬 것은 순서가 아니라 '0 속도가 API 의 **응답**을 기다리지 않는다' 이다."""
    _, url = gw
    _Fake.replies["/api/robot1/stop"] = (200, {"success": True, "message": "정지 확인"})
    _Fake.stall = 2.0                                          # 정지 API 가 늦게 답한다(스크립트의 curl -m 3 안)
    rc, out, direct = _run(tmp_path, url, "stop")
    done = [t for t, p in _Fake.done if p == "/api/robot1/stop"]
    assert direct and done, (direct, _Fake.log, out)
    assert direct[0] < done[0] - 1.0, "0 속도 직접 발행이 정지 API 응답(최대 1 s 대기)을 기다렸다"
    assert "HTTP 200" in out and rc == 0


def test_G8_0_속도_python_이_느려도_정지_API_는_그것이_끝나기_전에_도착한다(gw, tmp_path):
    """제3자 검수 G-8 그대로의 경우: python 블록이 오래 걸리면(검수 실측 0.66~2.22 s) 예전엔 HOLD 가 그만큼 늦었다.

    검수 R-script-2: 가짜 python 은 2.5 s — 실측 최악(2.22 s)보다 길다. 0 속도 python 상한(ESTOP_PUB_TIMEOUT_SEC)이
    그 아래로 내려가면(예: 2 s) 부하 걸린 현장에서 제대로 나가던 0 속도를 끊고 '시간 초과' 라 한다 — 여기서 빨강이 된다."""
    _, url = gw
    _Fake.replies["/api/robot1/stop"] = (200, {"success": True, "message": "정지 확인"})
    rc, out, direct = _run(tmp_path, url, "stop", py="slow")
    api = [t for t, p in _Fake.log if p == "/api/robot1/stop"]
    assert direct and api, (direct, _Fake.log, out)
    assert api[0] - direct[0] < 1.0, "정지 API(HOLD)가 0 속도 python 이 끝나기를 기다렸다 (%.2f s)" % (api[0] - direct[0])
    assert "0 속도 직접 발행 완료" in out and "HTTP 200" in out
    assert rc == 0, out


@pytest.mark.parametrize("py", ["hang", "hang_ignore_term"])
def test_G8_정지_API_는_0_속도_python_을_기다리지_않고__멈춘_python_은_상한에서_끊긴다(gw, tmp_path, py):
    """제3자 검수 G-8: 예전엔 0 속도 python 이 끝나야 정지 API(플릿 HOLD = Nav2 목표 취소)를 불렀다 —
    HOLD 가 python 시간(격리 0.6~2.2 s)만큼 늦었고, python 이 멈추면 API 는 끝내 안 불렸다(타임아웃 없음)."""
    _, url = gw
    _Fake.replies["/api/robot1/stop"] = (200, {"success": True, "message": "정지 확인"})
    t0 = time.time()
    rc, out, direct = _run(tmp_path, url, "stop", py=py)
    t1 = time.time()
    api = [t for t, p in _Fake.log if p == "/api/robot1/stop"]
    assert direct and api, (direct, _Fake.log, out)
    assert api[0] - direct[0] < 1.0, "정지 API 가 0 속도 python 이 끝나기를 기다렸다 — HOLD 가 늦는다"
    assert t1 - t0 < 12, "멈춘 0 속도 python 에 상한이 없다 (%.1f s)" % (t1 - t0)
    assert "시간 초과" in out and "0 속도 직접 발행 완료" not in out
    assert rc == 1, out                                        # API 가 200 이어도 0 속도는 모른다


def test_G12_0_속도_python_이_실패하면_완료라_하지_않고_현장_NIC_를_말한다__정지_API_는_불린다(gw, tmp_path):
    """제3자 검수 G-12: 예전엔 python 종료코드를 안 봐서 rclpy 가 실패해도 '0 속도 직접 발행 완료' 였다.
    CYCLONEDDS_URI 가 현장 NIC 에 묶인 설정이라 현장 밖에서는 실제로 그렇게 실패한다."""
    _, url = gw
    _Fake.replies["/api/robot1/stop"] = (200, {"success": True, "message": "정지 확인"})
    rc, out, direct = _run(tmp_path, url, "stop", py="fail")
    assert direct, out
    assert "0 속도 직접 발행 완료" not in out and "0 속도 직접 발행 실패" in out
    assert "CYCLONEDDS_URI" in out and "현장 NIC" in out
    assert [p for _, p in _Fake.log] == ["/api/robot1/stop"]
    assert rc == 1, out


@pytest.mark.parametrize("code,py,expect_rc,expect", [
    (200, "ok", 0, "0 속도 직접 발행 완료"),
    (202, "ok", 2, "확인하지 못했다"),
    (503, "ok", 1, "정지 API 실패: HTTP 503"),
    (202, "fail", 1, "0 속도 직접 발행 실패"),
])
def test_G12_비상정지_종료코드는_0_속도와_정지_확인_둘_다를_말한다(gw, tmp_path, code, py, expect_rc, expect):
    _, url = gw
    _Fake.replies["/api/robot1/stop"] = (code, {"success": code == 200, "reason": "NO_FLEET_COORDINATOR" if code == 503 else None,
                                                "message": "m"})
    rc, out, direct = _run(tmp_path, url, "stop", py=py)
    assert direct and expect in out, out
    assert rc == expect_rc, out


def test_G12_정지_API_가_응답하지_않으면_HOLD_가_나갔는지_모른다고_말한다(gw, tmp_path):
    """curl -m 3 시간 초과(28) — 게이트웨이는 살아 있을 수 있다. 'HTTP ERR' 같은 가짜 HTTP 코드도, 0 도 아니다."""
    _, url = gw
    _Fake.replies["/api/robot1/stop"] = (200, {"success": True, "message": "정지 확인"})
    _Fake.stall = 3.5                                          # 정지 API 의 -m 3 보다 길게
    rc, out, direct = _run(tmp_path, url, "stop")
    assert direct, out
    assert "정지 API 응답 없음" in out and "나갔는지 모른다" in out and "HTTP ERR" not in out, out
    assert rc == 1, out


def test_G7_게이트웨이에_닿지_못해도_비상정지의_0_속도_직접_발행은_남는다__성공이라_하지는_않는다(tmp_path):
    rc, out, direct = _run(tmp_path, _closed_port_url(), "stop")
    assert len(direct) == 1, "멈추는 쪽 직접 발행(0 속도)은 게이트웨이 없이도 나가야 한다: " + out
    assert "닿지 못했다" in out and "HOLD" in out
    assert rc == 1, out


@pytest.mark.parametrize("args,path,code,expect_rc", [
    (("goal", "0.5", "0.3"), "/api/robot1/goal", 200, 0),
    (("goal", "0.5", "0.3"), "/api/robot1/goal", 409, 1),
    (("goal", "0.5", "0.3"), "/api/robot1/goal", 503, 1),
    (("mission1",), "/api/robot1/mission", 200, 0),
    (("mission1",), "/api/robot1/mission", 202, 2),
    (("mission1",), "/api/robot1/mission", 409, 1),
])
def test_G12_목표_미션_종료코드가_게이트웨이_답을_말한다(gw, tmp_path, args, path, code, expect_rc):
    """제3자 검수 G-12: 예전엔 거절(409)·게이트웨이 고장(503)에도 종료코드 0 이었다. 202 = 보냈으나 모른다 = 2."""
    _, url = gw
    _Fake.replies[path] = (code, {"success": code == 200, "message": "m"})
    rc, out, direct = _run(tmp_path, url, *args)
    assert rc == expect_rc, out
    assert direct == []


@pytest.mark.parametrize("yaw", ["abc", "nan", "1e308", "-1e308"])
def test_G12_잘못된_Yaw_는_0도로_바꿔_보내지_않는다(gw, tmp_path, yaw):
    """예전엔 도→라디안 python 의 실패를 안 봐서 빈 값이 send_goal 의 기본 0.0 이 되어 목표가 나갔다.
    검수 R-script-1: 1e308° 는 유한하지만 라디안으로 바꾸면 inf — "inf" 가 JSON 을 깨서 (0,0,0) 목표가 됐다."""
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, "goal", "0.5", "0.3", yaw)
    assert _Fake.log == [], "잘못된 Yaw 로 목표를 보냈다: %s" % _Fake.bodies
    assert rc == 1 and "잘못된 Yaw" in out


@pytest.mark.parametrize("x,y", [("abc", "0.3"), ("0.5m", "0.3"), ("0.5", "abc"), ("0.5", "0.3,"),
                                 ("NaN", "0.3"), ("0.5", "-Infinity"), ("1e999", "0.3"),
                                 ('0.5, "x": 0.1', "0.3")])
def test_R_script_1_잘못된_X_Y_는_0_0_0_이나_다른_목표로_바꿔_보내지_않는다(gw, tmp_path, x, y):
    """검수 R-script-1: 예전엔 X·Y 를 JSON 에 그대로 끼워 넣었다. 게이트웨이는 깨진 본문을 {} 로 보고 기본값 0 을 써서
    잘못 친 좌표가 (0,0,0) 목표로 나갔고 "전송 완료"·종료코드 0 이었다. NaN·-Infinity·1e999 는 JSON 파서가 받아
    NaN·Infinity 목표가 됐고, '0.5, "x": 0.1' 은 x 를 0.1 로 바꿔 끼웠다."""
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, "goal", x, y, "90")
    assert _Fake.log == [], "잘못된 좌표로 목표를 보냈다: %s" % _Fake.bodies
    assert rc == 1 and "잘못된 좌표" in out and "완료" not in out, out
    assert direct == []


def test_R_script_1_대화형_메뉴에서_X_Y_를_비워도_0_0_0_목표로_보내지_않는다(gw, tmp_path):
    """메뉴에서 X·Y 에 Enter 만 치면 예전엔 'X=m, Y=m' 로 (0,0,0) 목표가 나갔다."""
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, stdin="1\n\n\n90\n\n0\n")
    assert _Fake.log == [], "빈 X·Y 로 목표를 보냈다: %s" % _Fake.bodies
    assert "잘못된 좌표" in out and "전송 완료" not in out, out


def test_Yaw_는_도에서_라디안으로_바뀌어_간다(gw, tmp_path):
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, "goal", "0.5", "0.3", "90")
    assert rc == 0, out
    (path, body), = _Fake.bodies
    assert path == "/api/robot1/goal" and body["x"] == 0.5 and body["y"] == 0.3
    assert abs(body["yaw"] - 1.5707963) < 1e-6


@pytest.mark.parametrize("x,y,ex,ey", [("-1.4", "0", -1.4, 0.0), ("1", "-0.3", 1.0, -0.3), ("+0.5", ".3", 0.5, 0.3)])
def test_R_script_1_바른_좌표는_그대로_간다(gw, tmp_path, x, y, ex, ey):
    """좌표 검사가 정수·음수·부호·앞 0 생략을 거절하지 않는다(과한 거절도 조작을 막는다)."""
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, "goal", x, y)
    assert rc == 0, out
    (path, body), = _Fake.bodies
    assert body == {"x": ex, "y": ey, "yaw": 0.0}


def test_R_script_1_대화형_메뉴의_바른_좌표는_그대로_간다(gw, tmp_path):
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, stdin="1\n0.5\n-0.3\n\n\n0\n")
    (path, body), = _Fake.bodies
    assert body == {"x": 0.5, "y": -0.3, "yaw": 0.0} and "전송 완료" in out, out


def test_G12_대화형_메뉴도_잘못된_Heading_을_0도로_바꿔_보내지_않는다(gw, tmp_path):
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    rc, out, direct = _run(tmp_path, url, stdin="1\n0.5\n0.3\nabc\n\n0\n")
    assert _Fake.log == [], "잘못된 Heading 으로 목표를 보냈다: %s" % _Fake.bodies
    assert "잘못된 Heading" in out



def test_검토S_느린_게이트웨이는_닿지_못한_것이_아니다__직접_발행하지_않는다(gw, tmp_path):
    """curl 이 시간 초과(28)면 게이트웨이는 살아 있다 — 이미 처리했거나 거절했을 수 있다."""
    _, url = gw
    _Fake.replies["/api/robot1/goal"] = (200, {"success": True})
    _Fake.stall = 3.0                                          # _post 의 -m 2 보다 길게
    rc, out, direct = _run(tmp_path, url, "goal", "0.5", "0.3")
    assert direct == [] and "응답 없음" in out
    assert rc == 1, out                                        # G-12: 처리됐는지 모르는 것을 성공이라 하지 않는다
