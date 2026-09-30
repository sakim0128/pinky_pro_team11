# -*- coding: utf-8 -*-
"""제어권 정책의 게이트웨이·화면 배선 — 정적(rclpy 없는 기계에서도 돈다).

규칙(관제 2026-09-28, 사용자 결정):
- (개편 2단계 2026-09-29: 원 저장소 단일 로봇 경로 목표·미션과 관측·캘리브레이션·비전 세계는 게이트웨이에서 지웠다)
- 움직이는 명령 세 경로(로봇 재개 · 좌표 전환 ①②③ · 플릿 start/resume/assign)는 전부 `_deny_if_cannot_move` 를 지난다.
- 멈추는 명령(fleet stop/estop · 로봇 정지)은 정책을 부르지 않는다.
- /api/status 의 view_only 는 정책이 말하고 control 블록이 같이 나간다 · /api/control(GET) · /api/control/{acquire,release}(POST).
- 화면(중계 콘솔 relay_console.html): 남이 쥐면 body.control-held 로 움직이는 버튼을 잠근다(숨기지 않는다) · 제어권 버튼 둘.
"""
import io
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
GW = os.path.join(os.path.dirname(HERE), "gateway_web")
SRC = io.open(os.path.join(GW, "gateway_web_server.py"), encoding="utf-8").read()
CONSOLE = io.open(os.path.join(GW, "static", "relay_console.html"), encoding="utf-8").read()


def _block(head, upto="\n        elif parsed.path"):
    i = SRC.index(head)
    j = SRC.find(upto, i + 1)
    return SRC[i:j if j > 0 else i + 3000]


def test_정책_객체는_모듈_수준이고_허용_파일_경로는_두_배치를_다_본다():
    assert "import control_policy" in SRC
    assert "CONTROL_POLICY = control_policy.ControlPolicy(LOCAL_CONTROL_IPS, control_policy.AllowList(_control_allow_path()))" in SRC
    body = SRC[SRC.index("def _control_allow_path"):SRC.index("CONTROL_POLICY =")]
    assert "'..', 'configs', 'control_allow.json'" in body and "'..', '..', 'configs', 'control_allow.json'" in body


def test_움직이는_세_경로는_전부_정책을_지난다():
    resume = _block("elif parsed.path in ROBOT_RESUME_PATHS:")
    assert "self._deny_if_cannot_move('remote robot resume', {'robot': robot})" in resume and "not in LOCAL_CONTROL_IPS" not in resume
    profile = _block("elif parsed.path in PROFILE_COMMANDS:")
    assert "self._deny_if_cannot_move('remote profile command ' + cmd, {'command': cmd})" in profile and "not in LOCAL_CONTROL_IPS" not in profile
    fleet = _block("elif parsed.path in ('/api/fleet/start', '/api/fleet/stop', '/api/fleet/estop', '/api/fleet/resume', '/api/fleet/assign'):")
    assert "fleet_cmd_name not in FLEET_STOP_COMMANDS and self._deny_if_cannot_move(" in fleet   # 멈추는 명령은 정책 밖
    assert "not in LOCAL_CONTROL_IPS" not in fleet


def test_로봇_정지는_정책을_부르지_않는다():
    stop = _block("if parsed.path in ROBOT_STOP_PATHS:")
    assert "_deny_if_cannot_move" not in stop and "not in LOCAL_CONTROL_IPS" not in stop


def test_거절_응답은_403_또는_409_이고_이유_코드를_싣는다():
    body = SRC[SRC.index("def _deny_if_cannot_move"):SRC.index("def do_POST")]
    assert "CONTROL_POLICY.may_move(ip)" in body and "code=CONTROL_POLICY.http_code(code)" in body
    assert "'reason': code" in body and "'error': 'Forbidden' if code == 'FORBIDDEN' else 'ControlHeld'" in body


def test_status_는_정책의_view_only_와_control_을_싣고_요청마다_touch_한다():
    st = SRC[SRC.index("elif parsed.path == '/api/status':"):SRC.index("# 5-B. 플릿 관제 상태 전용 JSON API")]
    assert "'view_only': CONTROL_POLICY.status(self.client_address[0])['view_only']" in st
    assert "'control': CONTROL_POLICY.status(self.client_address[0])" in st
    assert "not in LOCAL_CONTROL_IPS" not in st
    assert SRC.count("CONTROL_POLICY.touch(self.client_address[0])") == 2      # do_GET · do_POST
    assert "if parsed.path == '/api/control':" in SRC
    assert "if parsed.path in ('/api/control/acquire', '/api/control/release'):" in SRC
    assert "CONTROL_POLICY.acquire(ip, name=str(req_json.get('name') or ''))" in SRC and "CONTROL_POLICY.release(ip)" in SRC


def test_콘솔은_남이_쥐면_잠그고_숨기지_않는다():
    assert 'id="control"' in CONSOLE and 'id="acquire"' in CONSOLE and 'id="release"' in CONSOLE
    assert "body.control-held .move{opacity:.45;pointer-events:none}" in CONSOLE
    assert "document.body.classList.toggle('control-held', !!c.holder && !c.mine);" in CONSOLE
    assert "c.holder_ttl_s" in CONSOLE
    assert "post('/api/control/acquire'" in CONSOLE and "post('/api/control/release')" in CONSOLE
    # 보기 전용이면 움직이는 조작·제어권 버튼이 숨고(서버 view_only), 멈추는 버튼(.stop)은 남는다
    assert "body.view-only .move, body.view-only .move-only{display:none}" in CONSOLE
    assert "document.body.classList.toggle('view-only', !!h.view_only);" in CONSOLE
    assert 'id="acquire" class="move-only"' in CONSOLE
    for path in ("/api/fleet/stop", "/api/fleet/estop"):
        assert 'class="stop" data-post="%s"' % path in CONSOLE


def test_기본_허용_파일은_닫혀_있고_문서용_주소뿐이다():
    for cand in (os.path.join(os.path.dirname(os.path.dirname(HERE)), "configs", "control_allow.json"),
                 os.path.join(os.path.dirname(HERE), "configs", "control_allow.json")):
        if os.path.exists(cand):
            doc = json.load(io.open(cand, encoding="utf-8"))
            assert all(not r.get("enabled") for r in doc["controllers"])
            assert all(re.match(r"^198\.51\.100\.\d+$", r["ip"]) for r in doc["controllers"])
            return
    raise AssertionError("configs/control_allow.json 이 없다")
