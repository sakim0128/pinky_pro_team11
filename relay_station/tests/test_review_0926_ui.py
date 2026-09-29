# -*- coding: utf-8 -*-
"""REVIEW_20260926 G 후속: 게이트웨이를 재기동하면 로봇이 링크유실로 래치되고 코디네이터가 그 로봇을
"링크유실 래치 — 로봇 재개 필요" 로 세운다. 그런데 V2 에는 로봇 재개(`/api/pinkyN/resume`)가 없었다 →
웹만으로는 월요일 ② 로봇 지도 전환이 REFUSED 로 막혔다. 로봇 카드에 로봇 정지·재개와 세운 이유를 싣는다.
경로는 /api/pinkyN/stop · /api/pinkyN/resume — 이름은 pinkyN 하나다(2026-09-29).
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

STATIC = os.path.join(REPO, "relay_station", "gateway_web", "static")


# ---- V2 로봇 카드 · 로봇 정지·재개 -------------------------------------------------------------------

def _v2():
    js = open(os.path.join(STATIC, "fleet_control_v2.js"), encoding="utf-8").read()
    css = open(os.path.join(STATIC, "fleet_control_v2.css"), encoding="utf-8").read()
    return js, css


def test_V2_로봇_카드에_로봇_정지·재개가_있고_재개는_확인을_묻는다():
    """통합 검토 OPS-4 로 바꿨다: 예전 이 시험은 정지도 확인을 묻는 것을 고정했다 — 멈추는 명령은 묻지 않는다(플릿
    비상정지 버튼처럼). 움직일 수 있는 재개만 묻는다(test_review_0926_ui2 가 실제 Chrome 에서 잰다)."""
    js, css = _v2()
    assert 'data-robot-cmd="stop"' in js and 'data-robot-cmd="resume"' in js
    assert "`/api/${robotKey(index)}/${cmd}`" in js and "/api/robot" not in js
    body = js[js.index("async function robotAction"):]
    body = body[:body.index("\n  }\n")]
    ask = 'if (cmd === "resume" && !window.confirm('
    assert ask in body and "postJson" in body and "refresh()" in body
    assert body.index(ask) < body.index("postJson")      # 재개는 묻기 전에는 보내지 않는다
    assert ".robot-actions" in css


def test_V2_카드_버튼은_위임으로_받는다__카드가_다시_그려져도_산다():
    js, _ = _v2()
    init = js[js.index("function initControls"):]
    init = init[:init.index("document.querySelectorAll(\"[data-command]\")")]
    assert '$("dashboard-robots")' in init and 'closest("[data-robot-cmd]")' in init and "robotAction(" in init


def test_V2_세운_이유를_카드에_보이고_세운_로봇은_초록이_아니다():
    js, _ = _v2()
    assert "held: Boolean(f.held)" in js and "heldReason: f.held_reason" in js
    assert "로봇별 정지" in js and "r.heldReason" in js
    tone = js[js.index("function toneForRobot"):]
    tone = tone[:tone.index("\n  }\n")]
    assert 'if (r.held) return "warn"' in tone and tone.index("r.held") < tone.index('return "ok"')


def test_V2_카드_번호와_게이트웨이_로봇_재개·정지_경로가_맞는다():
    import gateway_web_server as g
    for i, name in ((1, "pinky1"), (2, "pinky2")):
        assert g.ROBOT_RESUME_PATHS["/api/%s/resume" % name] == name
        assert g.ROBOT_STOP_PATHS["/api/%s/stop" % name] == name
    assert not [p for p in list(g.ROBOT_STOP_PATHS) + list(g.ROBOT_RESUME_PATHS) if "robot" in p]


def test_V2_코디네이터_경고를_알림_띠에_싣는다__끊겼으면_옛_경고라_싣지_않는다():
    """통합 검토(플릿 묶음 열린 질문): ARRIVED_NOT_AT_GOAL · STALL_NO_REQUEST · ROBOT_IDLE_IN_RUNNING 이 화면에 없었다."""
    js = open(os.path.join(STATIC, "fleet_control_v2.js"), encoding="utf-8").read()
    body = js[js.index("function renderLink"):]
    body = body[:body.index("\n  }\n")]
    assert 'link === "ok" && state.fleet?.warning' in body
    assert 'split(" | ")' in body and "플릿 경고" in body

