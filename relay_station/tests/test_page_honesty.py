# -*- coding: utf-8 -*-
"""계획서 §5 금지 조항 5 — **도메인·연결 상태를 하드코딩해 보고하지 않는다** (G-6 재발 금지).

2026-09-11 에 다른 세션이 짚어 준(R-5) 그대로, 이 페이지가 그러고 있었다:

    rtt-router 에 `0.5 ms` 가 **HTML 에 박혀** 있었다 — 재 본 적 없는 숫자다
    로봇 셋의 배지가 **측정 전부터 초록 ONLINE**
    머리에 `● LAN 198.51.100.3` `● Tailscale 100.64.0.81` — 재는 곳이 없다
    태블릿 카드에 `198.51.100.2 (:8080)` — 낡았고 지금 당기는 포트는 :18082 다

⭐⭐ 지어낸 숫자는 **없는 것보다 나쁘다.** 없으면 사람이 잰다. 있으면 안 잰다.

⭐ 주소를 화면에 베끼지 않는다 — 서버가 **실제로 핑하는 목록**에서 온다.
   손으로 적어 두면 서버 목록을 고쳐도 화면은 옛 주소를 계속 보여 준다.
   (같은 세션에서 사유 표를 손으로 베껴 뒀다가 `LENS_CHANGED` 를 놓친 적이 있다.)

⚠️ **접속 안내는 다른 범주다.** SSH 명령이나 브라우저 주소는 *기록*이지 *현황*이 아니다.
   금지 조항 5 는 상태 **보고**를 막는 것이다. 그래서 지우지 않고 as-of 를 붙였다.
"""
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(os.path.dirname(_HERE), "gateway_web", "static", "index.html")

IPV4 = re.compile(r"\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b")

# 서버가 핑하는 노드들 (NetworkLatencyMonitor.targets 와 같은 키)
# ⭐ 서버가 핑하는 **고정 설비**만. 태블릿·폰은 들고 다녀서 주소가 바뀌므로
#    핑 목록에 없다 - 도달 여부는 소스 자신이 말한다.
NODES = ("router", "robot1", "robot2", "internet")


def _html():
    if not os.path.exists(HTML):
        pytest.skip("index.html 없음")
    with open(HTML, encoding="utf-8") as fh:
        return fh.read()


def _no_comments(text):
    """HTML 주석을 걷어낸다.

    ⭐ 주석에 적어 둔 **"예전에 이랬다"** 를 잡으면 거짓 실패다. 이 세션에서
       같은 방식으로 한 번 틀렸다 — 무엇을 보는지 먼저 좁힌다.
    """
    return re.sub(r"<!--.*?-->", "", text, flags=re.S)


_DIV_TAG = re.compile(r"<div\b|</div>")


def _block(text, anchor):
    """그 표시를 담은 요소의 **짝 맞는 끝**까지. `<div>` 를 센다.

    ⭐⭐ 처음엔 `find("</section>")` 같은 끝 표시를 찍었는데 그게 없어서 창이 페이지
       끝까지 벌어졌고 **접속 안내 패널까지 삼켜** 거짓 실패가 났다. 이 프로젝트에서
       좁은 문자열로 구조를 판정해 여러 번 틀렸는데, **넓게 자르는 것도 같은 병**이다.
       무엇을 보는지 구조로 좁힌다.
    """
    i = text.find(anchor)
    assert i > 0, "%s 를 못 찾았다" % anchor
    start = text.rfind("<div", 0, i)
    depth, k = 0, start
    while k < len(text):
        m = _DIV_TAG.search(text, k)
        if not m:
            break
        depth += 1 if m.group(0) != "</div>" else -1
        k = m.end()
        if depth == 0:
            return text[start:k]
    pytest.fail("%s 의 짝 맞는 끝을 못 찾았다" % anchor)


def _grid():
    """지연 격자 블록만. ⭐ 넓게 자르면 접속 안내(다른 범주)를 삼킨다."""
    return _block(_no_comments(_html()), 'id="latency-container"')


# ---- ⭐⭐ 지어낸 측정값 ------------------------------------------------------------

def test_재지_않은_숫자를_화면에_안_적는다():
    """⭐⭐ `0.5 ms` 가 HTML 에 박혀 있었다. 페이지를 열면 재 본 적 없는 값이 보였다.

    지어낸 숫자는 없는 것보다 나쁘다 — 없으면 사람이 재고, 있으면 안 잰다.
    """
    grid = _grid()
    for key in NODES:
        m = re.search(r'id="rtt-%s"[^>]*>([^<]*)<' % key, grid)
        assert m, "rtt-%s 를 못 찾았다" % key
        text = m.group(1).strip()
        assert not re.search(r"\d", text), \
            "rtt-%s 초기값에 숫자가 있다: %r — 잰 적 없는 값이다" % (key, text)


def test_측정_전에는_초록이_아니다():
    """상태를 아직 모르는데 초록으로 칠하면 그게 하드코딩된 보고다."""
    grid = _grid()
    for key in NODES:
        m = re.search(r'id="badge-%s"' % key, grid)
        assert m, "badge-%s 를 못 찾았다" % key
        # 그 span 태그 전체를 본다
        start = grid.rfind("<span", 0, m.start())
        tag = grid[start:grid.find(">", m.end()) + 1]
        assert "badge green" not in tag, \
            "badge-%s 가 측정 전부터 초록이다: %s" % (key, tag)
        assert "ONLINE" not in tag and "CONNECTED" not in tag, \
            "badge-%s 가 측정 전부터 연결됐다고 말한다" % key


def test_rtt_초기값에_좋다_나쁘다_색이_없다():
    """`-- ms` 에 rtt-good(초록)이 붙어 있었다 — 값이 없는데 좋다고 칠한 것이다."""
    grid = _grid()
    for key in NODES:
        m = re.search(r'<div class="([^"]*)" id="rtt-%s"' % key, grid)
        assert m, "rtt-%s 를 못 찾았다" % key
        cls = m.group(1)
        for bad in ("rtt-good", "rtt-warn", "rtt-bad", "rtt-mid"):
            assert bad not in cls, "rtt-%s 초기 클래스에 %s 가 있다" % (key, bad)


# ---- ⭐ 주소를 베끼지 않는다 --------------------------------------------------------

def test_지연_격자에_주소를_박지_않는다():
    """⭐ 서버가 **실제로 핑하는 주소**가 응답에 들어 있다. 손으로 적으면 서버 목록을
       고쳐도 화면은 옛 주소를 계속 보여 준다."""
    found = IPV4.findall(_grid())
    assert not found, "지연 격자에 주소가 박혀 있다: %s" % sorted(set(found))


def test_주소_칸이_있고_JS_가_채운다():
    t = _no_comments(_html())
    for key in NODES:
        assert 'id="ip-%s"' % key in t, "ip-%s 칸이 없다" % key
    assert "getElementById('ip-' + key)" in t, "갱신 함수가 주소를 안 채운다"
    assert "info.ip" in t, "서버가 준 주소를 안 쓴다"


def test_머리_배지에_주소를_안_박는다():
    """`● LAN 198.51.100.3` `● Tailscale 100.64.0.81` — 초록 점까지 붙어 있었는데
    재는 곳이 없었다. 기록은 docs/NETWORK_AND_PORTS.md 가 가진다."""
    head = _block(_no_comments(_html()), '<div class="badges">')
    found = IPV4.findall(head)
    assert not found, "머리 배지에 주소가 박혀 있다: %s" % sorted(set(found))


def test_상태_문구에_서브넷을_안_박는다():
    """`양호 (198.51.100.x)` — 사이트가 바뀌면 틀린 말이 된다. 실제로 두 번 어긋났다.

    ⚠️ **안내 패널의 `사설망(198.51.100.x) IP 규칙표` 는 대상이 아니다.** 거기는 규칙을
       설명하는 자리고, 여기서 막는 것은 **상태와 함께 보고되는** 서브넷이다.
       처음엔 파일 전체에서 그 문자열을 찾아 규칙표까지 걸었다 — 범주를 안 나눈 탓이다.
    """
    t = _no_comments(_html())
    i = t.find("fleet-lan-link")
    assert i > 0, "링크 상태 칸을 못 찾았다"
    assert not IPV4.search(t[max(0, i - 300):i + 300]), \
        "링크 상태 옆에 주소가 박혀 있다"
    j = t.find("p2pEl.textContent")
    assert j > 0, "링크 상태를 채우는 곳을 못 찾았다"
    assert "192.168" not in t[j:j + 300], "상태 문구에 서브넷을 박는다"


# ---- 접속 안내는 다른 범주 ---------------------------------------------------------

def test_접속_안내는_기록이라고_적혀_있다():
    """⚠️ 지우지 않는다 — 지우면 팀원이 들어올 길이 없다. 대신 **언제 기준인지**를 적는다.

    금지 조항 5 는 상태 **보고**를 막는 것이지 안내를 막는 것이 아니다.
    """
    t = _html()
    assert "기준 기록" in t, "접속 안내가 현황처럼 보인다 (as-of 가 없다)"
    assert "NETWORK_AND_PORTS" in t, "최신을 어디서 보는지 안 알려준다"
