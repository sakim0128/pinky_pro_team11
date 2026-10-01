# -*- coding: utf-8 -*-
"""MCV-1D — 멀티뷰가 지켜야 할 성질을 화면 소스에서 고정한다.

브라우저 없이 JS 동작을 시험할 수는 없다. 그래서 **되돌아가면 안 되는 성질**만 고정한다 —
전부 2026-09-10 에 실제로 문제가 됐던 것들이다.

    UI-02  소스 id 하드코딩 -> tablet-relay 가 버튼 배열에서 빠진 걸 아무도 못 알아챘다
    §3-②  <img> 상시 연결 -> lazy 웹캠이 계속 열려 LED 가 안 꺼진다
    §3-③  전역 boost -> 관측 경로의 ArUco 대비를 망친다
    §3-①  시각 상태 미표시 -> 나란히 놓인 영상을 같은 순간으로 오해한다
"""
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)
HTML = os.path.join(_RELAY, "gateway_web", "static", "index.html")

# ⚠️ 이 목록은 "JS 에 박혀 있으면 안 되는 id" 다(부정 목록). 소스가 늘면 같이 늘려야
#    가드가 새 id 를 덮는다. 2026-09-14: tablet-relay -> phone2 로 개명(기기 교체).
#    옛 id 도 남겨 둔다 — 되살아나 JS 에 박히는 것도 막을 값이다.
SOURCE_IDS = ("phone", "phone2", "tablet-relay", "relay-cam", "gazebo", "control",
              "pinky1", "pinky2")


def _html():
    if not os.path.exists(HTML):
        pytest.skip("index.html 없음")
    with open(HTML, encoding="utf-8") as fh:
        return fh.read()


def _multiview_js():
    """멀티뷰 JS 블록만 잘라낸다. 다른 코드의 하드코딩까지 트집잡지 않기 위해서다."""
    t = _html()
    start = t.find("// ===== MCV-1D 멀티뷰 =====")
    end = t.find("// --- 관제 카메라 다중 소스 및 AR", start + 10)
    if start < 0 or end < 0:
        pytest.fail("멀티뷰 JS 블록을 못 찾았다")
    return t[start:end]


# ---- 존재 --------------------------------------------------------------------

def test_멀티뷰_카드와_격자가_있다():
    t = _html()
    assert 'id="mv-card"' in t
    assert 'id="mv-grid"' in t


def test_멀티뷰가_api_sources_를_읽는다():
    assert "/api/sources" in _multiview_js()


# ---- UI-02: 소스 id 하드코딩 금지 ------------------------------------------------

def test_멀티뷰_JS_에_소스_id_가_박혀_있지_않다():
    """이게 09-10 의 사고다. 버튼 배열에 id 를 박아 둬서 tablet-relay 추가를 못 알아챘다.

    목록이 아니라 **성질**로 걸러야 새 소스가 저절로 따라온다.
    """
    js = _multiview_js()
    # 문자열 리터럴로 등장하는 소스 id 만 본다 (변수명·주석의 단어는 아니다)
    for sid in SOURCE_IDS:
        for quote in ("'", '"'):
            lit = quote + sid + quote
            assert lit not in js, "멀티뷰가 소스 id %s 를 박아 뒀다" % lit


def test_성질로_거른다_transport_와_trust():
    js = _multiview_js()
    assert "MV_REAL_TRANSPORTS" in js
    assert "'pull'" in js and "'local'" in js, "transport 성질로 걸러야 한다"
    assert "trust" in js and "trusted" in js


# ---- §3-②: lazy 웹캠을 상시로 열지 않는다 ------------------------------------------

def test_local_은_기본으로_열지_않는다():
    """<img> 를 붙이면 웹캠이 열리고 LED 가 켜진다. 09-09 에 없앤 문제다."""
    js = _multiview_js()
    assert "MV_DEFAULT_OPEN_TRANSPORTS" in js
    m = re.search(r"MV_DEFAULT_OPEN_TRANSPORTS\s*=\s*\[([^\]]*)\]", js)
    assert m, "기본 개방 목록을 못 찾았다"
    body = m.group(1)
    assert "'pull'" in body
    assert "'local'" not in body, "local(웹캠)을 기본으로 열면 LED 가 계속 켜진다"


def test_닫으면_연결을_실제로_끊는다():
    """display:none 만으로는 MJPEG 연결이 안 끊긴다. src 를 비워야 한다."""
    js = _multiview_js()
    assert re.search(r"img\.src\s*=\s*''", js), "닫을 때 src 를 비우지 않는다"


def test_폴링이_수요를_만들지_않는다는_것을_주석으로_남겼다():
    """/api/sources 는 probe 로 상태를 본다(MCV-1C). 그 근거가 코드에 남아야 다음 사람이 안 뒤집는다."""
    js = _multiview_js()
    assert "probe" in js


# ---- §3-③: 관측 경로 오염 금지 ---------------------------------------------------

def test_멀티뷰가_boost_나_watermark_를_붙이지_않는다():
    """boost 는 채도를 1.8배 올려 ArUco 흑백 대비를 망친다. 기본으로 켜면 안 된다."""
    js = _multiview_js()
    assert "boost" not in js
    assert "watermark" not in js


# ---- §3-①: 나란히 놓인 영상은 "같은 순간"이라는 주장이다 ------------------------------

def test_타일이_시각_상태를_표시한다():
    """나란히 놓인 영상을 **같은 순간**으로 오해하지 않게 시각 상태를 낸다.

    ⭐ 예전엔 `offsetMedianMs` 라는 **필드 이름**을 단언했다. 그 이름 하나가
       시계차·전송·큐를 다 담고 있었고, 화면은 그걸 `지연` 이라고 불렀다 —
       그래서 로컬 웹캠에 `지연 18837.8ms` 가 떴다(2026-09-12).
       지금은 서버가 **어긋남/지연/전송**으로 갈라서 준다. 그래서 이 시험도
       이름이 아니라 **성질**을 본다: 세 가지가 각각 화면에 있는가.
    """
    js = _multiview_js()
    assert "'어긋남 '" in js, "시각 어긋남을 표시하지 않으면 같은 순간으로 오해한다"
    assert "'지연 '" in js, "지연 칸이 없다"
    assert "offsetJitterMs" in js, "흔들림은 어긋남 중 시계차가 **아닌** 성분이다 — 버리면 안 된다"
    assert "clock" in js


def test_촬영_시각이_없으면_그렇게_말한다():
    """receive 를 capture 처럼 보이게 하면 없는 정확도를 주장하는 것이다."""
    js = _multiview_js()
    assert "capture" in js
    assert "수신 시각" in js


def test_지연_경고_임계가_상수로_있다():
    js = _multiview_js()
    m = re.search(r"MV_OFFSET_WARN_MS\s*=\s*(\d+)", js)
    assert m, "경고 임계가 없다"
    assert 100 <= int(m.group(1)) <= 2000, "임계 %s ms 가 비현실적이다" % m.group(1)


def test_유실_프레임을_보여준다():
    assert "framesLost" in _multiview_js()


def test_배지를_찾는_선택자와_만드는_속성이_짝이_맞는다():
    """짝이 어긋나면 배지가 **영영 갱신되지 않는다.** 화면은 멀쩡해 보이는데 숫자가 언다.

    실제로 처음 작성할 때 selector 만 있고 속성을 안 만들어 이 버그가 있었다.
    """
    js = _multiview_js()
    selector = re.search(r"querySelector\(\s*'\[([a-z-]+)=", js)
    assert selector, "배지 선택자를 못 찾았다"
    attr = selector.group(1)
    assert attr + '="' in js, "선택자는 %s 로 찾는데 그 속성을 만드는 곳이 없다" % attr
