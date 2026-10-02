# -*- coding: utf-8 -*-
"""index.html 폴링(fetchStatus) 가드·배지 매핑·고장 토글 숨김을 소스 텍스트로 고정한다.

배경: 요소(tablet-badge)를 지운 커밋 뒤에도 fetchStatus 가 그 요소를 가드 없이 역참조해
매 폴링 TypeError 로 이후 갱신(호스트 자원·지연·fleet_comm·discrepancy)이 전부 멎어 있었다.
브라우저가 없는 환경에서는 이 파일의 텍스트 고정이 유일한 자동 회귀다.

고정하는 것:

    G-1  요소를 id 로 잡아 쓰는 폴링 코드는 속성 접근 전에 존재 가드가 있다 (tablet-badge)
    G-2  정적이던 로봇·링크 배지를 API(runtime·p2p_link) 값으로 다시 그린다 — 보증 단어를 쓰지 않는다
    G-3  보기 전용 출처(view_only)에서는 고장 주입 버튼을 숨긴다
    G-4  고장 토글이 거부(비 2xx)되면 화면 상태를 바꾸지 않는다

검사는 순수 함수(`_problems_*`)로 만들고, 같은 함수가 **변이된 소스에서는 실패함**을 함께 시험한다
(구조적으로 실패할 수 없는 검사를 막는다).
"""
import io
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(os.path.dirname(_HERE), "gateway_web", "static", "index.html")

# 배지 둘 + 링크 배지 — 이 id 들은 JS 가 API 값으로 덮어써야 한다.
RUNTIME_BADGE_IDS = ("p1-runtime-badge", "p2-runtime-badge", "fleet-r1-badge", "fleet-r2-badge")


def _html():
    # skip 은 통과가 아닌데 exit 0 이라 통과처럼 보인다 — 파일이 없으면 시험의 전제가 깨진 것이라 실패다.
    if not os.path.exists(HTML):
        pytest.fail("index.html 이 없다 — 이 시험의 대상이 사라졌다: %s" % HTML)
    with io.open(HTML, encoding="utf-8") as fh:
        return fh.read()


def _problems_guard(text):
    """getElementById('tablet-badge') 를 변수에 받은 뒤, 첫 속성 접근 전에 `if (<변수>)` 가 있어야 한다."""
    out = []
    pat = re.compile(r"(?:const|let|var)\s+(\w+)\s*=\s*document\.getElementById\('tablet-badge'\)")
    found = list(pat.finditer(text))
    if not found:
        out.append("tablet-badge 를 잡는 코드가 없다 — 가드 검사의 전제가 사라졌다")
    for m in found:
        var = m.group(1)
        tail = text[m.end(): m.end() + 700]
        guard = re.search(r"if\s*\(\s*%s\s*\)" % re.escape(var), tail)
        access = re.search(r"\b%s\.(?:textContent|className|style|classList|innerHTML)" % re.escape(var), tail)
        if not access:
            continue  # 이 변수를 쓰지 않는 곳이면 가드가 필요 없다
        if not guard or guard.start() > access.start():
            out.append("변수 %s: 속성 접근 전에 if (%s) 가드가 없다" % (var, var))
    return out


def _problems_badges(text):
    out = []
    if "const RT = {" not in text:
        out.append("runtime → 배지 매핑 표(RT)가 없다")
    for key in ("PHYSICAL", "DOCKER", "OFFLINE"):
        if not re.search(r"\b%s:\s*\[" % key, text):
            out.append("RT 매핑에 %s 항이 없다" % key)
    if "RT.OFFLINE" not in text:
        out.append("알 수 없는 runtime 의 기본값(RT.OFFLINE)이 없다")
    for bid in RUNTIME_BADGE_IDS:
        if text.count("'%s'" % bid) < 1:
            out.append("JS 가 %s 를 덮어쓰지 않는다" % bid)
    if "OBSERVED_SHARED_PLANE" not in text:
        out.append("p2p_link 상태(OBSERVED_SHARED_PLANE) 매핑이 없다")
    # JS 가 보증 단어를 배지에 써 넣으면 안 된다 (정적 마크업의 초기값은 첫 폴링에 덮인다)
    if re.search(r"\.textContent\s*=\s*['\"]SECURED['\"]", text):
        out.append("JS 가 배지에 SECURED 를 써 넣는다")
    return out


def _problems_toggle_hide(text):
    if not re.search(r"getElementById\('btn-fault-toggle'\)", text):
        return ["btn-fault-toggle 를 잡는 코드가 없다"]
    if not re.search(r"\.hidden\s*=\s*\(?\s*data\.view_only\s*===\s*true\s*\)?", text):
        return ["view_only 출처에서 고장 토글을 숨기는 코드가 없다"]
    return []


def _problems_toggle_reject(text):
    m = re.search(r"async function toggleFaultPoseFix\(\)\s*\{", text)
    if not m:
        return ["toggleFaultPoseFix 를 못 찾았다"]
    body = text[m.end(): m.end() + 900]
    ok = re.search(r"if\s*\(\s*!\s*res\.ok\s*\)", body)
    js = re.search(r"await\s+res\.json\(\)", body)
    if not ok:
        return ["거부(비 2xx)를 거르는 if (!res.ok) 가 없다"]
    if not js or ok.start() > js.start():
        return ["if (!res.ok) 가 res.json() 보다 앞에 있지 않다 — 거부 응답을 상태로 읽는다"]
    return []


# ---- 실제 소스 ----

def test_G1_tablet_badge_역참조는_가드_안에_있다():
    assert _problems_guard(_html()) == []


def test_G2_정적_배지를_API_값으로_다시_그린다():
    assert _problems_badges(_html()) == []


def test_G3_보기_전용_출처에서_고장_토글을_숨긴다():
    assert _problems_toggle_hide(_html()) == []


def test_G4_고장_토글_거부를_상태로_읽지_않는다():
    assert _problems_toggle_reject(_html()) == []


# ---- 양성 대조군: 같은 검사가 변이된 소스에서는 실패해야 한다 ----

def _mutate_after(text, anchor, old, new):
    """anchor 이후 처음 나오는 old 만 바꾼다 — 파일 앞쪽의 같은 문구를 건드려 변이가 빗나가는 것을 막는다."""
    i = text.find(anchor)
    assert i >= 0, "변이 앵커를 못 찾았다: %s" % anchor
    j = text.find(old, i)
    assert j >= 0, "앵커 뒤에 %s 가 없다" % old
    return text[:j] + new + text[j + len(old):]


def test_대조군_가드를_지우면_G1_이_잡는다():
    text = _html()
    mutated = _mutate_after(text, "document.getElementById('tablet-badge')", "if (badge)", "if (true)")
    assert mutated != text, "변이가 적용되지 않았다 — 앵커가 바뀌었다"
    # 'if (true)' 는 변수 가드가 아니므로 문제가 나와야 한다
    assert _problems_guard(mutated), "가드를 지웠는데 G1 이 통과했다 — 검사가 눈먼 것이다"


def test_대조군_배지_매핑을_지우면_G2_가_잡는다():
    text = _html()
    mutated = text.replace("OBSERVED_SHARED_PLANE", "X_SHARED_PLANE")
    assert mutated != text
    assert _problems_badges(mutated)


def test_대조군_숨김을_지우면_G3_가_잡는다():
    text = _html()
    mutated = text.replace("fb.hidden = (data.view_only === true);", "")
    assert mutated != text
    assert _problems_toggle_hide(mutated)


def test_대조군_거부_가드를_지우면_G4_가_잡는다():
    text = _html()
    mutated = _mutate_after(text, "async function toggleFaultPoseFix", "if (!res.ok)", "if (false)")
    assert mutated != text
    assert _problems_toggle_reject(mutated)
