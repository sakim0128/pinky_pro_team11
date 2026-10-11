# -*- coding: utf-8 -*-
"""설계서 2026-09-12 (`DESIGN_20260912_GATEWAY_UI_REBUILD.md`) 의 화면 수락 기준.

사용자가 화면을 열었더니 영상 카드 둘이 검고 `지연 15~19초` 였다. 그 수가 맞다면
시스템이 못 쓸 물건이고, 틀렸다면 화면이 거짓말을 하고 있다. **후자였다.**

여기서 고정하는 것:

    U-1  어긋남과 지연이 **다른 칸**이다 (시계를 틀면 어긋남만 움직인다)
    U-2  불가능한 값은 **숫자 대신 사유**
    U-3  기본은 **닫힘**. 넷을 동시에 열면 무너진다
    U-4  도메인을 화면에 **박지 않는다**
    U-6  닫힌 것과 죽은 것을 **분모에서** 가른다
    U-8  빈 상태가 **왜** 비었는지 말한다
"""
import io
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(os.path.dirname(_HERE), "gateway_web", "static", "index.html")


def _html():
    # ⭐⭐ 예전엔 `pytest.skip` 이었다. 그런데 **skip 은 통과가 아닌데 exit=0 이라
    #    통과처럼 보인다** — 2026-09-12 에 이 레포에서 하루 세 번 밟은 함정이다.
    #    이 파일은 레포에 항상 있다. 없으면 그건 시험의 전제가 깨진 것이라 실패다.
    if not os.path.exists(HTML):
        pytest.fail("index.html 이 없다 — 이 시험의 대상이 사라졌다: %s" % HTML)
    with io.open(HTML, encoding="utf-8") as fh:
        return fh.read()


def _block(text, start, end):
    i = text.find(start)
    assert i > 0, "%s 를 못 찾았다" % start
    j = text.find(end, i + 1)
    assert j > i, "%s 를 못 찾았다" % end
    return text[i:j]


def _mv():
    """멀티뷰 JS 만 — **상수 정의부터**.

    ⭐ 처음엔 `function mvFmt` 부터 잘랐는데 `MV_OPEN_SOFT_CAP` 의 실측 주석이 그
       앞에 있어서 안 보였다. 창을 좁게 잡으면 '근거가 없다' 는 거짓 실패가 나고,
       넓게 잡으면 옆 화면을 삼킨다. 이 프로젝트가 양쪽으로 다 밟은 자리다.
    """
    return _block(_html(), "const MV_OFFSET_WARN_MS", "// ===== MCV-2A-UI")


# ---- ⭐⭐ U-1: 어긋남과 지연은 다른 칸 -------------------------------------------------

def test_지연이라는_이름으로_어긋남을_안_쓴다():
    """⭐⭐ 이게 이 라운드의 출발점이다.

    `'지연 ' + mvFmt(off, ...)` — `off` 는 `now - captureClock` 이라 **시계차가
    통째로 들어 있다.** 로컬 웹캠에 `지연 18837.8ms` 가 뜬 이유다.
    """
    js = _mv()
    assert not re.search(r"'지연 '\s*\+\s*mvFmt\(\s*off\b", js), \
        "어긋남(off)을 '지연' 이라고 부르고 있다"


def test_어긋남_칸과_지연_칸이_따로_있다():
    js = _mv()
    assert "'어긋남 '" in js, "어긋남 칸이 없다"
    assert "'지연 '" in js, "지연 칸이 없다"


def test_서버가_계산한_timing_을_쓴다():
    """⭐ 화면이 다시 계산하면 서버와 규칙이 갈린다. 서버 값을 그대로 쓴다."""
    js = _mv()
    assert "mvClockBadge(s.clock, s.timing)" in js, "서버의 timing 을 안 넘긴다"
    assert "t.pipelineMs" in js and "t.offsetMs" in js


def test_어긋남에_무엇이_섞였는지_화면이_말한다():
    """이름만 바꾸고 뜻을 안 적으면 다음 사람이 또 지연으로 읽는다."""
    js = _mv()
    assert "offsetIncludes" in js
    assert "pipelineBasis" in js


# ---- ⭐ U-2: 불가능한 값은 사유로 ----------------------------------------------------

def test_못_잰_값에_숫자_대신_사유를_쓴다():
    js = _mv()
    assert "mvWhy(" in js, "사유를 사람 말로 옮기는 곳이 없다"
    for code in ("NEGATIVE", "IMPLAUSIBLE_FOR_LOCAL", "NO_TIMING_HEADERS",
                 "NO_COMMON_CLOCK"):
        assert code in js, "사유 %s 에 화면 문구가 없다" % code


def test_사유_코드를_서버와_맞춘다():
    """⭐ 코드는 서버가, 말은 화면이 소유한다. 짝이 어긋나면 코드가 날것으로 뜬다."""
    import timing as T
    js = _mv()
    for name in dir(T):
        if name.startswith("WHY_"):
            assert getattr(T, name) in js, "화면이 사유 %s 를 모른다" % name


# ---- ⭐ U-3: 기본은 닫힘 ------------------------------------------------------------

def test_기본은_전부_닫힘이다():
    """⭐⭐ 실측: 1개 10.5 fps · 2개 10.8+8.9 · 3개 6.7+7.8+5.7 · **4개 붕괴**.
       예전 화면은 기본으로 넷을 열었다 — 그게 카드가 검은 진짜 이유다."""
    js = _mv()
    assert "mvOpen[s.id] = false;" in js, "기본 열림이 아직 있다"
    assert "MV_DEFAULT_OPEN_TRANSPORTS.indexOf" not in js, \
        "transport 로 기본 열림을 정하는 코드가 남아 있다"


def test_동시_열림_상한을_말한다():
    js = _mv()
    assert "MV_OPEN_SOFT_CAP" in js
    assert "권장" in js, "상한을 넘겨도 아무 말을 안 한다"


def test_닫힌_카드는_정지_프레임이다():
    """⭐ 스트림이 아니라 단발 요청이라 경합에 안 낀다. 빈 검은 칸이면 죽은 것과 구분이 안 된다."""
    js = _mv()
    assert "/api/calibration/still?src=" in js
    assert "정지 프레임" in js


def test_상한이_실측에서_왔다고_적었다():
    """⭐ 눈대중 값이면 다음 사람이 마음대로 올린다."""
    js = _mv()
    assert "실측" in js and "TimeoutError" in js


# ---- ⭐ U-4: 도메인을 박지 않는다 -----------------------------------------------------

def test_도메인_값이_화면에_박혀_있지_않다():
    """⭐ 도커 88 · 현장 8 인데 화면엔 10 이 박혀 있었다 — 한 화면이 자기 모순이었다."""
    found = re.findall(r"ROS_DOMAIN_ID=\d+", _html())
    assert not found, "도메인 값이 박혀 있다: %s" % sorted(set(found))


def test_도메인을_런타임에서_채운다():
    t = _html()
    assert 'id="hdr-ros-domain"' in t
    assert "ros-domain-live" in t
    assert "d.rosDomainId" in t, "런타임 값을 안 읽는다"


def test_못_읽는_도메인을_실측처럼_안_적는다():
    """⭐ 게이트웨이는 자기 도메인만 읽는다. 로봇2의 도메인은 못 읽는다."""
    t = _html()
    assert "설계값 11" in t, "못 읽는 값을 실측처럼 적고 있다"


# ---- ⭐ U-6: 닫힌 것과 죽은 것 --------------------------------------------------------

def test_닫힌_로컬_캠을_분모에서_뺀다():
    js = _mv()
    assert "const expected = real.filter" in js, "분모를 안 가른다"
    assert "s.transport !== 'local' || mvOpen[s.id]" in js
    assert "닫힘 " in js, "몇 개가 닫혔는지 안 말한다"


def test_분모가_모자라면_빨갛다():
    js = _mv()
    assert "live < expected.length" in js


# ---- ⭐ U-8: 빈 상태에 사유 ---------------------------------------------------------

def test_빈_상태가_서로_다른_문구다():
    """⭐ 소스가 없는 건지, 안 찍은 건지, 핀이 없는 건지 화면에서 갈려야 한다."""
    cal = _block(_html(), "// ===== MCV-2A-UI", "// --- 관제 카메라 다중 소스 및 AR")
    for phrase in ("실물 시점이 없다", "정지 프레임을 안 받았다", "핀이 없다"):
        assert phrase in cal, "빈 상태 문구 '%s' 가 없다" % phrase


# ---- 유실 카운터는 숨기지 않는다 -----------------------------------------------------

def test_유실을_숨기지_않고_뜻을_적는다():
    """설계서 §5: D-2 를 고치면 줄어야 하고, 안 줄면 다른 원인이다."""
    js = _mv()
    assert "유실 " in js
    assert "X-Frame-Seq" in js, "유실이 무엇을 세는지 안 적었다"


# ---- QA Q-1/Q-3 — **안 온 값을 0 으로 떨어뜨리지 않는다** ------------------------------
#
# Q-1 에서 배운 것: `0` 은 "쟀는데 0" 이라는 뜻이라 "안 왔다" 와 구분이 안 된다.
# 그리고 **방향이 문제다.** `0.0cm` 는 "완벽히 일치", `0.00 m` 남은 거리는 "도착했다" 로
# 읽힌다 — 못 쟀다는 사실이 **가장 좋은 소식으로 둔갑**한다.
#
# ⭐ 그래서 인스턴스가 아니라 **모양**을 막는다. Q-1 을 한 줄씩 고쳤으면 Q-3 의 세 자리는
#    그대로 남았을 것이다(실제로 남아 있었다).

FALLBACK = re.compile(r"\|\|\s*[01]\b|:\s*'?0(?:\.0)?'?\s*[,;)]")
SHOWS_NUMBER = ("toFixed(", "textContent")
DENOM_NAME = re.compile(r"\b(?:total|count|denom|전체)\w*\s*=")


def _script_lines():
    """주석을 뺀 페이지 스크립트 줄들 — (줄번호, 내용).

    ⭐ 주석을 안 빼면 "예전엔 `|| 0` 이었다" 는 **설명문이 스스로 걸린다.**
       (하루에 네 번째로 밟은 모양이라 여기서는 처음부터 뺀다.)
    """
    out = []
    for i, line in enumerate(_html().split(chr(10)), 1):
        s = line.strip()
        if s.startswith("//") or s.startswith("*") or s.startswith("<!--"):
            continue
        out.append((i, line))
    return out


ASSIGN = re.compile(r"\b(?:const|let|var)\s+(\w+)\s*=")


def _indent(line):
    return len(line) - len(line.lstrip())


def _fabricated_zero(lines):
    """[(줄번호, 내용)] — 없는 값을 0/1 로 떨어뜨려 **화면에 숫자로 내보내는** 자리.

    ⭐⭐ 같은 줄만 보면 **Q-1 자신을 못 잡는다.** Q-1 은 `const distCm = … : 0.0;` 로
       떨어뜨리고 **몇 줄 뒤에** `distCm.toFixed(1)` 로 찍었다. 그래서 폴백이 변수에
       담기면 그 변수가 곧 표시되는지 앞을 내다본다. 처음 판은 같은 줄만 봐서
       네 모양 중 셋만 잡았다.
    """
    bad = []
    for idx, (i, line) in enumerate(lines):
        if not FALLBACK.search(line):
            continue
        if any(k in line for k in SHOWS_NUMBER) or DENOM_NAME.search(line):
            bad.append((i, line.strip()))
            continue
        m = ASSIGN.search(line)
        if not m:
            continue
        # ⭐ 앞을 **몇 줄** 보느냐는 튜닝 손잡이다. 처음엔 8줄로 뒀는데 Q-1 의 실제
        #    간격이 11줄이라 못 잡았다. 그래서 숫자가 아니라 **구조**로 끊는다 —
        #    들여쓰기가 얕아지는 지점이 그 변수가 사는 블록의 끝이다.
        name, here = m.group(1), _indent(line)
        used = re.compile(r"\b%s\b" % re.escape(name))
        for _j, nxt in lines[idx + 1:]:
            if not nxt.strip():
                continue
            if _indent(nxt) < here:
                break
            if used.search(nxt) and any(k in nxt for k in SHOWS_NUMBER):
                bad.append((i, line.strip()))
                break
    return bad


def test_탐지기가_옛_모양을_잡는다():
    """⭐ 양성 대조군 — 이 넷이 실제로 화면에 있던 줄이다."""
    old = [
        (1, "+ (100 * (res.body.coveredFraction || 0)).toFixed(1) + '%');"),
        (2, "const total = c.cellsTotal || 1;"),
        (3, "remDistEl.textContent = `${(st.remaining_dist || 0).toFixed(2)} m`;"),
        # ⭐ Q-1 의 진짜 모양 — 떨어뜨리는 줄과 찍는 줄이 **떨어져 있다**
        (4, "const distCm = disc.dist_err_cm !== undefined ? disc.dist_err_cm : 0.0;"),
        (5, "const gradeBadge = document.getElementById('discrepancy-grade-badge');"),
        (6, "if (errDistEl) errDistEl.textContent = `${distCm.toFixed(1)} cm`;"),
    ]
    got = _fabricated_zero(old)
    assert [g[0] for g in got] == [1, 2, 3, 4], "옛 모양을 다 못 잡았다: %s" % got


def test_탐지기가_0나눗셈_가드는_안_잡는다():
    """⚠️ 음성 대조군 — 좌표 변환의 분모 보호는 **표시값이 아니다.**

    이걸 같이 막으면 거짓 양성이 생기고, 그러면 사람이 이 검사를 끈다.
    """
    ok = [(1, "function calToFrame(x, y) { const s = calScale() || 1; return [x / s, y / s]; }")]
    assert _fabricated_zero(ok) == []


def test_주석은_안_센다():
    """옛 모양을 설명하는 주석이 스스로 걸리면 영원히 빨갛다."""
    assert _fabricated_zero([(1, "        // 예전 판은 `cellsTotal || 1` 로 분모를 지어냈다")]) == []


def test_볼_줄이_있다():
    lines = _script_lines()
    assert len(lines) > 500, "페이지를 %d 줄밖에 못 읽었다" % len(lines)


def test_안_온_값을_0_으로_표시하지_않는다():
    """🔴 빨개지면 화면이 **못 잰 것을 좋은 소식으로** 바꿔 말하고 있다."""
    bad = _fabricated_zero(_script_lines())
    assert not bad, (
        "없는 값을 0/1 로 떨어뜨려 숫자로 표시한다 — 0 은 '쟀는데 0' 이라는 뜻이다."
        + chr(10) + "안 왔으면 사유를 쓴다(미측정/값 없음/미산출)." + chr(10) + chr(10)
        + chr(10).join("  index.html:%d  %s" % b for b in bad))


def test_못_잰_것들이_사유_문구를_갖는다():
    """반대 방향 — `|| 0` 만 지우고 사유를 안 적으면 `undefined` 가 뜬다."""
    h = _html()
    for phrase in ("미측정", "미산출", "값 없음", "비율 미제공"):
        assert phrase in h, "사유 문구 '%s' 가 없다" % phrase


def test_정착_전을_1970년으로_안_적는다():
    """QA Q-2: `new Date((r.settledAt || 0) * 1000)` 은 **1970-01-01** 을 낸다."""
    h = _html()
    assert "settledAt || 0" not in h, "정착한 적 없음이 '1970년에 정착함' 이 된다"
    assert "정착 전" in h, "정착 전 상태를 뭐라고 적는지가 없다"
