# -*- coding: utf-8 -*-
"""MCV-2A-UI — 핀 튜닝 화면이 되돌아가면 안 되는 성질.

브라우저 없이 JS 를 돌릴 수는 없다. 그래서 **틀리면 조용한 것**만 고정한다.

    좌표계   캔버스 좌표를 그대로 보내면 서버는 다른 사각형을 본다 (화면은 줄어든다)
    표현     "정확도를 안 쟀다" 와 "정확도가 좋다" 가 같은 색이면 거짓말이다
    짝       서버가 내는 상태·사유 코드에 UI 문구가 다 있어야 배지가 안 언다
    정지     움직이는 스트림 위에는 핀을 못 찍는다
"""
import os
import re

import pytest

import calibration as C
import drift as DR

_HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(os.path.dirname(_HERE), "gateway_web", "static", "index.html")

# ⚠️ 이 목록은 "JS 에 박혀 있으면 안 되는 id" 다(부정 목록). 소스가 늘면 같이 늘려야
#    가드가 새 id 를 덮는다. 2026-09-14: tablet-relay -> phone2 로 개명(기기 교체).
#    옛 id 도 남겨 둔다 — 되살아나 JS 에 박히는 것도 막을 값이다.
SOURCE_IDS = ("phone", "phone2", "tablet-relay", "relay-cam", "gazebo", "control",
              "pinky1", "pinky2")

SETTLED_GREEN = "#238636"
# 화면이 "흔들림 없음" 으로 쓰는 말. 감시가 안 돌 때 이 말이 나오면 안 된다.
CAL_DRIFT_OK_TEXT = "흔들림 없음"


def _html():
    if not os.path.exists(HTML):
        pytest.skip("index.html 없음")
    with open(HTML, encoding="utf-8") as fh:
        return fh.read()


def _js():
    t = _html()
    start = t.find("// ===== MCV-2A-UI 좌표계 정착 =====")
    end = t.find("// --- 관제 카메라 다중 소스 및 AR", start + 10)
    if start < 0 or end < 0:
        pytest.fail("캘리브레이션 JS 블록을 못 찾았다")
    return t[start:end]


# ---- 존재 --------------------------------------------------------------------

def test_카드와_캔버스와_미리보기가_있다():
    t = _html()
    for el in ('id="cal-card"', 'id="cal-canvas"', 'id="cal-preview"',
               'id="cal-state"', 'id="cal-acc"'):
        assert el in t, "%s 가 없다" % el


def test_정착_버튼이_있다():
    assert "calSettle" in _html()
    assert "/api/calibration/settle" in _js()


# ---- ⭐ 좌표계 ----------------------------------------------------------------

def test_핀을_원본_프레임_좌표로_들고_있는다():
    """캔버스는 화면 폭에 맞춰 줄어든다. 캔버스 좌표를 보내면 **서버가 다른 사각형을 본다.**

    화면에서는 멀쩡해 보이고, 화면 밖/최소넓이 검사만 이상하게 걸린다 — 조용한 종류의 오류다.
    """
    js = _js()
    assert "calToFrame" in js, "화면 좌표 -> 프레임 좌표 변환이 없다"
    assert re.search(r"calPins\[calDrag\]\s*=\s*calToFrame\(", js), \
        "핀을 옮길 때 프레임 좌표로 환산하지 않는다"
    assert re.search(r"px:\s*calToFrame\(", js), \
        "검증점도 프레임 좌표여야 한다"


def test_화면_크기가_바뀌면_다시_그린다():
    js = _js()
    assert "calResize" in js
    assert "'resize'" in js or '"resize"' in js


def test_엇갈린_사각형을_화면에서도_경고한다():
    """서버가 막기는 한다. 그래도 왜 안 되는지 그 자리에서 보여야 사람이 고친다."""
    js = _js()
    assert "calConvex" in js
    assert "뒤집" in js, "엇갈렸을 때 무슨 일이 나는지 말해 주지 않는다"


# ---- ⭐ 정직한 표현 ------------------------------------------------------------

def test_정확도_미측정을_좋은_것처럼_칠하지_않는다():
    """네 점의 오차는 구조상 0 이다. 검증점이 없으면 **아무것도 안 잰 것**이다."""
    js = _js()
    assert "정확도 미측정" in js
    m = re.search(r"if\s*\(!st\.accuracyMeasured\)\s*\{(.{0,220}?)\}", js, re.S)
    assert m, "미측정 분기를 못 찾았다"
    assert SETTLED_GREEN not in m.group(1), \
        "정확도를 안 쟀는데 정착과 같은 초록으로 칠하고 있다"


def test_왜곡보정을_안_했다고_화면에_적었다():
    """광각이면 가장자리가 휜다. 안 한 것을 안 했다고 말해야 한다(V-2: 3이 4보다 먼저)."""
    t = _html()
    assert "왜곡 보정" in t


def test_검증점이_풀이에_안_쓰인다고_적었다():
    t = _html()
    assert "풀이에 안 쓰는" in t or "풀이에 안 쓴" in t


# ---- ⭐ 서버 코드와 UI 문구의 짝 ---------------------------------------------------

def _table_keys(js, table_name):
    """`NAME = { KEY: ... }` 의 키들. ⭐ 부분문자열로 보면 DRIFTX 가 DRIFT 를 통과시킨다."""
    i = js.find(table_name + " = {")
    assert i >= 0, "%s 표가 없다" % table_name
    depth, j = 0, js.find("{", i)
    end = j
    for k in range(j, len(js)):
        if js[k] == "{":
            depth += 1
        elif js[k] == "}":
            depth -= 1
            if depth == 0:
                end = k
                break
    body = js[j:end]
    return set(re.findall(r"(?m)^\s*([A-Z_]+)\s*:", body))


def _constants(module, prefix):
    """그 모듈이 **지금** 내는 코드들. ⭐ 목록을 손으로 베끼지 않는다.

    이 시험이 사유 세 개를 박아 뒀던 탓에 서버에 `LENS_CHANGED` 가 늘어도
    초록이었다 — 배지는 그동안 코드를 날것으로 보여 주고 있었다.
    상수에서 끌어오면 **서버가 늘어나는 순간 여기가 빨개진다.**
    """
    out = {v for k, v in vars(module).items()
           if k.startswith(prefix) and isinstance(v, str)}
    assert out, "%s 에 %s* 상수가 없다" % (module.__name__, prefix)
    return out


def test_서버가_내는_상태에_UI_문구가_다_있다():
    """짝이 어긋나면 배지가 '?' 로 언다. 실제로 멀티뷰에서 같은 종류의 버그를 냈다.

    ⭐ 키를 **정확히** 본다. `DRIFT in js` 는 `DRIFTX` 로 오타 나도 통과한다 —
       이 프로젝트에서 좁은 문자열 검사로 구조를 판정해 여러 번 틀렸다.
    """
    keys = _table_keys(_js(), "CAL_STATE_TEXT")
    for state in _constants(C, "STATE_"):
        assert state in keys, "UI 가 %s 상태를 모른다 (있는 것: %s)" % (state, sorted(keys))


def test_서버가_내는_무효_사유에_UI_문구가_다_있다():
    """⭐ 목록을 서버에서 끌어온다. 사유가 늘면 이 시험이 먼저 빨개진다."""
    keys = _table_keys(_js(), "CAL_REASON_TEXT")
    reasons = _constants(C, "REASON_") | {DR.REASON_CAMERA_MOVED}
    for reason in reasons:
        assert reason in keys, \
            "UI 가 무효 사유 %s 를 모른다 (있는 것: %s)" % (reason, sorted(keys))


# ---- ⭐ 흔들림이 화면에 보이는가 (MCV-2A4) --------------------------------------------
#
# 안 불리는 모듈이 기능이 아니듯 **안 그려지는 상태도 감시가 아니다.**
# 서버는 `/api/calibration` 에 drift 를 싣고 있었고 화면은 그걸 버렸다 —
# 삼각대를 쳐도 배지는 '정착됨' 그대로였을 것이다.

def test_흔들림_배지와_설명칸이_있다():
    t = _html()
    for el in ('id="cal-drift"', 'id="cal-drift-note"'):
        assert el in t, "%s 가 없다" % el


def test_상태에서_흔들림을_꺼내_그린다():
    js = _js()
    assert "calDrawDrift(st.drift)" in js, "서버가 준 drift 를 화면이 안 읽는다"
    assert "function calDrawDrift" in js


def test_드리프트_상태에_UI_문구가_다_있다():
    keys = _table_keys(_js(), "CAL_DRIFT_TEXT")
    for state in _constants(DR, "STATE_"):
        assert state in keys, "UI 가 흔들림 상태 %s 를 모른다" % state


def test_드리프트_사유에_UI_문구가_다_있다():
    """why 는 '왜 그렇게 봤나' 다. 코드를 날것으로 보여 주면 사람이 못 읽는다."""
    keys = _table_keys(_js(), "CAL_DRIFT_WHY")
    for why in _constants(DR, "WHY_"):
        assert why in keys, "UI 가 흔들림 사유 %s 를 모른다 (있는 것: %s)" % (why, sorted(keys))


def test_모른다를_초록으로_칠하지_않는다():
    """⭐⭐ 기준이 없다는 것은 **안 움직였다는 뜻이 아니다.**

    정확도 미측정을 초록으로 칠하지 않는 것과 같은 성질이다. 이걸 초록으로 두면
    감시가 안 도는 상태와 통과한 상태가 화면에서 구분이 안 된다.
    """
    js = _js()
    i = js.find("CAL_DRIFT_TEXT = {")
    body = js[i:js.find("}", i)]
    line = [l for l in body.split(chr(10)) if DR.STATE_UNKNOWN + ":" in l]
    assert line, "UNKNOWN 줄을 못 찾았다"
    assert SETTLED_GREEN not in line[0], "흔들림을 모르는데 통과와 같은 초록이다"


def _no_comments(js):
    """줄 주석을 걷어낸다.

    ⭐ 처음 쓴 이 시험은 **주석에 적힌 말**을 잡고 실패했다 - 코드는 옳았다.
       이 프로젝트에서 좁은 문자열 검사로 구조를 판정해 여러 번 틀린 바로 그 자리다.
       무엇을 보는지 먼저 좁힌다.
    """
    out = []
    for line in js.split(chr(10)):
        k = line.find("//")
        out.append(line[:k] if k >= 0 else line)
    return chr(10).join(out)


def test_감시가_안_돌_때_흔들림_없음이라고_안_한다():
    """⭐ drift 가 아예 없는 것(기준 미저장)과 '안 움직였다' 는 다른 사실이다."""
    js = _no_comments(_js())
    i = js.find("function calDrawDrift")
    j = js.find("if (!d) {", i)
    assert j > 0, "drift 가 없을 때의 가지를 못 찾았다"
    branch = js[j:js.find("return;", j)]
    assert CAL_DRIFT_OK_TEXT not in branch, "감시가 안 도는데 '흔들림 없음' 이라고 적는다"
    assert "미감시" in branch, "감시가 안 돈다는 것을 말하지 않는다"


def test_못_잰_모서리를_0_으로_적지_않는다():
    js = _js()
    assert "못 쟀다" in js, "기준이 없는 모서리를 숫자로 채우고 있다"


# ---- ⭐ 영상 -> 평면도가 화면에서 불리는가 (MCV-2V) ------------------------------------

def test_영상월드_버튼이_있고_그_엔드포인트를_친다():
    t = _html()
    assert 'id="cal-vision-btn"' in t
    assert "calVisionBuild" in t
    assert "'/api/vision/world'" in _js(), "버튼이 있는데 아무 데도 안 친다"


def test_평면도가_라이다에서_안_왔다고_적었다():
    """⭐ 이게 이 산출물의 요점이다 — 라이다의 파생물이 아니라서 라이다를 검증한다."""
    t = _html()
    assert "라이다를 안 쓴다" in t


def test_평면도의_한계를_같이_보여준다():
    """⭐⭐ 한계를 안 적으면 사람이 이 격자를 **사실**로 읽는다.

    발자국은 실루엣이라 점유의 **상한**이고, 그림자는 카메라 높이를 몰라서
    안쪽을 과대추정한다. 산출물에 이미 적혀 있는 것을 화면이 버리면 안 된다.
    """
    js = _js()
    for key in ("footprintMethod", "shadowModel", "shadowCaveat",
                "cameraGroundPointIs"):
        assert key in js, "평면도의 한계 %s 를 화면이 안 보여준다" % key


def test_말이_안_되는_판을_말한다():
    js = _js()
    assert "sanity" in js


def test_모서리_순서를_화면에_적었다():
    """bl/br/tr/tl 은 코드다. 사람에겐 순서를 말해 줘야 엇갈리지 않는다."""
    t = _html()
    assert "좌하" in t and "우하" in t and "우상" in t and "좌상" in t


# ---- 정지 프레임 ---------------------------------------------------------------

def test_움직이는_스트림이_아니라_정지_프레임에_찍는다():
    js = _js()
    assert "/api/calibration/still" in js
    assert "/video_feed" not in js, "움직이는 화면 위에는 핀을 못 찍는다"


def test_관측_경로를_오염시키지_않는다():
    """boost 는 채도를 1.8배 올려 ArUco 대비를 망친다(V-2)."""
    js = _js()
    assert "boost" not in js
    assert "watermark" not in js


# ---- 소스 id 하드코딩 금지 --------------------------------------------------------

def test_캘리브레이션_JS_에_소스_id_가_박혀_있지_않다():
    """2026-09-10 의 사고. id 를 박으면 새 소스가 조용히 빠진다."""
    js = _js()
    for sid in SOURCE_IDS:
        for quote in ("'", '"'):
            assert (quote + sid + quote) not in js, \
                "캘리브레이션이 소스 id %s 를 박아 뒀다" % sid


def test_성질로_거른다():
    js = _js()
    assert "CAL_REAL_TRANSPORTS" in js
    assert "'pull'" in js and "'local'" in js
    assert "trusted" in js


# ---- ⭐ 관측 제외 영역이 화면에서 다뤄지는가 (MCV-2M) -------------------------------------

def test_마스크_칸이_있다():
    t = _html()
    for el in ('id="cal-mask-why"', 'id="cal-mask-btn"', 'id="cal-mask-list"'):
        assert el in t, "%s 가 없다" % el


def test_마스크를_그_엔드포인트에_저장한다():
    assert "'/api/calibration/masks'" in _js()


def test_가린_자리가_모른다라고_화면에_적었다():
    """⭐ 이게 이 기능의 안전 성질이다 — 비었다로 읽으면 로봇이 삼각대로 들어간다."""
    t = _html()
    assert "모른다" in t and "비었다가 아니다" in t


def test_라벨_없이는_못_만든다():
    """⭐ 서버도 막지만, 사람이 그린 **직후**에 말해야 고친다."""
    js = _no_comments(_js())
    i = js.find("function calMaskFinish")
    body = js[i:js.find("function calMaskDelete", i)]
    assert "무엇을 가리는지 먼저 적는다" in body
    assert "return" in body


def test_핀이_없어도_마스크를_그린다():
    """⭐ `calDraw` 는 핀이 없으면 일찍 반환한다. 삼각대는 핀보다 **먼저** 보인다 —
       정착 전에 그린 마스크가 안 보이면 사람은 저장이 안 된 줄 안다."""
    js = _no_comments(_js())
    i = js.find("function calDraw()")
    body = js[i:i + 700]
    call = body.find("calDrawMasks(ctx)")
    early = body.find("if (!calPins) return;")
    assert call > 0 and early > 0, "둘 중 하나를 못 찾았다"
    assert call < early, "마스크를 일찍 반환한 뒤에 그린다 — 핀 없으면 안 보인다"


def test_거부되면_화면을_되돌린다():
    """⭐ 화면에만 남아 있으면 사람은 **가려진 줄 안다.** 안 가려졌는데."""
    js = _no_comments(_js())
    i = js.find("async function calMaskSave")
    body = js[i:js.find("function calDrawMaskList", i)]
    assert "calSync()" in body, "거부된 뒤 서버 상태로 안 되돌린다"


def test_못_읽은_마스크를_없다로_접지_않는다():
    js = _js()
    assert "masksProblem" in js, "못 읽었다는 사실을 화면이 버린다"


def test_마스크와_검열이_눈에_다르다():
    """주인이 다르면 눈에도 달라야 한다 — 운영자가 지울 수 있는 것과 아닌 것이다."""
    js = _js()
    assert "#d29922" in js, "검열 상자 색이 없다"
    assert "#8b949e" in js or "139,148,158" in js, "마스크 색이 검열과 같다"


# ---- ⭐ 화각 점검이 화면에 있는가 (MCV-2G) -----------------------------------------------

def test_화각_점검_칸이_있다():
    t = _html()
    for el in ('id="cal-framing-badge"', 'id="cal-framing-note"', 'id="cal-guide-on"'):
        assert el in t, "%s 가 없다" % el
    assert "'/api/calibration/framing?src='" in _js()


def test_가이드를_핀보다_먼저_그린다():
    """⭐ 거치는 정착 **전**에 한다. 핀이 없으면 안 보이는 가이드는 쓸모가 없다."""
    js = _no_comments(_js())
    i = js.find("function calDraw()")
    body = js[i:i + 800]
    call, early = body.find("calDrawGuide(ctx"), body.find("if (!calPins) return;")
    assert call > 0 and early > 0
    assert call < early, "가이드를 일찍 반환한 뒤에 그린다"


def test_모르는_항목을_빈칸으로_두지_않는다():
    """⭐ 빈칸은 사람이 0 으로 읽는다. '아직 모른다' 라고 적는다."""
    js = _js()
    assert "아직 모른다" in js


def test_수평을_영상으로_추정하지_않는다고_적었다():
    """⭐ 금지 조항 6 — 추정을 관측 자리에 놓지 않는다."""
    js = _js()
    assert "앱이 안 보낸다" in js
    assert "추정" in js


def test_문턱을_잴_수_있는지_말한다():
    """⭐⭐ 1픽셀이 문턱보다 크면 감시는 도는 **척**만 한다."""
    js = _js()
    assert "thresholdMeasurable" in js
    assert "못 잰다" in js


def test_금지_띠가_어디서_왔는지_보여준다():
    """상수를 화면에 박지 않는다 — 서버가 출처를 같이 준다."""
    assert "edgeBandSource" in _js()
