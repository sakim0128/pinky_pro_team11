# -*- coding: utf-8 -*-
"""MCV-2G — 화각이 쓸 만한가를 **거치 시점에** 말한다.

⭐⭐ 이 유닛이 생긴 이유는 라이브에서 겪은 일이다: 네 모서리 중 **둘이 화면
   가장자리에 붙어** 흔들림 조각을 못 떴다. 감시가 절반만 돌고 있었는데 화면
   어디에도 그 말이 없었다.

여기서 고정하는 성질:

    끌어온다   금지 띠의 폭은 `drift.PATCH/2` 다. 베끼면 한쪽만 바뀐다
    같은 식    가이드의 판정과 `take_patches` 의 조건이 **같아야** 한다
    안 지어낸다 수평계는 앱이 보낼 때만. 영상으로 추정해 관측 자리에 놓지 않는다
    잴 수 있나  1픽셀이 문턱보다 크면 그 문턱은 못 재는 양이다
"""
import json

import numpy
import pytest

import calibration as C
import drift as D
import framing as F

ARENA = {"version": "t1", "widthCm": 270.0, "heightCm": 125.0, "landmarks": {}}
W, H = 1280, 720
FRAME = (W, H)
GOOD = [(200.0, 620.0), (1080.0, 620.0), (900.0, 200.0), (380.0, 200.0)]
# 라이브에서 실제로 나온 자리 — 아래 둘이 가장자리에 붙어 조각을 못 떴다
LIVE_BAD = [(38.4, 662.4), (1241.6, 662.4), (934.4, 115.2), (345.6, 115.2)]


@pytest.fixture
def arena(tmp_path):
    p = tmp_path / "arena.json"
    p.write_text(json.dumps(ARENA), encoding="utf-8")
    return C.load_arena(str(p))


def matrix_for(corners, arena):
    t = arena.corner_targets()
    return C.solve_homography(corners, [t[c] for c in C.CORNERS])


# ---- ⭐ 금지 띠를 끌어온다 ---------------------------------------------------------

def test_금지_띠는_조각_반쪽이다():
    """⭐ 베끼면 조각 크기를 바꿨을 때 가이드가 틀린 말을 한다."""
    assert F.EDGE_BAND_PX == D.PATCH // 2


def test_띠가_어디서_왔는지_산출물에_적는다():
    got = F.assess(FRAME)
    assert "drift.PATCH" in got["edgeBandSource"]
    assert got["edgeBandPx"] == D.PATCH // 2


def test_권장_상자가_안전_상자보다_안쪽이다():
    safe, target = F.guide_box(FRAME)
    assert target["x"] > safe["x"] and target["y"] > safe["y"]
    assert target["w"] < safe["w"] and target["h"] < safe["h"]


def test_상자가_화면_비율이다():
    """캔버스는 줄어든다. 픽셀로 주면 화면이 다른 자리에 그린다."""
    safe, target = F.guide_box(FRAME)
    for b in (safe, target):
        assert 0 <= b["x"] < 1 and 0 <= b["y"] < 1
        assert 0 < b["w"] <= 1 and 0 < b["h"] <= 1


# ---- ⭐⭐ 가이드와 조각 뜨기가 같은 식이어야 한다 -------------------------------------------

def test_가이드가_괜찮다면_조각도_떠진다():
    """⭐⭐ 다르면 화면은 괜찮다는데 조각은 안 떠진다 — 가장 나쁜 종류의 불일치다.

    무늬가 있는 화면을 써서 `MIN_PATCH_STD` 가 아니라 **자리** 때문에 실패하는지 본다.
    """
    rng = numpy.random.default_rng(7)
    img = numpy.repeat(numpy.repeat(
        rng.integers(0, 255, (H // 8, W // 8), dtype=numpy.uint8), 8, 0), 8, 1)
    frame = numpy.dstack([img[:H, :W]] * 3)
    assert F.corner_problems(GOOD, FRAME) == []
    assert all(r is not None for r in D.take_patches(frame, GOOD))


def test_라이브에서_못_뜬_그_자리를_잡는다():
    """⭐ 실제로 겪은 자리다. 아래 두 모서리가 가장자리에 붙어 있었다."""
    probs = F.corner_problems(LIVE_BAD, FRAME)
    assert [i for i, _w in probs] == [1, 2], probs
    assert "조각을 못 뜬다" in probs[0][1]


def test_문제를_사람이_읽는_말로_낸다():
    got = F.assess(FRAME, corners_px=LIVE_BAD)
    assert got["problems"], "가장자리에 붙었는데 아무 말도 안 한다"
    assert any("1번 모서리" in p for p in got["problems"])


# ---- ⭐ 잴 수 있는 문턱인가 --------------------------------------------------------

def test_잴_수_있으면_그렇다고_한다(arena):
    m = matrix_for(GOOD, arena)
    pr = F.precision(m, GOOD)
    assert pr["thresholdMeasurable"] is True, pr
    assert pr["driftThresholdCm"] == D.DRIFT_CM


def test_너무_멀면_문턱을_못_잰다고_말한다(arena):
    """⭐⭐ 1픽셀이 문턱보다 크면 2cm 흔들림은 **1픽셀도 못 움직인 것**으로 보인다.

    그 상태로 감시를 돌리면 도는 **척**만 한다. 그걸 말해 주는 게 이 유닛의 값어치다.
    """
    # 아레나가 화면의 작은 조각만 차지한다 = 1픽셀이 많은 cm
    tiny = [(600.0, 380.0), (680.0, 380.0), (672.0, 340.0), (608.0, 340.0)]
    m = matrix_for(tiny, arena)
    pr = F.precision(m, tiny)
    assert pr["thresholdMeasurable"] is False, pr
    got = F.assess(FRAME, corners_px=tiny, matrix=m)
    assert any("구분 못 한다" in p for p in got["problems"]), got["problems"]
    assert pr["minThresholdPx"] == F.MIN_THRESHOLD_PX


def test_먼_모서리가_더_나쁘다(arena):
    """원근이다 — 같은 1픽셀이 멀리서는 더 큰 cm 다."""
    m = matrix_for(GOOD, arena)
    near = F.cm_per_pixel(m, GOOD[0])
    far = F.cm_per_pixel(m, GOOD[2])
    assert far > near


# ---- 화면 점유 ----------------------------------------------------------------

def test_점유율을_낸다():
    cov = F.coverage(GOOD, FRAME)
    assert 0.2 < cov < 0.6, cov


def test_작으면_말해_준다(arena):
    tiny = [(600.0, 380.0), (680.0, 380.0), (672.0, 340.0), (608.0, 340.0)]
    got = F.assess(FRAME, corners_px=tiny)
    assert any("밖에 안 된다" in p for p in got["problems"])


def test_모서리가_없으면_점유율을_지어내지_않는다():
    got = F.assess(FRAME)
    assert got["coverage"] is None
    assert got["precision"] is None
    assert got["cornerProblems"] is None


# ---- ⭐ 수평계는 앱이 보낼 때만 -------------------------------------------------------

def test_앱이_안_보내면_수평을_지어내지_않는다():
    """⭐ 영상으로 기울기를 추정할 수야 있지만 추정을 관측 자리에 놓지 않는다(금지 6)."""
    assert F.level_from_sidecar({}) is None
    assert F.level_from_sidecar(None) is None
    assert F.assess(FRAME)["level"] is None


def test_보내면_읽고_기울면_말한다():
    lv = F.level_from_sidecar({"cameraPitch": 0.5, "cameraRoll": -1.0})
    assert lv["tilted"] is False
    lv = F.level_from_sidecar({"cameraPitch": 9.0, "cameraRoll": 0.0})
    assert lv["tilted"] is True
    got = F.assess(FRAME, sidecar={"cameraPitch": 9.0, "cameraRoll": 0.0})
    assert any("기울었다" in p for p in got["problems"])


def test_한쪽만_보내도_읽는다():
    lv = F.level_from_sidecar({"cameraRoll": 0.2})
    assert lv is not None and lv["pitchDeg"] is None


# ---- 입력 --------------------------------------------------------------------

def test_프레임_크기를_모르면_거부한다():
    """⭐ 0 으로 나눈 값을 화면에 그리느니 거부한다."""
    with pytest.raises(ValueError):
        F.guide_box((0, 0))


def test_문턱_최소값이_알고리즘_낟알에서_왔다():
    """⭐⭐ `cv2.minMaxLoc` 은 **정수 픽셀**을 준다 — ±1px 는 방법 자체의 양자화다.

    1px 만 넘으면 된다고 두면 잡음 한 칸이 곧 '흔들렸다' 가 된다. 튜닝이 아니라
    출력의 낟알에서 끌어온 수여야 근거가 있다(MCVA-75 의 정신).
    """
    assert F.MIN_THRESHOLD_PX >= 2.0
    assert "정수 픽셀" in F.precision(
        [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [(10.0, 10.0)])["minThresholdSource"]
