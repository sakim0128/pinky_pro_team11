# -*- coding: utf-8 -*-
"""MCV-2A4 — 정착한 뒤 카메라가 움직였나.

무효화 3종(아레나·크기·세션)은 **이걸 못 잡는다.** 삼각대를 치면 프레임 크기도 그대로,
세션도 그대로인데 세상이 옮겨간다. 화면은 멀쩡하고 좌표만 조용히 틀린다.

여기서 고정하는 성질:

    부분만 본다   화면 전체 유사도로 재면 로봇이 지나가기만 해도 떨어진다
    cm 로 판정    같은 픽셀이라도 멀리 있는 모서리는 훨씬 큰 cm 다
    기준선 필수   문턱을 상수로 안 박는다. 기준선이 없으면 **판정하지 않는다**
"""
import json
import os

import numpy
import pytest

import calibration as C
import drift as D

ARENA = {"version": "t1", "widthCm": 270.0, "heightCm": 125.0, "landmarks": {}}
W, H = 1280, 720
CORNERS_PX = [(200.0, 620.0), (1080.0, 620.0), (900.0, 200.0), (380.0, 200.0)]


@pytest.fixture
def arena(tmp_path):
    p = tmp_path / "arena.json"
    p.write_text(json.dumps(ARENA), encoding="utf-8")
    return C.load_arena(str(p))


@pytest.fixture
def matrix(arena):
    t = arena.corner_targets()
    return C.solve_homography(CORNERS_PX, [t[c] for c in C.CORNERS])


def scene(shift=(0, 0), moving_blob=None, seed=11):
    """구조가 있는 장면. shift 만큼 통째로 밀면 '카메라가 움직였다'."""
    rng = numpy.random.default_rng(seed)
    base = rng.integers(0, 255, (H + 200, W + 200), dtype=numpy.uint8)
    base = numpy.repeat(numpy.repeat(base[::8, ::8], 8, axis=0), 8, axis=1)
    base = base[:H + 200, :W + 200]
    ox, oy = 100 + shift[0], 100 + shift[1]
    img = base[oy:oy + H, ox:ox + W].copy()
    if moving_blob:
        x, y, r = moving_blob
        yy, xx = numpy.ogrid[:H, :W]
        img[((xx - x) ** 2 + (yy - y) ** 2) <= r * r] = 250
    return numpy.dstack([img] * 3)


def anchored(frame, matrix, corners=CORNERS_PX):
    refs = D.take_patches(frame, corners)
    measured = D.measure(frame, refs, corners, matrix)
    baseline = {"scores": [(m["score"] if m else None) for m in measured]}
    return refs, baseline


# ---- 기준선이 없으면 판정하지 않는다 -------------------------------------------------

def test_기준선이_없으면_모른다(matrix):
    """⭐ 모르는 것을 경보로 바꾸지 않는다."""
    frame = scene()
    refs = D.take_patches(frame, CORNERS_PX)
    measured = D.measure(frame, refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, baseline=None)
    assert state == D.STATE_UNKNOWN
    assert detail["why"] == "NO_BASELINE"


def test_잴_수_없으면_모른다(matrix):
    state, detail = D.classify([None, None, None, None], baseline={"scores": [1, 1, 1, 1]})
    assert state == D.STATE_UNKNOWN
    assert detail["why"] == "NO_MEASUREMENT"


def test_기준이_없는_모서리는_None_으로_남긴다(matrix):
    """⭐ 화면 가장자리에 걸린 모서리를 0 으로 채우면 없는 기준을 비교하게 된다."""
    frame = scene()
    edge = [(5.0, 5.0)] + CORNERS_PX[1:]
    refs = D.take_patches(frame, edge)
    assert refs[0] is None
    assert all(r is not None for r in refs[1:])


# ---- 안 움직였으면 통과 -----------------------------------------------------------

def test_그대로면_고정_상태다(matrix):
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    measured = D.measure(frame, refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, baseline)
    assert state == D.STATE_OK, detail
    assert detail["worstShiftCm"] < D.DRIFT_CM


def test_로봇이_지나가도_고정이다(matrix):
    """⭐⭐ 이 시험이 '부분만 본다' 는 설계의 이유다.

    화면 전체 유사도로 재면 여기서 떨어진다. 그러면 사람이 경보를 끄고,
    그 다음 진짜 흔들림도 안 본다.
    """
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    # 아레나 한가운데를 큰 물체가 지나간다
    moved = scene(moving_blob=(640, 420, 90))
    measured = D.measure(moved, refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, baseline)
    assert state == D.STATE_OK, detail


# ---- ⭐ 움직이면 잡는다 -----------------------------------------------------------

def test_카메라가_밀리면_잡는다(matrix):
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    shifted = scene(shift=(24, 16))
    measured = D.measure(shifted, refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, baseline)
    assert state == D.STATE_DRIFT, detail
    assert detail["why"] == "SHIFTED"
    assert detail["worstShiftCm"] >= D.DRIFT_CM


def test_옮겨간_양을_픽셀과_cm_둘_다_낸다(matrix):
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    measured = D.measure(scene(shift=(20, 0)), refs, CORNERS_PX, matrix)
    got = [m for m in measured if m]
    assert got
    for m in got:
        assert "dxPx" in m and "shiftCm" in m
    # 실제로 민 만큼이 나와야 한다 (부호는 조각이 어디서 발견됐나에 따른다)
    assert max(abs(m["dxPx"]) for m in got) >= 12


def test_같은_픽셀이라도_먼_모서리가_더_큰_cm_다(matrix):
    """⭐ 그래서 픽셀이 아니라 cm 로 판정한다. 픽셀로 하면 먼 모서리가 유리해진다."""
    near = D.shift_in_cm(matrix, CORNERS_PX[0], 10, 0)     # 아래쪽(가까움)
    far = D.shift_in_cm(matrix, CORNERS_PX[2], 10, 0)      # 위쪽(멂)
    assert far > near * 1.2, "먼 모서리 %.2f cm vs 가까운 %.2f cm" % (far, near)


def test_한_군데_가려지면_움직였다고_단정하지_않는다(matrix):
    """⭐ 조각 **하나**를 못 찾는 것은 '움직였다' 가 아니라 '모른다' 다.

    로봇이 모서리에 서 있으면 그 조각만 사라진다. 국소 가림이다.
    """
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    hidden = frame.copy()
    x, y = CORNERS_PX[0]
    hidden[int(y) - 70:int(y) + 70, int(x) - 70:int(x) + 70] = 120
    measured = D.measure(hidden, refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, baseline)
    assert state == D.STATE_UNKNOWN, detail
    assert detail["why"] == D.WHY_MATCH_LOST
    assert detail["lostCorners"] == 1


def test_전부_가려지면_정착을_유지하지_않는다(matrix):
    """⭐⭐ **몇 개를 놓쳤나로 뜻이 갈린다.**

    전부 놓쳤다는 것은 국소가 아니다 — 세상이 통째로 바뀐 것이다(크게 움직였거나,
    조명이 꺼졌거나, 렌즈가 가려졌거나). 어느 쪽인지는 **모른다.** 그래도 그 행렬이
    맞는지 확인할 방법이 없는데 맞다고 계속 말하면 안 된다(fail-closed).
    """
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    blank = numpy.full((H, W, 3), 120, dtype=numpy.uint8)
    measured = D.measure(blank, refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, baseline)
    assert state == D.STATE_DRIFT, detail
    assert detail["why"] == D.WHY_ALL_LOST


def test_전부_놓친_것을_움직였다고_부르지_않는다():
    """⭐ 모르는 것을 아는 것처럼 말하지 않는다 — 사유 코드가 따로다."""
    assert D.REASON_UNVERIFIABLE != D.REASON_CAMERA_MOVED


# ---- 문턱 -------------------------------------------------------------------

def test_문턱이_cm_단위로_드러난다(matrix):
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    measured = D.measure(frame, refs, CORNERS_PX, matrix)
    _st, detail = D.classify(measured, baseline)
    assert detail["driftCmThreshold"] == D.DRIFT_CM


def test_기준선_점수를_정착_직후에_기록한다(tmp_path, matrix):
    """⭐ 나중에 재면 이미 움직였을 수 있다. 그때 잰 값이어야 기준이다."""
    frame = scene()
    refs = D.take_patches(frame, CORNERS_PX)
    measured = D.measure(frame, refs, CORNERS_PX, matrix)
    meta = D.save_reference(str(tmp_path), "phone", refs, measured, CORNERS_PX)
    assert meta["scores"] and all(s is None or s > 0.9 for s in meta["scores"])


# ---- 보관 --------------------------------------------------------------------

def test_저장하고_다시_읽으면_같다(tmp_path, matrix):
    frame = scene()
    refs = D.take_patches(frame, CORNERS_PX)
    measured = D.measure(frame, refs, CORNERS_PX, matrix)
    D.save_reference(str(tmp_path), "phone", refs, measured, CORNERS_PX)
    back, meta = D.load_reference(str(tmp_path), "phone")
    assert meta["sourceId"] == "phone"
    for a, b in zip(refs, back):
        if a is None:
            assert b is None
        else:
            assert numpy.array_equal(a, b)


def test_기준이_없으면_안_지어낸다(tmp_path):
    refs, meta = D.load_reference(str(tmp_path), "없는소스")
    assert refs is None and meta is None


def test_망가진_기준도_안_지어낸다(tmp_path):
    (tmp_path / "drift_x.json").write_text("{이건 JSON 이 아니다", encoding="utf-8")
    (tmp_path / "drift_x.npz").write_bytes(b"not an npz")
    refs, meta = D.load_reference(str(tmp_path), "x")
    assert refs is None and meta is None


def test_지우면_사라진다(tmp_path, matrix):
    frame = scene()
    refs = D.take_patches(frame, CORNERS_PX)
    D.save_reference(str(tmp_path), "phone", refs,
                     D.measure(frame, refs, CORNERS_PX, matrix), CORNERS_PX)
    D.clear_reference(str(tmp_path), "phone")
    assert D.load_reference(str(tmp_path), "phone") == (None, None)


# ---- ⭐⭐ 무늬 없는 조각 (2026-09-11 라이브에서 발견한 결함) --------------------------------

def flat(value=28):
    """평평한 화면. 민무늬 바닥·흰 벽이 실제로 이렇다."""
    return numpy.full((H, W, 3), value, dtype=numpy.uint8)


def test_무늬가_없으면_기준을_안_뜬다(matrix):
    """⭐⭐ 정규화 상관의 분모는 **표준편차의 곱**이다. 평평하면 정의되지 않는다.

    OpenCV 는 그때 1.0 을 돌려주고, minMaxLoc 은 탐색창 **첫 칸**을 집는다.
    그래서 "정합 1.0 으로 62.27 cm 움직였다" 가 나왔다 — 아무것도 아닌 것에 대한
    최대 확신. 라이브에서 실제로 그렇게 보고했다.
    """
    refs = D.take_patches(flat(), CORNERS_PX)
    assert all(r is None for r in refs), "평평한 조각을 기준으로 떴다"


def test_평평한_화면에서_흔들림을_지어내지_않는다(matrix):
    """⭐ 그 결함의 **결과**를 고정한다 — 이게 사람이 보는 것이다."""
    frame = flat()
    refs = D.take_patches(frame, CORNERS_PX)
    measured = D.measure(frame, refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, {"scores": [None] * 4})
    assert state == D.STATE_UNKNOWN, detail
    assert detail["why"] == "NO_MEASUREMENT"
    assert "worstShiftCm" not in detail, "못 쟀는데 옮겨간 cm 를 말하고 있다"


def test_무늬가_있으면_그대로_뜬다(matrix):
    """문턱이 너무 세면 멀쩡한 모서리까지 버린다."""
    refs = D.take_patches(scene(), CORNERS_PX)
    assert any(r is not None for r in refs)


def test_문턱이_자료형의_한계에서_왔다():
    """⭐ 튜닝값이 아니다. 8비트에서 표준편차 1 미만은 양자화 잡음이다."""
    assert D.MIN_PATCH_STD == 1.0
    # 한 계조만 오가는 무늬는 잡음이지 무늬가 아니다
    noise = numpy.zeros((H, W, 3), dtype=numpy.uint8)
    noise[::2, :] = 28
    noise[1::2, :] = 29
    assert all(r is None for r in D.take_patches(noise, CORNERS_PX))


# ---- ⭐ 탐색창을 벗어난 변위는 **하한**이다 -------------------------------------------

def test_탐색창_끝에_붙으면_하한이라고_말한다(matrix):
    """⭐ 최고점이 창 테두리면 진짜 꼭대기를 가두지 못한 것이다.

    그 숫자를 측정처럼 말하면 **과소보고**다 — 실제로는 더 갔을 수 있다.
    """
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    far = scene(shift=(D.SEARCH + 40, 0))
    measured = D.measure(far, refs, CORNERS_PX, matrix)
    got = [m for m in measured if m]
    assert got
    assert any(m["atSearchLimit"] for m in got), "창을 벗어났는데 그렇다고 안 말한다"
    _st, detail = D.classify(measured, baseline)
    assert detail["atSearchLimit"] is True


def test_창_안이면_하한이_아니다(matrix):
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    measured = D.measure(scene(shift=(20, 12)), refs, CORNERS_PX, matrix)
    _st, detail = D.classify(measured, baseline)
    assert detail["atSearchLimit"] is False


def test_창을_벗어나도_흔들림으로_잡는다(matrix):
    """⭐ 하한이어도 문턱을 넘었으면 흔들린 것이다 — 모른다로 내리면 정착이 살아남는다."""
    frame = scene()
    refs, baseline = anchored(frame, matrix)
    measured = D.measure(scene(shift=(D.SEARCH + 40, 0)), refs, CORNERS_PX, matrix)
    state, detail = D.classify(measured, baseline)
    assert state == D.STATE_DRIFT, detail
