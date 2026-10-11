# -*- coding: utf-8 -*-
"""MCV-2V — 영상에서 만든 평면도가 지켜야 할 성질.

⭐ 합성 아레나로 시험한다 — **정답을 알기 때문이다.** 벽을 아는 cm 자리에 그려 놓고,
   복원된 벽이 그 자리로 돌아오는지 본다. 네 점의 재투영 오차처럼 구조상 0 이 되는
   숫자가 아니라, 진짜 대조군이 있는 측정이다.

⚠️ 다만 합성 그림에는 **벽 높이가 없다.** 바닥에 칠한 그림이라 실제 벽이 만드는
   번짐(독스트링 §호모그래피는 바닥 위의 점에만 맞다)은 여기서 안 나타난다.
   그래서 이 시험들은 **좌표 수학과 파이프라인**을 고정하지, 번짐 문제를 고정하지 않는다.
   그건 실기기에서만 잰다.
"""
import json
import os

import numpy
import pytest

import calibration as C
import vision_world as V

ARENA = {"version": "t1", "widthCm": 270.0, "heightCm": 125.0,
         "landmarks": {"block-nw": {"x": 97.0, "y": 70.5}}}
W, H = 1280, 720
# 아레나 네 모서리가 화면에서 차지하는 자리 (bl, br, tr, tl) — 원근 있는 사다리꼴
CORNERS_PX = [(60.0, 660.0), (1220.0, 660.0), (940.0, 120.0), (340.0, 120.0)]
RES = 5.0          # cm/칸 — 54 x 25. 그림자 광선이 파이썬 루프라 성기게 둔다
# 실제 아레나는 늘 외벽이 있다. 벽이 하나도 없는 화면은 '대비 없음' 으로
# 거부되는 게 맞고, 그건 test_대비가_없으면_벽을_지어내지_않는다 가 따로 본다.
OUTER_WALLS = [(0.0, 0.0, 270.0, 4.0), (0.0, 121.0, 270.0, 125.0),
               (0.0, 0.0, 4.0, 125.0), (266.0, 0.0, 270.0, 125.0)]


@pytest.fixture
def arena(tmp_path):
    p = tmp_path / "arena.json"
    p.write_text(json.dumps(ARENA), encoding="utf-8")
    return C.load_arena(str(p))


@pytest.fixture
def matrix(arena):
    t = arena.corner_targets()
    return C.solve_homography(CORNERS_PX, [t[c] for c in C.CORNERS])


def settled(matrix, source_id="synthetic"):
    return {"state": C.STATE_SETTLED, "homography": matrix,
            "sourceId": source_id, "settledAt": 1789000000}


def synth_frame(matrix, walls_cm, floor=210, wall=35, noise=0):
    """아레나 cm 좌표의 벽들을 **알려진 사영**으로 그려 넣은 프레임.

    바닥은 밝게, 벽은 어둡게. 높이는 없다(위 모듈 독스트링의 단서).
    """
    inv = V.invert_homography(matrix)
    img = numpy.zeros((H, W, 3), dtype=numpy.uint8)
    # 아레나 바닥 폴리곤
    import cv2

    def px(x, y):
        p = numpy.linalg.inv(inv) if False else None
        q = numpy.asarray(matrix, dtype=float)
        # cm -> px 는 inv 다
        r = inv @ numpy.array([x, y, 1.0])
        return (int(round(r[0] / r[2])), int(round(r[1] / r[2])))

    a = arena_poly = numpy.array(
        [px(0, 0), px(ARENA["widthCm"], 0),
         px(ARENA["widthCm"], ARENA["heightCm"]), px(0, ARENA["heightCm"])],
        dtype=numpy.int32)
    cv2.fillPoly(img, [arena_poly], (floor, floor, floor))
    for (x0, y0, x1, y1) in walls_cm:
        poly = numpy.array([px(x0, y0), px(x1, y0), px(x1, y1), px(x0, y1)],
                           dtype=numpy.int32)
        cv2.fillPoly(img, [poly], (wall, wall, wall))
    if noise:
        rng = numpy.random.default_rng(3)
        img = numpy.clip(img.astype(numpy.int16)
                         + rng.integers(-noise, noise + 1, img.shape), 0, 255
                         ).astype(numpy.uint8)
    return img


# ---- fail-closed (MCVA-72) ------------------------------------------------------

def test_정착_전에는_거부한다(arena, matrix):
    """⭐ 평면이 없으면 벽 좌표도 없다. 추정으로 월드를 만들지 않는다."""
    frame = synth_frame(matrix, OUTER_WALLS)
    for st in ({"state": C.STATE_UNCALIBRATED}, {"state": C.STATE_CALIBRATING},
               {"state": C.STATE_DRIFT, "homography": matrix}, None):
        with pytest.raises(V.VisionWorldError, match="정착"):
            V.build(frame, st, arena, res_cm=RES)


def test_대비가_없으면_벽을_지어내지_않는다(arena, matrix):
    """민무늬 화면에서 문턱을 억지로 잡으면 없는 벽이 생긴다."""
    flat = numpy.full((H, W, 3), 128, dtype=numpy.uint8)
    with pytest.raises(V.VisionWorldError, match="대비"):
        V.build(flat, settled(matrix), arena, res_cm=RES)


# ---- ⭐⭐ 모른다를 비었다로 접지 않는다 -----------------------------------------------

def test_화면_밖은_모른다다(arena, matrix):
    """아레나 일부가 화면 밖이면 그 칸은 **모른다**여야 한다 — 비었다가 아니다."""
    # ⭐ 검은 칠로 흉내내면 안 된다 — 그건 "화면 밖" 이 아니라 "어두운 벽" 이다.
    #    프레임 자체를 잘라야 아레나 일부가 진짜로 화면 밖으로 나간다.
    frame = synth_frame(matrix, OUTER_WALLS)[:, :W // 2]
    out = V.build(frame, settled(matrix), arena, res_cm=RES, wall_is_dark=True)
    grid = out["grid"]
    assert (grid == V.PGM_UNKNOWN).sum() > 0


def test_검열_상자_안은_모른다다(arena, matrix):
    """가려서 못 본 것을 바닥이라고 읽으면 안 된다."""
    frame = synth_frame(matrix, OUTER_WALLS)
    box = {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}
    out = V.build(frame, settled(matrix), arena, res_cm=RES, censor_box=box)
    assert out["provenance"]["censorBox"] == box
    assert (out["grid"] == V.PGM_UNKNOWN).sum() > 0


def test_벽_뒤_그림자는_비었다가_아니다(arena, matrix):
    """⭐⭐ 이 시험이 이 모듈의 안전 장치다.

    한 시점은 벽 뒤를 못 본다. 거기를 "비었다" 로 적으면 로봇이 그림자를 가로질러
    계획한다 — 이 파이프라인에서 가장 위험한 실수다.
    """
    # 아레나 가운데를 가로지르는 벽. 카메라는 아래쪽(y 작은 쪽)에서 본다.
    wall = (100.0, 55.0, 170.0, 70.0)
    frame = synth_frame(matrix, [wall])
    out = V.build(frame, settled(matrix), arena, res_cm=RES, wall_is_dark=True)
    grid = out["grid"]
    rows, cols = grid.shape
    # 벽 바로 뒤(y 큰 쪽) 같은 x 구간
    c0 = int(105 / RES)
    c1 = int(165 / RES)
    r_behind = int((ARENA['heightCm'] - 85.0) / RES)   # 행 0 = 위쪽
    behind = grid[max(0, r_behind - 1):r_behind + 2, c0:c1]
    assert behind.size > 0
    assert (behind == V.PGM_FREE).sum() == 0, (
        "벽 뒤가 비었다로 적혔다 — 로봇이 그림자를 가로지른다")
    assert (behind == V.PGM_UNKNOWN).sum() > 0


def test_격자값은_셋뿐이다(arena, matrix):
    out = V.build(synth_frame(matrix, [(100.0, 55.0, 170.0, 70.0)]), settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True)
    vals = set(numpy.unique(out["grid"]).tolist())
    assert vals <= {V.PGM_OCCUPIED, V.PGM_UNKNOWN, V.PGM_FREE}, vals


# ---- ⭐ 정답과 대조 ---------------------------------------------------------------

def test_아는_자리의_벽이_그_자리로_돌아온다(arena, matrix):
    """⭐ 합성이라 **정답을 안다.** 좌표 수학이 왕복해서 맞는지 보는 유일한 방법이다."""
    wall = (100.0, 20.0, 140.0, 60.0)          # cm
    out = V.build(synth_frame(matrix, [wall]), settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True)
    occ = (out["grid"] == V.PGM_OCCUPIED)
    assert occ.sum() > 0, "벽을 하나도 못 찾았다"
    rows, cols = occ.shape
    rr, cc = numpy.nonzero(occ)
    x0 = cc.min() * RES
    x1 = (cc.max() + 1) * RES
    # 행 0 이 아레나 위쪽이다 (cell_centers 규약)
    y1 = ARENA['heightCm'] - rr.min() * RES
    y0 = ARENA['heightCm'] - (rr.max() + 1) * RES
    tol = RES * 2.0
    assert abs(x0 - wall[0]) <= tol, "x0 %.1f vs %.1f" % (x0, wall[0])
    assert abs(x1 - wall[2]) <= tol, "x1 %.1f vs %.1f" % (x1, wall[2])
    assert abs(y0 - wall[1]) <= tol, "y0 %.1f vs %.1f" % (y0, wall[1])
    assert abs(y1 - wall[3]) <= tol, "y1 %.1f vs %.1f" % (y1, wall[3])


def test_문턱을_상수로_박지_않았다(arena, matrix):
    """조명이 통째로 어두워져도 벽을 찾아야 한다. 상수 문턱은 그때 조용히 틀린다."""
    wall = (100.0, 20.0, 140.0, 60.0)
    bright = V.build(synth_frame(matrix, [wall], floor=230, wall=60),
                     settled(matrix), arena, res_cm=RES, wall_is_dark=True)
    dim = V.build(synth_frame(matrix, [wall], floor=120, wall=25),
                  settled(matrix), arena, res_cm=RES, wall_is_dark=True)
    a = (bright["grid"] == V.PGM_OCCUPIED).sum()
    b = (dim["grid"] == V.PGM_OCCUPIED).sum()
    assert a > 0 and b > 0
    assert abs(a - b) <= max(a, b) * 0.4, "밝기에 따라 결과가 %d vs %d 로 갈린다" % (a, b)


# ---- 출처 (MCVA-73) --------------------------------------------------------------

def test_출처가_vision_이라고_박힌다(arena, matrix):
    out = V.build(synth_frame(matrix, OUTER_WALLS), settled(matrix), arena, res_cm=RES)
    assert out["version"].startswith("vision@")
    assert out["provenance"]["derivedFrom"] == "camera"
    assert "vision@" in out["sdf"]


def test_라이다_월드와_이름이_겹치지_않는다(arena, matrix):
    """⭐ `map2@`(라이다)를 덮어쓰면 서로를 대조할 수 없게 된다."""
    out = V.build(synth_frame(matrix, OUTER_WALLS), settled(matrix), arena, res_cm=RES)
    assert not out["version"].startswith("map2@")
    assert "map2@" not in out["sdf"]


def test_실루엣을_썼다는_것을_적는다(arena, matrix):
    """⭐ 벽 높이 때문에 점유의 **상한**이다. 쓰는 쪽이 그걸 알아야 한다."""
    out = V.build(synth_frame(matrix, [(100.0, 20.0, 140.0, 60.0)]), settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True)
    assert out["provenance"]["footprintMethod"] == V.FOOTPRINT_SILHOUETTE
    assert "상한" in V.FOOTPRINT_SILHOUETTE or "upper-bound" in V.FOOTPRINT_SILHOUETTE


def test_카메라_위치가_추정이라고_적는다(arena, matrix):
    """정확한 값이 아닌 것을 정확한 값처럼 적지 않는다."""
    out = V.build(synth_frame(matrix, OUTER_WALLS), settled(matrix), arena, res_cm=RES)
    assert out["provenance"]["cameraGroundPointIs"] == "estimate-from-pixel-density"


def test_얼마나_봤는지_적는다(arena, matrix):
    out = V.build(synth_frame(matrix, [(100.0, 20.0, 140.0, 60.0)]), settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True)
    cov = out["provenance"]["coverage"]
    assert cov["cellsTotal"] == cov["cellsSeen"] + (
        cov["cellsTotal"] - cov["cellsSeen"])
    assert 0.0 <= cov["seenFraction"] <= 1.0
    assert cov["cellsFree"] + cov["cellsOccupied"] + cov["cellsUnknown"] == cov["cellsTotal"]


# ---- nav2 산출물 ---------------------------------------------------------------

def test_pgm_이_읽히는_형식이다(arena, matrix):
    out = V.build(synth_frame(matrix, [(100.0, 20.0, 140.0, 60.0)]), settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True)
    pgm = out["pgm"]
    assert pgm.startswith(b"P5")
    rows, cols = out["grid"].shape
    head_end = pgm.index(b"255\n") + 4
    assert len(pgm) - head_end == rows * cols
    assert ("%d %d" % (cols, rows)).encode() in pgm[:head_end]


def test_yaml_이_모른다를_모른다로_읽게_한다(arena, matrix):
    """205 가 free_thresh 와 occupied_thresh **사이**에 있어야 unknown 으로 읽힌다."""
    out = V.build(synth_frame(matrix, OUTER_WALLS), settled(matrix), arena, res_cm=RES)
    y = out["yaml"]
    assert "mode: trinary" in y
    occ_t = float([l for l in y.splitlines() if l.startswith("occupied_thresh")][0].split(":")[1])
    free_t = float([l for l in y.splitlines() if l.startswith("free_thresh")][0].split(":")[1])
    p = 1.0 - (V.PGM_UNKNOWN / 255.0)          # negate:0 이므로 점유도 = 1 - 정규값
    assert free_t < p < occ_t, ("unknown(205)이 %.3f 인데 free<%.2f occ>%.2f 사이가 아니다"
                                % (p, free_t, occ_t))


def test_해상도와_원점이_yaml_과_맞는다(arena, matrix):
    out = V.build(synth_frame(matrix, OUTER_WALLS), settled(matrix), arena, res_cm=RES)
    y = out["yaml"]
    assert ("resolution: %.6f" % (RES / 100.0)) in y
    rows, cols = out["grid"].shape
    assert ("origin: [%.4f, %.4f, 0.0]"
            % (-(cols * RES / 100.0) / 2.0, -(rows * RES / 100.0) / 2.0)) in y


# ---- 벡터화 -------------------------------------------------------------------

def test_사각형들이_점유_칸을_빠짐없이_덮는다(arena, matrix):
    out = V.build(synth_frame(matrix, [(100.0, 20.0, 140.0, 60.0)]), settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True)
    occ = (out["grid"] == V.PGM_OCCUPIED)
    covered = numpy.zeros_like(occ)
    for r1, c1, r2, c2 in out["rects"]:
        covered[r1:r2 + 1, c1:c2 + 1] = True
    assert (occ & ~covered).sum() == 0, "덮이지 않은 점유 칸이 있다"
    assert (covered & ~occ).sum() == 0, "빈 칸을 벽으로 덮었다"


def test_사각형이_없으면_SDF_에_링크도_없다(arena, matrix):
    out = V.build(synth_frame(matrix, OUTER_WALLS), settled(matrix), arena, res_cm=RES)
    if not out["rects"]:
        assert "<link" not in out["sdf"]


# ---- 극성·온전성 (2026-09-11 라이브에서 드러난 것) ------------------------------------

def test_외벽에서_극성을_스스로_찾는다(arena, matrix):
    """⭐ 아레나 **바깥 테두리는 언제나 벽**이다. 사람에게 매번 묻지 않는다."""
    dark = V.build(synth_frame(matrix, OUTER_WALLS, floor=210, wall=35),
                   settled(matrix), arena, res_cm=RES)
    assert dark["provenance"]["wallIsDark"] is True
    assert dark["provenance"]["wallIsDarkSource"] == "auto-border"

    bright = V.build(synth_frame(matrix, OUTER_WALLS, floor=45, wall=200),
                     settled(matrix), arena, res_cm=RES)
    assert bright["provenance"]["wallIsDark"] is False


def test_확신이_없으면_극성을_안_고른다(arena, matrix):
    """⭐ 외벽이 화면 밖이면 테두리가 바닥이다. 억지로 고르면 **바닥 전체가 벽**이 된다.

    2026-09-11 라이브에서 실제로 그렇게 나왔다 — 비었다 0, 막혔다 5234.
    """
    inner_only = synth_frame(matrix, [(100.0, 20.0, 140.0, 60.0)])
    with pytest.raises(V.VisionWorldError, match="판정할 수 없다"):
        V.build(inner_only, settled(matrix), arena, res_cm=RES)


def test_빈_공간이_없는_평면도는_안_내보낸다(arena, matrix):
    """⭐⭐ 로봇이 갈 곳이 없는 월드는 있을 수 없다. 숫자가 말이 안 되면 거절한다."""
    # 극성을 일부러 뒤집어 바닥을 벽으로 읽게 만든다
    with pytest.raises(V.VisionWorldError, match="빈 공간|말이 안 된다"):
        V.build(synth_frame(matrix, OUTER_WALLS, floor=210, wall=35),
                settled(matrix), arena, res_cm=RES, wall_is_dark=False)


def test_극성의_출처를_적는다(arena, matrix):
    """자동으로 찾았는지 사람이 준 값인지 남아야 나중에 따질 수 있다."""
    out = V.build(synth_frame(matrix, [(100.0, 20.0, 140.0, 60.0)]),
                  settled(matrix), arena, res_cm=RES, wall_is_dark=True)
    assert out["provenance"]["wallIsDarkSource"] == "explicit"


# ---- MCV-2M 마스크 ------------------------------------------------------------

MASK = {"x": 0.0, "y": 0.0, "w": 0.35, "h": 0.35, "why": "삼각대 다리"}


def test_마스크_안은_모른다다_절대_비었다가_아니다(arena, matrix):
    """⭐⭐ 이 유닛의 안전 성질이다.

    삼각대 다리를 가려 놓고 그 칸을 **비었다**로 적으면 로봇이 삼각대로 들어간다 —
    충돌 방지를 하려다 충돌을 만든다. 가린 곳은 모른다.

    ⭐ 차분으로 본다. 같은 입력에 마스크만 더했을 때 칸은 **모른다 쪽으로만** 움직여야
       한다. 반대 방향이 하나라도 있으면 그건 마스킹이 아니라 지어내기다.
    """
    frame = synth_frame(matrix, OUTER_WALLS)
    plain = V.build(frame, settled(matrix), arena, res_cm=RES,
                    wall_is_dark=True)["grid"]
    got = V.build(frame, settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True, masks=[MASK])["grid"]

    changed = (plain != got)
    assert changed.sum() > 0, "마스크가 아무것도 안 바꿨다 - 배선이 안 됐다"
    # 바뀐 칸은 전부 '모른다' 가 됐어야 한다
    assert (got[changed] == V.PGM_UNKNOWN).all(),         "마스크가 칸을 모른다 아닌 값으로 바꿨다"
    # ⭐ 그리고 모른다였던 칸이 비었다/막혔다로 바뀐 일은 없어야 한다
    became_known = (plain == V.PGM_UNKNOWN) & (got != V.PGM_UNKNOWN)
    assert not became_known.any(), "마스크가 모르는 칸을 아는 칸으로 만들었다"


def test_마스크가_비었다를_모른다로_바꾼다(arena, matrix):
    """구체적으로: 가리기 전에 **비었다**였던 칸이 있어야 시험이 의미가 있다."""
    frame = synth_frame(matrix, OUTER_WALLS)
    plain = V.build(frame, settled(matrix), arena, res_cm=RES,
                    wall_is_dark=True)["grid"]
    got = V.build(frame, settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True, masks=[MASK])["grid"]
    was_free = (plain == V.PGM_FREE) & (got == V.PGM_UNKNOWN)
    assert was_free.sum() > 0,         "가리기 전에 비었다였던 칸이 없다 - 이 마스크로는 성질을 못 잰다"


def test_마스크를_라벨째_출처에_남긴다(arena, matrix):
    """⭐ 라벨이 없으면 나중에 이 평면도를 본 사람이 구멍을 **관측 실패**로 읽는다."""
    frame = synth_frame(matrix, OUTER_WALLS)
    out = V.build(frame, settled(matrix), arena, res_cm=RES,
                  wall_is_dark=True, masks=[MASK])
    p = out["provenance"]
    assert p["masks"] and p["masks"][0]["why"] == MASK["why"]
    assert "unknown" in p["maskedMeans"] and "never free" in p["maskedMeans"]


def test_마스크가_없으면_출처도_비어_있다(arena, matrix):
    frame = synth_frame(matrix, OUTER_WALLS)
    out = V.build(frame, settled(matrix), arena, res_cm=RES, wall_is_dark=True)
    assert out["provenance"]["masks"] == []


def test_검열과_마스크는_같은_계산을_쓴다(arena, matrix):
    """뜻은 다르지만(누가 선언했나) 결과는 같다 - 둘 다 '여기는 안 봤다'.

    ⭐ 같은 계산을 두 번 적으면 그중 하나가 틀린다 (이 세션에서 행 방향으로 겪었다).
    """
    frame = synth_frame(matrix, OUTER_WALLS)
    as_censor = V.build(frame, settled(matrix), arena, res_cm=RES,
                        wall_is_dark=True,
                        censor_box={k: MASK[k] for k in "xywh"})["grid"]
    as_mask = V.build(frame, settled(matrix), arena, res_cm=RES,
                      wall_is_dark=True, masks=[MASK])["grid"]
    assert (as_censor == as_mask).all(), "같은 상자인데 결과가 다르다"


# ---- MCVA-76 — "모른다" 를 원인별로 가른다 -----------------------------------
#
# ⭐⭐ 왜 필요한가: "화각을 넓힐까, 시점을 더할까" 에 답하려면 모르는 이유를 갈라야 한다.
#    화면 밖은 **화각이** 고치고, 벽 뒤 가림은 **각도가** 고친다. 렌즈를 아무리 넓혀도
#    벽 뒤는 안 보인다. 한 덩어리로 세면 그 질문에 답할 수가 없다.

def _built(arena, matrix, **kw):
    walls = OUTER_WALLS + [(97.0, 60.0, 130.0, 110.0)]     # 안쪽 블록 = 그림자를 만든다
    frame = synth_frame(matrix, walls)
    return V.build(frame, settled(matrix), arena, res_cm=RES, **kw)


def test_모르는_이유를_넷으로_가른다(arena, matrix):
    cov = _built(arena, matrix)["provenance"]["coverage"]
    assert set(cov["unknownBy"]) == {"outOfFrame", "censored", "masked", "shadowed"}


def test_원인의_합이_모르는_칸_수와_같다(arena, matrix):
    """⭐ 안 맞으면 어느 원인에도 안 잡히는 '모른다' 가 있다는 뜻이다."""
    cov = _built(arena, matrix)["provenance"]["coverage"]
    assert sum(cov["unknownBy"].values()) == cov["cellsUnknown"], cov


def test_검열_상자는_censored_로만_센다(arena, matrix):
    """상자를 씌워도 **화면 밖 몫은 안 변한다** — 원인이 섞이면 판단이 틀린다."""
    base = _built(arena, matrix)["provenance"]["coverage"]["unknownBy"]
    box = {"x": 0.30, "y": 0.30, "w": 0.25, "h": 0.25}
    with_box = _built(arena, matrix, censor_box=box)["provenance"]["coverage"]["unknownBy"]
    assert with_box["censored"] > 0
    assert base["censored"] == 0
    assert with_box["outOfFrame"] == base["outOfFrame"]
    assert with_box["shadowed"] >= 0


def test_화각과_가림이_따로_센다(arena, matrix):
    """⭐⭐ R-2 가 묻는 것이 이것이다 — 둘이 같은 칸을 세면 답을 못 준다."""
    cov = _built(arena, matrix)["provenance"]["coverage"]
    ub = cov["unknownBy"]
    # 안쪽 블록이 있으므로 가림이 존재해야 한다. 화각 몫과 별개로 잡혀야 한다.
    assert ub["shadowed"] > 0, ub
    assert ub["outOfFrame"] >= 0
    assert ub["shadowed"] + ub["outOfFrame"] <= cov["cellsUnknown"]


def test_화면_밖_몫이_따로_잡힌다(arena, matrix):
    """⭐⭐ 화각이 고칠 수 있는 몫. 프레임을 **잘라** 아레나 일부를 진짜로 밖에 둔다.

    ⭐ 처음 판에서는 이 경우를 안 만들어서 "화면 밖 몫을 0 으로" 뮤테이션이
       **살아남았다** — 합성 아레나가 화면을 꽉 채우면 그 값이 원래 0 이라
       시험이 아무것도 고정하지 못했다.
    """
    walls = OUTER_WALLS + [(97.0, 60.0, 130.0, 110.0)]
    frame = synth_frame(matrix, walls)[:, :W // 2]
    out = V.build(frame, settled(matrix), arena, res_cm=RES, wall_is_dark=True)
    cov = out["provenance"]["coverage"]
    ub = cov["unknownBy"]
    assert ub["outOfFrame"] > 0, ub
    assert sum(ub.values()) == cov["cellsUnknown"], cov
