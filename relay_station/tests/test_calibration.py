# -*- coding: utf-8 -*-
"""MCV-2A-UI / MCV-2S3 — 캘리브레이션이 조용히 틀리지 않게 하는 성질들.

여기 있는 시험은 전부 **통과 못 할 수 있어야** 의미가 있다.
네 점으로 푼 호모그래피의 재투영 오차는 구조상 0 이라 그걸 재는 시험은 **항상 통과한다** —
그런 건 여기 없다. 대신:

    · 나비넥타이(엇갈린 순서)를 거부하는가            <- 숫자로는 멀쩡하다
    · 풀이에 안 쓴 점으로 **진짜** 오차를 내는가       <- 이것만 측정이다
    · 아레나가 바뀌면 옛 정착이 내려오는가             <- 오늘 현장이 바뀐다
    · 영수증을 덮어쓰지 않는가                        <- 과거를 지우면 의심할 수 없다
"""
import json
import os

import pytest

import calibration as C


ARENA = {"version": "t1", "widthCm": 270.0, "heightCm": 125.0,
         "landmarks": {"block-nw": {"x": 97.0, "y": 70.5},
                       "kitchen-wall-n": {"x": 207.0, "y": 62.5}}}


@pytest.fixture
def arena_path(tmp_path):
    p = tmp_path / "arena.json"
    p.write_text(json.dumps(ARENA), encoding="utf-8")
    return str(p)


@pytest.fixture
def store(tmp_path, arena_path):
    return C.CalibrationStore(C.load_arena(arena_path),
                              state_dir=str(tmp_path / "cal"),
                              clock=lambda: 1000.0)


FRAME = (1280, 720)
# 원근이 있는 네 모서리 (bl, br, tr, tl) — 위쪽이 멀어서 좁다.
QUAD = [(180.0, 640.0), (1120.0, 660.0), (900.0, 210.0), (330.0, 200.0)]


def _true_homography():
    """QUAD -> 아레나 cm 의 참 행렬. 합성 시험의 기준이다."""
    t = C.Arena("t1", 270.0, 125.0).corner_targets()
    return C.solve_homography(QUAD, [t[c] for c in C.CORNERS])


def _invert_pixel(matrix, cm):
    """cm -> px. 검증점의 '참 픽셀'을 만들기 위한 역사영."""
    import numpy
    m = numpy.asarray(matrix, dtype=float)
    u, v = cm
    # (m0 - u*m2)·p = 0, (m1 - v*m2)·p = 0  에서 p=(x,y,1) 를 푼다
    r1 = m[0] - u * m[2]
    r2 = m[1] - v * m[2]
    a = numpy.asarray([[r1[0], r1[1]], [r2[0], r2[1]]], dtype=float)
    b = numpy.asarray([-r1[2], -r2[2]], dtype=float)
    x, y = numpy.linalg.solve(a, b)
    return (float(x), float(y))


# ---- 푸는 것 -------------------------------------------------------------------

def test_네_모서리는_정확히_아레나_모서리로_간다():
    h = _true_homography()
    a = C.Arena("t1", 270.0, 125.0)
    t = a.corner_targets()
    for px, name in zip(QUAD, C.CORNERS):
        u, v = C.project(h, px[0], px[1])
        assert u == pytest.approx(t[name][0], abs=1e-6)
        assert v == pytest.approx(t[name][1], abs=1e-6)


def test_겹친_점을_거부한다():
    bad = list(QUAD)
    bad[1] = (bad[0][0] + 2.0, bad[0][1] + 2.0)
    with pytest.raises(C.CalibrationError, match="겹친다"):
        C.check_corner_quad(bad, FRAME)


def test_한_줄에_있는_점을_거부한다():
    line = [(100.0, 100.0), (300.0, 100.0), (500.0, 100.0), (700.0, 100.0)]
    with pytest.raises(C.CalibrationError):
        C.check_corner_quad(line, FRAME)


def test_나비넥타이를_거부한다():
    """⭐ 핵심. 순서가 엇갈린 사각형도 호모그래피는 **멀쩡히 풀린다.**

    푼 값은 상을 뒤집어 놓는데 재투영 오차는 여전히 0 이다. 숫자로는 아무 이상이 없으므로
    **기하로 막아야 한다.**
    """
    bow = [QUAD[0], QUAD[1], QUAD[3], QUAD[2]]      # tr 과 tl 을 바꿨다
    assert not C.is_convex_quad(bow)
    with pytest.raises(C.CalibrationError, match="볼록"):
        C.check_corner_quad(bow, FRAME)
    # 그런데 풀리기는 한다 — 그래서 게이트가 필요하다는 증거다
    a = C.Arena("t1", 270.0, 125.0)
    t = a.corner_targets()
    C.solve_homography(bow, [t[c] for c in C.CORNERS])


def test_화면_밖의_점을_거부한다():
    out = list(QUAD)
    out[2] = (FRAME[0] + 40.0, 210.0)
    with pytest.raises(C.CalibrationError, match="화면 밖"):
        C.check_corner_quad(out, FRAME)


def test_너무_작은_사각형을_거부한다():
    tiny = [(100.0, 100.0), (140.0, 102.0), (139.0, 140.0), (99.0, 138.0)]
    with pytest.raises(C.CalibrationError, match="%"):
        C.check_corner_quad(tiny, FRAME)


def test_점이_네_개가_아니면_거부한다():
    with pytest.raises(C.CalibrationError):
        C.check_corner_quad(QUAD[:3], FRAME)
    with pytest.raises(C.CalibrationError):
        C.solve_homography(QUAD[:3], [(0, 0), (1, 0), (1, 1)])


# ---- 재는 것 -------------------------------------------------------------------

def test_풀이에_안_쓴_점이_진짜_오차를_낸다(store):
    """⭐⭐ 이 유닛의 존재 이유. 네 점의 오차는 0 이라 아무 말도 못 한다.

    검증점의 픽셀은 **참 행렬로부터** 만든다. 그러면 정확히 찍었을 때 오차 0,
    모서리를 흔들면 오차가 생긴다 — 시험이 **양쪽 다** 보여야 한다.
    """
    h = _true_homography()
    lm = store.arena.landmark("block-nw")
    px = _invert_pixel(h, lm)

    store.set_points("phone", QUAD, FRAME, publisher_session="s1",
                     verify=[{"name": "block-nw", "px": px}])
    r = store.settle("phone")
    assert r["accuracyMeasured"] is True
    assert r["maxVerifyErrorCm"] == pytest.approx(0.0, abs=0.05)

    # 모서리 하나를 20 px 흔들면 같은 검증점의 오차가 커진다
    shaken = list(QUAD)
    shaken[2] = (shaken[2][0] + 20.0, shaken[2][1] - 20.0)
    store.set_points("phone", shaken, FRAME, publisher_session="s1",
                     verify=[{"name": "block-nw", "px": px}])
    r2 = store.settle("phone")
    assert r2["maxVerifyErrorCm"] > 0.5, "검증점이 흔들림을 못 잡아내면 재는 게 아니다"


def test_검증점이_없으면_정확도를_안_쟀다고_적는다(store):
    store.set_points("phone", QUAD, FRAME)
    r = store.settle("phone")
    assert r["accuracyMeasured"] is False
    assert r["maxVerifyErrorCm"] is None
    assert r["verifyPoints"] == []


def test_영수증에_풀이점_재투영오차를_적지_않는다(store):
    """구조상 0 인 값을 적으면 그 0 을 정확도로 읽는 사람이 반드시 나온다."""
    store.set_points("phone", QUAD, FRAME)
    r = store.settle("phone")
    blob = json.dumps(r, ensure_ascii=False)
    assert "cornerResidual" not in blob
    assert "reprojectionErrorPx" not in blob


def test_아레나에_없는_지점은_검증점이_될_수_없다(store):
    with pytest.raises(C.CalibrationError, match="없는 지점"):
        store.set_points("phone", QUAD, FRAME,
                         verify=[{"name": "made-up", "px": [10.0, 10.0]}])


# ---- 무효화 -------------------------------------------------------------------

def test_아레나가_바뀌면_정착이_내려온다(tmp_path, arena_path):
    """오늘 현장이 바뀐다. 어느 아레나를 잰 건지 모르면 조용히 틀린다."""
    d = str(tmp_path / "cal")
    s1 = C.CalibrationStore(C.load_arena(arena_path), state_dir=d, clock=lambda: 1.0)
    s1.set_points("phone", QUAD, FRAME)
    s1.settle("phone")
    assert s1.state("phone")["state"] == C.STATE_SETTLED

    changed = dict(ARENA, version="t2", widthCm=235.0)
    open(arena_path, "w", encoding="utf-8").write(json.dumps(changed))
    s2 = C.CalibrationStore(C.load_arena(arena_path), state_dir=d, clock=lambda: 2.0)
    st = s2.state("phone")
    assert st["state"] == C.STATE_DRIFT
    assert st["reason"] == C.REASON_ARENA_CHANGED
    assert s2.homography_for("phone") is None, "무효인데 행렬을 내주면 안 된다"


def test_프레임_크기가_바뀌면_정착이_내려온다(store):
    """APP-1 이 1088x1088 을 1280x720 으로 바꾼다. 화각까지 바뀌므로 되살리지 않는다."""
    store.set_points("phone", QUAD, FRAME, publisher_session="s1")
    store.settle("phone")
    st = store.state("phone", frame_size=(1088, 1088), publisher_session="s1")
    assert st["state"] == C.STATE_DRIFT
    assert st["reason"] == C.REASON_FRAME_SIZE_CHANGED
    assert store.homography_for("phone", frame_size=(1088, 1088)) is None


def test_발행자가_재시작하면_정착이_내려온다(store):
    """앱이 다시 뜬 사이에 기기가 움직였을 수 있다. 안 움직였다는 보장이 없다."""
    store.set_points("phone", QUAD, FRAME, publisher_session="s1")
    store.settle("phone")
    st = store.state("phone", frame_size=FRAME, publisher_session="s2")
    assert st["state"] == C.STATE_DRIFT
    assert st["reason"] == C.REASON_PUBLISHER_RESTARTED


def test_조건이_그대로면_정착을_유지한다(store):
    store.set_points("phone", QUAD, FRAME, publisher_session="s1")
    store.settle("phone")
    st = store.state("phone", frame_size=FRAME, publisher_session="s1")
    assert st["state"] == C.STATE_SETTLED
    assert st["reason"] is None
    assert store.homography_for("phone", FRAME, "s1") is not None


# ---- fail-closed ---------------------------------------------------------------

def test_정착_전에는_행렬을_안_내준다(store):
    assert store.homography_for("phone") is None
    store.set_points("phone", QUAD, FRAME)
    assert store.state("phone")["state"] == C.STATE_CALIBRATING
    assert store.homography_for("phone") is None, \
        "찍는 중인 값을 관측으로 내보내면 캘리브레이션 없이 추정을 내는 것이다"


def test_모르는_소스도_예외를_안_던진다(store):
    st = store.state("듣도보도못한소스")
    assert st["state"] == C.STATE_UNCALIBRATED
    assert st["homography"] is None


def test_점이_없으면_정착을_거부한다(store):
    with pytest.raises(C.CalibrationError, match="놓인 점이 없다"):
        store.settle("phone")


def test_아레나_명세가_없으면_캘리브레이션도_없다(tmp_path):
    s = C.CalibrationStore(None, state_dir=str(tmp_path / "cal"))
    assert s.state("phone")["state"] == C.STATE_UNCALIBRATED
    with pytest.raises(C.CalibrationError, match="아레나"):
        s.set_points("phone", QUAD, FRAME)


def test_없는_아레나_파일은_None_이다_추정치를_만들지_않는다(tmp_path):
    assert C.load_arena(str(tmp_path / "없다.json")) is None


# ---- 영수증 (MCV-2S3) -----------------------------------------------------------

def test_영수증은_덧붙기만_한다(store):
    for _ in range(3):
        store.set_points("phone", QUAD, FRAME)
        store.settle("phone")
    recs = store.read_receipts("phone")
    assert len(recs) == 3, "덮어쓰면 과거 정착을 의심할 수 없다"


def test_영수증이_무엇을_잰_건지_다_적는다(store):
    store.set_points("phone", QUAD, FRAME, publisher_session="sess-9")
    r = store.settle("phone", note="현장 수정 직후")
    for key in ("sourceId", "settledAt", "arenaVersion", "frameSize",
                "publisherSession", "imageOrigin", "arenaOrigin",
                "corners", "homography", "undistorted", "note"):
        assert key in r, "영수증에 %s 가 없으면 나중에 재현할 수 없다" % key
    assert r["arena"]["widthCm"] == 270.0
    assert r["publisherSession"] == "sess-9"
    assert r["undistorted"] is False, "왜곡 보정을 안 했으면 안 했다고 적어야 한다"


def test_영수증이_좌표계_방향을_적는다(store):
    """화면은 위가 0, 아레나는 아래가 0 이다. 안 적으면 뒤집힌 채 '정합됐다'고 말한다."""
    store.set_points("phone", QUAD, FRAME)
    r = store.settle("phone")
    assert r["imageOrigin"] == "top-left"
    assert r["arenaOrigin"] == "bottom-left"


def test_정착하면_작업중_점이_치워진다(store):
    store.set_points("phone", QUAD, FRAME)
    store.settle("phone")
    assert store.state("phone")["state"] != C.STATE_CALIBRATING


def test_다시_띄워도_영수증을_읽어_정착을_이어간다(tmp_path, arena_path):
    d = str(tmp_path / "cal")
    s1 = C.CalibrationStore(C.load_arena(arena_path), state_dir=d)
    s1.set_points("phone", QUAD, FRAME, publisher_session="s1")
    s1.settle("phone")
    s2 = C.CalibrationStore(C.load_arena(arena_path), state_dir=d)
    assert s2.state("phone", FRAME, "s1")["state"] == C.STATE_SETTLED


def test_깨진_영수증_한_줄이_나머지를_못_읽게_하지_않는다(store):
    store.set_points("phone", QUAD, FRAME)
    store.settle("phone")
    with open(store.receipts_path, "a", encoding="utf-8") as fh:
        fh.write("{이건 JSON 이 아니다\n")
    assert len(store.read_receipts("phone")) == 1


# ---- 아레나가 코드에 안 박혀 있다 -------------------------------------------------

def test_아레나_치수를_코드에_안_박았다(tmp_path):
    """⭐ 원본이 바뀌면 알아채는가. 파일을 고쳤을 때 모듈이 따라와야 한다."""
    p = tmp_path / "arena.json"
    p.write_text(json.dumps(dict(ARENA, version="odd", widthCm=111.0,
                                 heightCm=222.0)), encoding="utf-8")
    a = C.load_arena(str(p))
    t = a.corner_targets()
    assert t["br"] == (111.0, 0.0)
    assert t["tl"] == (0.0, 222.0)


def test_소스에_270_이나_125_가_상수로_없다():
    """치수가 코드에 있으면 현장이 바뀐 날 조용히 옛 값으로 정착한다."""
    src = open(os.path.join(os.path.dirname(C.__file__), "calibration.py"),
               encoding="utf-8").read()
    body = src.split('"""', 2)[-1]          # 모듈 독스트링(설명문)은 제외
    for bad in ("270.0", "125.0", "235.0"):
        assert bad not in body, "아레나 치수 %s 가 코드에 박혀 있다" % bad
