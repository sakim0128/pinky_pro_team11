# -*- coding: utf-8 -*-
"""MCV-2M — 운영자가 선언하는 관측 제외 영역.

여기서 고정하는 성질:

    모른다     가린 자리는 '비었다' 가 아니다. 이게 이 유닛의 안전 성질이다
    거부       정착한 모서리·검증점을 덮는 마스크는 **거부한다**
    라벨       무엇을 가리는지 없으면 평면도에 설명 없는 구멍이 남는다
    전부 아니면 아무것도  하나라도 틀리면 목록을 안 바꾼다
"""
import io
import json

import pytest

import masks as M


def box(x, y, w, h, why="삼각대 다리"):
    return {"x": x, "y": y, "w": w, "h": h, "why": why}


# ---- 입력 검사 ----------------------------------------------------------------

def test_라벨이_없으면_거부한다():
    """⭐ 라벨 없는 마스크는 평면도에 **설명 없는 구멍**을 남긴다.

    나중에 본 사람은 그 구멍을 '관측이 실패했다' 로 읽는다. 실은 '일부러 안 봤다' 다.
    """
    with pytest.raises(M.MaskError) as e:
        M.normalize({"x": 0.1, "y": 0.1, "w": 0.2, "h": 0.2})
    assert "무엇을 가리는지" in str(e.value)


def test_너무_작으면_거부한다():
    """잘못 눌린 점 하나가 마스크가 되면 사람은 그게 있는 줄도 모른다."""
    with pytest.raises(M.MaskError):
        M.normalize(box(0.5, 0.5, 0.001, 0.001))


def test_화면_밖으로_나가면_거부한다():
    with pytest.raises(M.MaskError) as e:
        M.normalize(box(0.9, 0.1, 0.3, 0.2))
    assert "화면 밖" in str(e.value)


def test_숫자가_아니면_거부한다():
    with pytest.raises(M.MaskError):
        M.normalize({"x": "왼쪽", "y": 0.1, "w": 0.2, "h": 0.2, "why": "x"})


def test_라벨을_자른다():
    m = M.normalize(box(0.1, 0.1, 0.2, 0.2, why="가" * 200))
    assert len(m["why"]) == M.MAX_WHY


def test_화면을_다_덮으면_거부한다():
    """⭐ 튜닝한 문턱이 아니다 — 화면을 다 가리는 건 마스킹이 아니라 **소스를 끄는 것**이다."""
    with pytest.raises(M.MaskError) as e:
        M.parse([box(0.0, 0.0, 1.0, 1.0, why="전부")])
    assert "소스를 끄는 것" in str(e.value)


def test_하나라도_틀리면_아무것도_안_바꾼다():
    """부분 적용은 사람이 무엇이 들어갔는지 모르게 만든다."""
    with pytest.raises(M.MaskError):
        M.parse([box(0.1, 0.1, 0.2, 0.2), {"x": 0, "y": 0, "w": 0.2, "h": 0.2}])


# ---- 겹침 --------------------------------------------------------------------

def test_겹쳐도_100_퍼센트를_안_넘는다():
    """겹쳐 놓고 두 번 세면 '얼마나 가렸나' 가 거짓이 된다."""
    two = [M.normalize(box(0.0, 0.0, 0.5, 0.5, why="a")),
           M.normalize(box(0.25, 0.25, 0.5, 0.5, why="b"))]
    frac = M.union_fraction(two)
    assert 0.4 < frac < 0.5, frac          # 0.25+0.25-0.0625 = 0.4375
    assert frac <= 1.0


def test_없으면_0():
    assert M.union_fraction([]) == 0.0


# ---- ⭐⭐ 정착한 점을 덮으면 거부 ------------------------------------------------------

FRAME = (1280, 720)
# ⭐ **실물 모양**이다. 상태 딕트는 `verify`, 영수증은 `verifyPoints` 를 쓴다
#    (calibration.Calibration.state() / 영수증 참조). 픽스처가 실물과 달랐던 탓에
#    검증점 보호가 운영에서만 안 걸렸다 - 그래서 둘 다 시험한다.
STATE = {
    "corners": [[200.0, 620.0], [1080.0, 620.0], [900.0, 200.0], [380.0, 200.0]],
    "verify": [{"name": "block-nw", "px": [533.0, 265.0]}],
}
RECEIPT_SHAPE = {
    "corners": STATE["corners"],
    "verifyPoints": [{"name": "block-nw", "px": [533.0, 265.0]}],
}


def test_모서리를_덮으면_거부한다():
    """⭐⭐ 그 점을 안 본다고 선언하면 그 정합은 **못 보는 점으로 푼 행렬**이다.

    카메라를 옮길지 마스크를 줄일지는 사람이 정한다 — 우리가 조용히 한쪽을 안 고른다.
    """
    on_corner = box(0.10, 0.80, 0.15, 0.15, why="삼각대 다리")   # (128~320, 576~684)
    with pytest.raises(M.MaskError) as e:
        M.parse([on_corner], state=STATE, frame_size=FRAME)
    msg = str(e.value)
    assert "모서리" in msg
    assert "삼각대 다리" in msg, "어느 마스크가 문제인지 안 말한다"


def test_검증점을_덮어도_거부한다():
    """검증점은 정확도를 말하는 유일한 근거다. 그걸 가리면 오차가 거짓이 된다."""
    on_verify = box(0.38, 0.32, 0.1, 0.1, why="모니터 베젤")     # (486~614, 230~302)
    with pytest.raises(M.MaskError) as e:
        M.parse([on_verify], state=STATE, frame_size=FRAME)
    assert "검증점" in str(e.value)


def test_빈_자리는_통과한다():
    ok = box(0.0, 0.0, 0.12, 0.12, why="모니터 베젤")
    got = M.parse([ok], state=STATE, frame_size=FRAME)
    assert len(got) == 1 and got[0]["why"] == "모니터 베젤"


def test_정착_전에는_덮을_점이_없다():
    """아직 안 찍었으면 막을 이유가 없다. 순서를 강요하지 않는다."""
    got = M.parse([box(0.1, 0.8, 0.2, 0.2)], state=None, frame_size=FRAME)
    assert len(got) == 1


# ---- 점 조회 -----------------------------------------------------------------

def test_덮는_마스크를_라벨째_돌려준다():
    ms = [M.normalize(box(0.0, 0.0, 0.2, 0.2, why="베젤"))]
    got = M.covering(ms, (100, 100), FRAME)
    assert got and got["why"] == "베젤", "무엇이 가렸는지 말해야 사람이 고친다"
    assert M.covering(ms, (900, 600), FRAME) is None


# ---- 보관 --------------------------------------------------------------------

def test_저장하고_읽으면_같다(tmp_path):
    ms = M.parse([box(0.0, 0.0, 0.2, 0.2, why="베젤"),
                  box(0.7, 0.7, 0.2, 0.2, why="케이블")])
    M.save(str(tmp_path), "phone", ms)
    back, problem = M.load(str(tmp_path), "phone")
    assert problem is None
    assert [m["why"] for m in back] == ["베젤", "케이블"]


def test_없으면_빈_목록이고_문제도_없다(tmp_path):
    back, problem = M.load(str(tmp_path), "없는소스")
    assert back == [] and problem is None


def test_망가지면_빈_목록이되_조용하지_않다(tmp_path):
    """⭐⭐ 못 읽었을 때 빈 목록은 **안전한 방향**이다 (삼각대가 벽으로 읽힌다).

    그렇다고 조용히 넘어가면 안 된다 — 운영자는 마스크를 그려 놨는데 평면도에
    유령 벽이 생기고, 그걸 관측 실패로 읽는다.
    """
    (tmp_path / "masks_x.json").write_text("{이건 JSON 이 아니다", encoding="utf-8")
    back, problem = M.load(str(tmp_path), "x")
    assert back == []
    assert problem and "못 읽었다" in problem


def test_정착을_해제해도_마스크는_안_지운다(tmp_path):
    """⭐ 카메라 자리가 그대로면 삼각대도 그 자리다. 매번 다시 그리게 하지 않는다.

    (파일 이름이 다르다는 것으로 고정한다 — 같은 접두어면 나중에 같이 지워진다.)
    """
    assert "masks_" in M.path_for(str(tmp_path), "phone")
    assert "drift_" not in M.path_for(str(tmp_path), "phone")


def test_원자적으로_쓴다(tmp_path):
    """운영 중 파일을 0바이트로 자르지 않는다 (2026-09-08 에 실제로 겪었다)."""
    ms = M.parse([box(0.0, 0.0, 0.2, 0.2, why="베젤")])
    M.save(str(tmp_path), "phone", ms)
    assert not (tmp_path / "masks_phone.json.tmp").exists()
    raw = json.load(io.open(str(tmp_path / "masks_phone.json"), encoding="utf-8"))
    assert raw["sourceId"] == "phone"


def test_영수증_모양의_검증점도_본다():
    """⭐ 상태 딕트는 `verify`, 영수증은 `verifyPoints` 다. 한쪽만 읽으면
       시험은 초록인데 **운영에서만** 보호가 안 걸린다 (실제로 그랬다)."""
    on_verify = box(0.38, 0.32, 0.1, 0.1, why="모니터 베젤")
    with pytest.raises(M.MaskError) as e:
        M.parse([on_verify], state=RECEIPT_SHAPE, frame_size=FRAME)
    assert "검증점" in str(e.value)
