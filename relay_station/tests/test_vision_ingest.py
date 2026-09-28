# -*- coding: utf-8 -*-
"""연산 노드 좌표 수신구 — 순수 로직 시험.

🔴 이 시험들이 지키려는 성질은 하나다: **없는 것을 있다고 하지 않는다.**
   2026-09-18 감사에서 게이트웨이가 지어내던 것이 넷 나왔고(정지 API 무조건 성공 ·
   화면이 응답 미판독 · 로봇 좌표 하드코딩 · 맵 못 읽으면 합성 격자 무표시), 같은 병을
   새 경로에 다시 만들지 않으려고 쓴다.

⭐ 그래서 **뮤테이션이 통과 조건**이다. 아래 시험 중 여럿은 `vision_ingest` 의 신선도
   검사를 지우면 빨개지도록 짰다. 지워도 초록이면 그 시험은 검사가 아니다.
"""
import math

import pytest

import vision_ingest as VI


def _payload(**over):
    base = {"robotId": "robot1", "x": 0.5, "y": -0.25, "yaw": 1.57,
            "computedAtMs": 1_000_000}
    base.update(over)
    return base


# ---- validate ---------------------------------------------------------------

def test_정상_payload_는_통과하고_값이_실수로_정규화된다():
    ok, reason, norm = VI.validate(_payload())
    assert ok and reason is None
    assert norm["robotId"] == "robot1"
    assert norm["x"] == 0.5 and norm["y"] == -0.25 and norm["yaw"] == 1.57
    assert isinstance(norm["x"], float)


@pytest.mark.parametrize("missing", ["x", "y", "yaw", "computedAtMs"])
def test_빠진_필드는_0_으로_채우지_않고_거절한다(missing):
    """⭐ 0 으로 채우면 '안 왔다' 와 '원점에 있다' 가 같은 숫자가 된다."""
    p = _payload()
    del p[missing]
    ok, reason, norm = VI.validate(p)
    assert not ok and norm is None
    assert missing in reason


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_유한하지_않은_값은_거절한다(bad):
    ok, reason, _ = VI.validate(_payload(x=bad))
    assert not ok and VI.REJECT_NOT_FINITE in reason


def test_불리언은_숫자로_통과하지_못한다():
    """True 가 1 로 통과하면 좌표가 조용히 생긴다."""
    ok, reason, _ = VI.validate(_payload(x=True))
    assert not ok and VI.REJECT_TYPE in reason


@pytest.mark.parametrize("rid", [None, "", "   ", 7, [], {}])
def test_로봇_id_가_없거나_문자열이_아니면_거절한다(rid):
    ok, reason, _ = VI.validate(_payload(robotId=rid))
    assert not ok and reason == VI.REJECT_ROBOT_ID


def test_모르는_로봇_id_는_거절한다():
    ok, reason, _ = VI.validate(_payload(robotId="robot9"),
                                robot_ids={"robot1", "robot2"})
    assert not ok and reason == VI.REJECT_ROBOT_ID


def test_payload_가_객체가_아니면_거절한다():
    for junk in (None, [], "robot1", 3):
        ok, reason, _ = VI.validate(junk)
        assert not ok and reason == VI.REJECT_NOT_OBJECT


def test_선택_필드는_없으면_None_이지_기본값이_아니다():
    _, _, norm = VI.validate(_payload())
    assert norm["sourceIds"] is None
    assert norm["frameSeq"] is None
    assert norm["quality"] is None


# ---- 신선도 (뮤테이션 표적) ----------------------------------------------------

def test_한_번도_못_받으면_NEVER_이고_좌표가_없다():
    s = VI.VisionPoseStore()
    assert s.state("robot1", 0) == (VI.NEVER, None)
    assert s.pose("robot1", 0) is None
    rep = s.report(["robot1"], 0)["robot1"]
    assert rep["state"] == VI.NEVER and rep["observed"] is False
    assert "x" not in rep and "y" not in rep


def test_받은_직후에는_FRESH_다():
    s = VI.VisionPoseStore()
    _, _, norm = VI.validate(_payload())
    s.accept(norm, received_at_ms=5_000)
    st, age = s.state("robot1", 5_000)
    assert st == VI.FRESH and age == 0
    assert s.pose("robot1", 5_000)["x"] == 0.5


def test_문턱을_넘기면_STALE_이_된다():
    """🔴 뮤테이션 표적 — 신선도 검사를 지우면 이 시험이 빨개진다."""
    s = VI.VisionPoseStore(stale_after_ms=500)
    _, _, norm = VI.validate(_payload())
    s.accept(norm, received_at_ms=1_000)
    assert s.state("robot1", 1_000 + 500)[0] == VI.FRESH     # 경계는 아직 신선
    assert s.state("robot1", 1_000 + 501)[0] == VI.STALE     # 한 밀리초 넘으면 낡음


def test_낡으면_좌표를_아예_돌려주지_않는다():
    """🔴 뮤테이션 표적 — '일단 주고 나중에 판단' 을 못 하게 값 자체를 막는다."""
    s = VI.VisionPoseStore(stale_after_ms=500)
    _, _, norm = VI.validate(_payload())
    s.accept(norm, received_at_ms=1_000)
    assert s.pose("robot1", 1_400) is not None
    assert s.pose("robot1", 1_600) is None


def test_낡으면_보고에도_좌표가_안_실린다():
    """🔴 화면이 낡은 좌표를 찍는 경로를 막는다."""
    s = VI.VisionPoseStore(stale_after_ms=500)
    _, _, norm = VI.validate(_payload())
    s.accept(norm, received_at_ms=1_000)
    fresh = s.report(["robot1"], 1_100)["robot1"]
    assert fresh["observed"] is True and fresh["x"] == 0.5

    stale = s.report(["robot1"], 9_000)["robot1"]
    assert stale["state"] == VI.STALE and stale["observed"] is False
    assert "x" not in stale and "y" not in stale and "yaw" not in stale


def test_시계가_뒤로_가도_신선하다고_우기지_않는다():
    s = VI.VisionPoseStore()
    _, _, norm = VI.validate(_payload())
    s.accept(norm, received_at_ms=10_000)
    st, age = s.state("robot1", 9_000)      # 받은 시각이 미래
    assert st == VI.STALE and age < 0
    assert s.pose("robot1", 9_000) is None


def test_새로_받으면_다시_신선해진다():
    s = VI.VisionPoseStore(stale_after_ms=500)
    _, _, norm = VI.validate(_payload())
    s.accept(norm, received_at_ms=1_000)
    assert s.state("robot1", 5_000)[0] == VI.STALE
    s.accept(norm, received_at_ms=5_000)
    assert s.state("robot1", 5_000)[0] == VI.FRESH


# ---- 남의 시계는 신선도의 기준이 아니다 ------------------------------------------

def test_신선도는_받은_시각으로_재지_보낸_시각으로_재지_않는다():
    """⭐ 2026-09-19 실측에서 현장 폰 하나가 82 시간 어긋난 채 `clockSuspect:false`
       였다. 남의 시계를 기준으로 삼으면 그 기기가 틀린 만큼 우리가 속는다."""
    s = VI.VisionPoseStore(stale_after_ms=500)
    # 연산 노드가 '한참 전' 이라고 주장하지만, 방금 도착했다.
    _, _, norm = VI.validate(_payload(computedAtMs=1))
    s.accept(norm, received_at_ms=1_000_000)
    assert s.state("robot1", 1_000_000)[0] == VI.FRESH


def test_보낸_시각과의_차이는_버리지_않고_따로_적는다():
    s = VI.VisionPoseStore()
    _, _, norm = VI.validate(_payload(computedAtMs=1_000))
    rec = s.accept(norm, received_at_ms=1_120)
    assert rec["clockOffsetMs"] == 120
    assert rec["clockSuspect"] is False

    _, _, far = VI.validate(_payload(computedAtMs=0))
    rec2 = s.accept(far, received_at_ms=10_000)
    assert rec2["clockOffsetMs"] == 10_000
    assert rec2["clockSuspect"] is True
    rep = s.report(["robot1"], 10_000)["robot1"]
    assert rep["clockSuspect"] is True      # 값은 받되 경고는 낸다
    assert rep["observed"] is True


# ---- 상수는 유도의 산물이다 ----------------------------------------------------

def test_문턱이_실측_갱신율과_주행속도에서_유도된다():
    """박아 둔 기대값 대조 — 상수를 조용히 바꾸면 여기서 걸린다.

    노트10 실측 8.325 fps(2026-09-19 01:34) → 프레임 간격 120 ms.
    네 번 연속 결측 480 ms 를 500 ms 로 올림. 0.20 m/s 에서 10 cm = 격자 4 칸.
    """
    frame_gap_ms = 1000.0 / 8.325
    assert 115 < frame_gap_ms < 125
    assert VI.STALE_AFTER_MS >= 4 * frame_gap_ms      # 네 프레임은 견딘다
    assert VI.STALE_AFTER_MS == 500

    moved_cm = 0.20 * (VI.STALE_AFTER_MS / 1000.0) * 100.0
    assert math.isclose(moved_cm, 10.0)
    assert moved_cm / 2.5 == 4.0                      # 평면 격자 네 칸


def test_문턱이_0_이하면_만들_수_없다():
    for bad in (0, -1):
        with pytest.raises(ValueError):
            VI.VisionPoseStore(stale_after_ms=bad)


def test_로봇마다_따로_판정한다():
    s = VI.VisionPoseStore(stale_after_ms=500)
    _, _, r1 = VI.validate(_payload(robotId="robot1"))
    s.accept(r1, received_at_ms=1_000)
    rep = s.report(["robot1", "robot2"], 1_100)
    assert rep["robot1"]["state"] == VI.FRESH
    assert rep["robot2"]["state"] == VI.NEVER
    assert "x" not in rep["robot2"]
