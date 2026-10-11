# -*- coding: utf-8 -*-
"""U-1/U-2 — 어긋남과 지연이 **다른 칸**인가, 불가능한 값이 **숫자로** 나가지 않는가.

설계서(2026-09-12) 수락 기준:

    U-1  시계를 인위로 10초 틀면 → **지연 칸은 안 움직이고 어긋남 칸만 10초 변한다**
    U-2  음수 지연·로컬 1초 초과를 주입하면 → **숫자 대신 사유**

⭐ U-1 을 시험으로 옮기는 방법: 지연이 **발행자 자기 시계**로 잰 헤더에서만 나오면,
   게이트웨이 시계를 아무리 틀어도 그 값은 그대로다. 어긋남은 `now - captureClock`
   이라 그대로 밀린다. 그래서 곁표에서 어긋남만 밀어 보면 된다.
"""
import timing as T


def sidecar(offset=120.0, jitter=8.0, process=30.0, encode=5.0, kind="capture"):
    return {"source": kind, "offsetMedianMs": offset, "offsetJitterMs": jitter,
            "processMs": process, "encodeMs": encode}


# ---- ⭐⭐ U-1: 시계를 틀면 어긋남만 움직인다 ------------------------------------------

def test_시계를_10초_틀면_어긋남만_변한다():
    """⭐⭐ 이 시험이 U-1 그 자체다.

    게이트웨이 시계가 10초 밀리면 `now - captureClock` 이 10초 커진다. 지연은
    발행자 자기 시계로 잰 구간의 합이라 **한 밀리초도 안 변해야 한다.**
    """
    before = T.describe(sidecar(offset=120.0), transport="pull")
    after = T.describe(sidecar(offset=120.0 + 10000.0), transport="pull")

    assert after["offsetMs"] - before["offsetMs"] == 10000.0
    assert after["pipelineMs"] == before["pipelineMs"], \
        "시계를 틀었는데 지연이 움직였다 — 지연이 교차 시계를 쓰고 있다"
    assert after["pipelineMs"] == 35.0


def test_어긋남과_지연이_다른_칸이다():
    """한 칸에 합치면 어느 쪽도 못 고친다 — 어긋남은 시계를, 지연은 파이프라인을 고친다."""
    d = T.describe(sidecar(), transport="pull")
    assert d["offsetMs"] == 120.0
    assert d["pipelineMs"] == 35.0
    assert d["offsetMs"] != d["pipelineMs"]


def test_어긋남에_무엇이_섞였는지_말한다():
    """⭐ 이름만 바꾸고 뜻을 안 적으면 다음 사람이 또 지연으로 읽는다."""
    d = T.describe(sidecar())
    assert "시계차" in d["offsetIncludes"]
    assert "시계차와 무관" in d["pipelineBasis"]


def test_음수_어긋남은_정상이다():
    """⭐ 발행자 시계가 앞서면 음수다. 실제로 -2,917 ms 를 쟀다(2026-09-12).
       여기서 막으면 진짜 사실을 지우는 것이다 — 막아야 할 건 그걸 '지연'이라 부르는 쪽이다."""
    d = T.describe(sidecar(offset=-2917.0), transport="pull")
    assert d["offsetMs"] == -2917.0
    assert d["offsetWhy"] is None


# ---- ⭐ U-2: 불가능한 값은 숫자로 안 나간다 ------------------------------------------

def test_음수_지연은_사유로_나간다():
    d = T.describe(sidecar(process=-5.0), transport="pull")
    assert d["pipelineMs"] is None, "음수인데 숫자가 나갔다"
    assert d["pipelineWhy"] == T.WHY_NEGATIVE


def test_로컬인데_1초_넘으면_사유로_나간다():
    """로컬 웹캠이 초 단위 내부 지연일 수 없다 — 값을 의심한다."""
    d = T.describe(sidecar(process=1500.0), transport="local")
    assert d["pipelineMs"] is None
    assert d["pipelineWhy"] == T.WHY_IMPLAUSIBLE_LOCAL


def test_같은_값이라도_원격이면_안_막는다():
    """⭐ 원격은 네트워크·큐가 낀다. 로컬의 한계를 원격에 들이대면 멀쩡한 값을 지운다."""
    d = T.describe(sidecar(process=1500.0), transport="pull")
    assert d["pipelineMs"] == 1505.0


def test_자르지_않는다():
    """⭐ 범위로 자르면(clamp) 그럴듯한 숫자가 남고 **틀렸다는 사실이 사라진다**(설계서 §5)."""
    d = T.describe(sidecar(process=-5.0), transport="pull")
    assert d["pipelineMs"] is None
    assert d["pipelineMs"] != 0
    # 부분값은 남겨서 사람이 어디가 이상한지 본다
    assert d["pipelineParts"]["가공"] == -5.0


def test_이상한_입력을_0으로_안_만든다():
    for bad in ("abc", None, float("nan"), float("inf"), True):
        d = T.describe({"source": "capture", "offsetMedianMs": bad,
                        "processMs": bad, "encodeMs": bad})
        assert d["offsetMs"] is None or isinstance(d["offsetMs"], float)
        assert d["pipelineMs"] is None, bad


# ---- 못 재는 것은 못 잰다고 -----------------------------------------------------

def test_촬영시각이_없으면_어긋남을_안_지어낸다():
    d = T.describe({"source": "receive"})
    assert d["offsetMs"] is None
    assert d["offsetWhy"] == T.WHY_NO_HEADERS
    assert d["clockKind"] == "receive"


def test_구간_헤더가_없으면_지연은_미측정이다():
    d = T.describe({"source": "capture", "offsetMedianMs": 100.0})
    assert d["pipelineMs"] is None
    assert d["pipelineWhy"] == T.WHY_NO_HEADERS


def test_전송은_공통_시계가_없으면_미측정이다():
    """⭐ 어긋남에서 시계차를 못 빼면 전송만 떼어낼 수 없다."""
    d = T.describe(sidecar(), transport="pull")
    assert d["transportMs"] is None
    assert d["transportWhy"] == T.WHY_NO_COMMON_CLOCK


def test_시계차를_주면_전송을_낸다():
    """U-9(ROS 세션)가 시계차를 재 오면 그때 채워진다."""
    d = T.describe(sidecar(offset=3120.0), transport="pull", clock_skew_ms=3000.0)
    assert d["transportMs"] == 120.0
    assert d["clockSkewMs"] == 3000.0


def test_시계차를_줘도_지연은_안_변한다():
    """⭐ 지연은 발행자 내부 값이라 시계차와 무관하다 — U-1 과 같은 성질이다."""
    a = T.describe(sidecar(), transport="pull")
    b = T.describe(sidecar(), transport="pull", clock_skew_ms=9999.0)
    assert a["pipelineMs"] == b["pipelineMs"]


# ---- 한계를 값 옆에 적는다 -------------------------------------------------------

def test_로컬_상한이_선언값임을_적었다():
    """⭐ 실측 분포가 아니라 **선언한 한계**다. 그렇게 적어 둬야 나중에 실측으로 바꾼다."""
    import io
    import os
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "gateway_web", "timing.py")
    src = io.open(p, encoding="utf-8").read()
    assert "선언한 한계" in src and "0.18" in src
