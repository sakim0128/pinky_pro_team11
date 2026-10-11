# -*- coding: utf-8 -*-
"""FRA-76 — 조합 단의 시각 정밀도 문턱.

여기서 고정하는 성질:

    유도       문턱이 **격자와 속도에서 계산된다** — 손으로 고른 상수가 아니다
    fail-closed `captureClock` 이 없으면 통과가 아니라 UNMEASURABLE
    정직       판정보다 **implied error(cm)** 가 소비자에게 할 말을 준다
"""
import pytest

import fusion_clock as FC


# ---- 문턱이 물리에서 나온다 --------------------------------------------------

def test_문턱이_격자와_속도에서_계산된다():
    """⭐ 손 상수였다면 이 항등식이 깨진다."""
    g, v, n = 2.5, 0.35, 2
    assert FC.skew_budget_ms(g, v, n) == pytest.approx((g / 100.0) / (n * v) * 1000.0)


def test_격자가_커지면_문턱도_커진다():
    """더 성긴 격자는 더 큰 오차를 못 본다 — 그만큼 시계가 헐거워도 된다."""
    assert FC.skew_budget_ms(grid_cm=5.0) == pytest.approx(2 * FC.skew_budget_ms(grid_cm=2.5))


def test_속도가_빠르면_문턱이_좁아진다():
    assert FC.skew_budget_ms(v_mps=0.70) == pytest.approx(FC.skew_budget_ms(v_mps=0.35) / 2)


def test_마주_접근은_하나일_때의_절반이다():
    """상대속도가 두 배라 예산이 절반이다 — MCVA-41 시나리오."""
    assert FC.skew_budget_ms(movers=2) == pytest.approx(FC.skew_budget_ms(movers=1) / 2)


def test_기본값이_레포의_값_셋이다():
    """격자·속도가 바뀌면 문턱이 따라 움직여야 한다 — 여기 박아 두면 안 따라온다."""
    assert FC.GRID_CM == 2.5                 # vision_world.build(res_cm=2.5)
    assert FC.V_MAX_MPS == 0.35              # nav2_agile_params.yaml
    assert FC.skew_budget_ms() == pytest.approx(35.7, abs=0.1)


def test_잘못된_입력은_거부한다():
    for kw in ({"movers": 0}, {"v_mps": 0}, {"grid_cm": 0}, {"v_mps": -1}):
        with pytest.raises(ValueError):
            FC.skew_budget_ms(**kw)


# ---- 판정 --------------------------------------------------------------------

def test_현재_실측_170ms_는_초과다():
    """⭐⭐ 2026-09-11 앱 세션 실측. 이 값이 통과로 바뀌면 문턱이 헐거워진 것이다."""
    st, sev, d = FC.classify(170.0)
    assert st == FC.SKEW_EXCEEDS_GRID
    assert sev == "degraded"
    assert d["impliedErrorCm"] == pytest.approx(11.9, abs=0.05)


def test_예산_안쪽은_통과한다():
    st, sev, _ = FC.classify(30.0)
    assert st == FC.SKEW_WITHIN_GRID and sev == "ok"


def test_경계값은_초과로_친다():
    """⭐ `<` 이지 `<=` 가 아니다 — 격자와 **같은** 오차는 이미 한 칸을 옮긴다."""
    b = FC.skew_budget_ms()
    assert FC.classify(b)[0] == FC.SKEW_EXCEEDS_GRID
    assert FC.classify(b - 0.001)[0] == FC.SKEW_WITHIN_GRID


def test_어느_쪽이_빠른지는_상관없다():
    """어긋남의 부호는 뜻이 없다 — 크기만 문제다."""
    assert FC.classify(-170.0)[0] == FC.SKEW_EXCEEDS_GRID


def test_잴_수_없으면_통과가_아니다():
    """⭐ fail-closed. 못 잰 것을 맞았다고 하지 않는다."""
    st, sev, d = FC.classify(None)
    assert st == FC.SKEW_UNMEASURABLE and sev == "degraded"
    assert d["impliedErrorCm"] is None


# ---- 어긋남 계산 -------------------------------------------------------------

def test_captureClock_이_하나라도_없으면_None_이다():
    """⭐⭐ 빠진 값을 0 으로 치면 **못 잰 것이 잘 맞는 것처럼** 보인다."""
    assert FC.skew_ms_between([1789000000000, None]) is None
    assert FC.skew_ms_between([None]) is None
    assert FC.skew_ms_between([]) is None


def test_소스가_하나면_어긋남은_0_이다():
    assert FC.skew_ms_between([1789000000000]) == 0.0


def test_어긋남은_최대와_최소의_차다():
    assert FC.skew_ms_between([1000, 1170, 1100]) == 170.0


def test_implied_error_가_물리와_맞는다():
    """0.70 m/s 로 1 초면 70 cm."""
    assert FC.implied_error_cm(1000.0, v_mps=0.35, movers=2) == pytest.approx(70.0)


# ---- 보고 --------------------------------------------------------------------

def test_ok_면_저하항목을_안_만든다():
    st, sev, d = FC.classify(10.0)
    assert FC.degradation_for(st, sev, d) is None


def test_초과는_implied_error_까지_보고한다():
    """판정만 주면 소비자가 할 말이 없다."""
    st, sev, d = FC.classify(170.0)
    item = FC.degradation_for(st, sev, d)
    assert item["code"] == "FUSION_" + FC.SKEW_EXCEEDS_GRID
    assert item["impliedErrorCm"] == pytest.approx(11.9, abs=0.05)


# ---- 시계 기준 — 어떤 숫자가 시계 몇 개를 건너나 (U-1/U-9, 2026-09-12) --------
#
# 🔴 라이브: 화면이 `offset` 을 **`지연`** 이라고 적었고 그 값이 18,837.8 ms 였다.
#    같은 소스를 직접 당기면 첫 JPEG 이 0.18 초에 온다. 그 숫자는 지연이 아니었다.

def test_나이는_두_시계를_건넌다():
    assert FC.crosses_clocks("age") == (FC.CLOCK_OBSERVER, FC.CLOCK_SOURCE)


def test_어긋남은_소스끼리만_건넌다():
    """⭐ 그래서 어긋남은 관측자 시계가 약분돼 믿을 수 있다."""
    a, b = FC.crosses_clocks("skew")
    assert a == b == FC.CLOCK_SOURCE


def test_모르는_양은_지어내지_않는다():
    with pytest.raises(ValueError):
        FC.crosses_clocks("아무거나")


def test_오프셋을_모르면_나이를_못_믿는다():
    """⭐⭐ 나이 = 오프셋 + 지연. 오프셋을 모르면 남는 것을 지연이라 부를 수 없다."""
    st, sev, d = FC.classify_age(18837.8)
    assert st == FC.AGE_OFFSET_UNKNOWN and sev == "degraded"
    assert d["latencyMs"] is None, "모르는 지연을 숫자로 채우면 안 된다"


def test_실측_재현_오프셋이_나이를_삼킨다():
    """⭐⭐ 2026-09-12 라이브 값 그대로. 이 조합이 USABLE 로 바뀌면 문턱이 풀린 것이다."""
    st, sev, _ = FC.classify_age(18837.8, 26987.0)
    assert st == FC.AGE_OFFSET_DOMINATES and sev == "degraded"


def test_오프셋이_작으면_나이가_쓸_만하다():
    st, sev, d = FC.classify_age(180.0, 5.0)
    assert st == FC.AGE_USABLE and sev == "ok"
    assert d["latencyMs"] == pytest.approx(175.0), "오프셋을 알면 지연이 뽑힌다"


def test_나이가_없어도_지연을_지어내지_않는다():
    assert FC.classify_age(None, 5.0)[2]["latencyMs"] is None


# ---- 오프셋은 괄호로만 잰다 --------------------------------------------------

def test_괄호가_참값을_담는다():
    """관측자가 1000 에 묻고 21000 에 받았고 상대가 27000 이라 했다."""
    lo, hi, width = FC.offset_bracket_ms(1000, 27000, 21000)
    assert lo <= 26000 <= hi          # 참 오프셋이 무엇이든 이 안에 있다
    assert width == 20000


def test_왕복이_크면_아무것도_말하지_않는다():
    """⭐⭐ 2026-09-12: `docker exec` 왕복 20~27초로 재려 했더니 컨테이너 셋이
    서로 12초씩 다르게 나왔다 — 시계차가 아니라 exec 지연을 잰 것이다.
    괄호가 재려는 값보다 넓으면 그 측정은 **측정이 아니다.**
    """
    wide = FC.offset_bracket_ms(1000, 27000, 21000)
    assert FC.offset_is_resolved(wide, 35.7) is False


def test_왕복이_충분히_좁으면_가른다():
    tight = FC.offset_bracket_ms(1000, 1005, 1010)   # 왕복 10 ms
    assert FC.offset_is_resolved(tight, 35.7) is True


def test_시간이_거꾸로_가는_괄호는_거부한다():
    with pytest.raises(ValueError):
        FC.offset_bracket_ms(1000, 1005, 999)


def test_가르려는_크기가_0_이하면_거부한다():
    b = FC.offset_bracket_ms(0, 0, 10)
    for bad in (0, -1):
        with pytest.raises(ValueError):
            FC.offset_is_resolved(b, bad)
