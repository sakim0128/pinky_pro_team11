# -*- coding: utf-8 -*-
"""FRA-76 — 두 시점을 한 장면으로 합칠 때 **시계가 얼마나 맞아야 하는가**.

## 문턱은 취향이 아니라 물리에서 나온다

시계가 Δt 어긋난 두 시점을 한 장면으로 합치면, 그 장면 안에서 움직이는 것은

    e = v_rel · Δt

만큼 어긋난 자리에 놓인다. 출력이 쓰이는 격자가 g 일 때 **e < g 인 오차는 출력에
나타날 수 없다.** 그래서 유일하게 튜닝이 아닌 문턱은

    Δt_max = g / v_rel

이고, 값 셋이 전부 레포에 박혀 있다.

    g       2.5 cm     `vision_world.build(res_cm=2.5)` — 평면도 격자
    v       0.35 m/s   `nav2_agile_params.yaml: desired_linear_vel`
                       (프로필 셋 중 최대. cautious 0.18 · 기본 0.2 · agile 0.35)
    v_rel   2v         두 로봇이 마주 접근하는 경우 — MCVA-41 이 그 시나리오다

⭐ 그래서 이 모듈에는 **손으로 고른 상수가 없다.** 전부 위 셋에서 계산된다.
   속도 프로필이나 격자가 바뀌면 문턱이 따라 움직인다.

## ⭐⭐ 어긋남(skew)과 지연(latency)은 다른 양이다

    어긋남   **어디에** 있다고 보느냐를 틀리게 한다   -> 이 모듈
    지연     **언제** 아느냐를 늦춘다                 -> MCVA-41 의 2 초 예산

예산이 다르고 처방이 다르다. 둘을 한 숫자로 묶으면 어느 쪽도 못 고친다.

⭐ 어긋남에 파이프라인 적체가 **안 섞인다**는 것은 산술로 보장된다 —
   `age = now − captureClock` 에서 `captureClock` 이 약분되고 순수 벽시계 차만 남는다.
   (앱 세션 2026-09-11 실측 근거)

## ⭐ 못 잰 것을 맞았다고 하지 않는다

`captureClock` 이 없는 소스는 어긋남을 **잴 수 없다.** 그때는 통과가 아니라
`UNMEASURABLE` 이다 — 빈 목록이 "다 믿는다" 로 떨어지지 않는 것과 같은 이유다.
"""

# ---- 레포에서 온 값 셋 -------------------------------------------------------

GRID_CM = 2.5          # vision_world.build(res_cm=2.5)
V_MAX_MPS = 0.35       # nav2_agile_params.yaml — 프로필 셋 중 최대
APPROACHING_MOVERS = 2 # MCVA-41: 두 로봇이 마주 접근

# ---- 상태 --------------------------------------------------------------------

SKEW_WITHIN_GRID = "SKEW_WITHIN_GRID"        # 오차가 격자보다 작다 — 출력에 안 나타난다
SKEW_EXCEEDS_GRID = "SKEW_EXCEEDS_GRID"      # 격자를 넘는다 — 합치면 없는 어긋남을 만든다
SKEW_UNMEASURABLE = "SKEW_UNMEASURABLE"      # captureClock 이 없다 — 잴 수 없다

SEVERITY = {
    SKEW_WITHIN_GRID: "ok",
    SKEW_EXCEEDS_GRID: "degraded",
    SKEW_UNMEASURABLE: "degraded",
}


def skew_budget_ms(grid_cm=GRID_CM, v_mps=V_MAX_MPS, movers=APPROACHING_MOVERS):
    """Δt_max = g / (movers · v). 단위를 맞춰 ms 로 돌려준다.

    movers=1 은 정지한 아레나 안에서 로봇 하나가 움직이는 경우,
    movers=2 는 둘이 마주 접근하는 경우다(상대속도가 두 배).
    """
    if movers < 1:
        raise ValueError("movers 는 1 이상이어야 한다")
    if v_mps <= 0 or grid_cm <= 0:
        raise ValueError("격자와 속도는 양수여야 한다")
    return (grid_cm / 100.0) / (movers * v_mps) * 1000.0


def implied_error_cm(skew_ms, v_mps=V_MAX_MPS, movers=APPROACHING_MOVERS):
    """그 어긋남이 실제로 만드는 위치 오차. **판정보다 이 값이 더 쓸모 있다** —
    "170 ms 초과" 보다 "11.9 cm 어긋나 보인다" 가 소비자에게 할 말을 준다."""
    return (movers * v_mps) * (skew_ms / 1000.0) * 100.0


def skew_ms_between(capture_clocks):
    """소스들의 `captureClock`(epoch ms) 중 최대−최소. 하나라도 없으면 **None**.

    ⭐ 빠진 값을 0 으로 치면 어긋남이 작아 보인다 — 못 잰 것과 잘 맞는 것을
       같은 숫자로 만들면 안 된다.
    """
    if not capture_clocks:
        return None
    vals = list(capture_clocks)
    if any(v is None for v in vals):
        return None
    if len(vals) < 2:
        return 0.0
    return float(max(vals) - min(vals))


def classify(skew_ms, grid_cm=GRID_CM, v_mps=V_MAX_MPS, movers=APPROACHING_MOVERS):
    """(state, severity, detail). 합치기를 **막지 않는다** — 말을 정하는 데 쓴다.

    시청·관측을 끊는 것은 이 모듈의 일이 아니다(검열 판정과 같은 규약).
    """
    budget = skew_budget_ms(grid_cm, v_mps, movers)
    detail = {
        "skewMs": skew_ms,
        "budgetMs": round(budget, 1),
        "gridCm": grid_cm,
        "vMps": v_mps,
        "movers": movers,
        "impliedErrorCm": (None if skew_ms is None
                           else round(implied_error_cm(skew_ms, v_mps, movers), 2)),
    }
    if skew_ms is None:
        return SKEW_UNMEASURABLE, SEVERITY[SKEW_UNMEASURABLE], detail
    state = SKEW_WITHIN_GRID if abs(skew_ms) < budget else SKEW_EXCEEDS_GRID
    return state, SEVERITY[state], detail


def degradation_for(state, severity, detail):
    """readiness 의 degradations 에 넣을 항목. 문구는 내지 않는다 — UI 소유."""
    if severity == "ok":
        return None
    return {
        "code": "FUSION_" + state,
        "severity": severity,
        "skewMs": detail.get("skewMs"),
        "budgetMs": detail.get("budgetMs"),
        "impliedErrorCm": detail.get("impliedErrorCm"),
    }


# ============================================================================
# 시계 기준 — 어떤 숫자가 **시계 몇 개를 건너는가** (U-1/U-9 공통 정의, 2026-09-12)
# ============================================================================
#
# 🔴 라이브에서 드러난 것: 화면이 `offset`(어긋남)을 **`지연`** 이라고 적고 있었고
#    (당시 `static/index.html` 의 지연 표시), 그 값이 18,837.8 ms 였다. 같은 소스를 직접 당기면
#    첫 JPEG 이 0.18 초에 온다. **그 숫자는 지연이 아니었다.**
#
# ⭐⭐ 가르는 기준은 "무엇을 재나" 가 아니라 **시계를 몇 개 건너나** 다.
#
#   양                  식                          건너는 시계        믿을 수 있나
#   ──────────────────────────────────────────────────────────────────────────
#   소스 간 어긋남      captureClock_A − _B          소스 ↔ 소스        관측자 시계가 약분된다
#   시계 오프셋         now_관측자 − now_소스        관측자 ↔ 소스      이것이 **재야 할 값**이다
#   나이(age)           now_관측자 − captureClock    관측자 ↔ 소스      = 오프셋 + 지연. **섞여 있다**
#   지연(latency)       실제 전송 시간               0 (같은 시계)      오프셋을 알아야 뽑힌다
#
# ⭐ 앱 세션이 준 산술 — *"age 에서 captureClock 이 약분되고 순수 벽시계 차만 남는다"* —
#   는 **어긋남에는 맞고 나이 하나에는 안 맞는다.** 두 소스의 나이를 빼면 관측자의
#   `now` 가 약분되지만(그래서 어긋남은 믿을 수 있다), 나이 하나에는 관측자 시계가
#   그대로 남는다. **같은 식이 한 쓰임에서는 참이고 다른 쓰임에서는 허구다.**
#   판별자는 하나다 — **관측자 시계가 약분되는가.**
#
# ⚠️ 2026-09-12 실측 주의: 호스트↔컨테이너 오프셋을 `docker exec` 로 재려 했더니
#    왕복이 20~27초로 **재려는 값보다 컸다**. 같은 커널 시계를 쓰는 컨테이너 셋이
#    서로 12초씩 다르게 나왔다 — 그건 시계차가 아니라 exec 지연이다.
#    ⭐ **오프셋은 왕복으로 괄호를 쳐서만 잰다**(`offset_bracket_ms`). 괄호가
#       재려는 값보다 넓으면 그 측정은 **아무것도 말하지 않는다.**

CLOCK_OBSERVER = "observer"     # 이 값을 계산한 쪽 (게이트웨이/컨테이너)
CLOCK_SOURCE = "source"         # 프레임을 찍은 쪽 (기기/호스트 발행기)

# 나이(age) 판정
AGE_USABLE = "AGE_USABLE"                         # 오프셋을 알고, 작다
AGE_OFFSET_UNKNOWN = "AGE_OFFSET_UNKNOWN"         # 오프셋을 모른다 — 나이는 허구일 수 있다
AGE_OFFSET_DOMINATES = "AGE_OFFSET_DOMINATES"     # 오프셋이 나이보다 크다 — 값이 뜻을 잃었다

AGE_SEVERITY = {
    AGE_USABLE: "ok",
    AGE_OFFSET_UNKNOWN: "degraded",
    AGE_OFFSET_DOMINATES: "degraded",
}

# 어떤 양이 어느 시계를 건너는지 — 화면이 칸 옆에 적을 값 (U-9)
CROSSES_CLOCKS = {
    "skew": (CLOCK_SOURCE, CLOCK_SOURCE),
    "offset": (CLOCK_OBSERVER, CLOCK_SOURCE),
    "age": (CLOCK_OBSERVER, CLOCK_SOURCE),
    "latency": (CLOCK_SOURCE, CLOCK_SOURCE),
}


def crosses_clocks(quantity):
    """이 양이 건너는 시계 둘. 모르는 이름은 **지어내지 않는다.**

    ⭐ 화면은 숫자만 적으면 안 된다 — `지연 18837.8ms` 가 그래서 거짓말이 됐다.
       시계를 건너는 숫자는 **어느 시계를 건넜는지 함께** 적어야 읽는 사람이
       그 값을 믿을지 판단할 수 있다 (U-9 수락).
    """
    if quantity not in CROSSES_CLOCKS:
        raise ValueError("모르는 양: %r" % (quantity,))
    return CROSSES_CLOCKS[quantity]


def offset_bracket_ms(before_ms, remote_ms, after_ms):
    """왕복으로 **괄호 친** 오프셋 (lo, hi, width).

    관측자가 `before` 에 묻고 `after` 에 답을 받았고 상대가 `remote` 라고 했다면
    참 오프셋은 `[remote − after, remote − before]` 안에 있다.

    ⭐⭐ 한 점으로 내지 않는다. 2026-09-12 에 `docker exec` 왕복(20~27초)이
       재려던 값보다 커서, 컨테이너 셋이 서로 12초씩 다른 **허구의 오프셋**이 나왔다.
       괄호를 내면 그 상태가 `width` 로 **보인다.**
    """
    if after_ms < before_ms:
        raise ValueError("after 가 before 보다 앞설 수 없다")
    lo = remote_ms - after_ms
    hi = remote_ms - before_ms
    return (lo, hi, after_ms - before_ms)


def offset_is_resolved(bracket, need_ms):
    """그 괄호가 `need_ms` 규모의 값을 가를 만큼 좁은가.

    ⭐ 괄호가 재려는 값보다 넓으면 **아무것도 말하지 않는다.** 넓은 괄호를
       '측정했다' 로 세면 그게 곧 허구다.
    """
    if need_ms <= 0:
        raise ValueError("가르려는 크기는 양수여야 한다")
    return bracket[2] < need_ms


def classify_age(age_ms, offset_ms=None):
    """나이가 쓸 만한가. **나이 = 오프셋 + 지연** 이라 오프셋을 모르면 못 믿는다.

    (state, severity, detail)
    """
    detail = {
        "ageMs": age_ms,
        "offsetMs": offset_ms,
        "crosses": list(crosses_clocks("age")),
        # 오프셋을 알면 지연을 뽑을 수 있다. 모르면 **없다** — 0 으로 채우지 않는다.
        "latencyMs": (None if (age_ms is None or offset_ms is None)
                      else age_ms - offset_ms),
    }
    if age_ms is None or offset_ms is None:
        return AGE_OFFSET_UNKNOWN, AGE_SEVERITY[AGE_OFFSET_UNKNOWN], detail
    if abs(offset_ms) >= abs(age_ms):
        # 오프셋이 나이만큼 크면 남는 지연이 음수이거나 0 이다 — 값이 뜻을 잃었다.
        return AGE_OFFSET_DOMINATES, AGE_SEVERITY[AGE_OFFSET_DOMINATES], detail
    return AGE_USABLE, AGE_SEVERITY[AGE_USABLE], detail
