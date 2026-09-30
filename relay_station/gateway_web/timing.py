# -*- coding: utf-8 -*-
"""U-1/U-2 — **어긋남**과 **지연**은 다른 양이다. 한 칸에 합치지 않는다.

2026-09-12, 화면이 `지연 18837.8ms` 를 보여 줬다. 로컬 웹캠이 18.8초 지연일 수는 없다.
그 수는 지연이 아니라 **컨테이너와 호스트의 시계 어긋남**이었다.

    발행기는 **호스트**에서 `X-Capture-Clock` 을 찍는다
    게이트웨이는 **컨테이너**에서 `now - captureClock` 을 계산한다
    => 그 차이에는 시계 어긋남이 통째로 들어간다

실측(2026-09-12): 관제 세션이 쟀을 때 컨테이너가 **+26,987 ms** 앞섰고, 내가 두 시간 뒤에
쟀을 때는 **-2,917 ms** 였다. 부호까지 뒤집힌다 — Docker Desktop 의 VM 시계가 흔들린다.
⭐ 그래서 **상수로 보정할 수 없다.** 어긋남은 어긋남 칸에 그대로 두고, 지연은 어긋남과
   무관한 방법으로 구해야 한다.

## ⭐⭐ 지연은 **한 시계 안에서만** 잰다

발행자가 보내는 `X-Process-Ms` · `X-Encode-Ms` 는 **발행자 자기 시계**로 잰 구간이다.
빼기가 같은 시계 안에서 끝나므로 **두 기계의 시계가 아무리 어긋나도 안 변한다.**
그래서 이 값들의 합만 `지연` 이라고 부른다. 전송 구간은 공통 시계 없이는 못 재므로
**미측정**으로 남긴다(U-9 가 시계차를 재 오면 그때 채운다).

    어긋남 = now - captureClock      시계차 + 전송 + 큐  (섞여 있다. 그렇게 말한다)
    지연   = processMs + encodeMs    발행자 내부. 시계차와 무관
    전송   = 미측정                  공통 시계가 없다

## ⭐ 불가능한 값은 값이 아니다 — 자르지 않고 **사유**를 낸다

음수 가공 시간은 발행자가 잘못 보낸 것이다. 범위로 자르면(clamp) 그 사실이 사라지고
화면은 그럴듯한 숫자를 보여 준다. 자르지 않고 왜 못 쓰는지 말한다(설계서 §5).
"""

# 로컬 소스(중계에 직접 붙은 장치)의 내부 지연 상한. 이보다 크면 값을 의심한다.
#
# ⚠️ 이건 **선언한 한계**지 실측 분포가 아니다. 근거로 가진 것은 하나 —
#    같은 소스를 직접 당기면 첫 JPEG 이 0.18초에 온다(2026-09-12 관제 세션 실측).
#    1초는 그보다 다섯 배 넉넉한 자리이고, 설계서 U-2 가 그 값을 수락 기준으로 적었다.
#    실기기에서 분포를 재면 이 수를 그 근거로 바꾼다.
LOCAL_MAX_PIPELINE_MS = 1000.0

# 이 소스 종류는 중계가 장치를 **직접** 들고 있다 — 네트워크가 끼지 않는다.
LOCAL_TRANSPORTS = ("local",)

# 어긋남이 이보다 크면 화면이 경고한다. 시계를 맞추라는 신호지 지연 경고가 아니다.
OFFSET_WARN_MS = 500.0

KIND_CAPTURE = "capture"      # 발행자가 촬영 시각을 준다
KIND_RECEIVE = "receive"      # 안 준다 — 수신 시각밖에 없다

WHY_NO_SAMPLES = "NO_SAMPLES"           # 표본이 없다
WHY_NO_HEADERS = "NO_TIMING_HEADERS"    # 발행자가 구간 시간을 안 보낸다
WHY_NEGATIVE = "NEGATIVE"               # 음수 — 발행자가 잘못 보냈다
WHY_IMPLAUSIBLE_LOCAL = "IMPLAUSIBLE_FOR_LOCAL"   # 로컬인데 너무 크다
WHY_NO_COMMON_CLOCK = "NO_COMMON_CLOCK"  # 전송 구간: 공통 시계가 없다


def _num(v):
    """숫자면 float, 아니면 None. 문자열·None 을 조용히 0 으로 만들지 않는다."""
    if isinstance(v, bool) or v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if f != f or f in (float("inf"), float("-inf")):   # NaN·inf
        return None
    return f


def clock_offset(sidecar):
    """어긋남. (ms | None, why) — **지연이 아니다.**

    `now - captureClock` 이라 시계차·전송·큐가 **다 섞여 있다.** 음수도 정상이다
    (발행자 시계가 앞서면 그렇다). 그래서 여기서는 음수를 막지 않는다 —
    막아야 하는 것은 이 값을 **지연이라고 부르는 것**이다.
    """
    sc = sidecar or {}
    if sc.get("source") != KIND_CAPTURE:
        return None, WHY_NO_HEADERS
    v = _num(sc.get("offsetMedianMs"))
    if v is None:
        return None, WHY_NO_SAMPLES
    return v, None


def pipeline_latency(sidecar, transport=None):
    """지연. (ms | None, why, parts) — **발행자 자기 시계로 잰 구간만** 더한다.

    ⭐ 그래서 두 기계의 시계가 어긋나도 이 값은 안 변한다. U-1 의 수락 기준이 그거다.
    """
    sc = sidecar or {}
    parts = {}
    for key, name in (("processMs", "가공"), ("encodeMs", "인코드")):
        v = _num(sc.get(key))
        if v is not None:
            parts[name] = v
    if not parts:
        return None, WHY_NO_HEADERS, parts
    total = sum(parts.values())
    # ⭐ 자르지 않는다. 이상한 값은 계산이나 발행자가 틀렸다는 신호다(설계서 §5).
    if total < 0 or any(v < 0 for v in parts.values()):
        return None, WHY_NEGATIVE, parts
    if transport in LOCAL_TRANSPORTS and total > LOCAL_MAX_PIPELINE_MS:
        return None, WHY_IMPLAUSIBLE_LOCAL, parts
    return total, None, parts


def transport_latency(sidecar, clock_skew_ms=None):
    """전송 구간. 공통 시계가 있어야 잴 수 있다.

    ⭐ `clock_skew_ms` 는 U-9(ROS 세션)가 재 오는 컨테이너-호스트 시계차다.
       없으면 **미측정**이다 — 어긋남에서 시계차를 못 빼면 전송만 떼어낼 수 없다.
    """
    skew = _num(clock_skew_ms)
    if skew is None:
        return None, WHY_NO_COMMON_CLOCK
    off, why = clock_offset(sidecar)
    if off is None:
        return None, why
    return off - skew, None


def describe(sidecar, transport=None, clock_skew_ms=None):
    """화면이 쓸 한 덩어리. **칸을 섞지 않는다.**"""
    off, off_why = clock_offset(sidecar)
    lat, lat_why, parts = pipeline_latency(sidecar, transport)
    tr, tr_why = transport_latency(sidecar, clock_skew_ms)
    return {
        "clockKind": (sidecar or {}).get("source") or KIND_RECEIVE,
        # 어긋남 — 시계차가 섞여 있다는 것을 이름과 설명에 박아 둔다
        "offsetMs": off,
        "offsetWhy": off_why,
        "offsetJitterMs": _num((sidecar or {}).get("offsetJitterMs")),
        "offsetIncludes": "시계차 + 전송 + 큐 (섞여 있다)",
        "offsetWarnMs": OFFSET_WARN_MS,
        # 지연 — 발행자 자기 시계로 잰 구간만
        "pipelineMs": lat,
        "pipelineWhy": lat_why,
        "pipelineParts": parts,
        "pipelineBasis": "발행자 자기 시계 (시계차와 무관)",
        # 전송 — 공통 시계가 있어야 잴 수 있다
        "transportMs": tr,
        "transportWhy": tr_why,
        "clockSkewMs": _num(clock_skew_ms),
    }
