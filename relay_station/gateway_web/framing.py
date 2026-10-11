# -*- coding: utf-8 -*-
"""MCV-2G — 화각이 쓸 만한가. 거치하는 사람에게 **그 자리에서** 말해 준다.

정착은 카메라를 어디에 두느냐에 달려 있는데, 거치하는 사람은 화면만 본다. 화면은
멀쩡해 보여도 나중에 세 가지가 조용히 망가진다:

    모서리가 가장자리   흔들림 조각(96px)을 **못 뜬다** — 그 모서리는 감시에서 빠진다
    아레나가 작다       1픽셀이 몇 cm 인지가 커진다 — 2cm 문턱을 **잴 수 없게 된다**
    기울었다            수평이 안 맞으면 바닥 평면 가정이 더 나빠진다

⭐⭐ 2026-09-11 라이브에서 실제로 **네 모서리 중 둘이 화면 가장자리에 붙어** 조각을
   못 떴다. 흔들림 감시가 절반만 돌고 있었는데 화면 어디에도 그 말이 없었다.
   이 모듈은 그 사실을 **거치 시점에** 말하려는 것이다.

## ⭐ 가이드 박스를 눈대중 값으로 그리지 않는다

금지 띠의 폭은 `drift.PATCH / 2` 다. 조각을 뜨는 규칙에서 **끌어온다** — 조각 크기를
바꾸면 가이드도 같이 바뀐다. 상수를 베껴 두면 한쪽만 바뀌고 둘 중 하나가 틀린 말을 한다.

## ⭐ 모르는 것은 안 그린다

정착 전에는 원근을 모른다. 그래서 "아레나가 이렇게 보여야 한다"는 **사다리꼴을 못 그린다.**
그릴 수 있는 것은 **담기는 조건**뿐이다 — "네 모서리가 다 이 안에 들어와야 한다".
모양을 맞추라고 하면 거짓말이 되고, 담으라고 하면 참이다.
"""
import math

import drift

# 금지 띠 = 조각 반쪽. 여기 모서리가 오면 조각을 **못 뜬다**(drift.take_patches 참조).
# ⭐ 베끼지 않고 끌어온다.
EDGE_BAND_PX = drift.PATCH // 2

# 권장 상자는 안전 영역에서 이만큼 더 안으로. 광각일수록 가장자리 왜곡이 크고,
# 왜곡 보정은 아직 안 한다(V-2) — 그래서 가장자리를 더 피한다.
INSET_FRACTION = 0.06

# 아레나가 화면에서 이보다 적게 차지하면 정밀도가 급격히 나빠진다.
# ⭐ 튜닝값이 아니라 **문턱에서 끌어온 값**이다: 1픽셀이 DRIFT_CM 보다 커지면
#    2cm 흔들림은 애초에 **잴 수 없는** 양이 된다(아래 precision 참조).
MIN_COVERAGE = 0.10

LEVEL_OK_DEG = 3.0        # 이보다 기울면 말해 준다. 앱이 각도를 보낼 때만 쓴다.


def guide_box(frame_size, band_px=EDGE_BAND_PX, inset=INSET_FRACTION):
    """(safe, target) — 둘 다 화면 비율 {x,y,w,h}.

    safe   이 밖에 모서리가 오면 흔들림 조각을 못 뜬다 (딱딱한 제약)
    target 여기 안에 네 모서리를 다 담으면 좋다 (권장)
    """
    fw, fh = float(frame_size[0]), float(frame_size[1])
    if fw <= 0 or fh <= 0:
        raise ValueError("프레임 크기를 모른다")
    sx, sy = band_px / fw, band_px / fh
    safe = {"x": sx, "y": sy, "w": max(0.0, 1 - 2 * sx), "h": max(0.0, 1 - 2 * sy)}
    tx, ty = sx + inset, sy + inset
    target = {"x": tx, "y": ty, "w": max(0.0, 1 - 2 * tx), "h": max(0.0, 1 - 2 * ty)}
    return safe, target


def corner_problems(corners_px, frame_size, band_px=EDGE_BAND_PX):
    """가장자리에 너무 붙은 모서리들. [(번호, 이유)].

    ⭐ 이 판정은 `drift.take_patches` 의 조건과 **같은 식**이어야 한다.
       다르면 화면은 괜찮다는데 조각은 안 떠지는 일이 생긴다.
    """
    fw, fh = float(frame_size[0]), float(frame_size[1])
    out = []
    for i, (x, y) in enumerate(corners_px or []):
        if x - band_px < 0 or y - band_px < 0 or x + band_px > fw or y + band_px > fh:
            out.append((i + 1, "화면 가장자리에 붙어 흔들림 조각을 못 뜬다"))
    return out


def cm_per_pixel(matrix, px, dpx=1.0):
    """그 화면 자리에서 1픽셀이 몇 cm 인가. 멀수록 크다(원근)."""
    from calibration import project
    a = project(matrix, px[0], px[1])
    b = project(matrix, px[0] + dpx, px[1])
    c = project(matrix, px[0], px[1] + dpx)
    dx = math.hypot(b[0] - a[0], b[1] - a[1])
    dy = math.hypot(c[0] - a[0], c[1] - a[1])
    return max(dx, dy) / dpx


# 정합 최고점은 `cv2.minMaxLoc` 이 주는 **정수 픽셀**이다. 그래서 ±1px 는 방법 자체의
# 양자화지 세상의 움직임이 아니다. 문턱이 그 안이면 잡음과 흔들림을 구분 못 한다.
# ⭐ 여유를 한 칸씩 두어 3px. **튜닝값이 아니라 알고리즘 출력의 낟알**에서 나온 수다.
#    (계획서 MCVA-75 가 "추정치를 상수로 박지 말라"고 한 그 자리다 — 문턱 자체를
#     실측하진 못했지만, **잴 수 있는 문턱인지**는 이렇게 판정할 수 있다.)
MIN_THRESHOLD_PX = 3.0


def precision(matrix, corners_px):
    """네 모서리에서의 cm/px. 가장 나쁜 자리가 이 화각의 한계다.

    ⭐⭐ 이 수가 `drift.DRIFT_CM` 에 가까우면 **그 문턱은 못 재는 양**이다 —
       1~2 픽셀 움직인 것으로 보이는데 그건 정합 최고점의 양자화와 구분이 안 된다.
       문턱을 그대로 두면 감시가 도는 **척**만 한다.
    """
    vals = [cm_per_pixel(matrix, p) for p in (corners_px or [])]
    if not vals:
        return None
    worst = max(vals)
    in_px = (drift.DRIFT_CM / worst) if worst > 0 else 0.0
    return {
        "cmPerPixel": [round(v, 3) for v in vals],
        "worstCmPerPixel": round(worst, 3),
        "driftThresholdCm": drift.DRIFT_CM,
        # 문턱을 재려면 몇 픽셀이 움직여야 하나.
        "thresholdInPixels": round(in_px, 2) if worst > 0 else None,
        "minThresholdPx": MIN_THRESHOLD_PX,
        "minThresholdSource": "cv2.minMaxLoc 은 정수 픽셀 - ±1px 는 방법의 양자화",
        "thresholdMeasurable": bool(in_px >= MIN_THRESHOLD_PX),
    }


def coverage(corners_px, frame_size):
    """아레나 네 모서리가 화면의 몇 할을 감싸나 (신발끈 공식)."""
    if not corners_px or len(corners_px) != 4:
        return None
    fw, fh = float(frame_size[0]), float(frame_size[1])
    if fw <= 0 or fh <= 0:
        return None
    area = 0.0
    n = len(corners_px)
    for i in range(n):
        x1, y1 = corners_px[i]
        x2, y2 = corners_px[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return round(abs(area) / 2.0 / (fw * fh), 4)


def level_from_sidecar(sidecar):
    """앱이 보낸 수평(pitch/roll). **없으면 None** — 지어내지 않는다.

    ⭐ 수평계는 앱 쪽 몫이다(APP-3). 우리가 영상만 보고 기울기를 추정할 수도 있지만,
       그건 추정이고 추정을 관측 자리에 놓지 않는다(금지 조항 6).
    """
    if not sidecar:
        return None
    pitch, roll = sidecar.get("cameraPitch"), sidecar.get("cameraRoll")
    if pitch is None and roll is None:
        return None
    tilted = max(abs(pitch or 0.0), abs(roll or 0.0)) > LEVEL_OK_DEG
    return {"pitchDeg": pitch, "rollDeg": roll,
            "okDeg": LEVEL_OK_DEG, "tilted": tilted}


def assess(frame_size, corners_px=None, matrix=None, sidecar=None):
    """거치하는 사람에게 줄 한 덩어리. 모르는 항목은 **None 으로 남긴다.**"""
    safe, target = guide_box(frame_size)
    out = {
        "frameSize": list(frame_size),
        "safeBox": safe,
        "targetBox": target,
        "edgeBandPx": EDGE_BAND_PX,
        # ⭐ 그 띠가 어디서 왔는지 산출물에 적는다. 나중에 조각 크기를 바꾸면
        #    이 수도 따라 바뀐다는 것을 읽는 사람이 알 수 있게.
        "edgeBandSource": "drift.PATCH/2 (흔들림 조각 반쪽)",
        "level": level_from_sidecar(sidecar),
        "cornerProblems": None,
        "coverage": None,
        "coverageMin": MIN_COVERAGE,
        "precision": None,
        "problems": [],
    }
    if corners_px and len(corners_px) == 4:
        probs = corner_problems(corners_px, frame_size)
        out["cornerProblems"] = [{"corner": i, "why": w} for i, w in probs]
        for i, w in probs:
            out["problems"].append("%d번 모서리: %s" % (i, w))
        cov = coverage(corners_px, frame_size)
        out["coverage"] = cov
        if cov is not None and cov < MIN_COVERAGE:
            out["problems"].append(
                "아레나가 화면의 %.1f%% 밖에 안 된다 (권장 %.0f%% 이상) — "
                "가까이 가거나 화각을 좁힌다" % (100 * cov, 100 * MIN_COVERAGE))
    if matrix and corners_px:
        pr = precision(matrix, corners_px)
        out["precision"] = pr
        if pr and not pr["thresholdMeasurable"]:
            out["problems"].append(
                "가장 먼 모서리에서 1픽셀이 %.2f cm 다 — 흔들림 문턱 %.1f cm 가 "
                "%.1f 픽셀밖에 안 된다(최소 %.0f). 정합 최고점은 정수 픽셀이라 "
                "그 안에서는 잡음과 흔들림을 **구분 못 한다.** 더 가까이 거치한다."
                % (pr["worstCmPerPixel"], drift.DRIFT_CM,
                   pr["thresholdInPixels"] or 0.0, MIN_THRESHOLD_PX))
    lv = out["level"]
    if lv and lv["tilted"]:
        out["problems"].append(
            "카메라가 기울었다 (pitch %.1f° roll %.1f°, 권장 %.0f° 이내)"
            % (lv.get("pitchDeg") or 0.0, lv.get("rollDeg") or 0.0, LEVEL_OK_DEG))
    return out
