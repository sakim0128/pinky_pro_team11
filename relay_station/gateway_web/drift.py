# -*- coding: utf-8 -*-
"""MCV-2A4 — 정착한 뒤에 카메라가 움직였나.

정착은 **그 순간의 카메라 자세**에 묶여 있다. 삼각대를 누가 치거나 바람에 흔들리면
호모그래피는 그대로인데 세상이 옮겨간다. 그러면 화면은 멀쩡하고 좌표만 조용히 틀린다 —
무효화 3종(아레나·크기·세션)은 **어느 것도 이걸 못 잡는다.** 프레임 크기도 그대로고
세션도 그대로니까.

## ⭐⭐ 전체 화면을 비교하지 않는다 — 모서리 주변만 본다

화면 전체의 유사도(SSIM 같은 것)로 재면 **로봇이 지나가기만 해도 떨어진다.**
그러면 사람이 경보를 끄고, 그 다음에 진짜 흔들림이 와도 안 본다.

정착이 의존하는 것은 **네 모서리 핀이 가리키는 자리**다. 로봇은 아레나 가운데를
돌아다니지 모서리에 붙어 있지 않다. 그래서:

    정착할 때   네 핀 주변을 작은 조각으로 떠 둔다        (기준)
    감시할 때   지금 프레임에서 그 조각을 다시 찾는다      (어디로 갔나)

⭐ 이러면 "무엇이 얼마나 움직였나" 가 **픽셀 단위**로 나온다. 유사도 하나보다
   훨씬 말이 되는 값이고, cm 로 환산도 된다(호모그래피가 있으니까).

## ⭐ 문턱을 상수로 박지 않는다

계획서 §4.7.5 에 적어 둔 그대로다 — "SSIM 70%" 같은 값은 카메라·조명마다 다르다.
정착 직후의 정합도를 **기준선으로 기록**하고, 거기서 얼마나 내려갔는지로 판정한다.
기준선이 없으면 판정하지 않는다(모르는 것을 경보로 바꾸지 않는다).
"""
import io
import json
import os

import cv2
import numpy

STATE_OK = "ANCHORED"          # 기준과 같은 자리에 있다
STATE_DRIFT = "DRIFTED"        # 움직였다
STATE_UNKNOWN = "UNKNOWN"      # 기준이 없거나 못 쟀다

# 왜 그렇게 봤나. ⭐ **문자열이 아니라 이름으로 둔다** — 화면이 이 목록을 손으로
# 베끼면 서버에 사유가 늘어도 화면 시험이 초록이다(무효 사유에서 실제로 그랬다).
WHY_NO_MEASUREMENT = "NO_MEASUREMENT"      # 조각을 하나도 못 쟀다
WHY_NO_BASELINE = "NO_BASELINE"            # 기준선이 없다 - 판정하지 않는다
WHY_MATCH_LOST = "MATCH_LOST"              # 조각 **일부**를 못 찾았다 (국소 가림)
WHY_ALL_LOST = "ALL_PATCHES_LOST"          # 조각을 **전부** 못 찾았다
WHY_SHIFTED = "SHIFTED"                    # 문턱 이상 옮겨갔다
WHY_WITHIN = "WITHIN_THRESHOLD"            # 문턱 안이다

# 정착을 내릴 때 쓰는 사유. calibration 의 무효 3종이 **못 잡는** 자리다.
REASON_CAMERA_MOVED = "CAMERA_MOVED"
# 조각을 전부 놓쳤다. 움직였는지 조명이 바뀌었는지 **모른다** - 그래도 그 정합을
# 확인할 방법이 없으므로 정착을 유지하면 안 된다. 모른다를 움직였다로 부르지도 않는다.
REASON_UNVERIFIABLE = "ANCHOR_UNVERIFIABLE"

# 조각 크기(픽셀) — 모서리 주변에서 이만큼 떠 둔다.
PATCH = 96
# 그 조각을 다시 찾을 때 훑는 범위. 이보다 많이 움직였으면 어차피 정착을 다시 해야 한다.
SEARCH = 64

# 이 정도 어긋나면 움직인 것으로 본다. **픽셀이 아니라 cm 로 판정한다** —
# 같은 픽셀이라도 멀리 있는 모서리는 훨씬 큰 cm 다.
DRIFT_CM = 2.0
# 정합도가 기준선 대비 이만큼 떨어지면 "못 찾았다" 로 본다(가려졌거나 조명이 바뀌었거나).
MATCH_DROP = 0.35

# ⭐⭐ 무늬가 없는 조각은 **기준이 될 수 없다.**
#
# 정규화 상관(TM_CCOEFF_NORMED)의 분모는 **두 표준편차의 곱**이다. 조각이 평평하면
# 분모가 0 이라 상관이 정의되지 않고, OpenCV 는 그냥 1.0 을 돌려준다. 그러면
# minMaxLoc 이 탐색창의 **첫 칸**을 최고점으로 집고, 우리는
#
#     "정합 1.0 으로 62.27 cm 움직였다"
#
# 를 완전한 확신으로 보고한다. 아무것도 아닌 것에 대한 최대 확신이다.
# 2026-09-11 에 실제로 그렇게 나왔다 (dxPx 가 정확히 -SEARCH 였다 — 창 모서리).
# 게다가 기준선 점수도 1.0 이라 MATCH_DROP 도 안 걸린다.
#
# ⭐ 이 값은 튜닝한 문턱이 아니라 **자료형의 한계**다. 8비트 계조에서 표준편차 1 미만은
#    양자화 잡음이고, 거기엔 맞출 무늬가 없다. 현장에서도 민무늬 바닥·흰 벽이면 그렇다.
MIN_PATCH_STD = 1.0


class DriftError(ValueError):
    """사람이 고칠 수 있는 입력 문제."""


def _gray(frame_bgr):
    if frame_bgr is None:
        raise DriftError("프레임이 없다")
    if frame_bgr.ndim == 2:
        return frame_bgr
    return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)


def take_patches(frame_bgr, corners_px, patch=PATCH, masks=None):
    """네 핀 주변 조각을 뜬다. 화면 가장자리에 걸리면 그 모서리는 건너뛴다.

    ⭐ 건너뛴 것을 0 으로 채우지 않는다 — 없는 기준을 만들면 나중에 그걸 비교한다.

    ⭐⭐ **마스크에 걸치는 조각도 안 뜬다**(MCV-2M). 삼각대는 카메라와 한 몸이라
       카메라가 움직여도 같이 움직인다. 그것만이면 별일 아닌 것 같지만 — 누가 책상
       케이블을 건드리면 그 조각의 정합도가 떨어지고, `classify` 는 MATCH_LOST 를
       SHIFTED 보다 **먼저** 본다. 즉 흔들리는 케이블 하나가 **진짜 흔들림을 가린다.**
    """
    import masks as M
    gray = _gray(frame_bgr)
    h, w = gray.shape[:2]
    half = patch // 2
    out = []
    for i, (x, y) in enumerate(corners_px):
        xi, yi = int(round(x)), int(round(y))
        if xi - half < 0 or yi - half < 0 or xi + half > w or yi + half > h:
            out.append(None)
            continue
        rect = (xi - half, yi - half, xi + half, yi + half)
        if M.any_overlaps_rect(masks, rect, (w, h)):
            out.append(None)
            continue
        piece = gray[yi - half:yi + half, xi - half:xi + half].copy()
        # ⭐⭐ 무늬가 없으면 기준으로 안 쓴다 (MIN_PATCH_STD 주석 참조).
        #    없는 근거로 판정하느니 "못 쟀다" 가 낫다.
        if float(piece.std()) < MIN_PATCH_STD:
            out.append(None)
            continue
        out.append(piece)
    return out


def find_patch(frame_bgr, ref, at_px, search=SEARCH):
    """조각이 지금 어디 있나. (score, dx, dy) — 못 찾으면 None.

    정규화 상관으로 찾는다. 조명이 통째로 바뀌어도 구조가 같으면 점수가 유지된다.
    """
    if ref is None:
        return None
    gray = _gray(frame_bgr)
    h, w = gray.shape[:2]
    ph, pw = ref.shape[:2]
    cx, cy = int(round(at_px[0])), int(round(at_px[1]))
    x0 = max(0, cx - pw // 2 - search)
    y0 = max(0, cy - ph // 2 - search)
    x1 = min(w, cx + pw // 2 + search)
    y1 = min(h, cy + ph // 2 + search)
    win = gray[y0:y1, x0:x1]
    if win.shape[0] < ph or win.shape[1] < pw:
        return None
    res = cv2.matchTemplate(win, ref, cv2.TM_CCOEFF_NORMED)
    _mn, mx, _ml, ml = cv2.minMaxLoc(res)
    found_x = x0 + ml[0] + pw / 2.0
    found_y = y0 + ml[1] + ph / 2.0
    # ⭐ 최고점이 탐색창 **테두리**에 찍혔으면 진짜 꼭대기를 가두지 못한 것이다.
    #    그 변위는 측정이 아니라 **하한**이다 — 실제로는 더 갔을 수 있다.
    #    (이걸 그냥 숫자로 보고하면 "64px 움직였다" 로 과소보고가 된다.)
    at_limit = bool(ml[0] <= 0 or ml[1] <= 0
                    or ml[0] >= res.shape[1] - 1 or ml[1] >= res.shape[0] - 1)
    return (float(mx), found_x - at_px[0], found_y - at_px[1], at_limit)


def shift_in_cm(matrix, at_px, dx, dy):
    """화면에서 (dx,dy) 옮겨간 것이 아레나에서 몇 cm 인가.

    ⭐ 픽셀로 판정하면 **멀리 있는 모서리가 유리해진다** — 원근 때문에 같은 cm 가
       더 적은 픽셀이다. 판정은 cm 로 한다.
    """
    from calibration import project
    a = project(matrix, at_px[0], at_px[1])
    b = project(matrix, at_px[0] + dx, at_px[1] + dy)
    return float(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5)


def measure(frame_bgr, refs, corners_px, matrix, search=SEARCH):
    """모서리마다 (score, dxPx, dyPx, shiftCm). 기준이 없는 모서리는 None."""
    out = []
    for ref, at in zip(refs, corners_px):
        got = find_patch(frame_bgr, ref, at, search)
        if got is None:
            out.append(None)
            continue
        score, dx, dy, at_limit = got
        out.append({
            "score": round(score, 4),
            "dxPx": round(dx, 2),
            "dyPx": round(dy, 2),
            "shiftCm": round(shift_in_cm(matrix, at, dx, dy), 2),
            # True 면 그 cm 는 **적어도** 그만큼이라는 뜻이다 (탐색창을 벗어났다).
            "atSearchLimit": at_limit,
        })
    return out


def classify(measured, baseline=None, drift_cm=DRIFT_CM, match_drop=MATCH_DROP):
    """(state, detail). **기준선이 없으면 판정하지 않는다.**

    ⭐ 모르는 것을 경보로 바꾸지 않는다. 기준선은 정착 직후에 기록한다.
    """
    good = [m for m in measured if m]
    detail = {"corners": measured, "baseline": baseline,
              "driftCmThreshold": drift_cm}
    if not good:
        detail["why"] = WHY_NO_MEASUREMENT
        return STATE_UNKNOWN, detail
    if not baseline:
        detail["why"] = WHY_NO_BASELINE
        return STATE_UNKNOWN, detail

    worst_shift = max(m["shiftCm"] for m in good)
    detail["worstShiftCm"] = worst_shift
    # 하한인가. 그렇다면 화면은 "적어도" 라고 말해야 한다.
    detail["atSearchLimit"] = any(m.get("atSearchLimit") for m in good)
    base_scores = baseline.get("scores") or []
    drops = []
    for i, m in enumerate(measured):
        if not m or i >= len(base_scores) or base_scores[i] is None:
            continue
        drops.append(base_scores[i] - m["score"])
    detail["worstScoreDrop"] = round(max(drops), 4) if drops else None

    # ⭐⭐ **못 찾았다를 움직였다보다 먼저 본다.**
    #    조각을 못 찾으면 상관 최고점이 아무 데나 찍히고, 그 변위는 의미가 없다.
    #    그걸 "2.4 cm 움직였다" 로 보고하면 없는 사실을 숫자까지 붙여 말하는 것이다.
    #    가려졌거나 조명이 바뀐 것을 움직였다고 단정하지 않는다.
    lost = [i for i, d in enumerate(drops) if d >= match_drop]
    if lost:
        # ⭐⭐ **몇 개를 놓쳤나로 뜻이 갈린다.**
        #    하나만 놓쳤다 = 국소 가림이다 (로봇이 모서리에 서 있다). 모른다.
        #    전부 놓쳤다  = 국소가 아니다. 세상이 통째로 바뀌었다 - 크게 움직였거나
        #                  조명이 꺼졌거나 렌즈가 가려졌거나. 어느 쪽인지는 모른다.
        #    ⭐ 그래도 **정착을 유지하면 안 된다.** 그 행렬이 맞는지 확인할 방법이
        #      없는데 맞다고 계속 말하는 것이 되기 때문이다(fail-closed).
        #    ⭐ 그렇다고 "카메라가 움직였다" 고 부르지도 않는다 - 모르는 것을
        #      아는 것처럼 말하지 않는다. 사유를 따로 둔다.
        if len(lost) == len(drops):
            detail["why"] = WHY_ALL_LOST
            detail["lostCorners"] = len(lost)
            return STATE_DRIFT, detail
        detail["why"] = WHY_MATCH_LOST
        detail["lostCorners"] = len(lost)
        return STATE_UNKNOWN, detail
    if worst_shift >= drift_cm:
        detail["why"] = WHY_SHIFTED
        return STATE_DRIFT, detail
    detail["why"] = WHY_WITHIN
    return STATE_OK, detail


# ---- 기준 보관 ----------------------------------------------------------------

def save_reference(state_dir, source_id, refs, measured, corners_px):
    """정착 직후의 조각과 그때의 정합도를 남긴다.

    ⭐ 정합도 기준선을 **그 자리에서** 기록한다. 나중에 재면 이미 움직였을 수 있다.
    """
    os.makedirs(state_dir, exist_ok=True)
    npz = os.path.join(state_dir, "drift_%s.npz" % source_id)
    arrays = {}
    present = []
    for i, r in enumerate(refs):
        if r is None:
            present.append(False)
            continue
        present.append(True)
        arrays["p%d" % i] = r
    numpy.savez_compressed(npz, **arrays)
    meta = {
        "sourceId": source_id,
        "cornersPx": [list(map(float, c)) for c in corners_px],
        "present": present,
        "patch": PATCH,
        # 정착 직후의 점수 = 기준선. 상수가 아니라 **그때 잰 값**이다.
        "scores": [(m["score"] if m else None) for m in measured],
    }
    with io.open(os.path.join(state_dir, "drift_%s.json" % source_id),
                 "w", encoding="utf-8") as fh:
        fh.write(json.dumps(meta, ensure_ascii=False, indent=2))
    return meta


def load_reference(state_dir, source_id):
    """(refs, meta) 또는 (None, None). 없으면 감시하지 않는다 — 지어내지 않는다."""
    j = os.path.join(state_dir, "drift_%s.json" % source_id)
    npz = os.path.join(state_dir, "drift_%s.npz" % source_id)
    if not (os.path.exists(j) and os.path.exists(npz)):
        return None, None
    try:
        with io.open(j, encoding="utf-8") as fh:
            meta = json.load(fh)
        data = numpy.load(npz)
    except (OSError, ValueError):
        return None, None
    refs = []
    for i, present in enumerate(meta.get("present") or []):
        refs.append(data["p%d" % i] if present and ("p%d" % i) in data else None)
    return refs, meta


def clear_reference(state_dir, source_id):
    for suffix in (".json", ".npz"):
        p = os.path.join(state_dir, "drift_%s%s" % (source_id, suffix))
        if os.path.exists(p):
            os.remove(p)
