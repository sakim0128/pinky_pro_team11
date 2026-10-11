# -*- coding: utf-8 -*-
"""MCV-2V — 카메라 영상에서 아레나 평면도를 만든다. **라이다와 독립으로.**

## 왜 이게 있나 — 순환 논증을 끊는다

    기존   가제보 월드 = f(라이다 맵)      -> 그 월드로 라이다를 검증하면 **순환**
    여기   가제보 월드 = f(카메라 영상)    -> 라이다와 **독립** -> 검증에 쓸 수 있다

MCV 의 전제가 "카메라로 라이다를 의심한다" 였다. 그러려면 카메라만으로 만든 평면도가
있어야 한다. 그게 이 모듈이다. 산출물은 `map2@`(라이다) 것을 **덮어쓰지 않는다** —
`worldVersion` 에 `vision@` 로 출처를 박아 둘이 나란히 남는다(MCVA-73).

## ⭐⭐ 호모그래피는 **바닥 위의 점에만** 맞다

이게 이 모듈에서 가장 중요한 사실이다. 정착된 행렬은 **바닥 평면**의 사상이다.
높이 44 cm 벽은 바닥에 있지 않다. 그 벽의 **면과 윗변**은 바닥 평면으로 사영될 때
**카메라 반대쪽으로 번진다.**

    실제 벽 발자국        카메라에서 본 벽이 평면에 남기는 자국
    ┌──┐                 ┌──────┐   <- 카메라 반대 방향으로 길어진다
    └──┘                 └──────┘

그래서 워프한 그림을 그냥 이진화하면 벽이 **실제보다 두껍게** 나온다.
정확한 발자국은 **벽이 바닥과 만나는 선**(카메라에 가까운 쪽 모서리)이다.

이 판은 실루엣을 그대로 쓴다 — 그래서 결과를 `footprintMethod: silhouette-upper-bound`
로 적는다. **점유의 상한**이라는 뜻이다. 충돌 회피에서 점유를 크게 잡는 것은 안전한
방향이지만, **틀린 것은 틀린 것이다.** 값을 쓰는 쪽이 그걸 알아야 한다.

## ⭐⭐ 안 보이는 곳은 **비어 있는 것이 아니다**

한 시점은 벽 뒤를 못 본다. nav2 맵에는 값이 셋이다 — 비었다 / 막혔다 / **모른다**.
안 보이는 곳을 "비었다" 로 적으면 로봇이 벽 그림자를 가로질러 계획한다. 그래서:

    화면 밖            -> 모른다
    검열 상자 안       -> 모른다 (가려서 못 본다)
    벽 뒤 그림자       -> 모른다 (여기서 광선을 쏴서 구한다)

"비었다" 는 **카메라에서 그 칸까지 직선이 아무 점유도 안 지날 때만** 쓴다.
"모른다" 를 "비었다" 로 접는 것이 이 모듈에서 가장 위험한 실수다.
"""
import io
import json
import math
import time

import cv2
import numpy

# nav2 PGM 규약. 이 셋 말고 다른 값을 쓰지 않는다.
PGM_OCCUPIED = 0
PGM_UNKNOWN = 205
PGM_FREE = 254

WORLD_PREFIX = "vision@"

# 실루엣 그대로 쓴다는 표시. 벽 높이 때문에 **상한**이다(위 독스트링).
FOOTPRINT_SILHOUETTE = "silhouette-upper-bound"

# 테두리와 안쪽의 밝기 차가 이보다 작으면 극성을 **판정하지 않는다**.
POLARITY_MARGIN = 15.0

# 본 칸 중 이 비율을 넘게 벽이면 평면도로 말이 안 된다.
# ⭐ 임의의 수가 아니다 — 줄자 명세(270x125)의 실제 벽 면적에서 나온다:
#   외벽 4cm 테두리 + 중앙 블록 46x70.5 + 격벽 둘 ~= 전체의 20~30%.
#   극성이 뒤집히면 그 자리에 바닥이 들어와 ~89% 가 된다(2026-09-11 실측).
#   그 사이 어디든 되지만, 어느 쪽에도 가깝지 않은 값을 고른다.
MAX_OCCUPIED_FRACTION = 0.75


class VisionWorldError(ValueError):
    """사람이 고칠 수 있는 입력 문제."""


# ---- 평면 표본 ----------------------------------------------------------------

def invert_homography(matrix):
    """아레나 cm -> 화면 px 로 가는 역행렬."""
    m = numpy.asarray(matrix, dtype=numpy.float64)
    if m.shape != (3, 3):
        raise VisionWorldError("호모그래피가 3x3 이 아니다: %r" % (m.shape,))
    det = numpy.linalg.det(m)
    if not numpy.isfinite(det) or abs(det) < 1e-12:
        raise VisionWorldError("역행렬이 없다 (det=%.3g)" % det)
    return numpy.linalg.inv(m)


def cell_centers(arena, res_cm):
    """격자 칸 **중심**의 cm 좌표. (rows, cols, xs, ys).

    행 0 이 아레나 **위쪽**이다 — nav2 PGM 이 위에서 아래로 저장되기 때문이다.
    """
    if res_cm <= 0:
        raise VisionWorldError("해상도가 양수가 아니다")
    cols = int(math.floor(arena.width_cm / res_cm))
    rows = int(math.floor(arena.height_cm / res_cm))
    if cols < 2 or rows < 2:
        raise VisionWorldError("격자가 너무 성기다 (%dx%d)" % (cols, rows))
    xs = (numpy.arange(cols) + 0.5) * res_cm
    # ⭐⭐ 행 0 = 아레나 **위쪽**. 처음엔 오름차순으로 만들어 행 0 이 아래였는데,
    #    PGM 은 위 행부터 저장하고 rect_to_box 도 위가 0 이라고 가정한다.
    #    그 어긋남 때문에 맵과 SDF 가 통째로 상하 반전됐다(2026-09-11, 시험이 잡았다).
    #    한 곳에서 뒤집어 두면 나머지가 전부 같은 뜻이 된다.
    ys = arena.height_cm - (numpy.arange(rows) + 0.5) * res_cm
    return rows, cols, xs, ys


def cell_of(arena, res_cm, x_cm, y_cm):
    """cm -> (row, col). **cm 을 칸으로 바꾸는 곳은 여기 하나다.**

    ⭐ 행 0 이 위쪽이라는 규약을 세 군데에서 따로 적었더니 한 곳(카메라 칸)을 빼먹었고,
       그림자가 반대쪽으로 떨어졌다(2026-09-11). 규약을 여러 번 적으면 그중 하나는 틀린다.
    """
    rows, cols, _xs, _ys = cell_centers(arena, res_cm)
    r = int((arena.height_cm - y_cm) / res_cm)
    c = int(x_cm / res_cm)
    return (max(0, min(rows - 1, r)), max(0, min(cols - 1, c)))


def _drop_box(in_view, u, v, box, wpx, h):
    """그 상자에 떨어지는 칸을 **안 본 것**으로 만든다.

    ⭐ 검열 상자와 마스크가 같은 계산을 쓴다. 뜻은 다르지만(누가 선언했나) 결과는
       같다 - 둘 다 "여기는 안 봤다". 같은 계산을 두 번 적으면 그중 하나가 틀린다.
    """
    bx0 = float(box["x"]) * wpx
    by0 = float(box["y"]) * h
    bx1 = bx0 + float(box["w"]) * wpx
    by1 = by0 + float(box["h"]) * h
    inside = (u >= bx0) & (u < bx1) & (v >= by0) & (v < by1)
    return in_view & ~inside


def sample_plane(frame_bgr, matrix, arena, res_cm, censor_box=None, masks=None,
                 causes=None):
    """각 칸의 cm 좌표를 화면 px 로 되쏴서 밝기를 읽는다.

    (gray, in_view) — in_view 가 False 인 칸은 **모르는** 칸이다.

    ⭐ `causes` 에 dict 를 주면 **모르는 이유별 칸 수**를 채운다(MCVA-76).
       "모른다" 를 한 덩어리로 세면 화각을 넓혀야 하는지 시점을 더해야 하는지
       **답할 수가 없다** — 화면 밖은 화각이 고치고, 가림은 각도가 고친다.
    ⭐ cv2.warpPerspective 로 그림을 통째로 펴지 않는다. 칸마다 되쏘면
       **어느 칸이 화면 밖인지**가 자연스럽게 나오고, 그게 모른다/비었다를 가른다.
    """
    if frame_bgr is None:
        raise VisionWorldError("프레임이 없다")
    inv = invert_homography(matrix)
    rows, cols, xs, ys = cell_centers(arena, res_cm)
    gx, gy = numpy.meshgrid(xs, ys)
    ones = numpy.ones_like(gx)
    pts = numpy.stack([gx, gy, ones], axis=-1).reshape(-1, 3).T     # 3 x N
    proj = inv @ pts
    w = proj[2, :]
    good = numpy.abs(w) > 1e-9
    u = numpy.full(w.shape, -1.0)
    v = numpy.full(w.shape, -1.0)
    u[good] = proj[0, good] / w[good]
    v[good] = proj[1, good] / w[good]

    h, wpx = frame_bgr.shape[:2]
    in_view = good & (u >= 0) & (u < wpx) & (v >= 0) & (v < h)
    if causes is not None:
        # 화면 밖 = **화각이 고칠 수 있는** 몫이다.
        causes["outOfFrame"] = int((~in_view).sum())
        causes["censored"] = 0
        causes["masked"] = 0
    if censor_box:
        # 가려진 자리는 **못 본 것**이다. 흐린 화소를 바닥이라고 읽으면 안 된다.
        _before = in_view
        in_view = _drop_box(in_view, u, v, censor_box, wpx, h)
        if causes is not None:
            causes["censored"] = int((_before & ~in_view).sum())
    for m in (masks or []):
        # MCV-2M. 운영자가 "여긴 바닥이 아니다" 라고 한 자리 - 삼각대·베젤·케이블.
        # ⭐ 빼기만 한다. 그러면 build_grid 가 기본값인 **모른다**로 남긴다.
        #    비었다로 적으면 로봇이 삼각대로 들어간다.
        _before = in_view
        in_view = _drop_box(in_view, u, v, m, wpx, h)
        if causes is not None:
            causes["masked"] += int((_before & ~in_view).sum())

    gray_src = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    ui = numpy.clip(u.astype(numpy.int32), 0, wpx - 1)
    vi = numpy.clip(v.astype(numpy.int32), 0, h - 1)
    gray = gray_src[vi, ui].astype(numpy.float64)
    gray[~in_view] = numpy.nan
    return (gray.reshape(rows, cols), in_view.reshape(rows, cols))


def camera_ground_point(matrix, arena, res_cm):
    """카메라가 바닥 어디쯤에 있는지 — **추정**이다.

    ⭐ 가까울수록 cm 당 화소가 많다. 그래서 화소 밀도가 가장 높은 칸을 카메라 발치의
       대리값으로 쓴다. 내부 파라미터 없이 얻을 수 있는 것 중 가장 단순하고,
       그림자를 쏘는 **방향**을 정하는 데는 이걸로 충분하다.
    ⚠️ 정확한 카메라 위치가 아니다. 산출물에 estimate 라고 적는다.
    """
    inv = invert_homography(matrix)
    rows, cols, xs, ys = cell_centers(arena, res_cm)
    step = max(res_cm * 0.5, 0.5)

    def px(x, y):
        p = inv @ numpy.array([x, y, 1.0])
        if abs(p[2]) < 1e-9:
            return None
        return (p[0] / p[2], p[1] / p[2])

    best = None
    for y in ys:
        for x in xs:
            a = px(x, y)
            b = px(x + step, y)
            c = px(x, y + step)
            if not (a and b and c):
                continue
            # cm -> px 야코비안의 넓이 = 그 자리의 화소 밀도
            area = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            if best is None or area > best[0]:
                best = (area, x, y)
    if best is None:
        raise VisionWorldError("화소 밀도를 못 쟀다 — 행렬이 이상하다")
    return (best[1], best[2])


# ---- 점유·그림자 ---------------------------------------------------------------

def occupancy_from_gray(gray, in_view, wall_is_dark=True, min_range=12.0):
    """밝기로 벽/바닥을 가른다. 문턱은 **본 칸의 분포가 정한다.**

    ⭐ Otsu 를 쓴다. 처음엔 15/85 백분위로 잡았는데 **틀렸다** — 벽이 면적의 5% 면
       두 백분위가 **둘 다 바닥**에 걸려서 "대비가 없다" 가 나온다. 클래스 크기가
       한쪽으로 쏠린 이진화가 바로 Otsu 가 푸는 문제다.

    ⭐ 문턱을 상수로 박지 않는 이유는 그대로다 — 조명이 바뀌면 상수는 틀리고,
       틀린 줄도 모른다.
    """
    seen = gray[in_view]
    # ⭐ Otsu 는 민무늬에서도 숫자를 돌려주므로 이 검사가 먼저 있어야 한다.
    lo, hi = require_contrast(gray, in_view, min_range)
    vals = numpy.clip(seen, 0, 255).astype(numpy.uint8)
    thr, _ = cv2.threshold(vals.reshape(-1, 1), 0, 255,
                           cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thr = float(thr)
    # ⭐ OpenCV 의 Otsu 문턱은 **초과가 전경** 규약이다 — 이중봉 {35, 210} 에서 35 를
    #    돌려준다(낮은 쪽의 최대). 그래서 `< thr` 로 비교하면 경계값을 통째로 놓친다.
    #    2026-09-11 에 벽이 하나도 안 잡혔던 이유가 이 한 글자였다.
    occ = (gray <= thr) if wall_is_dark else (gray > thr)
    occ = occ & in_view
    return occ, {"threshold": round(thr, 2), "min": round(lo, 2), "max": round(hi, 2),
                 "method": "otsu", "wallIsDark": bool(wall_is_dark)}


def require_contrast(gray, in_view, min_range=12.0):
    """벽과 바닥을 가를 만한 밝기 차가 있나. 없으면 거절한다 — 지어내지 않는다."""
    seen = gray[in_view]
    if seen.size < 16:
        raise VisionWorldError("본 칸이 %d 개뿐이라 문턱을 못 정한다" % seen.size)
    lo = float(numpy.nanmin(seen))
    hi = float(numpy.nanmax(seen))
    if hi - lo < min_range:
        raise VisionWorldError("밝기 대비가 없다 (%.1f~%.1f) — 벽과 바닥을 못 가른다"
                               % (lo, hi))
    return lo, hi


def detect_wall_polarity(gray, in_view, border_cells=2):
    """벽이 바닥보다 어두운가. **아레나 바깥 테두리는 언제나 벽이다** — 그걸 쓴다.

    ⭐ 사람이 매번 고르게 두면 언젠가 틀린 값으로 돌린다. 우리가 이미 아는 사실
       (줄자 명세의 외벽)에서 끌어내는 편이 낫다.
    ⚠️ 외벽이 화면 밖이면 판정할 게 없다 → None. 그때는 호출자가 정해야 한다.
    """
    rows, cols = gray.shape
    b = max(1, int(border_cells))
    ring = numpy.zeros(gray.shape, dtype=bool)
    ring[:b, :] = True
    ring[-b:, :] = True
    ring[:, :b] = True
    ring[:, -b:] = True
    inner = ~ring
    rv = gray[ring & in_view]
    iv = gray[inner & in_view]
    if rv.size < 16 or iv.size < 16:
        return None
    rm = float(numpy.nanmedian(rv))
    im = float(numpy.nanmedian(iv))
    # ⭐ 확신이 없으면 **모른다고 한다.** 둘이 비슷하면 외벽이 화면 밖이거나 테두리가
    #    바닥이라는 뜻이고, 그때 억지로 고르면 극성이 뒤집혀 바닥 전체가 벽이 된다.
    if abs(rm - im) < POLARITY_MARGIN:
        return None
    return bool(rm < im)


def sanity_check(grid, in_view):
    """평면도가 말이 되나. **빈 공간이 없는 평면도는 있을 수 없다.**

    ⭐ 극성을 거꾸로 잡으면 바닥 전체가 벽이 된다. 그걸 그대로 내보내면 로봇이
       갈 곳이 없는 월드가 나오고, 사람은 "카메라가 이상한가" 를 한참 본다.
       숫자가 말이 안 되면 **내보내지 않는다.**
    """
    seen = int(in_view.sum())
    if seen < 16:
        raise VisionWorldError("본 칸이 %d 개뿐이다" % seen)
    occ = int((grid == PGM_OCCUPIED).sum())
    free = int((grid == PGM_FREE).sum())
    if free == 0:
        raise VisionWorldError(
            "빈 공간이 하나도 없다 (막힘 %d / 본 칸 %d) — 벽/바닥 밝기가 "
            "뒤집혔을 가능성이 높다. wallIsDark 를 확인한다" % (occ, seen))
    if occ > seen * MAX_OCCUPIED_FRACTION:
        raise VisionWorldError(
            "본 칸의 %.0f%% 가 벽이다 (상한 %.0f%%) — 평면도로 말이 안 된다. "
            "밝기 극성을 확인한다" % (100.0 * occ / seen,
                                    100.0 * MAX_OCCUPIED_FRACTION))
    return {"seen": seen, "occupied": occ, "free": free}


def boundary_ring(shape, cells=2):
    """격자 테두리 고리. 아레나 **외벽**이 여기 온다."""
    rows, cols = shape
    b = max(1, int(cells))
    ring = numpy.zeros(shape, dtype=bool)
    ring[:b, :] = True
    ring[-b:, :] = True
    ring[:, :b] = True
    ring[:, -b:] = True
    return ring


def cast_shadows(occ, in_view, cam_cell, max_steps=None, exclude=None):
    """카메라에서 각 칸까지 직선을 쏴서, 점유를 지나면 **모른다**로 만든다.

    ⭐⭐ 여기가 이 모듈의 안전 장치다. 벽 뒤를 "비었다" 로 적으면 로봇이 그
       그림자를 가로질러 계획한다. 한 시점은 벽 뒤를 **못 본다.**
    """
    rows, cols = occ.shape
    cr, cc = cam_cell
    # ⭐⭐ 광선을 막는 데는 **부풀린** 점유를 쓴다.
    #    정수 격자에서 대각선 광선은 얇은 벽 사이로 새어 나간다 — 그러면 벽 뒤가
    #    "비었다" 로 적히고, 그게 이 모듈에서 가장 위험한 오류다.
    #    의심스러우면 모른다 쪽으로 기운다. 점유를 넓게 잡는 것은 안전한 방향이다.
    blocker = cv2.dilate(occ.astype(numpy.uint8),
                         numpy.ones((3, 3), numpy.uint8), iterations=1).astype(bool)
    # ⭐⭐ 아레나 **외벽은 광선을 막지 않는다.** 카메라는 그 벽 바깥/위에서
    #    안을 내려다본다 — 외벽 너머로 본다는 뜻이다. 바닥 높이 광선 모형에서
    #    외벽을 차단으로 두면 **아레나 전체가 그림자**가 된다(2026-09-11 실측:
    #    비었다 0, 모른다 4249). 이건 모형의 한계이지 세상의 사실이 아니다.
    if exclude is not None:
        # ⭐ 제외 마스크도 **같이 부풀린다.** 안 그러면 부풀린 외벽이 테두리보다 한 칸
        #    더 안쪽까지 번져서, 아레나를 가로지르는 차단선이 그대로 남는다 —
        #    그러면 외벽을 뺀 의미가 없다(2026-09-11: 보이는 칸 1161/5400 이었다).
        exclude = cv2.dilate(exclude.astype(numpy.uint8),
                             numpy.ones((3, 3), numpy.uint8),
                             iterations=1).astype(bool)
        blocker = blocker & ~exclude
    visible = numpy.zeros_like(occ, dtype=bool)
    steps = max_steps or int(math.hypot(rows, cols) * 2)
    for r in range(rows):
        for c in range(cols):
            if not in_view[r, c]:
                continue
            if occ[r, c]:
                visible[r, c] = True        # 벽 자체는 보인다 (앞면을 봤으니까)
                continue
            n = max(abs(r - cr), abs(c - cc))
            n = min(n, steps)
            blocked = False
            for i in range(1, n):
                t = i / float(n)
                rr = int(round(cr + (r - cr) * t))
                ccx = int(round(cc + (c - cc) * t))
                if 0 <= rr < rows and 0 <= ccx < cols and blocker[rr, ccx]:
                    blocked = True
                    break
            visible[r, c] = not blocked
    return visible


def build_grid(occ, in_view, visible):
    """nav2 PGM 값 셋으로 만든다. **모른다가 기본이다.**"""
    grid = numpy.full(occ.shape, PGM_UNKNOWN, dtype=numpy.uint8)
    grid[occ & in_view] = PGM_OCCUPIED
    grid[(~occ) & in_view & visible] = PGM_FREE
    return grid


# ---- 벡터화 (map_to_world 와 같은 규약) ---------------------------------------------

def merge_rects(mask):
    """True 칸을 큰 직사각형으로 묶는다. `pinky_fleet/map_to_world.py` 와 같은 방식."""
    h, w = mask.shape
    used = numpy.zeros_like(mask, dtype=bool)
    rects = []
    for r in range(h):
        c = 0
        while c < w:
            if not mask[r, c] or used[r, c]:
                c += 1
                continue
            c2 = c
            while c2 + 1 < w and mask[r, c2 + 1] and not used[r, c2 + 1]:
                c2 += 1
            r2 = r
            while (r2 + 1 < h and mask[r2 + 1, c:c2 + 1].all()
                   and not used[r2 + 1, c:c2 + 1].any()):
                r2 += 1
            used[r:r2 + 1, c:c2 + 1] = True
            rects.append((r, c, r2, c2))
            c = c2 + 1
    return rects


def rect_to_box(rect, res_m, origin, rows):
    """격자 사각형 -> (cx, cy, sx, sy) 월드 좌표. map_to_world 의 행 뒤집기와 같다."""
    r1, c1, r2, c2 = rect
    size_x = (c2 - c1 + 1) * res_m
    size_y = (r2 - r1 + 1) * res_m
    cx = origin[0] + (c1 + (c2 - c1 + 1) / 2.0) * res_m
    cy = origin[1] + (rows - (r2 + (r2 - r1 + 1) / 2.0)) * res_m
    return cx, cy, size_x, size_y


# ---- 산출물 -------------------------------------------------------------------

def world_version(receipt, clock=time.time):
    """`vision@<정착시각>-<소스>` — 어디서 온 월드인지 이름이 말한다.

    ⭐ `map2@`(라이다)와 **같은 파일에 안 쓴다.** 둘이 나란히 남아야 서로를 대조한다.
    """
    src = (receipt or {}).get("sourceId") or "unknown"
    settled = int((receipt or {}).get("settledAt") or clock())
    return "%s%d-%s" % (WORLD_PREFIX, settled, src)


def to_nav2_map(grid, arena, res_cm, image_name="vision_map.pgm"):
    """(pgm 바이트, yaml 문자열). origin 은 아레나 중심 기준 — 줄자 규약과 같다."""
    rows, cols = grid.shape
    res_m = res_cm / 100.0
    header = ("P5\n# MCV-2V vision-derived occupancy. 205=unknown\n"
              "%d %d\n255\n" % (cols, rows))
    pgm = header.encode("ascii") + grid.tobytes()
    yaml = (
        "image: %s\n"
        "mode: trinary\n"
        "resolution: %.6f\n"
        "origin: [%.4f, %.4f, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        # ⭐⭐ free_thresh 가 0.25 면 nav2 가 205(모른다)를 **비었다로 읽는다.**
        #    negate:0 에서 점유도 p = (255-값)/255 이므로 205 -> p = 0.196 이고
        #    0.196 < 0.25 이면 free 다. 이 모듈이 막으려던 바로 그 오류가
        #    YAML 한 줄에 숨어 있었다(2026-09-11, 시험이 잡았다).
        #    0.15 면 free(254)=0.004 만 걸리고 205 는 두 문턱 사이 = 모른다.
        "free_thresh: 0.15\n"
        % (image_name, res_m,
           -(cols * res_m) / 2.0, -(rows * res_m) / 2.0))
    return pgm, yaml


SDF_LINK = """      <link name="%(name)s">
        <pose>%(cx).4f %(cy).4f %(cz).4f 0 0 0</pose>
        <collision name="c"><geometry><box><size>%(sx).4f %(sy).4f %(sz).4f</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>%(sx).4f %(sy).4f %(sz).4f</size></box></geometry></visual>
      </link>
"""


def to_sdf(rects, arena, res_cm, version, wall_height_m=0.44, provenance=None):
    """벽 사각형 -> Gazebo SDF. **출처를 주석과 이름에 박는다.**"""
    rows = int(math.floor(arena.height_cm / res_cm))
    cols = int(math.floor(arena.width_cm / res_cm))
    res_m = res_cm / 100.0
    origin = (-(cols * res_m) / 2.0, -(rows * res_m) / 2.0)
    links = []
    for i, rect in enumerate(rects):
        cx, cy, sx, sy = rect_to_box(rect, res_m, origin, rows)
        links.append(SDF_LINK % {"name": "wall_%03d" % i, "cx": cx, "cy": cy,
                                 "cz": wall_height_m / 2.0, "sx": sx, "sy": sy,
                                 "sz": wall_height_m})
    note = json.dumps(provenance or {}, ensure_ascii=False, indent=2)
    return (
        '<?xml version="1.0" ?>\n'
        '<sdf version="1.8">\n'
        '  <!-- %s\n'
        '       ⭐ 이 월드는 **카메라 영상**에서 나왔다. 라이다 맵의 파생물이 아니다.\n'
        '       그래서 라이다를 검증하는 데 쓸 수 있다 (순환이 아니다).\n'
        '       출처:\n%s\n  -->\n'
        '  <world name="%s">\n'
        '    <model name="arena_walls">\n'
        '      <static>true</static>\n'
        '%s'
        '    </model>\n'
        '  </world>\n'
        '</sdf>\n' % (version, note, version, "".join(links))
    )


# ---- 조립 --------------------------------------------------------------------

def build(frame_bgr, calib_state, arena, res_cm=2.5, censor_box=None,
          wall_is_dark=None, receipt=None, clock=time.time, masks=None):
    """정착된 정합에서 평면도를 만든다. **정착이 아니면 거부한다**(MCVA-72).

    돌려주는 것: {"grid", "rects", "sdf", "pgm", "yaml", "version", "provenance"}
    """
    from calibration import STATE_SETTLED
    if not calib_state or calib_state.get("state") != STATE_SETTLED:
        raise VisionWorldError(
            "정착되지 않았다 (%s) — 평면이 없으면 벽 좌표도 없다"
            % (calib_state or {}).get("state"))
    matrix = calib_state.get("homography")
    if not matrix:
        raise VisionWorldError("정착 상태인데 행렬이 없다")

    unknown_by = {}
    gray, in_view = sample_plane(frame_bgr, matrix, arena, res_cm, censor_box,
                                 masks=masks, causes=unknown_by)
    # ⭐ 극성을 기본값으로 두지 않는다. 테두리(= 언제나 벽)에서 끌어낸다.
    # ⭐ 대비 검사를 **극성 판정보다 먼저** 한다. 벽과 바닥을 아예 못 가르는 화면에서
    #    "밝은 쪽이 벽인가" 를 묻는 것은 순서가 뒤바뀐 질문이다.
    require_contrast(gray, in_view)
    polarity_src = 'explicit'
    if wall_is_dark is None:
        wall_is_dark = detect_wall_polarity(gray, in_view)
        polarity_src = 'auto-border'
        if wall_is_dark is None:
            raise VisionWorldError(
                '벽이 밝은지 어두운지 판정할 수 없다 (외벽이 화면 밖이다) — '
                'wallIsDark 를 명시한다')
    occ, thr_info = occupancy_from_gray(gray, in_view, wall_is_dark)
    rows, cols = occ.shape
    cam_xy = camera_ground_point(matrix, arena, res_cm)
    cam_cell = cell_of(arena, res_cm, cam_xy[0], cam_xy[1])
    ring = boundary_ring(occ.shape)
    visible = cast_shadows(occ, in_view, cam_cell, exclude=ring)
    # 가림 = **각도가 고치는** 몫이다. 화각을 아무리 넓혀도 벽 뒤는 안 보인다.
    unknown_by["shadowed"] = int((in_view & ~visible).sum())
    grid = build_grid(occ, in_view, visible)
    sanity = sanity_check(grid, in_view)

    rects = merge_rects(occ & in_view)
    version = world_version(receipt or calib_state, clock)
    seen = int(in_view.sum())
    provenance = {
        "worldVersion": version,
        "derivedFrom": "camera",
        "sourceId": (receipt or calib_state or {}).get("sourceId"),
        "arenaVersion": getattr(arena, "version", None),
        "settledAt": (receipt or calib_state or {}).get("settledAt"),
        "resolutionCm": res_cm,
        "gridSize": [cols, rows],
        # ⭐ 실루엣을 그대로 썼다 = 벽 높이 때문에 **점유의 상한**이다.
        "footprintMethod": FOOTPRINT_SILHOUETTE,
        "cameraGroundPointCm": [round(cam_xy[0], 1), round(cam_xy[1], 1)],
        "cameraGroundPointIs": "estimate-from-pixel-density",
        # ⭐ 그림자 모형의 한계를 산출물에 적는다. 카메라는 바닥에 있지 않고
        #    **위에서 내려다본다** — 높이를 모르므로 바닥 높이 광선으로 근사했다.
        #    그 근사는 안쪽 벽 뒤를 **실제보다 길게** 그림자로 잡고(안전한 방향),
        #    외벽은 아예 차단에서 뺐다(그러지 않으면 전부 모른다가 된다).
        "shadowModel": "floor-level-rays;boundary-excluded",
        "shadowCaveat": "camera height unknown - interior shadows are an over-estimate",
        "coverage": {
            "cellsTotal": int(rows * cols),
            "cellsSeen": seen,
            "cellsUnknown": int((grid == PGM_UNKNOWN).sum()),
            "cellsFree": int((grid == PGM_FREE).sum()),
            "cellsOccupied": int((grid == PGM_OCCUPIED).sum()),
            "seenFraction": round(seen / float(rows * cols), 4),
            # ⭐⭐ MCVA-76 — "모른다" 를 **원인별로** 가른다.
            #    outOfFrame  화면 밖   -> 화각을 넓히면 준다
            #    shadowed    벽 뒤     -> 화각으로는 **안 준다**. 시점을 더해야 한다
            #    censored    검열 상자 -> 규칙이 정한다
            #    masked      운영자 선언
            #    이 넷을 안 가르면 "화각을 키울까 시점을 더할까" 에 답할 수 없다.
            "unknownBy": dict(unknown_by),
        },
        "threshold": thr_info,
        "wallIsDark": bool(wall_is_dark),
        "wallIsDarkSource": polarity_src,
        "sanity": sanity,
        "censorBox": (dict(censor_box) if censor_box else None),
        # MCV-2M. ⭐ 라벨까지 남긴다. 안 그러면 나중에 이 평면도를 본 사람이
        #    구멍을 보고 **관측이 실패했다**고 읽는다. 실은 일부러 안 본 것이다.
        "masks": [dict(m) for m in (masks or [])],
        "maskedMeans": "not-observed (unknown), never free",
        "builtAt": clock(),
    }
    pgm, yaml = to_nav2_map(grid, arena, res_cm)
    return {
        "grid": grid,
        "rects": rects,
        "sdf": to_sdf(rects, arena, res_cm, version, provenance=provenance),
        "pgm": pgm,
        "yaml": yaml,
        "version": version,
        "provenance": provenance,
    }
