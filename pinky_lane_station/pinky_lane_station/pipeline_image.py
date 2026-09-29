"""이미지 전처리·디버그 오버레이 (ROS-free).

새 세그 모델은 학습 때 이미지 상위 30 % 를 검정(0)으로 채웠다 → 추론 입력도 똑같이 마스킹한다.
오버레이는 **마스킹하지 않은 원본** 위에 그린다 (모델이 본 영역은 회색 경계선으로 표시).
"""

import numpy as np

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None

# 클래스별 오버레이 색 (BGR)
CLASS_COLORS = {
    'lane': (0, 255, 0),
    'left_lane': (0, 255, 0),
    'right_lane': (255, 200, 0),
    'crosswalk': (0, 220, 255),
    'cone': (0, 140, 255),
    'traffic_light': (0, 140, 255),
    'barricade': (0, 0, 255),
}
DEFAULT_COLOR = (200, 200, 200)
TARGET_COLOR = (0, 0, 255)
POINT_COLOR = (0, 255, 0)
MASK_LINE_COLOR = (128, 128, 128)


def mask_top_rows(height, frac):
    """마스킹되는 행 수 = int(frac·H). frac 는 [0, 1] 로 클램프."""
    frac = min(1.0, max(0.0, float(frac)))
    return int(frac * int(height))


def mask_top(img, frac=0.30, fill=0):
    """상위 frac 비율의 행을 fill 로 채운 **새 배열** 을 돌려준다. 원본은 바꾸지 않는다."""
    out = np.array(img, copy=True)
    n = mask_top_rows(out.shape[0], frac)
    if n > 0:
        out[:n] = fill
    return out


def draw_debug(img_original, instances, result, mask_frac=0.30, infer_ms=0.0,
               stop_row_frac=0.80, draw_polygons=False):
    """원본 이미지 위에 검출 bbox·차선 중심점·샘플 행·마스크 경계를 그린 새 이미지."""
    if cv2 is None:
        raise RuntimeError('python3-opencv 가 필요합니다')
    dbg = np.array(img_original, copy=True)
    H, W = dbg.shape[:2]

    if draw_polygons:
        overlay = dbg.copy()
        for inst in instances:
            pts = np.array(inst.polygon, dtype=np.int32).reshape(-1, 1, 2)
            if len(pts) >= 3:
                cv2.fillPoly(overlay, [pts], CLASS_COLORS.get(inst.cls, DEFAULT_COLOR))
        cv2.addWeighted(overlay, 0.3, dbg, 0.7, 0, dbg)

    for inst in instances:
        color = CLASS_COLORS.get(inst.cls, DEFAULT_COLOR)
        x0, y0, x1, y1 = (int(round(v)) for v in inst.bbox)
        cv2.rectangle(dbg, (x0, y0), (x1, y1), color, 2)
        cv2.putText(dbg, f'{inst.cls} {inst.conf:.2f}', (x0, max(14, y0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    n_mask = mask_top_rows(H, mask_frac)
    if 0 < n_mask < H:
        for x in range(0, W, 16):                                   # 회색 점선 = 모델이 본 영역의 위 경계
            cv2.line(dbg, (x, n_mask), (min(W - 1, x + 8), n_mask), MASK_LINE_COLOR, 1)

    y = int(result.target_y)
    stop_row = int(stop_row_frac * H)
    cv2.line(dbg, (0, y), (W, y), (255, 255, 0), 1)                 # 샘플 행
    cv2.line(dbg, (0, stop_row), (W, stop_row), (0, 200, 255), 1)   # 횡단보도 정지 행
    cv2.line(dbg, (W // 2, 0), (W // 2, H), (255, 0, 0), 1)         # 화면 중앙선
    if result.left_seen:
        cv2.circle(dbg, (int(result.left_x), y), 5, POINT_COLOR, -1)
    if result.right_seen:
        cv2.circle(dbg, (int(result.right_x), y), 5, POINT_COLOR, -1)
    if result.quality_name not in ('LOST', 'STALE'):
        cv2.circle(dbg, (int(result.target_x), y), 7, TARGET_COLOR, -1)   # 차선 중심점
    cv2.putText(dbg, f'{result.quality_name} e={result.error_x:+.2f} cw={int(result.crosswalk_detected)} '
                     f'half={result.half_lane_px:.0f}px {infer_ms:.0f}ms',
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, TARGET_COLOR, 2)
    return dbg
