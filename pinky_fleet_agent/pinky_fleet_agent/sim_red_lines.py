"""시험용 빨간 선 카메라 모형 — 바닥 빨간 선(점)을 로봇 자세에서 본 덩어리 [(x_norm, width_frac, bottom_frac)] 로. ROS-free.

교차로 seek 동작(maneuver.RedLineSeeker)을 폐루프로 돌리는 테스트들(로봇·관제·relay)이 같이 쓴다.
카메라: 전방 near~far(m) 바닥이 화면 하단 1.0 ~ 0.55·H, 수평 반화각 half_fov. 선 폭은 고정 width.
선이 far_bottom(0.80) 행에 오는 거리 = ARRIVE_AHEAD (≈ 0.256 m) — 정지 판정(red_line_stop_row_frac 0.80)과 같은 행.
"""

import math

NEAR, FAR = 0.10, 0.45
TOP_ROW = 0.55
HALF_FOV = math.radians(30.0)
WIDTH = 0.4
STOP_ROW = 0.80
ARRIVE_AHEAD = NEAR + (1.0 - STOP_ROW) / (1.0 - TOP_ROW) * (FAR - NEAR)


def red_blobs(pose, lines):
    x, y, yaw = pose
    out = []
    c, s = math.cos(yaw), math.sin(yaw)
    for lx, ly in lines:
        dx, dy = lx - x, ly - y
        fx, fy = dx * c + dy * s, -dx * s + dy * c
        if not NEAR <= fx <= FAR:
            continue
        xn = -(fy / fx) / math.tan(HALF_FOV)
        if abs(xn) > 1.0:
            continue
        out.append((xn, WIDTH, 1.0 - (fx - NEAR) / (FAR - NEAR) * (1.0 - TOP_ROW)))
    return out


def red_line_detected(blobs):
    """관제 red_line_detected (하단 ≥ 0.80·H) 흉내 — 디바운스 없음."""
    return any(b[2] >= STOP_ROW for b in blobs)


def junction_red_lines(red_x, branch=0.40, straight=0.70, forward=0.20):
    """T자 교차로: x 축을 따라 들어온다. 로봇이 red_x 에서 서도록 입구 선을 red_x + ARRIVE_AHEAD 에 둔다.
    나가는 선: 좌 (red_x+forward, +branch), 우 (red_x+forward, −branch), 직진 (red_x+straight, 0)."""
    return [(red_x + ARRIVE_AHEAD, 0.0), (red_x + forward, branch), (red_x + forward, -branch),
            (red_x + straight, 0.0)]
