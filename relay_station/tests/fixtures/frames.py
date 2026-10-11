# -*- coding: utf-8 -*-
"""합성 프레임 생성기 — 로봇도 카메라도 없이 관측 파이프라인을 시험하기 위한 것.

    make_aruco_frame(center=(320,240))
        └─> JPEG bytes + truth dict {marker_id, center_px, corners_px}
                                          │
    detect_aruco(jpeg) ───────────────────┘  같은 좌표가 나와야 한다
                                             (오차를 픽셀로 잰다 = MCV-2B 의 판정 방식)

마커 위치를 우리가 **정하고** 만들기 때문에, 검출 결과와 대조할 진리값이 있다.
실제 카메라로는 진리값을 모르므로 이 픽스처가 T8(ArUco 검출)의 유일한 정량 판정 수단이다.
"""
import base64
import json

import cv2
import numpy as np

# 아레나 실측 규격 (FIELD_ARENA_SPECIFICATION.md). 호모그래피 테스트(T7)가 쓴다.
ARENA_W_CM = 270.0
ARENA_H_CM = 125.0

ARUCO_DICT = cv2.aruco.DICT_4X4_50


def _dictionary():
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICT)


def make_jpeg(width=640, height=480, color=(40, 44, 52), quality=85):
    """단색 배경 JPEG. 소스 파이프라인이 '그냥 프레임'을 다루는지 볼 때 쓴다."""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = color
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("cv2.imencode 실패")
    return buf.tobytes()


def make_aruco_frame(marker_id=7, center=(320, 240), marker_px=120,
                     width=640, height=480, quality=90, background=(40, 44, 52)):
    """마커를 **아는 위치**에 그린 프레임과 그 진리값을 함께 돌려준다."""
    if marker_px % 2 != 0:
        raise ValueError("marker_px 는 짝수여야 중심이 정확히 떨어진다")
    cx, cy = center
    half = marker_px // 2
    if cx - half < 0 or cy - half < 0 or cx + half > width or cy + half > height:
        raise ValueError("마커가 프레임 밖으로 나간다: center=%r marker_px=%d" % (center, marker_px))

    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = background

    marker = cv2.aruco.generateImageMarker(_dictionary(), marker_id, marker_px)
    img[cy - half:cy + half, cx - half:cx + half] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("cv2.imencode 실패")

    truth = {
        "marker_id": marker_id,
        "center_px": (float(cx), float(cy)),
        "corners_px": [
            (float(cx - half), float(cy - half)),
            (float(cx + half), float(cy - half)),
            (float(cx + half), float(cy + half)),
            (float(cx - half), float(cy + half)),
        ],
        "marker_px": marker_px,
        "size": (width, height),
    }
    return buf.tobytes(), truth


def detect_aruco(jpeg_bytes):
    """JPEG 에서 마커를 찾아 [(id, (cx, cy))] 로 돌려준다. 못 찾으면 빈 리스트."""
    arr = np.frombuffer(jpeg_bytes, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return []
    detector = cv2.aruco.ArucoDetector(_dictionary(), cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(img)
    if ids is None:
        return []
    out = []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        pts = marker_corners.reshape(-1, 2)
        out.append((int(marker_id), (float(pts[:, 0].mean()), float(pts[:, 1].mean()))))
    return out


def make_corrupt_bytes():
    """JPEG 매직도 없고 디코딩도 안 되는 쓰레기. 수신부가 조용히 삼키는지 본다."""
    return b"not-an-image-at-all-" + bytes(range(32))


def wrap_json_base64(jpeg_bytes, key="image", data_url=False):
    """자체개발 앱이 쓸 수 있는 JSON base64 봉투. 수신부가 실제로 지원한다."""
    b64 = base64.b64encode(jpeg_bytes).decode("ascii")
    if data_url:
        b64 = "data:image/jpeg;base64," + b64
    return json.dumps({key: b64}).encode("utf-8")


def wrap_multipart(jpeg_bytes, boundary=b"----mcvtestboundary"):
    """multipart/form-data 흉내. 수신부는 JPEG 매직으로 잘라낸다."""
    head = (b"--" + boundary + b"\r\n"
            b'Content-Disposition: form-data; name="frame"; filename="f.jpg"\r\n'
            b"Content-Type: image/jpeg\r\n\r\n")
    tail = b"\r\n--" + boundary + b"--\r\n"
    return head + jpeg_bytes + tail
