# -*- coding: utf-8 -*-
"""픽스처 자기검증.

픽스처가 틀리면 그 위에 쌓은 모든 테스트가 조용히 거짓말한다.
T8(ArUco 검출)의 판정이 이 파일의 정확성에 통째로 걸려 있으므로 먼저 검증한다.
"""
import urllib.request

import pytest

from fixtures.frames import (ARENA_H_CM, ARENA_W_CM, detect_aruco, make_aruco_frame,
                             make_corrupt_bytes, make_jpeg, wrap_json_base64,
                             wrap_multipart)
from fixtures.mjpeg_server import FixtureMjpegServer, iter_mjpeg_frames

JPEG_SOI = b"\xff\xd8\xff"
JPEG_EOI = b"\xff\xd9"


def test_make_jpeg_는_유효한_jpeg다(jpeg):
    assert jpeg.startswith(JPEG_SOI)
    assert jpeg.endswith(JPEG_EOI)
    assert len(jpeg) > 500


def test_아루코_마커가_그린_자리에서_검출된다(aruco):
    jpeg_bytes, truth = aruco
    found = detect_aruco(jpeg_bytes)
    assert len(found) == 1, "마커 1개를 그렸는데 %d 개가 잡혔다" % len(found)
    marker_id, (cx, cy) = found[0]
    assert marker_id == truth["marker_id"]
    tx, ty = truth["center_px"]
    # JPEG 손실 + 검출 서브픽셀 오차를 감안해도 2 px 안이어야 한다.
    assert abs(cx - tx) <= 2.0 and abs(cy - ty) <= 2.0, \
        "검출 %r 이 진리값 %r 에서 2px 넘게 벗어났다" % ((cx, cy), (tx, ty))


@pytest.mark.parametrize("center", [(160, 120), (480, 360), (100, 400)])
def test_마커_위치를_바꿔도_따라온다(center):
    """상수를 박아 놓고 통과하는 테스트가 아님을 보인다 — 위치를 바꾸면 결과도 바뀐다."""
    jpeg_bytes, truth = make_aruco_frame(center=center)
    found = detect_aruco(jpeg_bytes)
    assert len(found) == 1
    _, (cx, cy) = found[0]
    tx, ty = truth["center_px"]
    assert abs(cx - tx) <= 2.0 and abs(cy - ty) <= 2.0


def test_마커가_없는_프레임에서는_아무것도_안_잡힌다(jpeg):
    assert detect_aruco(jpeg) == []


def test_깨진_바이트는_검출도_예외도_없이_빈결과(  ):
    assert detect_aruco(make_corrupt_bytes()) == []


def test_봉투_헬퍼가_원본_jpeg를_보존한다(jpeg):
    assert JPEG_SOI in wrap_multipart(jpeg)
    body = wrap_json_base64(jpeg)
    assert body.startswith(b"{") and b"image" in body
    body_url = wrap_json_base64(jpeg, data_url=True)
    assert b"data:image/jpeg;base64," in body_url


def test_아레나_규격은_실측값이다():
    """FIELD_ARENA_SPECIFICATION.md 의 270x125 cm. T7 호모그래피가 이 값을 쓴다."""
    assert (ARENA_W_CM, ARENA_H_CM) == (270.0, 125.0)


def test_마커가_프레임_밖이면_만들지_않는다():
    with pytest.raises(ValueError):
        make_aruco_frame(center=(10, 10), marker_px=120)


def test_픽스처_서버가_게이트웨이와_같은_계약을_낸다():
    frames = [make_jpeg(color=(c, c, c)) for c in (10, 60, 110)]
    with FixtureMjpegServer(frames, frame_interval_s=0.005) as srv:
        with urllib.request.urlopen(srv.feed_url("cam1"), timeout=5) as resp:
            ctype = resp.headers.get("Content-Type", "")
            assert "multipart/x-mixed-replace" in ctype
            got = list(iter_mjpeg_frames(resp, max_frames=3, timeout_s=5))
    assert len(got) == 3
    assert all(f.startswith(JPEG_SOI) and f.endswith(JPEG_EOI) for f in got)


def test_픽스처_서버가_소스_목록을_낸다():
    """T2 가 만들 /api/sources 계약을 관측기가 미리 개발할 수 있게 한다."""
    import json
    with FixtureMjpegServer([make_jpeg()], sources=("cam1", "cam2")) as srv:
        with urllib.request.urlopen(srv.base_url + "/api/sources", timeout=5) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    ids = [s["id"] for s in payload["sources"]]
    assert ids == ["cam1", "cam2"]
