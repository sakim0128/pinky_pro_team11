# -*- coding: utf-8 -*-
"""중계 내장 캠 소스 — 실제 장치 없이 검증한다.

capture_factory 를 주입해 가짜 카메라를 끼운다. 그래서 CI 에서도, 캠이 없는
기계에서도 돈다. 실제 장치 검증은 라이브 실측이 따로 한다.

    FakeCapture(프레임 목록) ──> LocalCameraSource ──> (jpeg, stamp, connected)
                                                          │
                                    Source(jpeg_provider=)┘  ← 레지스트리 계약
"""
import io
import json
import os
import tempfile
import time

import numpy as np
import pytest

from local_camera import (LocalCameraSource, load_local_sources,
                          DEFAULT_JPEG_QUALITY)

JPEG_SOI = b"\xff\xd8\xff"


def _wait_for(predicate, timeout=8.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class FakeCapture:
    """cv2.VideoCapture 흉내. read() 가 (ok, ndarray) 를 돌려준다."""

    def __init__(self, fail_after=None, never_ok=False, color=40):
        self.reads = 0
        self.released = False
        self.fail_after = fail_after
        self.never_ok = never_ok
        self._color = color

    def read(self):
        self.reads += 1
        if self.never_ok:
            return False, None
        if self.fail_after is not None and self.reads > self.fail_after:
            return False, None
        img = np.zeros((48, 64, 3), dtype=np.uint8)
        # 프레임마다 픽셀을 바꿔 '정지 화면'과 구분되게 한다.
        img[:] = (self._color, (self.reads * 7) % 255, 90)
        return True, img

    def release(self):
        self.released = True


# ---- 기본 동작 ---------------------------------------------------------------

def test_프레임을_캡처해_jpeg로_보관한다():
    cap = FakeCapture()
    src = LocalCameraSource(device=0, capture_factory=lambda: cap).start()
    try:
        assert _wait_for(lambda: src.stats()["frames"] >= 3)
        jpeg, stamp, connected = src.get_latest_jpeg()
        assert connected is True
        assert jpeg is not None and jpeg.startswith(JPEG_SOI)
        assert stamp > 0
    finally:
        src.stop()


def test_소스_레지스트리_계약을_만족한다():
    src = LocalCameraSource(device=0, capture_factory=FakeCapture).start()
    try:
        assert _wait_for(lambda: src.stats()["frames"] >= 1)
        got = src.get_latest_jpeg()
        assert isinstance(got, tuple) and len(got) == 3
        jpeg, stamp, connected = got
        assert isinstance(jpeg, (bytes, bytearray))
        assert isinstance(stamp, float)
        assert isinstance(connected, bool)
    finally:
        src.stop()


def test_프레임이_실제로_바뀐다_정지화면이_아니다():
    src = LocalCameraSource(device=0, capture_factory=FakeCapture).start()
    try:
        assert _wait_for(lambda: src.stats()["frames"] >= 2)
        a = src.get_latest_jpeg()[0]
        assert _wait_for(lambda: src.get_latest_jpeg()[0] != a, timeout=4.0)
    finally:
        src.stop()


# ---- 정직성 ------------------------------------------------------------------

def test_캠이_없어도_게이트웨이는_죽지_않는다():
    """열기 실패를 예외로 터뜨리면 게이트웨이가 통째로 못 뜬다."""
    def boom():
        raise RuntimeError("장치 없음")

    src = LocalCameraSource(device=99, capture_factory=boom).start()
    try:
        assert _wait_for(lambda: src.stats()["errors"] >= 1, timeout=6.0)
        assert src.get_latest_jpeg() == (None, 0.0, False)
    finally:
        src.stop()


def test_stale_이면_connected_False_이고_프레임을_안_준다():
    now = [1000.0]
    src = LocalCameraSource(device=0, stale_after_s=2.0, clock=lambda: now[0],
                            capture_factory=FakeCapture).start()
    try:
        assert _wait_for(lambda: src.stats()["frames"] >= 1)
        assert src.get_latest_jpeg()[2] is True
        now[0] += 10.0
        jpeg, _, connected = src.get_latest_jpeg()
        assert connected is False, "낡았는데 connected 면 지표가 거짓말한다"
        assert jpeg is None, "낡은 프레임을 최신처럼 내보내면 안 된다"
    finally:
        src.stop()


def test_read_가_계속_실패하면_장치를_다시_연다():
    caps = []

    def factory():
        c = FakeCapture(fail_after=2)
        caps.append(c)
        return c

    src = LocalCameraSource(device=0, capture_factory=factory).start()
    try:
        assert _wait_for(lambda: len(caps) >= 2, timeout=10.0), \
            "연속 실패인데 재오픈하지 않았다"
        assert caps[0].released is True, "옛 핸들을 놓지 않으면 장치를 붙잡고 있다"
    finally:
        src.stop()


def test_target_fps_로_솎아낸다():
    """30 fps 장치를 그대로 인코딩하면 8코어에서 ArUco 에 쓸 CPU 가 없다."""
    src = LocalCameraSource(device=0, target_fps=5,
                            capture_factory=FakeCapture).start()
    try:
        time.sleep(1.2)
        frames = src.stats()["frames"]
        assert 1 <= frames <= 12, "5 fps 목표인데 1.2초에 %d 프레임" % frames
    finally:
        src.stop()


# ---- 설정 -------------------------------------------------------------------

def test_localSources_를_읽는다():
    got = load_local_sources({"localSources": [
        {"id": "relay-cam", "device": 0, "label": "중계 캠", "targetFps": 15},
    ]})
    assert len(got) == 1
    assert got[0]["id"] == "relay-cam"
    assert got[0]["device"] == 0
    assert got[0]["trust"] == "trusted"
    assert got[0]["targetFps"] == 15
    assert got[0]["jpegQuality"] == DEFAULT_JPEG_QUALITY


def test_localSources_가_없으면_빈_목록():
    assert load_local_sources({}) == []


def test_enabled_false_는_건너뛴다():
    got = load_local_sources({"localSources": [
        {"id": "a", "device": 0, "enabled": False},
        {"id": "b", "device": 1},
    ]})
    assert [s["id"] for s in got] == ["b"]


@pytest.mark.parametrize("bad,msg", [
    ({"localSources": [{"device": 0}]}, "id"),
    ({"localSources": [{"id": "a"}]}, "device"),
    ({"localSources": [{"id": "a", "device": 0, "trust": "maybe"}]}, "trust"),
    ({"localSources": [{"id": "a", "device": 0}, {"id": "a", "device": 1}]}, "중복"),
    ({"localSources": "nope"}, "리스트"),
])
def test_형식을_어기면_조용히_버리지_않고_예외(bad, msg):
    with pytest.raises(ValueError) as ei:
        load_local_sources(bad)
    assert msg in str(ei.value)


def test_device_0_은_유효하다_falsy_함정():
    """device=0 을 'if not e.get(device)' 로 검사하면 0 이 빠진다.

    /dev/video0 이 가장 흔한 장치인데 그게 조용히 사라지면 최악이다.
    """
    got = load_local_sources({"localSources": [{"id": "cam", "device": 0}]})
    assert got[0]["device"] == 0


def test_실제_배포된_설정에_relay_cam_이_있다():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo = os.path.dirname(here)
    cfg = os.path.join(repo, "relay_station", "configs", "video_sources.json")
    if not os.path.exists(cfg):
        pytest.skip("configs/video_sources.json 없음")
    with io.open(cfg, encoding="utf-8") as fh:
        doc = json.load(fh)
    got = load_local_sources(doc)
    ids = [s["id"] for s in got]
    assert "relay-cam" in ids, "중계 캠이 설정에 없다 - 항상 있는 유일한 실물 시점이다"
