# -*- coding: utf-8 -*-
"""MCV-1C — 중계 캠은 필요할 때만 연다 (LED 의 정답은 boot 플래그가 아니다).

다른 세션이 LED 를 끄려고 `--no-camera` 를 기본값으로 박았는데, 그 플래그는 폰·태블릿 pull 까지
끊어 실물 시점이 0 이 됐다. 여기서 고정하는 성질:

    lazy   start() 는 장치를 열지 않는다. 첫 수요가 연다
    idle   마지막 수요 뒤 idle_release_s 가 지나면 장치를 놓는다 (release 호출 = LED 꺼짐)
    probe  상태 조회는 수요가 아니다 — /api/safety 폴링이 카메라를 켜 두면 안 된다
    hold   관측 세션이 잡고 있으면 유휴여도 놓지 않는다
    reopen 놓은 뒤 다시 수요가 오면 다시 연다

각 테스트는 실제 장치 없이 FakeCapture 로 돈다. release() 가 불렸는지가 LED 의 대리 지표다.
"""
import time

import numpy as np

from local_camera import LocalCameraSource, load_local_sources, DEFAULT_IDLE_RELEASE_S


def _wait_for(predicate, timeout=8.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class FakeCapture:
    def __init__(self):
        self.reads = 0
        self.released = False

    def read(self):
        self.reads += 1
        img = np.zeros((48, 64, 3), dtype=np.uint8)
        img[:] = (40, (self.reads * 7) % 255, 90)
        time.sleep(0.005)                       # 실제 카메라처럼 read 가 조금 막힌다
        return True, img

    def release(self):
        self.released = True


class Factory:
    """열릴 때마다 새 FakeCapture 를 만들고 전부 기억한다 — opens 와 release 를 센다."""

    def __init__(self):
        self.caps = []

    def __call__(self):
        cap = FakeCapture()
        self.caps.append(cap)
        return cap


# ---- lazy: start 는 열지 않는다 ---------------------------------------------------

def test_lazy_start_는_장치를_열지_않는다():
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=True).start()
    try:
        time.sleep(0.3)
        assert f.caps == [], "수요가 없는데 열었다 — LED 가 켜진다"
        assert src.is_open() is False
        assert src.stats()["opens"] == 0
    finally:
        src.stop()


def test_lazy_가_아니면_예전처럼_바로_연다():
    """기존 동작을 깨지 않는다 — lazy 는 옵션이다."""
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=False).start()
    try:
        assert _wait_for(lambda: src.stats()["frames"] >= 1)
        assert len(f.caps) == 1
    finally:
        src.stop()


# ---- 수요가 연다 -------------------------------------------------------------------

def test_첫_수요가_장치를_열고_프레임이_온다():
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=True).start()
    try:
        jpeg, _, connected = src.get_latest_jpeg()
        assert jpeg is None and connected is False, "여는 중에는 없다고 정직하게 말한다"
        assert _wait_for(lambda: src.stats()["frames"] >= 1)
        assert len(f.caps) == 1
        jpeg, stamp, connected = src.get_latest_jpeg()
        assert connected is True and jpeg is not None and stamp > 0
    finally:
        src.stop()


def test_probe_는_수요가_아니다_장치를_열지_않는다():
    """이게 핵심이다. /api/safety 가 1초마다 상태를 물어도 카메라가 켜지면 안 된다."""
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=True).start()
    try:
        for _ in range(20):
            jpeg, stamp, connected = src.probe()
            assert jpeg is None and connected is False
            time.sleep(0.01)
        assert f.caps == [], "probe 가 장치를 열었다"
        assert src.stats()["lastDemandAt"] == 0.0
    finally:
        src.stop()


# ---- 유휴면 놓는다 ---------------------------------------------------------------

def test_수요가_끊기면_idle_release_뒤에_장치를_놓는다():
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=True, idle_release_s=0.4).start()
    try:
        src.get_latest_jpeg()
        assert _wait_for(lambda: src.stats()["frames"] >= 1)
        assert _wait_for(lambda: f.caps[0].released, timeout=3.0), "유휴인데 놓지 않았다 — LED 가 안 꺼진다"
        assert _wait_for(lambda: src.is_open() is False, timeout=3.0)
        assert src.stats()["releases"] == 1
        # 놓은 뒤에는 stale 로 떨어져 connected=False 가 된다 (거짓 '살아 있음' 금지)
        assert _wait_for(lambda: src.probe()[2] is False, timeout=5.0)
    finally:
        src.stop()


def test_놓은_뒤_다시_수요가_오면_다시_연다():
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=True, idle_release_s=0.3).start()
    try:
        src.get_latest_jpeg()
        assert _wait_for(lambda: len(f.caps) == 1 and f.caps[0].released, timeout=3.0)
        assert _wait_for(lambda: not src.is_open(), timeout=3.0)
        src.get_latest_jpeg()                    # 두 번째 수요
        assert _wait_for(lambda: len(f.caps) == 2, timeout=3.0), "다시 열지 않았다"
        assert _wait_for(lambda: src.stats()["frames"] >= 2, timeout=3.0)
        assert src.stats()["opens"] == 2
    finally:
        src.stop()


def test_hold_중에는_유휴여도_놓지_않는다():
    """관측 세션은 프레임을 뽑아가지 않아도(관측기가 별도 프로세스) 카메라를 잡아 둬야 한다."""
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=True, idle_release_s=0.2).start()
    try:
        src.hold(True)
        assert _wait_for(lambda: src.stats()["frames"] >= 1)
        time.sleep(0.8)                          # idle_release_s 의 4배
        assert src.is_open() is True
        assert f.caps[0].released is False
        src.hold(False)
        assert _wait_for(lambda: f.caps[0].released, timeout=3.0), "hold 를 풀었는데 놓지 않았다"
    finally:
        src.stop()


def test_stop_뒤에는_수요가_와도_다시_열지_않는다():
    f = Factory()
    src = LocalCameraSource(device=0, capture_factory=f, lazy=True).start()
    src.get_latest_jpeg()
    assert _wait_for(lambda: len(f.caps) == 1)
    src.stop()
    src.get_latest_jpeg()
    time.sleep(0.3)
    assert len(f.caps) == 1, "종료된 소스가 다시 열었다"


# ---- 설정 ---------------------------------------------------------------------------

def test_localSources_는_기본이_lazy_다():
    got = load_local_sources({"localSources": [{"id": "relay-cam", "device": 0}]})
    assert got[0]["lazy"] is True
    assert got[0]["idleReleaseSec"] == DEFAULT_IDLE_RELEASE_S


def test_localSources_로_lazy_를_끌_수_있다():
    got = load_local_sources({"localSources": [
        {"id": "relay-cam", "device": 0, "lazy": False, "idleReleaseSec": 3}]})
    assert got[0]["lazy"] is False
    assert got[0]["idleReleaseSec"] == 3.0
