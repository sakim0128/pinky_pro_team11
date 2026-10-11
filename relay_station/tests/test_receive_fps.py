# -*- coding: utf-8 -*-
"""MCV-1B5 — **수신** fps 와 **서빙** fps 는 다른 사실이다.

`Source.fps` 는 스트림을 뽑아갈 때만 갱신된다. 그래서 아무도 안 보고 있으면
프레임이 멀쩡히 들어와도 0 이다. 2026-09-10 에 `connected=True fps=0` 이
실제로 사람을 오해시켰다 — "연결됐다는데 왜 0 이지".

    fps          서빙 fps. 보는 사람이 있을 때만 오른다
    receiveFps   **도착한** fps. 보는 사람과 무관하다

⭐ 둘을 한 이름으로 부르면 "끊겼다" 와 "아무도 안 본다" 를 구분 못 한다.
"""
import time

import pytest

from mjpeg_puller import MjpegPuller, RECV_FPS_WINDOW_S, _recv_fps
from fixtures.frames import make_jpeg
from fixtures.mjpeg_server import FixtureMjpegServer


def _wait_for(pred, timeout=8.0, interval=0.02):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(interval)
    return False


# ---- 계산 --------------------------------------------------------------------

def test_두_장_미만이면_0():
    """한 장으로는 간격을 모른다. 숫자를 지어내지 않는다."""
    assert _recv_fps([], 100.0) == 0.0
    assert _recv_fps([99.0], 100.0) == 0.0


def test_간격에서_fps_를_낸다():
    t0 = 100.0
    times = [t0 + i * 0.1 for i in range(11)]      # 10 간격, 1.0 초
    assert _recv_fps(times, times[-1]) == pytest.approx(10.0, abs=0.2)


def test_창_밖은_안_센다():
    now = 100.0
    old = [now - RECV_FPS_WINDOW_S - 5 + i * 0.1 for i in range(10)]
    assert _recv_fps(old, now) == 0.0


def test_멈추면_0_으로_떨어진다():
    """창이 지나면 0 이 된다 — 마지막 값을 붙들면 죽은 소스가 살아 보인다."""
    times = [50.0 + i * 0.1 for i in range(11)]
    assert _recv_fps(times, 50.0 + 1.0) > 0
    assert _recv_fps(times, 50.0 + 1.0 + RECV_FPS_WINDOW_S + 1) == 0.0


# ---- ⭐ 아무도 안 봐도 수신 fps 는 오른다 ----------------------------------------------

def test_아무도_안_봐도_수신_fps_가_오른다():
    """⭐⭐ 이 시험이 이 유닛의 이유다.

    puller 는 스스로 당겨온다. 서빙 fps 는 0 인데 수신은 돌고 있다 —
    그 둘이 같은 이름이면 "끊겼다" 로 오해한다.
    """
    frames = [make_jpeg(color=(30, 40, 50 + i * 20)) for i in range(6)]
    with FixtureMjpegServer(frames, frame_interval_s=0.02, sidecar=True) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.sidecar()["receiveFps"] > 0, timeout=8.0)
            sc = p.sidecar()
            assert sc["receiveFps"] > 0, sc
        finally:
            p.stop()


def test_프레임이_없으면_수신_fps_는_0():
    p = MjpegPuller("http://127.0.0.1:9/video", name="unit")
    assert p.sidecar()["receiveFps"] == 0.0


# ---- 표면 --------------------------------------------------------------------

def test_소스가_fps_의_종류를_말한다():
    """⭐ 이름에 없던 사실을 이름 옆에 둔다."""
    import source_registry as S
    calls = {"n": 0}

    def provider():
        calls["n"] += 1
        return (b"x", time.time(), True)

    src = S.Source("s", provider, S.PULL, S.TRUSTED,
                   sidecar=lambda: {"receiveFps": 12.3})
    st = src.status()
    assert st["fpsKind"] == "served"
    assert st["receiveFps"] == 12.3


def test_곁표가_없는_소스도_안_깨진다():
    """중계 내장 캠처럼 곁표가 얇은 소스도 있다."""
    import source_registry as S
    src = S.Source("s", lambda: (b"x", time.time(), True), S.LOCAL, S.TRUSTED)
    st = src.status()
    assert st["fpsKind"] == "served"
    assert "receiveFps" in st


def test_서빙과_수신이_같은_칸이_아니다():
    import source_registry as S
    src = S.Source("s", lambda: (b"x", time.time(), True), S.PULL, S.TRUSTED,
                   sidecar=lambda: {"receiveFps": 30.0})
    st = src.status()
    assert st["fps"] != st["receiveFps"] or st["fps"] == 0.0
    assert st["fps"] == 0.0, "아직 아무도 안 뽑아갔으니 서빙 fps 는 0 이다"
    assert st["receiveFps"] == 30.0, "그런데 수신은 30 이다 — 이게 요점이다"
