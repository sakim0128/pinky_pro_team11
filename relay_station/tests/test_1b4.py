# -*- coding: utf-8 -*-
"""MCV-1B4 — 가공 소요 시간(X-Process-Ms)을 곁표에 넣는다.

2026-09-10 실기기: 폰 영상이 2.8초 늦는 원인을 **이 헤더 하나가 갈랐다.**

    폰      X-Process-Ms 44~51 ms   JPEG 183 KB   offset 2432~2837 ms
    태블릿  X-Process-Ms 15~25 ms   JPEG 135 KB   offset 21~98 ms

가공이 프레임당 50 ms 를 쓰면 30 fps 입력을 18 fps 로밖에 못 내보내고 그 차이가 큐로 쌓인다.
그런데 이 헤더는 **계약(CAMERA_SOURCE_LOOPBACK_v1)에 없다** - 앱이 보내고 있었을 뿐이라
아무도 안 보고 있었다. 상시로 보면 같은 병목을 다음엔 즉시 짚는다.

⭐ 이 값은 `offsetMedianMs` 안에 들어 있는 **아는 성분**이다.
   나머지가 전송·큐라는 것을 이 값이 있어야 말할 수 있다 - 그게 이 유닛의 존재 이유다.
"""
import time

import pytest

from mjpeg_puller import (MjpegPuller, parse_part_headers, CLOCK_WINDOW_S,
                          H_CAPTURE_CLOCK, H_PROCESS_MS, H_PROCESS_RULES)
from fixtures.frames import make_jpeg
from fixtures.mjpeg_server import FixtureMjpegServer


def _frames(n):
    return [make_jpeg(color=(40, 44, 40 + i * 30)) for i in range(n)]


def _wait_for(predicate, timeout=8.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _puller(clock=None):
    return MjpegPuller("http://127.0.0.1:9/video", name="unit", clock=clock or time.time)


# ---- 파서 -------------------------------------------------------------------

def test_가공_헤더를_뽑는다():
    pre = (b"--raasframe\r\nContent-Type: image/jpeg\r\n"
           b"X-Process-Ms: 43.9\r\nX-Process-Rules: v1@04:17:32\r\n\r\n")
    got = parse_part_headers(pre)
    assert got[H_PROCESS_MS] == "43.9"
    assert got[H_PROCESS_RULES] == "v1@04:17:32"


def test_가공_헤더가_없어도_수신은_계속된다():
    """계약에 없는 헤더다. 안 보내는 상대가 있어도 아무것도 깨지면 안 된다."""
    p = _puller()
    p._note_sidecar({H_CAPTURE_CLOCK: "1000000"}, 1000.0)
    sc = p.sidecar()
    assert sc["processMs"] is None
    assert sc["processMedianMs"] is None
    assert sc["processSamples"] == 0
    assert sc["source"] == "capture", "가공 헤더가 없다고 곁표가 죽으면 안 된다"


# ---- 값 ---------------------------------------------------------------------

def test_마지막_가공_시간과_규칙_버전을_낸다():
    p = _puller()
    p._note_sidecar({H_PROCESS_MS: "43.9", H_PROCESS_RULES: "v1@04:17:32"}, 1000.0)
    sc = p.sidecar()
    assert sc["processMs"] == pytest.approx(43.9)
    assert sc["processRules"] == "v1@04:17:32"


def test_중앙값을_낸다_한_프레임의_튐에_흔들리지_않게():
    """가공 시간은 프레임마다 흔들린다(폰 실측 44~51 ms). 마지막 값만 보면 오판한다."""
    clk = _Clock(1000.0)
    p = _puller(clk)
    for i, ms in enumerate((44.0, 50.0, 47.0, 300.0, 45.0)):   # 300 은 한 번 튄 것
        clk.t = 1000.0 + i * 0.1
        p._note_sidecar({H_PROCESS_MS: str(ms)}, clk.t)
    sc = p.sidecar()
    assert sc["processSamples"] == 5
    assert sc["processMedianMs"] == pytest.approx(47.0), "중앙값은 튄 값에 안 끌려간다"
    assert sc["processMs"] == pytest.approx(45.0), "마지막 값은 그대로 낸다"


def test_창_밖의_가공_표본은_버린다():
    clk = _Clock(1000.0)
    p = _puller(clk)
    p._note_sidecar({H_PROCESS_MS: "50"}, 1000.0)
    assert p.sidecar()["processSamples"] == 1
    clk.t = 1000.0 + CLOCK_WINDOW_S + 5.0
    assert p.sidecar()["processSamples"] == 0


def test_숫자가_아닌_가공_시간은_통계를_오염시키지_않는다():
    p = _puller()
    p._note_sidecar({H_PROCESS_MS: "n/a", H_PROCESS_RULES: "v9"}, 1000.0)
    sc = p.sidecar()
    assert sc["processMs"] is None
    assert sc["processSamples"] == 0
    assert sc["processRules"] == "v9", "규칙 버전은 문자열이라 그대로 남는다"


def test_규칙_버전은_마지막으로_본_값을_유지한다():
    """한 프레임에 규칙 헤더가 빠져도 '규칙을 모르는 상태'로 되돌아가지 않는다."""
    p = _puller()
    p._note_sidecar({H_PROCESS_MS: "20", H_PROCESS_RULES: "v1@08:18:51"}, 1000.0)
    p._note_sidecar({H_PROCESS_MS: "21"}, 1000.1)
    assert p.sidecar()["processRules"] == "v1@08:18:51"


# ---- 통합: 픽스처가 앱처럼 보낸다 ------------------------------------------------

def test_픽스처_스트림에서_가공_시간을_줍는다():
    with FixtureMjpegServer(_frames(3), frame_interval_s=0.01, sidecar=True,
                            process_ms=47.5, process_rules="v1@test") as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.sidecar()["processSamples"] >= 3)
            sc = p.sidecar()
            assert sc["processMs"] == pytest.approx(47.5)
            assert sc["processMedianMs"] == pytest.approx(47.5)
            assert sc["processRules"] == "v1@test"
        finally:
            p.stop()


def test_가공_헤더를_안_보내는_스트림도_정상이다():
    """구식 상대(가공 단 없이 앱이 직접 서빙)를 흉내낸다."""
    with FixtureMjpegServer(_frames(3), frame_interval_s=0.01, sidecar=True) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 3)
            sc = p.sidecar()
            assert sc["source"] == "capture"
            assert sc["processMedianMs"] is None
            assert sc["processSamples"] == 0
        finally:
            p.stop()


def test_가공_시간이_offset_보다_작다는_것을_말할_수_있다():
    """이 유닛의 존재 이유 - offsetMedian 에서 아는 성분을 빼면 나머지가 전송·큐다.

    폰 실측: offset 2783 ms 인데 가공은 47 ms 였다. 즉 2736 ms 가 다른 데서 왔다.
    가공 시간이 없으면 "가공이 느린 거 아니냐"를 반박할 수 없다.
    """
    clk = _Clock(1000.0)
    p = _puller(clk)
    for i in range(5):
        clk.t = 1000.0 + i * 0.1
        capture_ms = int((clk.t - 2.783) * 1000.0)          # 2783 ms 늦게 도착
        p._note_sidecar({H_CAPTURE_CLOCK: str(capture_ms), H_PROCESS_MS: "47"}, clk.t)
    sc = p.sidecar()
    assert sc["offsetMedianMs"] == pytest.approx(2783.0, abs=5.0)
    assert sc["processMedianMs"] == pytest.approx(47.0)
    unexplained = sc["offsetMedianMs"] - sc["processMedianMs"]
    assert unexplained > 2000, "가공으로 설명 안 되는 부분이 남는다: %.1f ms" % unexplained
