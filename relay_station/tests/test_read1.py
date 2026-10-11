# -*- coding: utf-8 -*-
"""MjpegPuller 는 온 만큼만 읽는다 — 지연이 쌓이지 않게.

2026-09-10 실기기 대조군(폰 198.51.100.7, 같은 순간 순차, 읽기 방식만 다르게):
    read(8192)     median 2960.1 ms  처음 2625.8 -> 마지막 3270.6   12초에 +645 ms 누적
    read1(65536)   median 2942.3 ms  처음 2951.3 -> 마지막 2914.4   안정

`read(n)` 은 **n 바이트가 찰 때까지 막힌다.** 생산자가 빠르면 소켓 버퍼가 차고 지연이 누적된다.
같은 함정을 외부 레포의 시청 중계가 2026-09-06 에 겪었다(251 -> 171 ms).

시간을 재는 시험은 흔들린다. 그래서 여기서는 **어느 메서드를 부르는지**를 고정한다 —
그게 이 수정의 실체이고, 누가 read 로 되돌리면 이 시험이 문다.
"""
import pytest

from mjpeg_puller import MjpegPuller, DEFAULT_READ_CHUNK


class RecordingResp:
    """read1 과 read 를 둘 다 가진 응답. 어느 쪽이 불렸는지 기록한다."""

    def __init__(self, chunks, has_read1=True):
        self._chunks = list(chunks)
        self.calls = []
        if not has_read1:
            del self.read1

    def read1(self, size):
        self.calls.append(("read1", size))
        return self._chunks.pop(0) if self._chunks else b""

    def read(self, size):
        self.calls.append(("read", size))
        return self._chunks.pop(0) if self._chunks else b""

    def close(self):
        pass


class NoRead1Resp:
    """구식 응답 객체 — read 만 있다."""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self.calls = []

    def read(self, size):
        self.calls.append(("read", size))
        return self._chunks.pop(0) if self._chunks else b""

    def close(self):
        pass


def _puller():
    return MjpegPuller("http://127.0.0.1:9/video", name="unit")


def test_read1_이_있으면_read1_을_쓴다():
    """이게 이 수정의 실체다. read 로 되돌리면 지연이 다시 쌓인다."""
    resp = RecordingResp([b"", ])
    got = MjpegPuller._read_available(resp, 4096)
    assert resp.calls == [("read1", 4096)]
    assert got == b""


def test_read1_이_없으면_read_로_떨어진다():
    """픽스처·목 중에는 read1 이 없는 것이 있다. 그 때문에 수신이 멈추면 안 된다."""
    resp = NoRead1Resp([b"abc"])
    got = MjpegPuller._read_available(resp, 4096)
    assert resp.calls == [("read", 4096)]
    assert got == b"abc"


def test_소비_루프가_read1_을_쓴다():
    """헬퍼만 고치고 루프가 옛 호출을 그대로 쓰면 아무것도 안 바뀐다."""
    jpeg = b"\xff\xd8\xff" + b"x" * 40 + b"\xff\xd9"
    part = b"--frame\r\nContent-Type: image/jpeg\r\nX-Frame-Seq: 1\r\n\r\n" + jpeg + b"\r\n"
    resp = RecordingResp([part, b""])
    p = _puller()
    p._consume(resp)
    assert resp.calls, "아무것도 안 읽었다"
    assert all(name == "read1" for name, _ in resp.calls), resp.calls
    assert p.stats()["frames"] == 1


def test_읽기_청크가_한_프레임을_한_번에_담을_만큼_크다():
    """8 KB 였을 때 폰 프레임(~180 KB)을 스무 번 넘게 나눠 읽어 뒤처졌다.

    read1 은 온 만큼만 주므로 크게 잡아도 대기하지 않는다 - 크게 잡을 이유만 있고 대가는 없다.
    """
    assert DEFAULT_READ_CHUNK >= 32768


def test_구식_응답으로도_프레임을_뽑는다():
    """read1 이 없어도 동작은 같아야 한다. 관용을 없애지 않는다."""
    jpeg = b"\xff\xd8\xff" + b"y" * 30 + b"\xff\xd9"
    part = b"--frame\r\nContent-Length: 35\r\n\r\n" + jpeg + b"\r\n"
    resp = NoRead1Resp([part, b""])
    p = _puller()
    p._consume(resp)
    assert p.stats()["frames"] == 1
    assert all(name == "read" for name, _ in resp.calls)
