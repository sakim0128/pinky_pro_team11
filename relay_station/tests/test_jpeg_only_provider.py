# -*- coding: utf-8 -*-
"""G-A — ROS 매니저를 감싸는 어댑터가 정직한지.

무엇이 틀렸었나 (2026-09-09 실측):
    connected = (jpeg is not None) 로 근사했는데, Gazebo·로봇 매니저는
    "기다리는 중" 플레이스홀더를 **항상** 들고 있어 jpeg 가 절대 None 이 아니었다.
    그래서 Gazebo 프로세스가 죽어 있는데도 conn=True, 로봇 Publisher 0 인데 conn=True.
    stamp 도 time.time() 이라 물어볼 때마다 '지금'이었다.

이제 **바이트가 바뀌었는지**로 판정한다. 플레이스홀더는 안 바뀌므로 자연히 stale 이 된다.
"""
import pytest

from source_registry import jpeg_only_provider

PLACEHOLDER = b"\xff\xd8\xff" + b"waiting-for-camera-topic" + b"\xff\xd9"


def _frame(n):
    return b"\xff\xd8\xff" + ("frame-%d" % n).encode() + b"\xff\xd9"


# ---- 거짓말이 사라졌는지 -------------------------------------------------------

def test_안_바뀌는_플레이스홀더는_절대_connected_가_안_된다():
    """이게 원래 버그다. Gazebo 가 죽어도 True 였다."""
    now = [1000.0]
    p = jpeg_only_provider(lambda: PLACEHOLDER, stale_after_s=3.0,
                           clock=lambda: now[0])
    for _ in range(5):
        now[0] += 1.0
        jpeg, stamp, connected = p()
        assert connected is False, "안 바뀌는 프레임을 연결로 읽으면 지표가 거짓말한다"
        assert stamp == 0.0


def test_첫_호출은_기준선만_잡고_연결로_치지_않는다():
    """첫 프레임이 플레이스홀더일 수 있다. 그걸 '도착'으로 세면 다시 거짓말이다."""
    p = jpeg_only_provider(lambda: PLACEHOLDER, clock=lambda: 1000.0)
    _, stamp, connected = p()
    assert connected is False
    assert stamp == 0.0


def test_바이트가_바뀌면_연결로_친다():
    now = [1000.0]
    seq = [PLACEHOLDER, PLACEHOLDER, _frame(1), _frame(2)]
    idx = [0]

    def getter():
        v = seq[min(idx[0], len(seq) - 1)]
        idx[0] += 1
        return v

    p = jpeg_only_provider(getter, stale_after_s=3.0, clock=lambda: now[0])
    assert p()[2] is False          # 기준선
    assert p()[2] is False          # 안 바뀜
    now[0] += 0.5
    assert p()[2] is True           # 바뀜 -> 도착
    now[0] += 0.5
    assert p()[2] is True


def test_바뀐_뒤_조용해지면_다시_False_가_된다():
    """스트림이 얼면 '연결됨'으로 남아 있으면 안 된다."""
    now = [1000.0]
    seq = [_frame(1), _frame(2)]
    idx = [0]

    def getter():
        v = seq[min(idx[0], len(seq) - 1)]
        idx[0] += 1
        return v

    p = jpeg_only_provider(getter, stale_after_s=2.0, clock=lambda: now[0])
    p()                              # 기준선
    now[0] += 0.1
    assert p()[2] is True            # 바뀜
    now[0] += 10.0                   # 이후 같은 프레임만 반복
    assert p()[2] is False, "얼어붙은 스트림이 연결됨으로 남으면 안 된다"


# ---- 화면은 유지한다 ----------------------------------------------------------

def test_stale_여도_jpeg_는_그대로_돌려준다():
    """거짓말이었던 것은 화면이 아니라 지표다.

    '기다리는 중' 카드를 보여주는 건 사람에게 유용하다. 다만 그것을
    connected 로 세지 않을 뿐이다.
    """
    p = jpeg_only_provider(lambda: PLACEHOLDER, clock=lambda: 1000.0)
    jpeg, _, connected = p()
    assert jpeg == PLACEHOLDER, "화면까지 없애면 사용자가 이유를 모른다"
    assert connected is False


def test_getter_가_None_을_줘도_안전하다():
    p = jpeg_only_provider(lambda: None, clock=lambda: 1000.0)
    jpeg, stamp, connected = p()
    assert (jpeg, stamp, connected) == (None, 0.0, False)


# ---- 레지스트리와 함께 --------------------------------------------------------

def test_레지스트리_status_가_거짓_connected_를_안_낸다():
    """Source.status() 가 provider 를 부르므로 여기까지 이어져야 한다."""
    from source_registry import Source, ROS, TRUSTED
    now = [1000.0]
    src = Source("gazebo", jpeg_only_provider(lambda: PLACEHOLDER,
                                              clock=lambda: now[0]),
                 ROS, TRUSTED, clock=lambda: now[0])
    st = src.status()
    assert st["connected"] is False, "죽은 ROS 소스가 connected 로 보고되면 안 된다"
    assert st["lastFrameStamp"] is None
