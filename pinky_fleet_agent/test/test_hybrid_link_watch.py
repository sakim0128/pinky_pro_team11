"""hybrid_link_watch — hybrid_agent_node 의 데드맨 (ROS 불필요).

link_watch.py 와 규칙이 다른 부분(아무 명령이나 무장 · pulse/admit_command)과,
timeout 0 이면 감시하지 않는다는 규칙을 확인한다.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.hybrid_link_watch import ARMED, INITIAL, LOST, RESTORED, LinkWatch  # noqa: E402


def test_any_command_arms():
    w = LinkWatch(3.0, 1.0)
    assert w.state == INITIAL
    assert w.pulse(10.0, heartbeat=False) == (INITIAL, ARMED)


def test_fires_once_after_timeout_and_restores_with_grace():
    w = LinkWatch(3.0, 1.0)
    w.pulse(10.0)
    assert w.poll(12.0) is None
    assert w.poll(13.5) == LOST
    assert w.poll(13.6) is None                      # 전이 때 한 번만
    assert w.pulse(14.0) == (LOST, RESTORED)
    assert not w.admit_command(14.5)                 # 복구 직후 재전송 방어
    assert w.admit_command(15.1)


def test_poll_gap_longer_than_timeout_is_a_clock_jump():
    w = LinkWatch(3.0, 1.0)
    w.pulse(10.0)
    w.poll(10.1)
    assert w.poll(5000.0) is None                    # NTP 로 시계가 건너뛰었다


def test_zero_timeout_never_fires():
    w = LinkWatch(0.0, 1.0)
    w.pulse(10.0)
    # 같은 시각에 두 번 불려도(간격 0) LOST 를 내지 않는다
    assert w.poll(10.5) is None
    assert w.poll(10.5) is None
    assert w.poll(99.0) is None
    assert w.admit_command(99.0)
