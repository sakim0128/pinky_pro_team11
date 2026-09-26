"""drive_command_gate — 우선순위 · 타임아웃 · 비상정지 전이 (ROS 불필요)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.drive_command_gate import CommandGateCore, GateSource  # noqa: E402


def gate():
    return CommandGateCore(teleop_timeout_sec=0.5, mission_timeout_sec=0.5)


def test_idle_without_input():
    assert gate().step(10.0) == (0.0, 0.0, GateSource.IDLE)


def test_mission_passes_then_times_out():
    g = gate()
    g.feed_mission(0.25, 0.1, now=100.0)
    assert g.step(100.1) == (0.25, 0.1, GateSource.MISSION)
    assert g.step(100.6) == (0.0, 0.0, GateSource.IDLE)


def test_teleop_preempts_mission_until_it_times_out():
    g = gate()
    g.feed_mission(0.15, 0.0, now=200.0)
    g.feed_teleop(0.40, -0.3, now=200.05)
    assert g.step(200.1) == (0.40, -0.3, GateSource.TELEOP)
    g.feed_mission(0.15, 0.0, now=200.5)
    assert g.step(200.6) == (0.15, 0.0, GateSource.MISSION)


def test_estop_overrides_everything():
    g = gate()
    g.feed_mission(0.2, 0.0, now=300.0)
    g.feed_teleop(0.5, 0.5, now=300.0)
    assert g.set_estop(True) is True
    assert g.step(300.1) == (0.0, 0.0, GateSource.ESTOP)
    # 비상정지 중에 들어온 입력도 내보내지 않는다
    g.feed_mission(0.2, 0.0, now=300.2)
    assert g.step(300.2) == (0.0, 0.0, GateSource.ESTOP)


def test_release_does_not_replay_inputs_cached_before_or_during_estop():
    g = gate()
    g.feed_teleop(0.5, 0.5, now=300.0)
    g.set_estop(True)
    g.feed_mission(0.2, 0.0, now=300.1)
    assert g.set_estop(False) is True
    # 두 입력 모두 timeout 안이지만 해제 전에 받은 것이라 버린다
    assert g.step(300.2) == (0.0, 0.0, GateSource.IDLE)
    g.feed_mission(0.1, 0.0, now=300.3)
    assert g.step(300.35) == (0.1, 0.0, GateSource.MISSION)


def test_repeated_same_estop_value_keeps_inputs():
    """에이전트는 래치 동안 true 를 1 s 마다 다시 낸다. 해제 쪽 false 가 되풀이돼도 주행이 끊기면 안 된다."""
    g = gate()
    g.feed_mission(0.2, 0.0, now=400.0)
    assert g.set_estop(False) is False
    assert g.step(400.1) == (0.2, 0.0, GateSource.MISSION)

    g.set_estop(True)
    assert g.set_estop(True) is False
    assert g.step(400.2) == (0.0, 0.0, GateSource.ESTOP)
