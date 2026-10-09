"""항공뷰 이탈 보정 (overhead_correction) — 관제가 보낸 차선 중앙 이탈로 차선 주행 조향에 작은 보정을 더한다."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pinky_fleet_agent.drive_fsm import CRUISE, JUNCTION_PASS, JUNCTION_STOP  # noqa: E402
from pinky_fleet_agent.overhead_correction import CorrectionParams, OverheadCorrection  # noqa: E402
from test_vision_driver import VisionSim  # noqa: E402


def test_left_of_center_turns_right_and_is_capped():
    c = OverheadCorrection(CorrectionParams(k_lat=2.0, k_head=0.5, max_omega=0.3))
    c.update(0.0, 0.05, 0.0, True)
    assert c.omega(0.1) == pytest.approx(-0.10)
    c.update(0.0, -0.04, 0.2, True)                              # 오른쪽으로 4 cm, 코스보다 왼쪽을 본다
    assert c.omega(0.1) == pytest.approx(-(2.0 * -0.04 + 0.5 * 0.2))
    c.update(0.0, 0.5, 0.0, True)
    assert c.omega(0.1) == pytest.approx(-0.3)                   # 상한


def test_no_correction_when_inactive_stale_or_disabled():
    c = OverheadCorrection(CorrectionParams(timeout=0.5))
    assert c.omega(0.0) is None                                  # 받은 적 없음
    c.update(0.0, 0.05, 0.0, False)
    assert c.omega(0.1) is None                                  # 관제가 보정 없음(교차로 구간 · 3 cm 미만)
    c.update(1.0, 0.05, 0.0, True)
    assert c.omega(1.4) is not None and c.omega(1.6) is None     # 항공뷰 끊김
    off = OverheadCorrection(CorrectionParams(enabled=False))
    off.update(0.0, 0.05, 0.0, True)
    assert off.omega(0.1) is None


def test_driver_adds_correction_only_while_cruising():
    s = VisionSim(red_x=99.0)                                    # 빨간 선 없음 — 차선 주행만
    s.run(2.0)
    assert s.log[-1][1].state == CRUISE
    w_plain = s.log[-1][1].omega
    s.d.set_correction(s.t, 0.05, 0.0, True)                     # 왼쪽으로 5 cm
    s.run(0.2)
    out = s.log[-1][1]
    assert out.omega < w_plain - 0.05 and '항공뷰 보정 +5 cm' in out.reason
    s.run(0.6)                                                   # 새 값이 없다 — 0.5 s 뒤 보정 없음
    assert '항공뷰 보정' not in s.log[-1][1].reason


def test_correction_not_applied_while_stopped_at_junction():
    s = VisionSim()
    s.run(12, until=lambda o: o.state == JUNCTION_STOP)
    s.d.set_correction(s.t, 0.08, 0.0, True)
    out = s.run(0.2)
    assert out.state == JUNCTION_STOP and out.v == 0.0 and out.omega == 0.0


def test_pre_granted_robot_stops_one_second_then_passes():
    """관제가 남은 거리로 통행권을 미리 주면(clear 1) 빨간 선에서 1 s 만 서고 바로 교차로 동작."""
    s = VisionSim()
    s.clear = 1
    s.run(15, until=lambda o: o.state == JUNCTION_PASS)
    stops = [t for t, o in s.log if o.state == JUNCTION_STOP]
    assert s.log[-1][1].state == JUNCTION_PASS
    assert 0.95 <= max(stops) - min(stops) <= 1.2
