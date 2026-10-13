"""교차로 seek 동작 — 각도·거리 대신 새 빨간 선을 찾아 그 앞까지 (maneuver.RedLineSeeker) + 드라이버 폐루프.

카메라는 sim_red_lines 모형: 바닥 빨간 선(점)을 로봇 자세에서 본 덩어리 [(x_norm, width, bottom)].
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.drive_fsm import CRUISE, JUNCTION_PASS, JUNCTION_STOP, RED_LINE_STOP  # noqa: E402
from pinky_fleet_agent.lane_control import QUALITY_BOTH, QUALITY_SINGLE  # noqa: E402
from pinky_fleet_agent.lane_driver import CMD_CLEARANCE, CMD_START, DriverParams, LaneDriver  # noqa: E402
from pinky_fleet_agent.maneuver import ManeuverExecutor, ManeuverParams, RedLineSeeker, parse_steps  # noqa: E402
from pinky_fleet_agent.sim_red_lines import (  # noqa: E402
    ARRIVE_AHEAD, junction_red_lines, red_blobs, red_line_detected,
)

DT = 0.05


def seek(direction, lines, pose=(0.0, 0.0, 0.0), seconds=40.0, params=None, frame_every=2, extra=None):
    """RedLineSeeker 를 유니사이클로 돌린다. 카메라 프레임은 frame_every 틱마다. extra(pose, blobs) 로 덩어리를 덧붙인다."""
    s = RedLineSeeker(direction, params or ManeuverParams())
    x, y, yaw = pose
    blobs, frame, log = [], None, []
    for i in range(int(seconds / DT)):
        t = i * DT
        if i % frame_every == 0:
            blobs = red_blobs((x, y, yaw), lines)
            if extra:
                blobs = blobs + extra((x, y, yaw), blobs)
            frame = t
        v, w, done, why = s.command((x, y, yaw), blobs, frame)
        log.append((s.phase, v, w, why, math.degrees(yaw)))
        if done or s.failed:
            return s, (x, y, yaw), log
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += w * DT
    return s, (x, y, yaw), log


def test_parse_steps_accepts_seek_and_normalises_sign():
    assert parse_steps(['seek', 'seek', 'seek'], [1.0, -3.0, 0.2]) == [('seek', 1.0), ('seek', -1.0), ('seek', 0.0)]
    with pytest.raises(ValueError):
        parse_steps(['jump'], [1.0])


@pytest.mark.parametrize('direction, sign', [(1.0, 1), (-1.0, -1)])
def test_turn_goes_forward_then_rotates_and_stops_in_front_of_new_line(direction, sign):
    lines = junction_red_lines(0.0)
    s, (x, y, yaw), log = seek(direction, lines)
    assert s.done and not s.failed
    assert 0.19 <= x <= 0.23                                        # 20 cm 전진 뒤 제자리 회전
    assert sign * y > 0.10 and abs(math.degrees(yaw) - sign * 90.0) < 12.0
    target = lines[1] if sign > 0 else lines[2]
    assert math.hypot(target[0] - x, target[1] - y) == pytest.approx(ARRIVE_AHEAD, abs=0.03)   # 새 선 앞 (정지 행)
    rotating = [deg for ph, v, w, why, deg in log if ph == 'rotate']
    assert rotating and all(abs(w) <= 0.3 + 1e-9 for _, v, w, _, _ in log if _ == 'rotate')


def test_line_visible_before_min_turn_is_not_taken():
    # 회전을 시작할 때 정면 가운데에 선이 있다(직진 방향 선). 좌회전은 그것을 무시하고 왼쪽 선으로 간다
    lines = [(0.20 + 0.35, 0.0), (0.20, 0.40)]
    s, (x, y, yaw), _ = seek(1.0, lines)
    assert s.done and y > 0.10 and abs(math.degrees(yaw) - 90.0) < 12.0


def test_straight_ignores_entry_line_underfoot_and_goes_to_next_line():
    red_x = 0.0
    lines = junction_red_lines(red_x)
    s, (x, y, yaw), log = seek(0.0, lines)
    assert s.done and not s.failed
    assert x == pytest.approx(lines[3][0] - ARRIVE_AHEAD, abs=0.02) and abs(y) < 1e-9 and abs(yaw) < 1e-9
    assert log[0][0] == 'ignore'


def test_straight_entry_line_still_visible_after_ignore_distance_is_not_new():
    # 입구 선이 발밑(하단 ≥ 도착 행)에 아직 보여도 새 선이 아니다 — 다음 선까지 간다
    p = ManeuverParams(seek_ignore_distance=0.0)
    lines = junction_red_lines(0.0)
    s, (x, _, _), _ = seek(0.0, lines, params=p)
    assert s.done and x == pytest.approx(lines[3][0] - ARRIVE_AHEAD, abs=0.02)


def test_approach_ignores_a_different_line_jumping_in_below():
    # 접근 중 발밑 가까이(하단 0.97) 가운데 덩어리가 끼어들어도(입구 선 조각) 목표를 바꾸지 않는다
    lines = junction_red_lines(0.0)
    s, (x, _, _), _ = seek(0.0, lines, extra=lambda pose, blobs: [(0.0, 0.4, 0.97)] if blobs else [])
    assert s.done and x == pytest.approx(lines[3][0] - ARRIVE_AHEAD, abs=0.02)


def test_turn_fails_at_max_angle_and_stays_stopped():
    s, (x, y, yaw), log = seek(-1.0, [])
    assert s.failed and not s.done
    assert math.degrees(-yaw) == pytest.approx(150.0, abs=2.0)
    assert '못 찾음' in log[-1][3] and log[-1][1] == 0.0 and log[-1][2] == 0.0
    v, w, done, why = s.command((x, y, yaw), [], 99.0)
    assert (v, w, done) == (0.0, 0.0, False) and '못 찾음' in why


def test_straight_fails_after_max_distance():
    s, (x, _, _), _ = seek(0.0, [(3.0, 0.0)])
    assert s.failed and x == pytest.approx(0.10 + 0.80, abs=0.02)


def test_same_frame_is_counted_once():
    s = RedLineSeeker(0.0, ManeuverParams(seek_ignore_distance=0.0, seek_confirm=2))
    blob = [(0.0, 0.4, 0.6)]
    for _ in range(5):
        s.command((0.0, 0.0, 0.0), blob, 1.0)                        # 같은 프레임 5 번
    assert s.phase == 'scan'
    s.command((0.0, 0.0, 0.0), blob, 2.0)
    assert s.phase == 'approach'
    s2 = RedLineSeeker(0.0, ManeuverParams(seek_ignore_distance=0.0))
    for _ in range(5):
        s2.command((0.0, 0.0, 0.0), blob, None)                      # STALE — 새 프레임 아님
    assert s2.phase == 'scan'


def test_executor_runs_seek_step_and_reports_turn_sign():
    ex = ManeuverExecutor([('seek', -1.0)], ManeuverParams())
    assert ex.last_turn_sign == -1.0 and '빨간 선 찾기(우)' in ex.describe()
    lines = junction_red_lines(0.0)
    x = y = yaw = 0.0
    for i in range(int(40 / DT)):
        v, w, done, why = ex.command(i * DT, (x, y, yaw), red_blobs((x, y, yaw), lines), i * DT)
        if done:
            break
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += w * DT
    assert ex.done and not ex.failed and y < -0.10
    assert ManeuverExecutor([('seek', 0.0)]).last_turn_sign == 0.0


# ------------------------------------------------------------ 드라이버 폐루프 (lane_only + 계획)

class SeekSim:
    """lane_only 드라이버 + seek 계획 + 빨간 선 모형. 입구 선은 red_x 에서 정지 행에 온다."""

    def __init__(self, direction, red_x=0.5, lines=None, exit_red_line_stop=True, quality=QUALITY_BOTH):
        p = DriverParams()
        p.lane_only = True
        p.exit_red_line_stop = exit_red_line_stop
        p.fsm.search_on_single = False                               # 현장 lane_agent.yaml 과 같게
        self.quality = quality
        self.d = LaneDriver(p)
        self.d.set_plan(3, [('seek', direction)], 1)
        self.lines = junction_red_lines(red_x) if lines is None else lines
        self.x = self.y = self.yaw = 0.0
        self.t = 0.0
        self.clear = 0
        self.log = []
        self.d.set_command(CMD_START, self.t)

    def run(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            k = int(round(self.t / DT))
            if k % 2 == 0:
                self.d.set_command(CMD_CLEARANCE, self.t, route_seq=3, clear_until=self.clear)
            if k % 3 == 0:
                blobs = red_blobs((self.x, self.y, self.yaw), self.lines)
                self.d.set_lane_path(self.t, self.t, self.quality, 0.0, False,
                                     red_line=red_line_detected(blobs), red_obs=blobs)
            self.d.update_us(1.0)
            self.d.update_odom(self.t, self.x, self.y, self.yaw)
            out = self.d.tick(self.t, 0.0, 0.0, 0.0)
            self.x += out.v * math.cos(self.yaw) * DT
            self.y += out.v * math.sin(self.yaw) * DT
            self.yaw += out.omega * DT
            self.log.append((self.t, out))
        return self.log[-1][1]

    def states(self):
        return [o.state for _, o in self.log]


@pytest.mark.parametrize('direction', [1.0, -1.0, 0.0])
def test_driver_entry_stop_grant_seek_then_red_line_stop_then_cruise(direction):
    s = SeekSim(direction)
    assert s.run(12).state == JUNCTION_STOP                          # 입구 선에서 허가 대기
    assert s.x == pytest.approx(0.5, abs=0.03)
    s.clear = 1
    s.run(40)
    states = s.states()
    i_pass = [i for i, st in enumerate(states) if st == JUNCTION_PASS]
    assert i_pass and any('빨간 선 찾기' in s.log[i][1].reason for i in i_pass)
    after = states[i_pass[-1] + 1:]
    assert RED_LINE_STOP in after[:3]                                # 새 선 앞: 1 s 정지
    runs = sum(1 for i in range(1, len(after)) if after[i] == RED_LINE_STOP != after[i - 1])
    assert runs == 1 and after[-1] == CRUISE
    assert s.d._junction_passed
    if direction > 0:
        assert s.y > 0.10
    elif direction < 0:
        assert s.y < -0.10
    else:
        assert abs(s.y) < 1e-9


def test_driver_seek_failure_stays_stopped_in_junction_with_reason():
    s = SeekSim(1.0, lines=junction_red_lines(0.5)[:1])              # 입구 선만 있다
    s.run(12)
    s.clear = 1
    out = s.run(40)
    assert out.state == JUNCTION_PASS and out.v == 0.0 and out.omega == 0.0
    assert '못 찾음' in out.reason and not s.d._junction_passed


@pytest.mark.parametrize('direction', [1.0, -1.0, 0.0])
def test_driver_exit_line_no_stop_keeps_seek_speed_into_cruise(direction):
    s = SeekSim(direction, exit_red_line_stop=False)
    s.run(12)
    s.clear = 1
    s.run(40)
    states = s.states()
    assert RED_LINE_STOP not in states
    i_pass = [i for i, st in enumerate(states) if st == JUNCTION_PASS]
    i_end = i_pass[-1]
    assert states[i_end + 1] == CRUISE and states[-1] == CRUISE and s.d._junction_passed
    seek_v = ManeuverParams().seek_speed
    vs = [o.v for _, o in s.log[i_end - 2:]]
    assert min(vs[:30]) >= seek_v - 1e-9                           # 선 앞에서도 0 으로 떨어지지 않는다
    v_max = s.d.p.control.v_max
    k_max = next(k for k, v in enumerate(vs) if v >= v_max - 1e-6)
    assert k_max * DT <= (v_max - seek_v) / s.d.p.control.accel_slew + 0.2   # seek 속도에서 바로 v_max 로 가속
    assert s.d.mission_stage == 'vision:after_junction'


def test_driver_exit_no_stop_with_single_lane_goes_cruise_not_search():
    s = SeekSim(1.0, exit_red_line_stop=False, quality=QUALITY_SINGLE)
    s.run(12)
    s.clear = 1
    s.run(40)
    states = s.states()
    i_end = [i for i, st in enumerate(states) if st == JUNCTION_PASS][-1]
    assert states[i_end + 1] == CRUISE and s.log[i_end + 1][1].v > 0.0


def test_lane_agent_yaml_exit_red_line_stop_off_but_test_run_still_stops():
    import yaml
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg = yaml.safe_load(open(os.path.join(here, 'params', 'lane_agent.yaml'), encoding='utf-8'))
    params = cfg['pinky_lane_agent']['ros__parameters']
    assert params['exit_red_line_stop'] is False and params['fsm']['red_line_stop'] is True
    assert params['maneuver']['seek_speed'] == pytest.approx(0.05)
    # 계획 없는 테스트 주행: 빨간 선마다 그대로 선다
    p = DriverParams()
    p.lane_only = True
    p.exit_red_line_stop = False
    d = LaneDriver(p)
    d.set_command(CMD_START, 0.0)
    t, states = 0.0, []
    for i in range(120):
        t += DT
        d.set_command(CMD_CLEARANCE, t)
        d.set_lane_path(t, t, QUALITY_BOTH, 0.0, False, red_line=20 <= i < 30)
        states.append(d.tick(t, 0.0, 0.0, 0.0).state)
    assert RED_LINE_STOP in states
