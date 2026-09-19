"""데드맨 스위치(LinkWatch) 단위 테스트.

link_watch 는 rclpy 에 의존하지 않으므로 ROS 없이 그대로 돌릴 수 있다.

    python3 -m pytest pinky_fleet_agent/test -q
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.link_watch import (  # noqa: E402
    ARMED, LOST, RESTORED, LinkWatch,
)

TIMEOUT = 3.0


def armed_watch(timeout=TIMEOUT, restore_grace=1.0, t=100.0):
    """하트비트를 한 번 받아 무장된 상태로 만들어 돌려준다."""
    w = LinkWatch(timeout, restore_grace)
    assert w.on_command(t, heartbeat=True) == ARMED
    return w


def beat(w, t):
    return w.on_command(t, heartbeat=True)


def run(w, start, stop, step=0.1):
    """실제 호출자처럼 10Hz 로 poll 하며 LOST 가 뜬 시각을 돌려준다.

    한 번에 몇 초를 건너뛰며 poll 하면 시계 점프 가드에 걸린다. 그건 코드가 맞고
    테스트가 틀린 것이다 - agent 는 _publish_state 안에서 10Hz 로 부른다.
    """
    t = start
    while t < stop:
        t = round(t + step, 6)
        if w.poll(t) == LOST:
            return t
    return None


# --- 무장 조건 --------------------------------------------------------

def test_starts_disarmed():
    w = LinkWatch(TIMEOUT)
    assert not w.armed
    assert w.alive(0.0)


def test_never_fires_before_the_first_heartbeat():
    """관제 PC 를 아직 안 켠 로봇은 몇 시간이 지나도 조용해야 한다."""
    w = LinkWatch(TIMEOUT)
    for t in (0.0, 1.0, 60.0, 3600.0, 36000.0):
        assert w.poll(t) is None
    assert w.alive(36000.0)


def test_plain_command_does_not_arm():
    """하트비트를 모르는 옛 관제 PC 와 섞여도 목표가 취소되면 안 된다."""
    w = LinkWatch(TIMEOUT)
    w.on_command(100.0)                      # CMD_GOTO 같은 보통 명령
    assert not w.armed
    assert w.poll(200.0) is None
    assert w.alive(200.0)


def test_first_heartbeat_arms_once():
    w = LinkWatch(TIMEOUT)
    assert w.on_command(100.0, heartbeat=True) == ARMED
    assert w.armed
    assert w.on_command(101.0, heartbeat=True) is None


def test_disabled_timeout_never_arms():
    w = LinkWatch(0.0)
    assert w.on_command(100.0, heartbeat=True) is None
    assert not w.armed
    assert w.poll(10_000.0) is None
    assert w.alive(10_000.0)


# --- 끊김 감지 --------------------------------------------------------

def test_alive_while_beats_keep_arriving():
    w = armed_watch()
    for i in range(1, 100):
        t = 100.0 + i
        beat(w, t)
        assert w.poll(t) is None, f't={t} 에서 잘못 발동'
    assert w.alive(199.0)


def test_fires_after_timeout():
    w = armed_watch()
    fired = run(w, 100.0, 110.0)
    assert fired is not None
    assert abs(fired - 103.0) < 0.15, fired
    assert w.lost
    assert not w.alive(fired)


def test_boundary_is_exclusive():
    w = armed_watch()
    assert w.poll(100.05) is None
    assert w.poll(100.0 + TIMEOUT - 1e-9) is None
    assert w.poll(100.0 + TIMEOUT) == LOST


def test_lost_is_edge_triggered():
    """10Hz 로 불리므로, 계속 LOST 를 뱉으면 취소 요청을 초당 10번 쏟는다."""
    w = armed_watch()
    fired = run(w, 100.0, 110.0)
    assert fired is not None
    for i in range(100):
        assert w.poll(fired + (i + 1) * 0.1) is None


def test_poll_gap_longer_than_timeout_is_treated_as_a_clock_jump():
    """자기 타이머가 통째로 멈췄다면 관제가 죽은 건지 우리가 굳은 건지 알 수 없다.

    그럴 땐 기준을 다시 잡고, 정말 끊겼으면 timeout 뒤에 잡는다.
    """
    w = armed_watch()
    assert w.poll(105.0) is None              # 5초 공백 -> 재기준
    assert run(w, 105.0, 112.0) is not None   # 그래도 결국 잡는다


def test_any_command_refreshes_the_clock():
    """하트비트가 몇 개 유실돼도 진짜 명령이 오면 링크는 살아 있는 것이다."""
    w = armed_watch()
    assert w.poll(102.9) is None
    w.on_command(102.9)                      # 하트비트가 아닌 명령
    assert w.poll(105.0) is None
    assert w.poll(105.9) == LOST


# --- 복구 ------------------------------------------------------------

def test_restore_reported_once():
    w = armed_watch()
    fired = run(w, 100.0, 110.0)
    assert fired is not None
    assert beat(w, fired) == RESTORED
    assert not w.lost
    assert beat(w, fired + 0.5) is None
    assert w.alive(fired + 0.5)


def test_poll_stays_quiet_after_restore():
    w = armed_watch()
    fired = run(w, 100.0, 110.0)
    beat(w, fired)
    assert w.poll(fired + 0.1) is None
    assert run(w, fired, fired + 8.0) is not None   # 다시 끊기면 다시 잡는다


# --- 재전송 방어 (RELIABLE QoS) ----------------------------------------

def test_motion_commands_accepted_before_any_loss():
    w = armed_watch()
    assert w.accepts_motion_command(100.0)
    assert w.accepts_motion_command(200.0)


def test_motion_commands_ignored_right_after_restore():
    """끊겼다 붙는 순간 밀린 CMD_GOTO 가 재전송된다. 그걸 받으면 혼자 출발한다."""
    w = armed_watch()
    fired = run(w, 100.0, 110.0)
    beat(w, fired)
    assert not w.accepts_motion_command(fired)
    assert not w.accepts_motion_command(fired + 0.5)
    assert w.accepts_motion_command(fired + 1.0)   # 사람이 누르는 출발은 이 뒤다


def test_restore_grace_zero_disables_the_guard():
    w = armed_watch(restore_grace=0.0)
    fired = run(w, 100.0, 110.0)
    beat(w, fired)
    assert w.accepts_motion_command(fired)


# --- 시계 점프 (RTC 없는 라즈베리파이) ------------------------------------

def test_ntp_jump_forward_does_not_fire():
    """부팅 뒤 첫 NTP 동기화로 시계가 몇 시간 건너뛰어도 멈추면 안 된다."""
    w = armed_watch()
    assert w.poll(100.1) is None
    assert w.poll(100.2) is None
    assert w.poll(100.2 + 7200.0) is None    # 2시간 점프
    assert w.poll(100.3 + 7200.0) is None    # 기준이 다시 잡혔다
    assert w.alive(100.3 + 7200.0)


def test_clock_step_backward_does_not_fire():
    w = armed_watch()
    assert w.poll(100.1) is None
    assert w.poll(90.0) is None
    assert w.poll(90.1) is None


def test_still_fires_after_a_clock_jump_settles():
    w = armed_watch()
    w.poll(100.1)
    w.poll(100.2 + 7200.0)                   # 점프로 기준 재설정
    base = 100.2 + 7200.0
    fired = run(w, base, base + 8.0)
    assert fired is not None
    assert abs(fired - (base + 3.0)) < 0.2, fired


# --- 기타 -------------------------------------------------------------

def test_silence_is_seconds_since_last_command():
    w = LinkWatch(TIMEOUT)
    assert w.silence(100.0) is None
    w.on_command(100.0)
    assert w.silence(103.25) == 3.25


def test_timeout_accepts_string_from_ros_param():
    w = LinkWatch('3.0', '1.0')
    assert w.timeout == 3.0
    assert w.restore_grace == 1.0


def test_scripted_timeline_matches_the_default():
    """1Hz 하트비트, 3초 타임아웃. 관제가 t=110 에 죽으면 t=113 에 잡힌다."""
    w = LinkWatch(3.0)
    w.on_command(100.0, heartbeat=True)
    t = 100.0
    while t < 110.0:                          # 관제 PC 정상
        t = round(t + 1.0, 3)
        w.on_command(t, heartbeat=True)
        assert w.poll(t) is None

    fired_at = run(w, t, 130.0)               # 관제 PC 사망, 하트비트 없음
    assert fired_at is not None
    assert abs(fired_at - 113.0) < 0.15, fired_at
