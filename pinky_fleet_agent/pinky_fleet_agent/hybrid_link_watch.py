#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""단방향 통신 링크 감시자 (데드맨 스위치 + 복구 은혜 기간).

전이:
    INITIAL  --(pulse/on_command)-->  ARMED  --(timeout 초과)-->  LOST
                                        ^                           |
                                        +-----(grace 경과)----+    (pulse)
                                                             |      |
                                                          RESTORED <-+

규칙:
    * LOST 동안은 로봇이 멈춘다 (이동 명령 무시).
    * RESTORED (복구 은혜 기간) 동안은 하트비트만 받고 "이동 명령" 은 무시한다.
      RELIABLE QoS 가 끊긴 동안 쌓아 둔 과거의 이동 명령을 재전송할 때 생기는
      급발진을 막기 위해서다.
    * 은혜 기간이 지나면 ARMED 로 승격되어 다시 이동 명령을 받는다.
    * timeout 이 0 이하면 감시하지 않는다 (LOST 를 내지 않는다).

``link_watch.py``(agent_node · lane_agent_node 용)와 무엇이 다른가:
    * **아무 명령이나** 받으면 무장한다. hybrid_agent_node 는 FleetCommand HEARTBEAT 뿐 아니라
      10 Hz 로 오는 LaneCommand CLEARANCE 도 생존 신호로 받는다(`LaneCommand.msg` 머리말).
      link_watch.py 는 하트비트로만 무장한다 — 옛 관제 PC 와 섞일 때를 위한 규칙이다.
    * :meth:`pulse` 가 (이전 상태, 지금 상태) 를 돌려주고, :meth:`admit_command` 가 이동 명령
      수용을 한 번에 판단한다.
    규칙이 다르므로 link_watch.py 를 고치지 않고 파일을 나눴다. hybrid_agent_node 만 이 파일을 쓴다.
"""

INITIAL = 'INITIAL'
ARMED = 'ARMED'
LOST = 'LOST'
RESTORED = 'RESTORED'


class LinkWatch:
    """마지막으로 명령을 받은 시각을 들고 연결 생존을 판단한다.

    :param timeout: 이 시간(초) 동안 명령이 없으면 끊긴 것으로 본다. 0 이하면 비활성.
    :param restore_grace: 연결 복구 직후 이 시간(초) 동안은 이동 명령을 무시한다.
    """

    def __init__(self, timeout: float, restore_grace: float = 1.0):
        self.timeout = float(timeout)
        self.restore_grace = float(restore_grace)
        self.armed = False
        self.lost = False
        self.last_command = None
        self.last_pulse = None
        self.restored_at = None
        self._last_poll = None
        self.state = INITIAL

    @property
    def enabled(self) -> bool:
        return self.timeout > 0.0

    def pulse(self, now: float, heartbeat: bool = False):
        now = float(now)
        prev = self.state
        self.last_command = now
        self.last_pulse = now

        if not self.armed or self.state == INITIAL:
            self.armed = True
            self.lost = False
            self.state = ARMED
        elif self.lost or self.state == LOST:
            self.lost = False
            self.restored_at = now
            self.state = RESTORED
        elif self.state == RESTORED:
            if self.restored_at is not None and (now - self.restored_at) >= self.restore_grace:
                self.state = ARMED

        return prev, self.state

    def on_command(self, now: float, heartbeat: bool = False):
        """명령 수신 시 상태 전이 이름 돌려줌 (ARMED / RESTORED)."""
        now = float(now)
        self.last_command = now
        self.last_pulse = now
        if not self.armed:
            self.armed = True
            self.state = ARMED
            return ARMED
        if self.lost:
            self.lost = False
            self.restored_at = now
            self.state = RESTORED
            return RESTORED
        if self.state == RESTORED and self.restored_at is not None:
            if (now - self.restored_at) >= self.restore_grace:
                self.state = ARMED
        return None

    def poll(self, now: float):
        """주기적 호출로 timeout 감지. 연결이 끊겼을 때만 LOST 반환, 정상이면 None."""
        now = float(now)
        last_poll, self._last_poll = self._last_poll, now
        if not self.enabled or not self.armed or self.lost:
            return None

        if last_poll is not None and now >= last_poll and (now - last_poll) > self.timeout:
            self.last_command = now
            return None

        if self.last_command is not None and (now - self.last_command) > self.timeout:
            self.lost = True
            self.state = LOST
            return LOST

        if self.state == RESTORED and self.restored_at is not None:
            if (now - self.restored_at) >= self.restore_grace:
                self.state = ARMED

        return None

    def silence(self, now: float):
        """마지막 명령 이후 경과 시간 (s)."""
        if self.last_command is None:
            return float('inf')
        return max(0.0, float(now) - self.last_command)

    def alive(self, now: float) -> bool:
        """연결 생존 여부."""
        if not self.armed or not self.enabled:
            return True
        return not self.lost and (self.silence(now) or 0.0) < self.timeout

    def accepts_motion_command(self, now: float) -> bool:
        """복구 은혜 기간 확인 (stale command rejection)."""
        if self.restored_at is None or self.restore_grace <= 0.0:
            return True
        return (float(now) - self.restored_at) >= self.restore_grace

    def admit_command(self, now: float) -> bool:
        """명령 수용 가능 여부."""
        self.poll(now)
        return self.state == ARMED and self.accepts_motion_command(now)
