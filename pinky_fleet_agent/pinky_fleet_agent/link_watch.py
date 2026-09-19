"""관제 PC 와의 연결이 살아 있는지 판단하는 데드맨 스위치.

rclpy 에 의존하지 않는다 (시각을 초 단위 float 로만 받는다). 덕분에 ROS 없이
``pinky_fleet_agent/test/test_link_watch.py`` 로 그대로 검증할 수 있다.
``map_paths.py`` 와 같은 이유의 분리다.

관제 PC 의 coordinator 와 GUI 가 각각 ``CMD_HEARTBEAT`` 를 주기적으로 보내고,
에이전트는 어떤 명령이든 받을 때마다 :meth:`on_command` 를 부른다. ``timeout`` 동안
아무것도 오지 않으면 연결이 끊긴 것으로 보고 주행을 끊는다.

세 가지 규칙이 오작동을 막는다.

**1. 하트비트를 한 번이라도 받아야 무장한다.**
    기동 직후 관제 PC 가 아직 없을 때 발동하지 않는다. 더 중요한 건 섞인 버전이다 —
    관제 PC 가 하트비트를 모르는 옛 빌드면 무장하지 않고 조용히 예전처럼 동작한다.
    "아무 명령이나 받으면 무장" 으로 하면 옛 관제 PC 가 보낸 목표가 3초 뒤 취소되어
    로봇이 아예 못 움직인다.

**2. 멈출 것이 있을 때만 발동한다.**
    목표도 HOLD 도 없으면 끊겨도 할 일이 없다.

**3. 시계가 튀면 발동하지 않는다.**
    핑키(라즈베리파이)에는 RTC 가 없어서 부팅 뒤 처음 NTP 에 붙는 순간 시계가 몇
    시간씩 건너뛴다. 그 한 번 때문에 멀쩡한 로봇이 서면 안 된다. :meth:`poll` 이
    자기 호출 간격을 보고 비정상이면 기준 시각을 다시 잡는다.
"""

#: :meth:`LinkWatch.poll` 반환값 — 연결이 방금 끊어졌다 (전이 시점에 한 번만).
LOST = 'LOST'
#: :meth:`LinkWatch.on_command` 반환값 — 하트비트를 처음 받아 감시를 시작했다.
ARMED = 'ARMED'
#: :meth:`LinkWatch.on_command` 반환값 — 끊겼던 연결이 방금 돌아왔다.
RESTORED = 'RESTORED'


class LinkWatch:
    """마지막으로 명령을 받은 시각을 들고 연결 생존을 판단한다.

    :param timeout: 이 시간(초) 동안 명령이 없으면 끊긴 것으로 본다. 0 이하면 비활성.
    :param restore_grace: 연결 복구 직후 이 시간(초) 동안은 이동 명령을 무시한다.
        ``/pinkyN/command`` 는 RELIABLE QoS 라, 끊겼다 붙으면 **끊기기 전에 보낸
        CMD_GOTO 가 재전송되어** 로봇이 혼자 출발할 수 있다. 데드맨이 막으려던 바로
        그 사고가 전송 계층으로 들어오는 셈이다. 재전송분은 복구 직후 몇 ms 안에
        몰려 오고 사람이 누르는 출발은 몇 초 뒤라, 이 창으로 구분한다.
    """

    def __init__(self, timeout, restore_grace=1.0):
        self.timeout = float(timeout)
        self.restore_grace = float(restore_grace)
        self.armed = False
        self.lost = False
        self.last_command = None
        self.restored_at = None
        self._last_poll = None

    @property
    def enabled(self):
        return self.timeout > 0.0

    # --- 입력 --------------------------------------------------------

    def on_command(self, now, heartbeat=False):
        """명령을 받았다. 무장/복구 전이가 일어나면 그 이름을 돌려준다."""
        now = float(now)
        self.last_command = now

        if heartbeat and not self.armed and self.enabled:
            self.armed = True
            self._last_poll = now
            return ARMED

        if self.lost:
            self.lost = False
            self.restored_at = now
            return RESTORED
        return None

    def poll(self, now):
        """주기적으로 호출한다. 연결이 **방금** 끊어졌으면 :data:`LOST`.

        전이 시점에만 한 번 돌려준다. 그렇지 않으면 10Hz 로 취소 요청을 쏟아낸다.
        """
        now = float(now)
        last_poll, self._last_poll = self._last_poll, now

        if not self.armed or self.lost:
            return None

        # 시계가 튀었다. 직전 호출과의 간격이 타임아웃보다 크다는 것은 10Hz 로 도는
        # 호출자에서는 일어날 수 없는 일이다 (시각이 뒤로 간 경우도 같이 걸린다).
        if last_poll is None or now < last_poll or (now - last_poll) > self.timeout:
            self.last_command = now
            return None

        if (now - self.last_command) < self.timeout:
            return None
        self.lost = True
        return LOST

    # --- 조회 --------------------------------------------------------

    def silence(self, now):
        """마지막 명령 이후 흐른 시간(초). 받은 적이 없으면 ``None``."""
        if self.last_command is None:
            return None
        return max(0.0, float(now) - self.last_command)

    def alive(self, now):
        """연결이 살아 있다고 볼 수 있으면 True.

        무장 전이거나 비활성이면 **살아 있는 것으로 친다.** 판단 근거가 없을 때
        로봇을 세우는 쪽으로 기울면 기동 직후마다 오작동한다.
        """
        if not self.armed or not self.enabled:
            return True
        return not self.lost and (self.silence(now) or 0.0) < self.timeout

    def accepts_motion_command(self, now):
        """지금 들어온 이동 명령을 믿어도 되는가 (복구 직후 재전송 방어)."""
        if self.restored_at is None or self.restore_grace <= 0.0:
            return True
        return (float(now) - self.restored_at) >= self.restore_grace
