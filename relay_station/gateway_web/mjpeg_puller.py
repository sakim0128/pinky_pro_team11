#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""원격 MJPEG 을 당겨와 최신 프레임을 들고 있는 puller (MCV-1A 의 pull transport).

왜 필요한가:
    폰과 태블릿은 각자 자기 `:18082` 로 시청 중계를 서빙한다. 게이트웨이의 기존 슬롯은
    앱이 밀어넣는 push(`/api/camera/upload`) 하나뿐이라, **아무도 안 밀면 비어 있다.**
    실제로 2026-09-09 실측에서 폰·태블릿이 각자 22.8 fps / 12.8 fps 로 흐르는데
    게이트웨이 레지스트리는 전부 fps 0 이었다. 당겨오는 주체가 없었기 때문이다.

    이 모듈이 그 주체다. 소스 레지스트리의 jpeg_provider 계약을 그대로 만족한다.

        MjpegPuller(url)              백그라운드 스레드
              │  GET url (multipart/x-mixed-replace)
              │  JPEG 매직으로 프레임을 잘라 최신 것만 보관
              ▼
        get_latest_jpeg() -> (jpeg|None, stamp, connected)   ← Source 가 그대로 받는다

설계 결정:
  * **표준 라이브러리만 쓴다.** cv2 도 rclpy 도 안 쓴다 → 로봇 없이, ROS 없이 유닛테스트된다.
  * **최신 프레임만 보관한다.** 큐를 두면 느린 소비자 때문에 지연이 쌓인다.
    시청 갈래의 유실 정책은 DROP_TO_STAY_LIVE 다.
  * **유계 backoff 로 재접속한다.** 상대가 꺼져 있어도 CPU 를 태우지 않는다.
  * **stale 을 시각으로 판정한다.** 소켓이 열려 있어도 프레임이 안 오면 connected=False.
    소켓 생존을 연결로 읽으면 "완벽 수신 LIVE" 오탐이 재발한다.
"""
import collections
import io
import json
import os
import re
import statistics
import subprocess
import threading
import time
import urllib.error
import urllib.request

JPEG_SOI = b"\xff\xd8\xff"
JPEG_EOI = b"\xff\xd9"

DEFAULT_STALE_AFTER_S = 3.0
DEFAULT_CONNECT_TIMEOUT_S = 5.0
# read1 은 온 만큼만 돌려주므로 크게 잡아도 대기하지 않는다.
# 8 KB 였을 때 폰 프레임(~180 KB)을 스무 번 넘게 나눠 읽어 뒤처졌다.
DEFAULT_READ_CHUNK = 65536
# 매직을 못 찾는 쓰레기가 흘러와도 메모리가 무한정 늘지 않게 한다.
MAX_BUFFER_BYTES = 4 * 1024 * 1024

BACKOFF_START_S = 0.5
BACKOFF_MAX_S = 5.0

# MCV-1B1 곁표 — 앱이 프레임마다 파트 헤더로 실어 보낸다.
# 계약: CAMERA_SOURCE_LOOPBACK_v1 (앱의 MJPEG 서버)
#   X-Capture-Clock     프레임 획득 시각 (epoch ms). 발행 시각이 아니다
#   X-Publisher-Session 앱 세션 id. 바뀌면 프레임 순번이 리셋된다
#   X-Frame-Seq         단조 증가 프레임 번호
H_CAPTURE_CLOCK = "x-capture-clock"
H_PUBLISHER_SESSION = "x-publisher-session"
H_FRAME_SEQ = "x-frame-seq"
# MCV-1B4 - 계약(CAMERA_SOURCE_LOOPBACK_v1)에는 없는데 앱이 보내고 있었다.
# 2026-09-10 에 폰이 2.8초 늦는 원인을 이 값 하나가 갈랐다:
#   폰 44~51 ms / 태블릿 15~25 ms  -> 가공이 30 fps 입력을 18 fps 로밖에 못 냄 -> 큐
# R-5 "왕복 2초 예산"의 실측 성분이다. offsetMedian 안에 이만큼이 들어 있다.
H_PROCESS_MS = "x-process-ms"
H_PROCESS_BLUR = "x-process-blur"
# 앱 v0.2.1 부터. 인코딩은 가공(검열)이 아니므로 **이름이 다르다** —
# 그 헤더의 존재가 검열 증거로 읽히면 안 되기 때문이다.
H_ENCODE_MS = "x-encode-ms"
H_ENCODE_RULES = "x-encode-rules"
# 렌즈. 광각 1280x720 과 일반 1280x720 은 크기가 같아 크기 검사로는 못 가른다.
H_CAMERA_LENS = "x-camera-lens"
# MCV-2G / APP-3. 앱이 자이로로 잰 수평. ⭐ **안 오면 안 오는 대로 둔다** —
# 영상으로 기울기를 추정해 이 자리에 넣지 않는다(금지 조항 6: 추정을 관측으로 안 낸다).
H_CAMERA_PITCH = "x-camera-pitch"
H_CAMERA_ROLL = "x-camera-roll"
H_PROCESS_RULES = "x-process-rules"
# 🔴 2026-09-19: **앱이 보내는데 우리가 버리고 있었다.** 카메라앱 세션이 양쪽 소스를
#    대조해 찾았다. 회전은 사람이 버튼으로 정하는 값이고(센서를 안 따라간다) 현장
#    설정이 `rotationDegrees 90` 이므로, 이걸 모르는 소비자는 **x·y 축이 뒤바뀐**
#    좌표를 만든다. 호모그래피에 직접 걸리는 양이라 조용히 버리면 안 된다.
H_CAMERA_ROTATION = "x-camera-rotation"
H_CAMERA_ZOOM = "x-camera-zoom"
# 시계 기준 3종. **앱이 아직 안 보낸다** — 여기 먼저 등재해 둔다(①정적설정 → ②생산자).
# 순서를 뒤집으면 앱을 재설치하고도 값이 여기서 버려져 "아무 변화가 없는" 상태가 된다.
H_CAPTURE_BASIS = "x-capture-basis"
H_PUBLISH_CLOCK = "x-publish-clock"
H_CLOCK_SUSPECT = "x-clock-suspect"
# ⭐ 여기 없는 헤더는 parse_part_headers 가 **버린다.** 상수만 만들고 이 목록에
#    안 넣으면 기능이 죽은 채로 유닛테스트는 통과한다 — 2026-09-10 에 실제로 그러했다
#    (X-Process-Blur). 짝이 맞는지는 test_censor_box 가 강제한다.
#
# 🔴 그런데 이 주석은 **통제가 아니었다.** 반대 방향으로 또 뚫렸다 — 우리가 상수를
#    안 만든 것이 아니라 **상대가 새 헤더를 보내기 시작한 것**이고, 그건 이 레포 안에서
#    아무 시험도 못 본다(앱은 다른 레포다). 그래서 목록을 늘리는 것으로 끝내지 않고
#    **모르는 `x-` 헤더를 세어 곁표에 싣는다**(`unknownHeaders`). 다음 어긋남은
#    사람이 두 레포를 손으로 대조할 때가 아니라 **그 자리에서** 보인다.
SIDECAR_HEADERS = (H_CAPTURE_CLOCK, H_PUBLISHER_SESSION, H_FRAME_SEQ,
                   H_PROCESS_MS, H_PROCESS_RULES, H_PROCESS_BLUR,
                   H_ENCODE_MS, H_ENCODE_RULES, H_CAMERA_LENS,
                   H_CAMERA_PITCH, H_CAMERA_ROLL,
                   H_CAMERA_ROTATION, H_CAMERA_ZOOM,
                   H_CAPTURE_BASIS, H_PUBLISH_CLOCK, H_CLOCK_SUSPECT)

#: `unknownHeaders` 에 담을 이름 개수 상한. 값은 **안 담는다**(내용 유출 방지).
UNKNOWN_HEADER_CAP = 12

#: 조회를 매 줄마다 다시 인코딩하지 않는다.
_SIDECAR_KEYS = frozenset(h.encode("ascii") for h in SIDECAR_HEADERS)

#: tailnet 주소 → 현재 LAN endpoint 해석 결과를 이만큼 재사용한다.
#: 짧으면 `tailscale ping` 을 쉴 새 없이 부르고, 길면 기기가 옮겨 간 걸 늦게 안다.
DISCOVER_TTL_S = 30.0

#: `tailscale ping` 이 직통 경로를 찾으면 `pong from ... via 198.51.100.17:41801 in 3ms` 를 낸다.
_VIA_RE = re.compile(r"via (\d{1,3}(?:\.\d{1,3}){3}):(\d+)")


def tailscale_lan_endpoint(tailnet_ip, runner=None, timeout_s=8.0):
    """tailnet 주소를 **지금 이 자리의 LAN 주소**로 바꾼다. 못 찾으면 None.

    ## 왜 이것이 필요한가 (2026-09-19 실측)

    이 파일은 이미 *"장소마다 도달 경로가 다르다"* 고 적어 두고 후보 목록으로 버티고
    있었다. 그런데 후보 목록은 **사이트가 늘 때마다 자란다** — `tablet-relay` 는 벌써
    셋이고, 폰 둘은 핫스팟 주소 **하나뿐**이라 현장 LAN 으로 옮기자 아무 데도 못 닿았다
    (소스 10개 전부 미접속).

    ⭐ 근본 해결은 **발견**이다. `configs/video_sources.json` 의 `_why` 가 이미
       그렇게 적어 뒀다 — tailscale 이 피어의 LAN endpoint 를 알고 있다.

    ## 실측

    2026-09-19 에 중계에서 세 기기로 확인했다 — 직통 LAN 주소를 얻어 `:18086` 이 3 ms 에
    열렸고, 앱이 꺼진 기기는 `ConnectionRefused`, 없는 기기는 timeout 이었다.
    **주소는 여기 안 적는다** — 이동 기기 주소를 코드에 두지 않는 것이 이 레포의 규칙이고
    (`tests/test_no_baked_addresses.py` 가 강제한다), 실제로 초판에 적었다가 걸렸다.
    수치와 주소는 `docs/DECISIONS_20260919_SOURCE_DISCOVERY.md` 에 있다.

    ⚠️ **tailnet 주소로 직접 TCP 를 걸면 안 된다.** 중계는 shared-in 노드라 `tailscale
       ping` 은 되는데 앱 포트 TCP 는 전부 timeout 이다(이 파일 `__init__` 주석의
       2026-09-09 실측과 같은 사실을 2026-09-19 에 재확인). 반드시 LAN 주소로 **바꿔서**
       붙는다. 노트북에서는 tailnet 직결이 되는데 중계에서는 안 되는 **비대칭**이다.

    ⭐ 그리고 이 경로는 실패 이유를 **가른다**: `ConnectionRefused` 는 "기기는 있는데
       앱이 꺼짐", timeout 은 "기기가 없음". 후보 목록만 쓰면 둘 다 `connected:false` 로
       뭉개진다.
    """
    if not tailnet_ip:
        return None
    if runner is None:
        runner = _default_runner
    try:
        out = runner(["tailscale", "ping", "-c", "1", "--timeout", "3s", str(tailnet_ip)],
                     timeout_s)
    except Exception:                                  # noqa: BLE001 - 발견 실패는 치명적이지 않다
        return None
    if not out:
        return None
    m = _VIA_RE.search(out)
    return m.group(1) if m else None


def _default_runner(argv, timeout_s):
    """시험이 갈아 끼울 수 있게 분리한다 — CI 에는 tailscale 이 없다."""
    proc = subprocess.run(argv, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", timeout=timeout_s)
    return (proc.stdout or "") + (proc.stderr or "")


def discovered_url(discover, runner=None):
    """`{"tailnetIp":..., "port":..., "path":...}` → 지금 붙을 수 있는 URL 또는 None."""
    if not isinstance(discover, dict):
        return None
    ip = tailscale_lan_endpoint(discover.get("tailnetIp"), runner=runner)
    if not ip:
        return None
    port = int(discover.get("port") or 18086)
    path = discover.get("path") or "/processed"
    if not path.startswith("/"):
        path = "/" + path
    return "http://%s:%d%s" % (ip, port, path)


# 시계 오차 통계를 내는 창. 짧으면 지터에 흔들리고 길면 이동을 못 따라간다.
CLOCK_WINDOW_S = 60.0

# 수신 fps 를 세는 창. 짧으면 튀고 길면 끊긴 걸 늦게 안다.
RECV_FPS_WINDOW_S = 5.0


def parse_part_headers(preamble, unknown_out=None):
    """파트 헤더 블록에서 곁표를 뽑는다. 없으면 빈 dict.

    ⭐ 관용을 없애지 않는다. 헤더가 **있으면** 쓰고, 없으면 지금까지처럼 매직 바이트로만
    자른다. 상대 구현(앱·가공 단·픽스처·다른 게이트웨이)이 제각각이어도 프레임 수신은
    계속돼야 한다 — 곁표가 없다고 영상을 끊으면 그건 개선이 아니라 회귀다.
    """
    out = {}
    if not preamble:
        return out
    for line in preamble.split(b"\r\n"):
        i = line.find(b":")
        if i <= 0:
            continue
        key = line[:i].strip().lower()
        if key not in _SIDECAR_KEYS:
            # ⭐ `x-` 로 시작하는 것만 센다. `Content-Type`·`Content-Length` 는 파트의
            #    정상 구성요소라 "모르는 곁표" 가 아니다.
            if (unknown_out is not None and key.startswith(b"x-")
                    and len(unknown_out) < UNKNOWN_HEADER_CAP):
                unknown_out.add(key.decode("ascii", "ignore"))
            continue
        out[key.decode("ascii")] = line[i + 1:].strip().decode("ascii", "ignore")
    return out


def _as_float(value):
    """숫자가 아니면 None. 상대가 이상한 값을 보내도 통계를 오염시키지 않는다."""
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return None


def _recv_fps(times, now, window=RECV_FPS_WINDOW_S):
    """창 안에 실제로 **도착한** 프레임 수 / 창 길이.

    ⭐ 서빙 fps 와 다른 사실이다. 아무도 안 봐도 프레임은 오고 있을 수 있고,
       그때 fps 0 을 보여 주면 사람이 소스가 죽은 줄 안다.
    """
    recent = [t for t in times if (now - t) <= window]
    if len(recent) < 2:
        return 0.0
    span = recent[-1] - recent[0]
    if span <= 0:
        return 0.0
    return round((len(recent) - 1) / span, 1)


def parse_blur(value):
    """`x,y,w,h` (프레임 비율 0~1) → dict. 못 읽으면 None.

    ⭐ **헤더 부재와 못 읽음은 다르다.** 부재는 "상자가 없다"이고 못 읽음은 "모른다"다.
       둘을 같은 None 으로 뭉치면, 규약이 바뀐 날 관측기가 조용히 "안 가려졌다"고 믿는다.
       그래서 호출자는 원문(raw)도 함께 들고 다닌다.
    """
    if not value:
        return None
    parts = str(value).split(",")
    if len(parts) != 4:
        return None
    try:
        x, y, w, h = (float(p) for p in parts)
    except (TypeError, ValueError):
        return None
    for v in (x, y, w, h):
        if not (v == v and abs(v) != float("inf")):      # NaN·inf 거르기
            return None
    if w <= 0.0 or h <= 0.0:
        return None
    return {"x": x, "y": y, "w": w, "h": h}


def _as_int(value):
    """숫자가 아니면 None. 상대가 이상한 값을 보내도 통계를 오염시키지 않는다."""
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return None


# ---- MCV-1B6: 프레임의 실제 크기 ------------------------------------------------

# SOFn 마커. 여기에 폭·높이가 들어 있다. SOF4(0xC4)·SOF8(0xC8)·SOF12(0xCC) 는
# 프레임 헤더가 아니라 허프만/산술/JPEG-LS 라서 뺀다.
_SOF_MARKERS = frozenset((0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                          0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF))


def jpeg_dimensions(data):
    """JPEG 바이트 → (폭, 높이). 못 읽으면 None.

    ⭐ 앱이 말하는 해상도가 아니라 **프레임이 말하는 해상도**다.
       설정값과 실물이 다를 수 있다는 걸 2026-09-10 에 비싸게 배웠다.
    """
    if not data or len(data) < 10:
        return None
    i, n = 2, len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in _SOF_MARKERS:
            height = (data[i + 5] << 8) | data[i + 6]
            width = (data[i + 7] << 8) | data[i + 8]
            if width > 0 and height > 0:
                return (width, height)
            return None
        if marker == 0xD8 or marker == 0x01 or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        if marker == 0xFF:
            i += 1                      # 채움 바이트
            continue
        seg = (data[i + 2] << 8) | data[i + 3]
        if seg < 2:
            return None
        i += 2 + seg
    return None


class MjpegPuller:
    """원격 MJPEG URL 하나를 당겨와 최신 JPEG 을 들고 있는다."""

    def __init__(self, url, name=None, stale_after_s=DEFAULT_STALE_AFTER_S,
                 connect_timeout_s=DEFAULT_CONNECT_TIMEOUT_S,
                 clock=time.time, opener=None, discover=None, resolver=None):
        # url 은 문자열 하나이거나 후보 목록이다. 장소마다 도달 경로가 다르기 때문이다:
        # 교육장에서는 기기가 같은 LAN(198.51.100.x)에 있고, 집에서는 tailnet 뿐인데
        # 중계는 shared-in 노드라 tailnet TCP 가 막힌다(2026-09-09 실측).
        # 설정을 장소에 묶으면 이동할 때마다 죽는다 - 같은 날 DDS 에서 이미 겪었다.
        self.urls = [url] if isinstance(url, str) else list(url)
        # ⭐ `discover` 가 있으면 후보가 비어 있어도 된다 — 주소를 **찾아서** 쓴다.
        self.discover = discover if isinstance(discover, dict) else None
        self._resolver = resolver          # 시험이 갈아 끼운다
        self._discovered_url = None
        self._discovered_at = 0.0
        self._discover_fails = 0
        if not self.urls and not self.discover:
            raise ValueError("url 후보가 비어 있다 (discover 도 없다)")
        self.url = self.urls[0] if self.urls else None
        self.active_url = None
        self.name = name or (self.urls[0] if self.urls
                             else "discover:%s" % self.discover.get("tailnetIp"))
        self.stale_after_s = stale_after_s
        self.connect_timeout_s = connect_timeout_s
        self._clock = clock
        # 테스트에서 갈아끼울 수 있게 주입 가능하게 둔다.
        self._opener = opener or (lambda u, t: urllib.request.urlopen(u, timeout=t))

        self._lock = threading.Lock()
        self._latest = None
        self._latest_stamp = 0.0
        self._frames = 0
        self._connects = 0
        self._errors = 0
        self._last_error = None

        # MCV-1B1 곁표 상태. 마지막 파트의 값과, 창 안의 오차 표본.
        self._capture_clock_ms = None
        self._frame_seq = None
        self._publisher_session = None
        self._frames_lost = 0
        self._seq_gaps = 0
        self._sidecar_frames = 0
        self._offsets = collections.deque()   # (receivedAt, offsetSec)
        # MCV-1B4 가공 소요 시간. 마지막 값과 창 안의 표본.
        self._process_ms = None
        self._process_rules = None
        self._process_samples = collections.deque()   # (receivedAt, ms)
        # MCV-2R 검열 상자. 마지막 프레임에 **실제로 쓰인** 상자다.
        # raw 를 함께 들고 있는 이유는 parse_blur 의 독스트링에.
        self._censor_box = None
        self._censor_raw = None
        # 앱이 내는 것. encode 는 가공과 **다른 양**이라 따로 들고 있는다.
        self._encode_ms = None
        self._encode_rules = None
        self._camera_lens = None
        # MCV-2G 앱이 보내는 수평. 없으면 None 으로 남는다 - 지어내지 않는다.
        self._camera_pitch = None
        # 🔴 2026-09-19 신설. 앱이 보내는데 우리가 버리고 있던 둘.
        self._camera_rotation = None
        self._camera_zoom = None
        # 우리가 모르는 x- 헤더의 **이름만** 모은다. 값은 안 담는다.
        self._unknown_headers = set()
        # 🔴 2026-09-19 앱 vc18/vc19 가 내보내기 시작한 시계 셋.
        #    화이트리스트에만 넣고 여기서 안 꺼내면 **파싱해 놓고 버린다** —
        #    실제로 등재 직후 그 상태였다(meta.get 호출 0곳).
        self._capture_basis = None
        self._clock_suspect_app = None
        # ⭐ `publish − capture` 는 **기기 한 시계 안의 뺄셈**이라 시계차가 상쇄된다.
        #    그래서 이것의 흔들림은 **기기 내부 지연**이고, `offsetJitterMs`(시계+전송+큐)
        #    에서 그 성분을 가려내는 열쇠다.
        self._dev_latency = collections.deque()   # (receivedAt, ms)
        self._camera_roll = None
        # MCV-1B6 마지막 프레임의 **실제** 픽셀 크기. /status 의 설정값이 아니다.
        self._frame_size = None
        # MCV-1B5 **수신** fps. Source 의 fps 는 서빙 fps 라 아무도 안 보면 0 이다 -
        # 그러면 'connected=True fps=0' 이 되어 사람이 끊긴 줄 안다. 둘은 다른 사실이다.
        self._recv_times = collections.deque()

        self._stop = threading.Event()
        self._thread = None

    # ---- 소스 레지스트리 계약 -------------------------------------------------

    def get_latest_jpeg(self):
        """(jpeg|None, stamp, connected) — Source 의 jpeg_provider 가 기대하는 형태."""
        with self._lock:
            jpeg = self._latest
            stamp = self._latest_stamp
        connected = self._is_fresh(stamp)
        if not connected:
            # 낡은 프레임을 살아 있는 것처럼 내보내지 않는다.
            return None, stamp, False
        return jpeg, stamp, True

    def _is_fresh(self, stamp):
        if stamp <= 0.0:
            return False
        return (self._clock() - stamp) <= self.stale_after_s

    def sidecar(self):
        """MCV-1B1/1B2/1B3 — 이 소스의 시각 품질과 손실을 숫자로 낸다.

        ⭐ **중앙값과 지터를 한 이름으로 부르지 않는다.**
          offsetMedianMs = (기기 시계 오차 + 전송 지연) 의 합
          offsetJitterMs = 그 표본의 중앙값 절대편차 = 전송 지연의 흔들림만
        둘을 하나로 뭉치면 "시계가 3초 틀렸다"와 "네트워크가 3초 밀렸다"를 구분 못 한다.

        ⭐ source 는 **마지막 파트 기준**이다. 곁표가 오다가 끊기면 정직하게 receive 로 떨어진다.
        """
        now = self._clock()
        with self._lock:
            offsets = [o for t, o in self._offsets if (now - t) <= CLOCK_WINDOW_S]
            dev = [x for t, x in self._dev_latency if (now - t) <= CLOCK_WINDOW_S]
            capture_ms = self._capture_clock_ms
            proc = [ms for t, ms in self._process_samples
                    if (now - t) <= CLOCK_WINDOW_S]
            out = {
                "source": "capture" if capture_ms is not None else "receive",
                "captureClockMs": capture_ms,
                "frameSeq": self._frame_seq,
                "publisherSession": self._publisher_session,
                "processMs": self._process_ms,
                "processRules": self._process_rules,
                "censorBox": (dict(self._censor_box) if self._censor_box else None),
                "censorBoxRaw": self._censor_raw,
                "encodeMs": self._encode_ms,
                "encodeRules": self._encode_rules,
                "cameraLens": self._camera_lens,
                "cameraPitch": self._camera_pitch,
                "cameraRotation": self._camera_rotation,
                "cameraZoom": self._camera_zoom,
                # ⭐ 비어 있어야 정상이다. 차 있으면 **상대가 우리가 모르는 것을
                #    보내고 있다**는 뜻이고, 그건 조용히 버려지고 있다는 뜻이다.
                "unknownHeaders": sorted(self._unknown_headers),
                "cameraRoll": self._camera_roll,
                "framesWithSidecar": self._sidecar_frames,
                "framesLost": self._frames_lost,
                "seqGaps": self._seq_gaps,
                "frameWidth": (self._frame_size[0] if self._frame_size else None),
                "frameHeight": (self._frame_size[1] if self._frame_size else None),
                "receiveFps": _recv_fps(self._recv_times, now),
                "windowSec": CLOCK_WINDOW_S,
                "samples": len(offsets),
                "offsetMedianMs": None,
                "offsetJitterMs": None,
                # 앱 vc18+ 가 선언하는 시계 기준. 없으면 None — 옛 빌드라는 뜻이다.
                "captureBasis": self._capture_basis,
                "clockSuspectApp": self._clock_suspect_app,
                # `publish − capture`. 기기 한 시계 안의 값이라 **시계차와 무관**하다.
                "deviceLatencyMedianMs": None,
                "deviceLatencyJitterMs": None,
                "deviceLatencySamples": 0,
            }
        # 가공 시간의 중앙값. offsetMedian 안에 들어 있는 **아는 성분**이다 -
        # 나머지가 전송·큐라는 것을 이 값이 있어야 말할 수 있다.
        out["processMedianMs"] = (round(statistics.median(proc), 1) if proc else None)
        out["processSamples"] = len(proc)
        if offsets:
            median = statistics.median(offsets)
            out["offsetMedianMs"] = round(median * 1000.0, 1)
            out["offsetJitterMs"] = round(
                statistics.median([abs(o - median) for o in offsets]) * 1000.0, 1)
        # ⭐ 기기 내부 지연. `offsetJitterMs` 는 **시계차 + 전송 + 큐** 의 합이고
        #    이건 그중 **기기 안에서 일어난 몫**이다 — 둘을 나란히 두면 사람이 가를 수 있다.
        #    ⚠️ 빼서 '전송 지터' 를 만들지 않는다. 중앙값절대편차는 그렇게 안 쪼개진다.
        if dev:
            dmed = statistics.median(dev)
            out["deviceLatencyMedianMs"] = round(dmed, 1)
            out["deviceLatencyJitterMs"] = round(
                statistics.median([abs(x - dmed) for x in dev]), 1)
            out["deviceLatencySamples"] = len(dev)
        return out

    def _note_sidecar(self, meta, now):
        """파트 헤더에서 뽑은 곁표를 반영한다. 호출자는 _lock 을 잡고 있지 않다."""
        capture_ms = _as_int(meta.get(H_CAPTURE_CLOCK))
        seq = _as_int(meta.get(H_FRAME_SEQ))
        session = meta.get(H_PUBLISHER_SESSION)
        process_ms = _as_float(meta.get(H_PROCESS_MS))
        process_rules = meta.get(H_PROCESS_RULES)
        blur_raw = meta.get(H_PROCESS_BLUR)
        encode_ms = _as_float(meta.get(H_ENCODE_MS))
        encode_rules = meta.get(H_ENCODE_RULES)
        camera_lens = meta.get(H_CAMERA_LENS)
        camera_pitch = _as_float(meta.get(H_CAMERA_PITCH))
        camera_roll = _as_float(meta.get(H_CAMERA_ROLL))
        camera_rotation = _as_float(meta.get(H_CAMERA_ROTATION))
        camera_zoom = _as_float(meta.get(H_CAMERA_ZOOM))
        capture_basis = meta.get(H_CAPTURE_BASIS)
        publish_clock_ms = _as_int(meta.get(H_PUBLISH_CLOCK))
        clock_suspect_raw = meta.get(H_CLOCK_SUSPECT)
        clock_suspect_app = (None if clock_suspect_raw is None
                             else str(clock_suspect_raw).strip().lower() == "true")
        with self._lock:
            if meta:
                self._sidecar_frames += 1
            self._capture_clock_ms = capture_ms
            if seq is not None:
                session_changed = (session != self._publisher_session)
                if (self._frame_seq is not None and not session_changed
                        and seq > self._frame_seq + 1):
                    # 순번이 건너뛰었다 = 그만큼 잃었다. 조용히 넘어가지 않는다.
                    self._frames_lost += seq - self._frame_seq - 1
                    self._seq_gaps += 1
                self._frame_seq = seq
            self._publisher_session = session
            self._process_ms = process_ms
            if process_rules is not None:
                self._process_rules = process_rules
            # 상자는 프레임마다 바뀔 수 있다(규칙 핫 리로드). 마지막 값을 그대로 쓴다 —
            # 헤더가 사라지면 상자가 꺼진 것이므로 None 으로 **되돌아가야 한다.**
            if meta:
                self._censor_raw = blur_raw
                self._censor_box = parse_blur(blur_raw)
                # 헤더가 사라지면 값도 사라져야 한다 - 상자와 같은 이유다.
                self._encode_ms = encode_ms
                self._encode_rules = encode_rules
                self._camera_lens = camera_lens
                # 헤더가 사라지면 값도 사라진다 - 렌즈·상자와 같은 이유다.
                self._camera_pitch = camera_pitch
            if camera_rotation is not None:
                self._camera_rotation = camera_rotation
            if camera_zoom is not None:
                self._camera_zoom = camera_zoom
            if capture_basis:
                self._capture_basis = capture_basis
            if clock_suspect_app is not None:
                self._clock_suspect_app = clock_suspect_app
            # ⭐ 둘 다 있을 때만 뺀다. 한쪽이 없으면 **지어내지 않는다.**
            if publish_clock_ms is not None and capture_ms is not None:
                self._dev_latency.append((now, float(publish_clock_ms - capture_ms)))
                while (self._dev_latency
                       and (now - self._dev_latency[0][0]) > CLOCK_WINDOW_S):
                    self._dev_latency.popleft()
                self._camera_roll = camera_roll
            if process_ms is not None:
                self._process_samples.append((now, process_ms))
                while (self._process_samples
                       and (now - self._process_samples[0][0]) > CLOCK_WINDOW_S):
                    self._process_samples.popleft()
            if capture_ms is not None:
                self._offsets.append((now, now - (capture_ms / 1000.0)))
                while self._offsets and (now - self._offsets[0][0]) > CLOCK_WINDOW_S:
                    self._offsets.popleft()

    def stats(self):
        with self._lock:
            return {
                "url": self.url,
                "urls": list(self.urls),
                "activeUrl": self.active_url,
                # ⭐ 발견 상태를 숨기지 않는다 — `discoveredUrl` 이 None 인데 소스가
                #    안 붙으면 "주소를 못 찾은 것" 이고, 주소는 찾았는데 안 붙으면
                #    "앱이 꺼진 것" 이다. 둘은 처방이 다르다.
                "discover": (dict(self.discover) if self.discover else None),
                "discoveredUrl": self._discovered_url,
                "discoverFails": self._discover_fails,
                "frames": self._frames,
                "connects": self._connects,
                "errors": self._errors,
                "lastError": self._last_error,
                "lastFrameStamp": self._latest_stamp,
                "framesWithSidecar": self._sidecar_frames,
                "framesLost": self._frames_lost,
                "frameSize": (list(self._frame_size) if self._frame_size else None),
            }

    # ---- 수명주기 -------------------------------------------------------------

    def start(self):
        if self._thread is not None:
            return self
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="mjpeg-puller-%s" % self.name, daemon=True)
        self._thread.start()
        return self

    def stop(self, timeout=3.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    def _resolve_discovered(self, force=False):
        """발견 결과를 TTL 안에서 재사용한다. 실패하면 None 이고 고정 후보로 떨어진다."""
        if not self.discover:
            return None
        now = self._clock()
        if (not force and self._discovered_url
                and (now - self._discovered_at) < DISCOVER_TTL_S):
            return self._discovered_url
        url = discovered_url(self.discover, runner=self._resolver)
        with self._lock:
            self._discovered_url = url
            self._discovered_at = now
            if url is None:
                self._discover_fails += 1
        return url

    def _candidates(self, force_discover=False):
        """이번 바퀴에 시도할 주소들. **발견한 것이 먼저**, 고정 후보가 뒤.

        ⭐ 발견이 먼저인 이유: 고정 후보는 *지난 사이트* 의 주소다. 먼저 시도하면
           매번 timeout 을 기다린 뒤에야 옳은 주소에 닿는다.
        ⚠️ 발견이 실패해도 고정 후보로 계속 돈다 — 발견은 **fail-open** 이다.
           tailscale 이 없는 소비자(도커 복제본·팀원 기기)도 그대로 동작해야 한다.
        """
        found = self._resolve_discovered(force=force_discover)
        out = ([found] if found else []) + list(self.urls)
        return out or ([found] if found else [])

    # ---- 내부 ----------------------------------------------------------------

    def _run(self):
        backoff = BACKOFF_START_S
        idx = 0
        while not self._stop.is_set():
            cands = self._candidates()
            if not cands:
                # 발견도 실패하고 고정 후보도 없다 — 기다렸다 다시 찾는다.
                if self._stop.wait(backoff):
                    return
                backoff = min(backoff * 2, BACKOFF_MAX_S)
                continue
            candidate = cands[idx % len(cands)]
            try:
                resp = self._opener(candidate, self.connect_timeout_s)
            except Exception as exc:                      # noqa: BLE001 - 어떤 실패든 재시도한다
                self._note_error(exc)
                idx += 1
                # 후보를 한 바퀴 다 돌았을 때만 기다린다 - 첫 후보가 죽었다고
                # 두 번째를 5초 뒤에 보는 것은 느리다.
                if idx % len(cands) == 0:
                    # ⭐ 한 바퀴를 다 돌아 전부 실패했으면 **주소가 낡았을 수 있다.**
                    #    기다리기 전에 다시 찾는다 — 기기가 옮겨 갔을 때 이게 복구다.
                    self._resolve_discovered(force=True)
                    if self._stop.wait(backoff):
                        return
                    backoff = min(backoff * 2, BACKOFF_MAX_S)
                continue

            with self._lock:
                self._connects += 1
                self.active_url = candidate
                frames_at_connect = self._frames
            backoff = BACKOFF_START_S
            try:
                self._consume(resp)
            except Exception as exc:                      # noqa: BLE001
                self._note_error(exc)
            finally:
                with self._lock:
                    self.active_url = None
                    got = self._frames - frames_at_connect
                try:
                    resp.close()
                except Exception:
                    pass
            # ⭐ **접속됐다는 것과 데이터가 온다는 것은 다른 사실이다.**
            # 이 접속이 프레임을 한 장도 못 냈으면 후보를 넘긴다. 안 넘기면 200 과
            # Content-Type 까지 정상으로 주고 본문을 한 바이트도 안 보내는 주소에
            # **영영 묶인다** — 읽기 실패는 같은 후보로 재접속하기 때문이다.
            # 2026-09-12 교육장 실측: 태블릿 앱 :18086/processed 가 그 상태였고,
            # 멀쩡한 :18082/video 가 urlCandidates 에 **이미 있었는데** 8분 넘게 안 쓰였다.
            #
            # ⚠️ 한 장이라도 받았으면 넘기지 않는다. 잘 되던 주소가 망 깜빡으로 끊긴 것을
            #    후보 회전으로 갚으면, 검증된 경로를 버리고 더 나쁜 것으로 가게 된다.
            if got == 0:
                idx += 1
            if self._stop.wait(BACKOFF_START_S):
                return

    @staticmethod
    def _read_available(resp, size):
        """온 만큼만 읽는다. **read(n) 은 n 바이트가 찰 때까지 막힌다.**

        생산자가 소비자보다 빠르면 소켓 버퍼가 차고 지연이 **누적**된다.
        2026-09-10 실기기 대조군(폰, 같은 순간 순차, 읽기 방식만 다르게):
            read(8192)    처음 2625.8 -> 마지막 3270.6 ms   12초에 +645 ms 누적
            read1(65536)  처음 2951.3 -> 마지막 2914.4 ms   안정
        read1 이 없는 응답 객체(구식 목·픽스처)는 read 로 떨어진다.
        """
        reader = getattr(resp, "read1", None)
        if callable(reader):
            return reader(size)
        return resp.read(size)

    def _consume(self, resp):
        """응답 스트림에서 JPEG 을 잘라 최신 것만 보관한다.

        파트 헤더의 Content-Length 를 믿지 않고 **매직 바이트로 자른다** — 상대 구현이
        헤더를 조금 다르게 써도 견디게 하려는 것이다(게이트웨이·픽스처·앱이 다 다르다).

        MCV-1B1: 그 관용은 유지하되, JPEG 앞에 남은 바이트(경계선 + 파트 헤더)에서
        곁표가 **있으면** 줍는다. 없으면 전과 완전히 같게 동작한다.
        """
        buf = b""
        while not self._stop.is_set():
            chunk = self._read_available(resp, DEFAULT_READ_CHUNK)
            if not chunk:
                return
            buf += chunk
            if len(buf) > MAX_BUFFER_BYTES:
                # 매직이 없는 쓰레기 스트림. 버리고 재접속한다.
                raise ValueError("버퍼 상한 초과 — MJPEG 이 아닌 응답으로 보인다")
            while True:
                start = buf.find(JPEG_SOI)
                if start < 0:
                    break
                end = buf.find(JPEG_EOI, start + 3)
                if end < 0:
                    break
                # JPEG 앞의 바이트 = 이 파트의 경계선과 헤더. 곁표가 여기 있다.
                preamble = buf[:start]
                frame = buf[start:end + 2]
                buf = buf[end + 2:]
                now = self._clock()
                self._note_sidecar(
                    parse_part_headers(preamble, self._unknown_headers), now)
                size = jpeg_dimensions(frame)
                with self._lock:
                    self._latest = frame
                    self._latest_stamp = now
                    self._frames += 1
                    self._recv_times.append(now)
                    while (self._recv_times
                           and (now - self._recv_times[0]) > RECV_FPS_WINDOW_S):
                        self._recv_times.popleft()
                    if size:
                        self._frame_size = size

    def _note_error(self, exc):
        with self._lock:
            self._errors += 1
            self._last_error = "%s: %s" % (type(exc).__name__, exc)


# ---- 설정 -------------------------------------------------------------------

DEFAULT_CONFIG_PATH = os.environ.get(
    "VIDEO_SOURCES_CONFIG",
    os.path.expanduser("~/pinky_pro/src/pinky_pro_team11/configs/video_sources.json"),
)

_ALLOWED_TRUST = ("trusted", "untrusted")

# 도커 복제본 전용. 호스트에서 도는 발행기(docker/host_camera_publisher.py)를 당겨온다.
# 윈도우에서는 USB 장치를 컨테이너에 못 넘기므로 호스트가 발행하고 컨테이너가 pull 한다 -
# 현장의 폰·태블릿과 **같은 경로**라 우회가 아니라 오히려 충실하다.
HOST_CAMERA_URL_ENV = "MCV_HOST_CAMERA_URL"
HOST_CAMERA_ID = "docker-host-cam"


def load_pull_sources(path=None):
    """pull 소스 목록을 설정 파일에서 읽는다. 주소를 코드에 박지 않기 위한 것이다.

    파일이 없으면 **빈 목록**을 돌려준다 — 없는 것과 잘못된 것은 다르다.
    항목이 형식을 어기면 그 항목만 버리지 않고 **예외를 던진다**: 조용히 빠진 소스는
    "왜 안 보이지"로 몇 시간을 태운다.
    """
    p = path or DEFAULT_CONFIG_PATH
    if not os.path.exists(p):
        return []
    with io.open(p, encoding="utf-8") as fh:
        doc = json.load(fh)

    entries = doc.get("pullSources") or []
    if not isinstance(entries, list):
        raise ValueError("pullSources 는 리스트여야 한다: %r" % type(entries).__name__)

    out = []
    seen = set()
    for i, e in enumerate(entries):
        if not isinstance(e, dict):
            raise ValueError("pullSources[%d] 가 객체가 아니다" % i)
        if not e.get("id"):
            raise ValueError("pullSources[%d] 에 'id' 이 없다" % i)
        urls = e.get("urlCandidates") or ([e["url"]] if e.get("url") else [])
        # ⭐ `discover` 가 있으면 고정 후보가 없어도 된다 — 주소를 찾아서 쓴다.
        #    이동 기기의 주소는 사이트마다 바뀌므로, 고정 후보를 늘리는 대신 **안 변하는**
        #    tailnet 주소만 적고 접속 직전에 현재 LAN endpoint 로 바꾼다.
        discover = e.get("discover")
        if discover is not None:
            if not isinstance(discover, dict) or not discover.get("tailnetIp"):
                raise ValueError(
                    "pullSources[%d].discover 는 'tailnetIp' 를 가진 객체여야 한다" % i)
        if not urls and not discover:
            raise ValueError("pullSources[%d] 에 'url' 도 'discover' 도 없다" % i)
        if urls and (not isinstance(urls, list)
                     or not all(isinstance(u, str) and u for u in urls)):
            raise ValueError("pullSources[%d].urlCandidates 는 문자열 목록이어야 한다" % i)
        trust = e.get("trust", "trusted")
        if trust not in _ALLOWED_TRUST:
            raise ValueError("pullSources[%d].trust 는 %r 중 하나여야 한다: %r"
                             % (i, _ALLOWED_TRUST, trust))
        if e["id"] in seen:
            raise ValueError("pullSources 에 중복 id: %r" % e["id"])
        seen.add(e["id"])
        if e.get("enabled") is False:
            continue
        out.append({
            "id": e["id"],
            "url": urls[0] if urls else None,
            "urls": urls,
            "discover": discover,
            "label": e.get("label") or e["id"],
            "trust": trust,
            "viewpoint": bool(e.get("viewpoint", True)),
            "staleAfterSec": float(e.get("staleAfterSec", DEFAULT_STALE_AFTER_S)),
        })

    # 도커 복제본이 호스트의 USB 웹캠을 당겨올 때만 쓰는 소스.
    # ⭐ 설정 **파일**에 안 적는다. 적으면 현장 게이트웨이에도 영영 안 붙는 소스가 하나
    #    생기고, "왜 빨간색이지"를 누군가 또 따라간다. 없는 곳에서는 아예 없어야 한다.
    host_url = (os.environ.get(HOST_CAMERA_URL_ENV) or "").strip()
    if host_url:
        if HOST_CAMERA_ID in seen:
            raise ValueError("pullSources 에 이미 %r 이 있다 — 환경변수와 충돌한다"
                             % HOST_CAMERA_ID)
        out.append({
            "id": HOST_CAMERA_ID,
            "url": host_url,
            "urls": [host_url],
            "label": "호스트 웹캠 (복제본)",
            "trust": "trusted",
            "staleAfterSec": DEFAULT_STALE_AFTER_S,
        })
    return out
