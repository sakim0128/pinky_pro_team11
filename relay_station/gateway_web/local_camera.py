#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""중계 노트북에 직접 붙은 카메라 (MCV-1A 의 local transport).

왜 필요한가:
    계획서의 소스 목록은 폰 push · 태블릿 pull · Gazebo · 로봇 온보드 넷이었고
    **중계 자신의 웹캠이 빠져 있었다.** 그런데 실물 시점 셋 중

        폰       사용자가 들고 다닌다. 껐다 켠다. tailnet 제약을 받는다
        태블릿   같음
        중계 캠  중계에 직접 붙어 있다. 게이트웨이가 살아 있으면 항상 있다   <- 이것

    중계 캠만이 **항상 있는 실물 시점**이다. 2026-09-09 실측: LG Camera
    /dev/video0, 640x480 @ 30.1 fps. 폰·태블릿을 끈 상태에서도 살아 있다.

    그리고 tailnet 제약(중계는 다른 tailnet 의 shared-in 노드라 홈랩 기기에
    TCP 로 못 닿는다)을 **유일하게 우회**한다 — 네트워크를 안 타니까.

MCV-1C (2026-09-10) — 필요할 때만 연다:
    상시 캡처 루프는 노트북 캠 LED 를 계속 켜 둔다. 그래서 다른 세션이 `--no-camera` 로
    통째로 껐는데, 그 플래그는 pull 소스까지 끊어 실물 시점이 0 이 됐다.
    LED 의 정답은 boot 플래그가 아니라 **수요가 있을 때만 장치를 여는 것**이다.

        lazy=True         start() 는 등록만 한다. 첫 수요(get_latest_jpeg)가 장치를 연다
        idle_release_s    마지막 수요 뒤 이 시간이 지나면 장치를 놓는다 (LED 꺼짐)
        hold(True)        관측 세션처럼 상시 수요가 있는 소비자가 잡아 둔다
        probe()           상태만 본다. **수요로 치지 않는다** — 안 그러면 /api/safety 폴링이
                          카메라를 영원히 켜 둔다

설계 결정:
  * **최신 프레임만 보관한다.** pull 쪽과 같은 이유(DROP_TO_STAY_LIVE).
  * **JPEG 로 인코딩해서 보관한다.** 레지스트리 계약이 JPEG 바이트다.
  * **열기 실패를 예외로 터뜨리지 않는다.** 캠이 없거나 권한이 없어도
    게이트웨이는 떠야 한다. connected=false 로 정직하게 보고할 뿐이다.
  * 🟡 `/dev/video0` 권한은 로그인 세션 ACL 에 의존한다(`crw-rw----+`).
"""
import threading
import time

try:
    import cv2
except Exception:                                    # pragma: no cover
    cv2 = None

DEFAULT_STALE_AFTER_S = 3.0
DEFAULT_JPEG_QUALITY = 80
DEFAULT_IDLE_RELEASE_S = 10.0
REOPEN_BACKOFF_START_S = 1.0
REOPEN_BACKOFF_MAX_S = 15.0
READ_FAIL_LIMIT = 15


class _IdleRelease(Exception):
    """수요가 끊겨 장치를 놓는다. 오류가 아니다."""


class LocalCameraSource:
    """V4L2 장치 하나를 열어 최신 JPEG 을 들고 있는다."""

    def __init__(self, device=0, name=None, width=None, height=None,
                 target_fps=None, jpeg_quality=DEFAULT_JPEG_QUALITY,
                 stale_after_s=DEFAULT_STALE_AFTER_S, clock=time.time,
                 capture_factory=None, lazy=False, idle_release_s=DEFAULT_IDLE_RELEASE_S):
        self.device = device
        self.name = name or ("video%s" % device)
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.jpeg_quality = jpeg_quality
        self.stale_after_s = stale_after_s
        self.lazy = bool(lazy)
        self.idle_release_s = float(idle_release_s or 0.0)
        self._clock = clock
        # 테스트에서 가짜 카메라를 끼울 수 있게 주입 가능하게 둔다.
        self._capture_factory = capture_factory or self._open_v4l2

        self._lock = threading.Lock()
        self._latest = None
        self._latest_cv_frame = None
        self._latest_stamp = 0.0
        self._frames = 0
        self._opens = 0
        self._releases = 0
        self._errors = 0
        self._last_error = None
        self._last_demand = 0.0
        self._held = False
        self._started = False

        self._thread_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    # ---- 소스 레지스트리 계약 -------------------------------------------------

    def get_latest_jpeg(self):
        """(jpeg|None, stamp, connected) — Source 의 jpeg_provider 계약. **수요다.**"""
        now = self._clock()
        with self._lock:
            self._last_demand = now
            jpeg = self._latest
            stamp = self._latest_stamp
        if self.lazy and self._started:
            self._ensure_running()
        if stamp <= 0.0 or (now - stamp) > self.stale_after_s:
            # 낡은 프레임을 살아 있는 것처럼 내보내지 않는다.
            return None, stamp, False
        return jpeg, stamp, True

    def probe(self):
        """(None, stamp, connected) — 상태만 본다. 수요로 치지 않으므로 장치를 열지 않는다."""
        with self._lock:
            stamp = self._latest_stamp
        if stamp <= 0.0 or (self._clock() - stamp) > self.stale_after_s:
            return None, stamp, False
        return None, stamp, True

    def sidecar(self):
        """MCV-1B2 — 이 소스의 시각은 **수신 시각**이다. 정직하게 표기한다.

        V4L2 는 cap.read() 가 돌아온 시점밖에 안 준다. 촬영은 그보다 앞이고
        그 차이를 우리는 모른다. capture 라고 부르면 없는 정확도를 주장하는 것이다.
        """
        with self._lock:
            frame = self._latest_cv_frame
        size = None
        if frame is not None and getattr(frame, "shape", None):
            # shape = (h, w, c). 요청한 해상도가 아니라 **받은 배열**이다.
            size = (int(frame.shape[1]), int(frame.shape[0]))
        return {"source": "receive", "captureClockMs": None,
                "frameSeq": None, "publisherSession": None,
                "frameWidth": (size[0] if size else None),
                "frameHeight": (size[1] if size else None),
                "samples": 0, "offsetMedianMs": None, "offsetJitterMs": None}

    def get_latest_cv_frame(self):
        """(cv_frame|None, stamp, connected) — 원시 OpenCV BGR 프레임. 수요다."""
        with self._lock:
            frame = self._latest_cv_frame
            stamp = self._latest_stamp
            self._last_demand = self._clock()
        if self.lazy and self._started:
            self._ensure_running()
        if stamp <= 0.0 or (self._clock() - stamp) > self.stale_after_s:
            return None, stamp, False
        return frame, stamp, True

    def hold(self, flag):
        """상시 수요를 건다/푼다. 관측 세션이 켜지면 잡고, 꺼지면 놓는다."""
        self._held = bool(flag)
        if self._held and self._started:
            self._ensure_running()

    def is_open(self):
        t = self._thread
        return t is not None and t.is_alive()

    def stats(self):
        with self._lock:
            return {
                "device": self.device,
                "frames": self._frames,
                "opens": self._opens,
                "releases": self._releases,
                "errors": self._errors,
                "lastError": self._last_error,
                "lastFrameStamp": self._latest_stamp,
                "lastDemandAt": self._last_demand,
                "lazy": self.lazy,
                "held": self._held,
                "open": self.is_open(),
            }

    # ---- 수명주기 -------------------------------------------------------------

    def start(self):
        self._started = True
        if self.lazy and not self._held:
            return self                              # 수요가 올 때 연다
        self._ensure_running()
        return self

    def stop(self, timeout=3.0):
        self._started = False
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=timeout)
        with self._thread_lock:
            self._thread = None

    def _ensure_running(self):
        with self._thread_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run, name="local-cam-%s" % self.name, daemon=True)
            self._thread.start()

    # ---- 내부 ----------------------------------------------------------------

    def _open_v4l2(self):
        if cv2 is None:
            raise RuntimeError("cv2 를 쓸 수 없다")
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError("장치를 열지 못했다: %r" % (self.device,))
        if self.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        return cap

    def _encode(self, frame):
        ok, buf = cv2.imencode(".jpg", frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG 인코딩 실패")
        return buf.tobytes()

    def _idle(self, now):
        """lazy 이고 아무도 잡지 않았고 마지막 수요가 오래됐으면 True."""
        if not self.lazy or self._held or self.idle_release_s <= 0.0:
            return False
        with self._lock:
            last = self._last_demand
        return (now - last) > self.idle_release_s

    def _run(self):
        backoff = REOPEN_BACKOFF_START_S
        min_interval = (1.0 / self.target_fps) if self.target_fps else 0.0
        released = False
        try:
            while not self._stop.is_set() and not released:
                if self._idle(self._clock()):
                    break                            # 열기도 전에 수요가 끊겼다
                try:
                    cap = self._capture_factory()
                except Exception as exc:              # noqa: BLE001 - 캠이 없어도 게이트웨이는 산다
                    self._note_error(exc)
                    if self._stop.wait(backoff):
                        return
                    backoff = min(backoff * 2, REOPEN_BACKOFF_MAX_S)
                    continue

                with self._lock:
                    self._opens += 1
                backoff = REOPEN_BACKOFF_START_S
                fails = 0
                last_emit = 0.0
                try:
                    while not self._stop.is_set():
                        now = self._clock()
                        if self._idle(now):
                            raise _IdleRelease()
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            fails += 1
                            if fails >= READ_FAIL_LIMIT:
                                raise RuntimeError("연속 read 실패 %d회 — 장치를 다시 연다" % fails)
                            time.sleep(0.02)
                            continue
                        fails = 0
                        now = self._clock()
                        if min_interval and (now - last_emit) < min_interval:
                            time.sleep(0.001)
                            continue                  # 목표 fps 로 솎아낸다
                        last_emit = now
                        jpeg = self._encode(frame)
                        with self._lock:
                            self._latest = jpeg
                            self._latest_cv_frame = frame
                            self._latest_stamp = now
                            self._frames += 1
                except _IdleRelease:
                    released = True
                    with self._lock:
                        self._releases += 1
                except Exception as exc:              # noqa: BLE001
                    self._note_error(exc)
                finally:
                    try:
                        cap.release()
                    except Exception:
                        pass
                if released:
                    break
                if self._stop.wait(REOPEN_BACKOFF_START_S):
                    return
        finally:
            # 유휴로 나갔으면 다음 수요가 새 스레드를 만들 수 있게 자리를 비운다.
            with self._thread_lock:
                if self._thread is threading.current_thread():
                    self._thread = None

    def _note_error(self, exc):
        with self._lock:
            self._errors += 1
            self._last_error = "%s: %s" % (type(exc).__name__, exc)


# ---- 설정 -------------------------------------------------------------------

_ALLOWED_TRUST = ("trusted", "untrusted")


def load_local_sources(doc):
    """video_sources.json 의 localSources 절을 읽는다.

    pull 쪽과 같은 규칙: 형식을 어기면 조용히 버리지 않고 예외를 던진다.
    """
    entries = doc.get("localSources") or []
    if not isinstance(entries, list):
        raise ValueError("localSources 는 리스트여야 한다: %r" % type(entries).__name__)

    out = []
    seen = set()
    for i, e in enumerate(entries):
        if not isinstance(e, dict):
            raise ValueError("localSources[%d] 가 객체가 아니다" % i)
        if not e.get("id"):
            raise ValueError("localSources[%d] 에 'id' 이 없다" % i)
        if "device" not in e:
            raise ValueError("localSources[%d] 에 'device' 가 없다" % i)
        trust = e.get("trust", "trusted")
        if trust not in _ALLOWED_TRUST:
            raise ValueError("localSources[%d].trust 는 %r 중 하나여야 한다: %r"
                             % (i, _ALLOWED_TRUST, trust))
        if e["id"] in seen:
            raise ValueError("localSources 에 중복 id: %r" % e["id"])
        seen.add(e["id"])
        if e.get("enabled") is False:
            continue
        out.append({
            "id": e["id"],
            "device": e["device"],
            "label": e.get("label") or e["id"],
            "trust": trust,
            "width": e.get("width"),
            "height": e.get("height"),
            "targetFps": e.get("targetFps"),
            "jpegQuality": int(e.get("jpegQuality", DEFAULT_JPEG_QUALITY)),
            "staleAfterSec": float(e.get("staleAfterSec", DEFAULT_STALE_AFTER_S)),
            "viewpoint": bool(e.get("viewpoint", True)),
            "lazy": bool(e.get("lazy", True)),
            "idleReleaseSec": float(e.get("idleReleaseSec", DEFAULT_IDLE_RELEASE_S)),
        })
    return out
