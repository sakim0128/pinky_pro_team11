# -*- coding: utf-8 -*-
"""소스 레지스트리 (MCV-1A).

게이트웨이엔 카메라 매니저가 세 개(TabletStreamIngest·GazeboCameraManager·
RobotCameraManager) 따로 있고, /video_feed·/control_feed·/gazebo_feed·
/robot_camera_feed 네 개의 MJPEG 루프가 거의 똑같이 복붙돼 있었다. 이 모듈이
그 셋을 하나의 표면(id 로 조회, ?src= 로 스트림)으로 묶는다.

    등록                         조회·스트림
    register(Source(...))  ──>   get("phone").latest_jpeg()
                                 sources_public()  -> /api/sources
                                 trusted_ids()     -> 관측·보정이 쓸 소스만

⭐ 결정 1A (eng review): 각 소스에 trust 등급이 있다.
    - TRUSTED   : 관측(ArUco)·pose 보정이 이 소스만 쓴다
    - UNTRUSTED : 시청은 되지만 관측 입력으로 쓰지 않는다
   무인증 /api/camera/upload 로 들어오는 폰 프레임은 기본 UNTRUSTED 다. Phase 3 에서
   가짜 프레임 한 장이 로봇 위치를 오염시키는 경로를 여기서 끊는다.

이 모듈은 cv2·numpy·표준 라이브러리만 쓴다(rclpy 없음) → 로봇 없이 유닛테스트된다.
프레임 소스는 콜러블(`jpeg_provider() -> (bytes|None, stamp, connected)`)로 주입받아
게이트웨이의 기존 매니저든 픽스처든 똑같이 물린다.
"""
import threading
import time

import timing

TRUSTED = "trusted"
UNTRUSTED = "untrusted"
_TRUST_LEVELS = (TRUSTED, UNTRUSTED)

# transport: 프레임이 이 소스에 도달하는 방식. 표시용 + 정책 판단용.
PUSH = "push"      # 앱이 POST 로 밀어넣음 (/api/camera/upload). 발신자 미인증
PULL = "pull"      # 게이트웨이가 원격 URL 을 당겨옴
ROS = "ros"        # ROS2 토픽 구독 (Gazebo·로봇 온보드)
LOCAL = "local"    # 중계에 직접 붙은 장치 (V4L2). 네트워크를 안 탄다
_TRANSPORTS = (PUSH, PULL, ROS, LOCAL)

# 소스가 살아있다고 볼 최대 무프레임 시간(초). 이보다 오래 프레임이 없으면 stale.
DEFAULT_STALE_AFTER_S = 3.0


def jpeg_only_provider(getter, stale_after_s=DEFAULT_STALE_AFTER_S, clock=time.time):
    """jpeg 하나만 내는 매니저를 (jpeg, stamp, connected) 3-튜플로 감싼다.

    ⚠️ `connected = (jpeg is not None)` 로 근사하면 **항상 True** 가 된다.
    Gazebo·로봇 매니저는 "기다리는 중" 플레이스홀더를 늘 들고 있기 때문이다.
    2026-09-09 실측: Gazebo 프로세스가 죽었는데 conn=True, 로봇 Publisher 0 인데 conn=True.

    그래서 **바이트가 바뀌었는지**로 새 프레임 도착을 판정한다.

        첫 호출        기준선만 잡는다. 연결로 치지 않는다 (플레이스홀더일 수 있으므로)
        바이트 변함    새 프레임이 왔다 -> stamp 갱신
        바이트 그대로  아무것도 안 왔다 -> stamp 그대로 -> 곧 stale

    jpeg 자체는 그대로 돌려준다. "기다리는 중" 카드를 보여주는 건 사람에게 유용하다 —
    거짓말이었던 것은 **화면이 아니라 지표**다.
    """
    state = {"last": None, "stamp": 0.0}
    lock = threading.Lock()

    def provider():
        jpeg = getter()
        now = clock()
        with lock:
            if jpeg is not None:
                if state["last"] is None:
                    state["last"] = jpeg          # 기준선. 연결로 치지 않는다
                elif jpeg != state["last"]:
                    state["last"] = jpeg
                    state["stamp"] = now
            stamp = state["stamp"]
        fresh = stamp > 0.0 and (now - stamp) <= stale_after_s
        return jpeg, stamp, fresh

    return provider


class Source:
    """한 영상 소스. jpeg_provider 를 주입받아 최신 프레임과 메타를 노출한다.

    jpeg_provider() 는 (jpeg_bytes|None, stamp, connected) 를 돌려주는 콜러블이다.
    게이트웨이의 TabletStreamIngest.get_latest_jpeg 가 정확히 이 형태다.
    """

    def __init__(self, source_id, jpeg_provider, transport, trust,
                 label=None, stale_after_s=DEFAULT_STALE_AFTER_S, clock=time.time,
                 probe=None, sidecar=None, viewpoint=True):
        if trust not in _TRUST_LEVELS:
            raise ValueError("trust 는 %r 중 하나여야 한다: %r" % (_TRUST_LEVELS, trust))
        if transport not in _TRANSPORTS:
            raise ValueError("transport 는 %r 중 하나여야 한다: %r" % (_TRANSPORTS, transport))
        if not callable(jpeg_provider):
            raise TypeError("jpeg_provider 는 콜러블이어야 한다")
        self.id = source_id
        self.transport = transport
        self.trust = trust
        self.label = label or source_id
        self.stale_after_s = stale_after_s
        self._provider = jpeg_provider
        # probe 는 상태만 본다 - 수요로 치지 않는다. lazy 카메라가 /api/safety 폴링에
        # 켜지지 않게 하려는 것이다(MCV-1C). 없으면 provider 로 상태를 본다.
        self._probe = probe if callable(probe) else None
        # sidecar 는 이 소스의 시각 품질을 낸다(MCV-1B2). 없으면 receive 로 본다 -
        # 없는 정확도를 주장하지 않는 쪽이 기본값이어야 한다.
        self._sidecar = sidecar if callable(sidecar) else None
        self._clock = clock
        # ⭐ **찍히는 것과 아레나를 보는 것은 다른 사실이다.** 중계 내장 캠은 노트북 화면
        #    방향이라 사람 얼굴을 찍는다 — 살아 있다고 실물 시점으로 세면
        #    `/api/safety` 의 realViewpoints 가 부풀고 "관측 가능" 이 거짓이 된다.
        #    기본은 True 다(대부분의 소스는 시점이다). 아닌 것만 설정이 False 로 끈다.
        self.viewpoint = bool(viewpoint)
        # fps 는 스트림이 실제로 뽑아갈 때만 갱신된다(관측된 값). 추정하지 않는다.
        self._lock = threading.Lock()
        self._served = 0
        self._last_served_at = 0.0
        self._fps = 0.0
        self._fps_window_start = 0.0
        self._fps_window_count = 0

    def latest_jpeg(self):
        """(jpeg_bytes|None, stamp, connected). 스트림 루프가 이걸 뽑아간다."""
        jpeg, stamp, connected = self._provider()
        if jpeg is not None:
            self._tick(stamp)
        return jpeg, stamp, connected

    def _tick(self, stamp):
        now = self._clock()
        with self._lock:
            self._served += 1
            self._last_served_at = now
            if self._fps_window_start == 0.0:
                self._fps_window_start = now
            self._fps_window_count += 1
            span = now - self._fps_window_start
            if span >= 1.0:
                self._fps = self._fps_window_count / span
                self._fps_window_start = now
                self._fps_window_count = 0

    def is_trusted(self):
        return self.trust == TRUSTED

    def age_of_last_frame(self):
        """마지막으로 프레임을 뽑아간 뒤 경과 초. 한 번도 없으면 None.

        ⭐ 이건 '뽑아간' 시각이지 '수신한' 시각이 아니다. 아무도 안 보면 갱신 안 된다.
        연결 판정은 provider 의 connected 를 함께 본다.
        """
        with self._lock:
            if self._last_served_at == 0.0:
                return None
            return self._clock() - self._last_served_at

    def status(self):
        _, stamp, connected = (self._probe or self._provider)()
        age = self.age_of_last_frame()
        with self._lock:
            fps = round(self._fps, 1)
            served = self._served
        return {
            "id": self.id,
            "transport": self.transport,
            "trust": self.trust,
            "label": self.label,
            "connected": bool(connected),
            # ⭐ `fps` 는 **서빙** fps 다 - 아무도 안 보면 0 이다. 그 사실이 이름에 없어서
            #    "connected=True fps=0" 이 사람을 오해시켰다. 종류를 함께 낸다.
            "fps": fps,
            "fpsKind": "served",
            "receiveFps": (self.clock_info() or {}).get("receiveFps"),
            "framesServed": served,
            "ageOfLastFrameSec": None if age is None else round(age, 3),
            "lastFrameStamp": stamp if stamp else None,
            "clock": self.clock_info(),
            # U-1/U-2. ⭐ 어긋남과 지연을 **다른 칸**으로 낸다. 예전엔 화면이
            #    `now - captureClock`(= 시계차 + 전송 + 큐)을 `지연` 이라 불렀고,
            #    그래서 로컬 웹캠에 `지연 18837.8ms` 가 떴다(2026-09-12).
            "timing": timing.describe(self.clock_info(), self.transport),
        }

    def clock_info(self):
        """이 소스 프레임의 시각이 촬영 시각인지 수신 시각인지, 그리고 그 통계.

        ⭐ 품질이 다른 것을 같은 이름으로 부르지 않는다 — 앱은 촬영 시각을 보내고
        중계 내장 캠은 수신 시각밖에 모른다. 융합기가 가중치를 정하려면 그 차이를 알아야 한다.
        """
        if self._sidecar is None:
            return {"source": "receive"}
        try:
            info = self._sidecar()
        except Exception:                       # noqa: BLE001 - 곁표가 죽어도 소스는 산다
            return {"source": "receive"}
        return dict(info) if info else {"source": "receive"}


class SourceRegistry:
    """소스 id -> Source. 등록·조회·목록·신뢰 필터."""

    def __init__(self):
        self._lock = threading.Lock()
        self._sources = {}
        self._order = []
        self._default_id = None

    def register(self, source, default=False):
        if not isinstance(source, Source):
            raise TypeError("Source 인스턴스를 등록한다")
        with self._lock:
            if source.id in self._sources:
                raise ValueError("이미 등록된 소스 id: %r" % source.id)
            self._sources[source.id] = source
            self._order.append(source.id)
            if default or self._default_id is None:
                self._default_id = source.id
        return source

    def get(self, source_id):
        with self._lock:
            return self._sources.get(source_id)

    def default(self):
        with self._lock:
            if self._default_id is None:
                return None
            return self._sources[self._default_id]

    def resolve(self, source_id=None):
        """?src= 값을 Source 로. 없거나 미지정이면 기본 소스."""
        if source_id is None:
            return self.default()
        return self.get(source_id)

    def all_sources(self):
        """등록 순서대로 Source 객체를 돌려준다."""
        with self._lock:
            return [self._sources[i] for i in self._order]

    def ids(self):
        with self._lock:
            return list(self._order)

    def trusted_ids(self):
        """관측·보정이 입력으로 쓸 수 있는 소스만. 결정 1A 의 게이트."""
        with self._lock:
            return [sid for sid in self._order if self._sources[sid].is_trusted()]

    def sources_public(self):
        """GET /api/sources 응답용. 자격증명·내부 URL 은 절대 넣지 않는다."""
        with self._lock:
            ordered = [self._sources[sid] for sid in self._order]
            default_id = self._default_id
        return {
            "default": default_id,
            "sources": [s.status() for s in ordered],
        }


def clock_alignment(registry):
    """MCVA-13 — 살아 있는 실물 시점 **쌍**의 시각 오차를 숫자로 낸다.

    각 소스는 (수신시각 - 촬영시각) 의 중앙값을 낸다. 그 안에는 중계 시계가 공통으로
    들어 있으므로 **두 중앙값의 차**가 곧 두 기기 시계의 어긋남이다(중계 시계가 상쇄된다).

    ⭐ 촬영 시각이 없는 소스(clockSource=receive)가 낀 쌍은 **판정 불가**로 낸다.
    숫자를 지어내지 않는다 — 없는 정확도를 주장하는 것이 이 프로젝트의 반복 실패였다.
    """
    live = []
    for sid in real_viewpoint_ids(registry):
        src = registry.get(sid)
        if src is None:
            continue
        live.append((sid, src.clock_info()))

    pairs, unavailable = [], []
    for i in range(len(live)):
        for j in range(i + 1, len(live)):
            a_id, a = live[i]
            b_id, b = live[j]
            a_med, b_med = a.get("offsetMedianMs"), b.get("offsetMedianMs")
            if a_med is None or b_med is None:
                reason = ("NO_CAPTURE_CLOCK"
                          if "receive" in (a.get("source"), b.get("source"))
                          else "NO_SAMPLES")
                unavailable.append({"a": a_id, "b": b_id, "reason": reason})
                continue
            pairs.append({
                "a": a_id,
                "b": b_id,
                "offsetDiffMs": round(a_med - b_med, 1),
                "jitterMs": {a_id: a.get("offsetJitterMs"), b_id: b.get("offsetJitterMs")},
                "samples": {a_id: a.get("samples"), b_id: b.get("samples")},
            })
    return {"pairs": pairs, "unavailable": unavailable,
            "liveIds": [sid for sid, _ in live]}


# 다중 시점 융합에 필요한 최소 실물 시점 수. 하나로는 가림도 대조도 안 된다.
MIN_REAL_VIEWPOINTS = 2

# 실물 시점으로 세는 transport. ros 는 Gazebo(가상)와 로봇 온보드가 섞여 있고,
# push 는 출처 미상(untrusted)이라 관측 입력으로 쓰지 않는다.
REAL_VIEWPOINT_TRANSPORTS = (LOCAL, PULL)


def real_viewpoint_ids(registry):
    """지금 살아 있는 실물 시점 id 목록. 관측·보정의 입력 후보다."""
    out = []
    for src in registry.all_sources():
        if src.transport not in REAL_VIEWPOINT_TRANSPORTS:
            continue
        if not src.is_trusted():
            continue
        # 아레나를 안 보는 소스는 살아 있어도 시점이 아니다(내장 캠 등).
        if not getattr(src, "viewpoint", True):
            continue
        if src.status()["connected"]:
            out.append(src.id)
    return out


SEVERITY_ORDER = ("ok", "info", "degraded", "safety")


# ---- pose 가용성 (R-1, 2026-09-12) ------------------------------------------
#
# 🔴 실측: `/robot1/pose` 가 **Publisher 1 인데 18초간 메시지 0** 인 동안
#    `poseCorrectionPossible` 이 true 였다. 원인은 AMCL `update_min_d: 0.05` —
#    **정지한 로봇은 pose 를 안 낸다.** 발행자가 있는 것과 값이 오는 것은 다른 사실이다.
#
# ⭐⭐ 이 프로젝트의 판정 기준이 `topic list` -> `Publisher count` -> **실제 유량**
#    으로 늘어왔다. 여기만 두 번째에 머물러 있었다.
#
# ⭐ 값이 한 건도 안 왔으면 보정은 **불가능**이다. 모르면 가능하다고 하지 않는다.
POSE_USABLE = "POSE_USABLE"
POSE_NO_PUBLISHER = "POSE_NO_PUBLISHER"          # 발행자가 없다
POSE_NEVER_RECEIVED = "POSE_NEVER_RECEIVED"      # 발행자는 있는데 값이 안 왔다
POSE_FLOW_UNMEASURED = "POSE_FLOW_UNMEASURED"    # 호출자가 유량을 안 줬다 — fail-closed
POSE_PUBLISHERS_UNMEASURED = "POSE_PUBLISHERS_UNMEASURED"   # 발행자 수를 못 쟀다 — fail-closed


def pose_availability(pose_publishers, pose_msgs=None):
    """(state, usable). **발행자 수로 답하지 않는다.**

    🔴 인자 이름이 `robot_publishers` 였다. 2026-09-14 실측에서 그 이름이 결함을 낳았다 —
       호출자가 **로봇당 탐침 5토픽(odom·pose·image_raw·compressed·scan)의 합계**를
       넘기고 있었고, 그래서 `/robot1/pose` 발행자가 0 인데도 `/robot1/scan` 하나 때문에
       `publishers=1` 이 되어 `POSE_NO_PUBLISHER` 가 아니라 `POSE_NEVER_RECEIVED` 가 나왔다.
       바로 이 함수가 **일부러 갈라 놓은 두 상태**가 상류에서 뭉개진 것이다.
       이름을 `pose_publishers` 로 바꾼다 — 무엇을 받아야 하는지 이름이 말하게 한다.

    ⚠️ 아직 **신선도는 안 본다.** 한 번 받은 뒤 로봇이 움직이면 그 값은 낡지만,
       그걸 판정하려면 마지막 pose 이후의 **odom 이동량**이 필요하다(`/robot1/odom`
       은 15 Hz 로 계속 온다). 그 판정은 `fusion_clock` 과 같은 물리를 쓴다 —
       이동량이 격자를 넘으면 그 pose 는 더 이상 그 자리를 말하지 않는다.
       여기서는 **없는 것을 있다고 하지 않는 것**까지만 고친다.
    """
    if pose_publishers is None:
        # 못 쟀다. 모르는 것을 "없다"(0)로 적으면 이 레포가 반복해 밟은 함정이다.
        return POSE_PUBLISHERS_UNMEASURED, False
    if pose_publishers <= 0:
        return POSE_NO_PUBLISHER, False
    if pose_msgs is None:
        # 호출자가 유량을 안 넘겨줬다. 모르는 것을 통과로 치면 이 결함이 되돌아온다.
        return POSE_FLOW_UNMEASURED, False
    if pose_msgs <= 0:
        return POSE_NEVER_RECEIVED, False
    return POSE_USABLE, True


def readiness(registry, robot_publishers=0, observing=False, robot_pose_msgs=None,
              pose_publishers=None):
    """저하 안내 UI 가 읽을 판정. 문구가 아니라 코드와 숫자만 낸다.

    severity:
      safety    안전 기능이 실제로 죽음 -> 모달
      degraded  미뤄질 뿐 안전은 그대로 -> 배너
      info      지금 아무것도 그것에 기대지 않는다 -> 배지

    ⭐⭐ `robot_publishers` 와 `pose_publishers` 는 **다른 수량이다.**
      robot_publishers  로봇 토픽 아무거나의 발행자 수  -> NO_ROBOT_DATA 판정용
      pose_publishers   `/robotN/pose` 만의 발행자 수   -> pose 가용성 판정용
      2026-09-14 실측에서 이 둘이 뭉개져 있었다 — 호출자가 탐침 5토픽 합계를 하나로
      넘겼고, `/robot1/scan` 발행자 1 때문에 pose 발행자가 0 인데도 `publishers=1` 이
      되어 `POSE_NO_PUBLISHER` 가 `POSE_NEVER_RECEIVED` 로 가려졌다.
      ⚠️ `pose_publishers` 를 안 주면 **모르는 것으로 친다**(POSE_PUBLISHERS_UNMEASURED).
         옛 뜻(합계)으로 답하지 않는다 — 틀린 초록보다 모른다가 낫다.

    observing (MCV-1C, 계획서 §4.5.3):
      관측 세션이 꺼져 있으면 실물 시점 부족은 safety 가 아니라 info 다.
      아무도 그 시점에 기대고 있지 않은데 빨강이 상시로 뜨면 사람이 모달을 무시하게 되고,
      그러면 진짜 빨강도 같이 무시된다(사용자 승인 A안의 취지).
    """
    live = real_viewpoint_ids(registry)
    degradations = []

    if len(live) < MIN_REAL_VIEWPOINTS:
        degradations.append({
            "code": "VIEWPOINTS_INSUFFICIENT",
            "severity": "safety" if observing else "info",
            "have": len(live),
            "need": MIN_REAL_VIEWPOINTS,
            "liveIds": list(live),
        })

    pose_state, pose_ok = pose_availability(pose_publishers, robot_pose_msgs)

    if robot_publishers <= 0:
        # 충돌 예측 자체는 관측만으로도 된다 - 로봇 데이터는 위치 '보정'에 쓴다.
        # 그래서 안전 정지가 아니라 저하다.
        degradations.append({
            "code": "NO_ROBOT_DATA",
            "severity": "degraded",
            "publishers": robot_publishers,
        })
    elif not pose_ok:
        # ⭐ 발행자는 있는데 값이 안 온다. NO_ROBOT_DATA 와 **다른 사실**이라 코드도 다르다 —
        #    같은 코드로 묶으면 "시뮬이 안 떴다" 와 "로봇이 서 있다" 를 못 가른다.
        degradations.append({
            "code": pose_state,
            "severity": "degraded",
            "publishers": robot_publishers,
            "poseMsgs": robot_pose_msgs,
        })

    worst = "ok"
    for d in degradations:
        if SEVERITY_ORDER.index(d["severity"]) > SEVERITY_ORDER.index(worst):
            worst = d["severity"]

    return {
        "observation": {"active": bool(observing)},
        "realViewpoints": {"connected": len(live), "required": MIN_REAL_VIEWPOINTS,
                           "ids": list(live)},
        "collisionPredictionPossible": len(live) >= MIN_REAL_VIEWPOINTS,
        # ⭐ 발행자 수가 아니라 **값이 왔는가**로 판정한다 (R-1).
        "poseCorrectionPossible": len(live) >= MIN_REAL_VIEWPOINTS and pose_ok,
        "poseAvailability": {"state": pose_state, "publishers": pose_publishers,
                             "msgs": robot_pose_msgs},
        "degradations": degradations,
        "worstSeverity": worst,
    }
