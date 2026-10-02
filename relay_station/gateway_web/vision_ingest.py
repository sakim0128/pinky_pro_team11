# -*- coding: utf-8 -*-
"""연산 노드(태블릿)가 낸 좌표를 중계가 받는 자리 — **순수 로직**.

⭐ ROS 도 HTTP 도 import 하지 않는다. 기기 없이 시험할 수 있어야 하기 때문이다
   (`calibration.py`·`fusion_clock.py` 와 같은 이유의 분리).

🔴 이 모듈의 존재 이유는 좌표를 예쁘게 만드는 것이 아니라 **연산 노드가 조용해졌을 때
   낡은 좌표를 계속 보여 주지 않는 것**이다. 2026-09-18 감사에서 게이트웨이가 지어내던
   것이 넷 나왔다(정지 API 무조건 성공 · 화면이 응답 미판독 · 로봇 좌표 하드코딩 ·
   맵 못 읽으면 합성 격자 무표시). 같은 병을 새 경로에 다시 만들지 않는다.

## 시각은 **받은 쪽 시계**로 잰다

신선도 판정에 연산 노드가 보낸 `computedAtMs` 를 쓰지 않는다. 그 기기의 시계를 믿을
근거가 없기 때문이다 — 2026-09-19 01:34 실측에서 현장 폰 하나가 `captureClock` 이
**82 시간** 어긋난 채 `clockSuspect: false` 를 냈다. 남의 시계를 신선도의 기준으로
삼으면 그 기기가 틀린 만큼 우리가 속는다.

그래서 신선도는 **중계가 받은 시각**으로 재고, `computedAtMs` 는 버리지 않고
`clockOffsetMs = receivedAtMs - computedAtMs` 로 **따로 보관**한다. 그 값이 크면
연산 노드의 시계가 틀린 것이고, 그건 좌표를 버릴 이유가 아니라 **알려야 할 사실**이다.

## STALE_AFTER_MS 는 왜 500 인가 (지어낸 값이 아니다)

- 연산 노드의 갱신율 상한은 **가장 느린 카메라**가 정한다. **노트10**을 2026-09-19
  01:34 에 tailnet 으로 재니 8.325 fps = 프레임 간격 **120 ms** 였다.
  (기기 주소는 여기 안 적는다 — 이동 기기라 사이트마다 바뀐다. 후보는
  `configs/video_sources.json` 이 가지고, `tests/test_no_baked_addresses.py` 가 강제한다.)
- 네 번 연속 결측 = 480 ms. 여기서 **500 ms** 를 잡는다 — 한두 프레임 빠졌다고
  깜빡이지 않으면서, 네 번 이상 빠지면 말을 바꾼다.
- 주행 속도 0.20 m/s(팀원 A 실제값) 에서 500 ms 는 **10 cm** 이동이다. 평면 격자
  2.5 cm 의 **네 칸**이다. 그보다 낡은 값은 "지금 거기 있다" 가 아니다.

## 🔴 그런데 그 8.325 는 **밝은 조건의 값**이다 — 조명이 지배 변수다

**같은 노트10**을 중계가 2026-09-17 21:53(실내 야간, 현장 LAN)에 재니 **1.741 fps**
= 간격 **574 ms** 였다(`configs/video_sources.json` 의 `phone._why`, 끝단 8초 읽기
1.88 fps 로 교차 확인). **28시간 사이 5배 차이다.**

⭐ 원인이 특정된다. 두 관측의 `encodeMs` 가 거의 같고(30.2 vs 33.2) 해상도도 둘 다
   1088x1088 이다 — **인코딩이 병목이 아니라 센서가 느려진 것**이다. AE 가 어두운 곳에서
   노출을 늘리는 것이 유일하게 설명되는 기전이고, 앱은 `CONTROL_AE_TARGET_FPS_RANGE` 를
   **설정하는 호출부가 0곳**이라 하한이 없다.

## 그 조명에서 이 문턱이 무엇을 하는가 (정확히)

나이는 **마지막 수신으로부터** 재므로 톱니(0 → 간격)다. 그래서:

  fps 8.325 (간격 120 ms) → FRESH 100 %  · 연속 결측 4.2 프레임 견딤
  fps 1.741 (간격 574 ms) → FRESH  87 %  · 연속 결측 0.9 프레임 견딤

🔴 **"항상 STALE" 이 아니다.** 어두워도 87%는 FRESH 다. 깨지는 것은 *"연속 네 프레임
   결측을 견딘다"* 는 성질이고(500 >= 4x574 이 거짓), 한 프레임만 빠져도 낡는다.
   (2026-09-19 에 내가 "항상 STALE" 이라고 두 곳에 잘못 보고했다. 톱니를 안 봤다.)

⚠️ **그래서 처방은 문턱을 올리는 것이 아니다.** 574 ms 간격을 네 프레임 견디려면
   문턱이 2,296 ms 여야 하는데, 그러면 0.20 m/s 에서 **46 cm(격자 18칸)** 오차를
   허용하게 된다. 안전 성질을 느슨하게 하는 것이 답일 수 없다 — **fps 를 올린다**
   (= 조명을 개선한다). 문턱은 주행에서 유도된 값이므로 그대로 둔다.

⚠️ 갱신율이나 주행 속도가 바뀌면 이 값도 다시 유도한다. 상수로 박아 두고 잊지 않는다.
⚠️ **실측값에는 기기 이름과 조명 조건을 같이 적는다.** 값만 적었더니 같은 기기의 5배
   차이가 "다른 기기였나" 논쟁으로 번졌다.
"""

import math
import os

# ---- 상태 -------------------------------------------------------------------

FRESH = "VISION_FRESH"      # 최근에 받았다 — 좌표를 써도 된다
STALE = "VISION_STALE"      # 받은 적은 있는데 낡았다 — **좌표를 쓰면 안 된다**
NEVER = "VISION_NEVER"      # 한 번도 못 받았다 — "원점에 있다" 와 구분된다

#: 이 시간(ms) 넘게 새 좌표가 안 오면 STALE. 유도는 모듈 독스트링 참조.
STALE_AFTER_MS = 500

#: 연산 노드 시계가 이만큼 어긋나면 값은 받되 **경고를 같이 낸다**.
#: 500 ms = STALE 문턱과 같은 크기 — 그보다 큰 어긋남은 신선도 판단을 흔든다.
CLOCK_WARN_MS = 500

# ---- 거절 사유 ---------------------------------------------------------------

REJECT_NOT_OBJECT = "NOT_AN_OBJECT"
REJECT_MISSING = "MISSING_FIELD"
REJECT_TYPE = "BAD_TYPE"
REJECT_NOT_FINITE = "NOT_FINITE"
REJECT_ROBOT_ID = "BAD_ROBOT_ID"
REJECT_INVALID_FRAME = "INVALID_FRAME_ID"
REJECT_INVALID_ROBOT = "INVALID_ROBOT_NAME"
REJECT_RANGE = "OUT_OF_RANGE"
REJECT_INVALID_EVENT = "INVALID_EVENT_TYPE"

_REQUIRED_NUMBERS = ("x", "y", "yaw")


def _is_number(v):
    # bool 은 int 의 하위형이라 먼저 걷어낸다 — True 가 1 로 통과하면 안 된다.
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_strict_int(v):
    # bool 및 float 형태(1.0, 1.5)를 엄격히 제외한 순수 정수 검증
    return isinstance(v, int) and not isinstance(v, bool)


def _finite(v):
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def validate_pose_fix(payload, allowed_robots=("pinky1", "pinky2")):
    """태블릿/외부 비전이 보낸 canonical PoseFix 검사 (Track R: R-D1).
    
    규약 (Strict Validation):
      - header 필수, header.frame_id == map
      - header.stamp.sec 및 header.stamp.nanosec 필수 strict integer
        (sec >= 0, 0 <= nanosec <= 999_999_999)
      - robot_name in allowed_robots ({pinky1, pinky2})
      - finite x/y/yaw
      - seq, marker_id, n_markers: strict integer (음수/float 절삭 금지)
      - marker_range, reproj_error, pipeline_latency: non-negative finite float
    """
    if not isinstance(payload, dict):
        return False, REJECT_NOT_OBJECT, None

    rname = payload.get("robot_name")
    if not isinstance(rname, str) or not rname.strip():
        return False, REJECT_INVALID_ROBOT, None
    rname = rname.strip()
    if allowed_robots is not None and rname not in allowed_robots:
        return False, REJECT_INVALID_ROBOT, None

    header = payload.get("header")
    if not isinstance(header, dict):
        return False, "%s:header" % REJECT_MISSING, None

    frame_id = header.get("frame_id")
    if frame_id != "map":
        return False, REJECT_INVALID_FRAME, None

    stamp = header.get("stamp")
    if not isinstance(stamp, dict):
        return False, "%s:header.stamp" % REJECT_MISSING, None

    if "sec" not in stamp or "nanosec" not in stamp:
        return False, "%s:header.stamp" % REJECT_MISSING, None

    stamp_sec = stamp["sec"]
    if not _is_strict_int(stamp_sec):
        return False, "%s:header.stamp.sec" % REJECT_TYPE, None
    if stamp_sec < 0:
        return False, "%s:header.stamp.sec" % REJECT_RANGE, None

    stamp_nanosec = stamp["nanosec"]
    if not _is_strict_int(stamp_nanosec):
        return False, "%s:header.stamp.nanosec" % REJECT_TYPE, None
    if not (0 <= stamp_nanosec <= 999_999_999):
        return False, "%s:header.stamp.nanosec" % REJECT_RANGE, None

    for key in _REQUIRED_NUMBERS:
        if key not in payload:
            return False, "%s:%s" % (REJECT_MISSING, key), None
        val = payload[key]
        if not _is_number(val):
            return False, "%s:%s" % (REJECT_TYPE, key), None
        if not _finite(val):
            return False, "%s:%s" % (REJECT_NOT_FINITE, key), None

    seq = payload.get("seq", 0)
    if not _is_strict_int(seq):
        return False, "%s:seq" % REJECT_TYPE, None
    if seq < 0:
        return False, "%s:seq" % REJECT_RANGE, None

    stamp_is_robot_clock = payload.get("stamp_is_robot_clock", False)
    if not isinstance(stamp_is_robot_clock, bool):
        return False, "%s:stamp_is_robot_clock" % REJECT_TYPE, None

    marker_id = payload.get("marker_id", 0)
    if not _is_strict_int(marker_id):
        return False, "%s:marker_id" % REJECT_TYPE, None

    marker_range = payload.get("marker_range", 0.0)
    if not _is_number(marker_range) or not _finite(marker_range):
        return False, "%s:marker_range" % REJECT_TYPE, None
    if float(marker_range) < 0.0:
        return False, "%s:marker_range" % REJECT_RANGE, None

    reproj_error = payload.get("reproj_error", 0.0)
    if not _is_number(reproj_error) or not _finite(reproj_error):
        return False, "%s:reproj_error" % REJECT_TYPE, None
    if float(reproj_error) < 0.0:
        return False, "%s:reproj_error" % REJECT_RANGE, None

    n_markers = payload.get("n_markers", 0)
    if not _is_strict_int(n_markers):
        return False, "%s:n_markers" % REJECT_TYPE, None
    if n_markers < 0:
        return False, "%s:n_markers" % REJECT_RANGE, None

    pipeline_latency = payload.get("pipeline_latency", 0.0)
    if not _is_number(pipeline_latency) or not _finite(pipeline_latency):
        return False, "%s:pipeline_latency" % REJECT_TYPE, None
    if float(pipeline_latency) < 0.0:
        return False, "%s:pipeline_latency" % REJECT_RANGE, None

    out = {
        "robot_name": rname,
        "frame_id": frame_id,
        "stamp_sec": stamp_sec,
        "stamp_nanosec": stamp_nanosec,
        "stamp_is_robot_clock": stamp_is_robot_clock,
        "seq": seq,
        "x": float(payload["x"]),
        "y": float(payload["y"]),
        "yaw": float(payload["yaw"]),
        "marker_id": marker_id,
        "marker_range": float(marker_range),
        "reproj_error": float(reproj_error),
        "n_markers": n_markers,
        "pipeline_latency": float(pipeline_latency),
    }
    return True, None, out


# ── 태블릿 좌표 → 팀11 로봇 위치 토픽 (2026-09-29) ─────────────────────────────
# 팀11 로봇(lane_robot.launch.xml)의 pose_fuser_node 는 PoseFix 를 받지 않고 /<robot>/overhead_pose
# (geometry_msgs/PoseStamped) 만 받는다. 게이트웨이는 검증을 통과한 PoseFix 를 그 이름으로도 낸다.
# 같은 로봇에 팀11 상부 추적기(overhead_tracker_node)를 같이 띄울 때만 끈다 — 한 토픽에 두 출처가 섞이면
# pose_fuser 의 점프 게이트가 번갈아 거부해 위치가 멈춘다.
POSE_FIX_TO_OVERHEAD_ENV = "RELAY_POSE_FIX_TO_OVERHEAD"


def pose_fix_to_overhead_enabled(environ=None):
    """기본 켜짐. 0 · false · no · off 면 끈다."""
    env = os.environ if environ is None else environ
    v = str(env.get(POSE_FIX_TO_OVERHEAD_ENV, "1")).strip().lower()
    return v not in ("0", "false", "no", "off")


def yaw_to_quat_zw(yaw):
    """평면 yaw(rad) → 쿼터니언 (z, w). x = y = 0."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def validate_zone_event(payload, allowed_robots=("pinky1", "pinky2")):
    """태블릿 비전 구역 이벤트 canonical payload 검사.
    
    규약:
      - robot_name in allowed_robots (또는 legacy 'robot')
      - event_type in {'ENTER', 'PRESENT', 'EXIT'} (또는 legacy 'event')
      - zone_id: 비어있지 않은 문자열
      - confidence: 0.0 ~ 1.0 유효 범위
      - sequence: strict integer >= 0 (있을 경우)
    """
    if not isinstance(payload, dict):
        return False, REJECT_NOT_OBJECT, None

    rname = payload.get("robot_name") or payload.get("robot")
    if not isinstance(rname, str) or not rname.strip():
        return False, REJECT_INVALID_ROBOT, None
    rname = rname.strip()
    if allowed_robots is not None and rname not in allowed_robots:
        return False, REJECT_INVALID_ROBOT, None

    camera_id = payload.get("camera_id", "")
    if not isinstance(camera_id, str):
        return False, "%s:camera_id" % REJECT_TYPE, None
    camera_id = camera_id.strip()

    zone_id = payload.get("zone_id", "")
    if not isinstance(zone_id, str) or not zone_id.strip():
        return False, "%s:zone_id" % REJECT_MISSING, None
    zone_id = zone_id.strip()

    event_type = payload.get("event_type") or payload.get("event")
    if not isinstance(event_type, str) or event_type.strip().upper() not in ("ENTER", "PRESENT", "EXIT"):
        return False, REJECT_INVALID_EVENT, None
    event_type = event_type.strip().upper()

    confidence = payload.get("confidence", 1.0)
    if not _is_number(confidence) or not _finite(confidence):
        return False, "%s:confidence" % REJECT_TYPE, None
    confidence = float(confidence)
    if not (0.0 <= confidence <= 1.0):
        return False, "%s:confidence" % REJECT_RANGE, None

    ts = payload.get("timestamp")
    if ts is not None:
        if not _is_number(ts) or not _finite(ts):
            return False, "%s:timestamp" % REJECT_TYPE, None
        if float(ts) < 0:
            return False, "%s:timestamp" % REJECT_RANGE, None
        source_timestamp = float(ts)
    else:
        source_timestamp = 0.0

    raw_seq = payload.get("sequence")
    if raw_seq is not None:
        if not _is_strict_int(raw_seq):
            return False, "%s:sequence" % REJECT_TYPE, None
        if raw_seq < 0:
            return False, "%s:sequence" % REJECT_RANGE, None
        seq = int(raw_seq)
    else:
        seq = None

    session_id = payload.get("session_id") or payload.get("boot_id")
    if session_id is not None:
        session_id = str(session_id).strip()

    reset = bool(payload.get("reset", False))

    norm = {
        "robot_name": rname,
        "camera_id": camera_id,
        "zone_id": zone_id,
        "event_type": event_type,
        "confidence": round(confidence, 3),
        "timestamp": round(source_timestamp, 4),
        "sequence": seq,
        "session_id": session_id,
        "reset": reset,
    }
    return True, None, norm
