# -*- coding: utf-8 -*-
"""`/api/ops/overview` · `/api/ops/diagnostics` (R-5) — 관제 화면이 "어느 구간이 끊겼나" 에 답하게 한다.

순수 로직이다(ROS 무의존). 게이트웨이 본체는 rclpy 를 module-level 에서 import 하므로 본체 안에 두면
유닛시험이 닿지 못한다(`mjpeg_serving.py` 를 가른 것과 같은 이유).

## 정직성 규칙 (지시서 R-5 ④: 데이터 없는 칸은 "미수신", 초록 기본값 금지)
- 로봇 진단(`/pinkyN/diag`)을 받은 적이 없거나 `STALE_AFTER_SEC` 넘게 끊겼으면 그 로봇의 칸은 **전부
  "미수신"** 이다. 마지막 값을 계속 보여 주지 않는다 — 끊긴 로봇이 멀쩡해 보이면 그게 가장 위험하다.
- 신선도는 **중계가 받은 시각**(중계 시계)으로 판정한다. 로봇 시계는 기기마다 어긋난다
  (09-23 실측: 같은 현장에서 −0.45 s ~ +2.71 s).
- 로봇이 보내지 않는 항목(camera·lidar·motor_watchdog·obstacle)은 처음부터 "미수신" 이다.
- 화면(`fleet_control_v2.js` `statusClass`)은 단어로 색을 고른다: OK·LIVE → 초록, WARN·WAIT → 노랑,
  FAIL·LOST → 빨강, 그 밖(미수신 포함) → 회색. 여기서 만드는 단어는 그 규칙에 맞춘다.
"""

import math

NO_DATA = "미수신"
STALE_AFTER_SEC = 5.0           # 진단은 1 Hz — 5번 연속 못 받으면 끊긴 것으로 본다(수락: 30 s 안 미수신)
ROBOT_STATE_LIVE_SEC = 2.0      # RobotState·LaneStatus 는 10 Hz
FIX_FRESH_SEC = 2.0             # PoseFix 가 이보다 오래되면 위치 보정이 멈춘 것

# 로봇이 아예 보내지 않는 칸 — 값을 지어내지 않는다
UNREPORTED = ("obstacle", "motor_watchdog", "camera", "lidar")

DRIVE_NAMES = {0: "IDLE", 1: "CRUISE", 2: "WAIT_CLEARANCE", 7: "ARRIVED", 8: "ESTOP", 9: "LINK_LOST"}


def json_safe(obj):
    """NaN·inf 를 None 으로 — 표준 JSON 이 아니라서 브라우저 JSON.parse 가 화면 전체를 멈춘다."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    return obj


def _num(v):
    return isinstance(v, (int, float)) and not (isinstance(v, float) and (math.isnan(v) or math.isinf(v)))


def diag_freshness(entry, now, stale_after=STALE_AFTER_SEC):
    """entry = (diag dict, 받은 시각) 또는 None → ('NEVER'|'STALE'|'FRESH', 나이 s 또는 None)."""
    if not entry:
        return "NEVER", None
    _, t = entry
    age = max(0.0, now - t)
    return ("FRESH" if age <= stale_after else "STALE"), age


def _pose_fuser_word(d):
    fs = d.get("fix_status")
    if fs is None:
        return NO_DATA
    recv = fs.get("received_age_s")
    if _num(recv) and recv > STALE_AFTER_SEC:
        return "LOST 상태 끊김 %.0fs" % recv
    acc, rej, age = fs.get("accepted", 0), fs.get("rejected", 0), fs.get("fix_age_s")
    if acc == 0:
        return "WAIT fix 없음 (거부 %d)" % rej
    if not _num(age):
        return "WARN fix 나이 모름 (수락 %d·거부 %d)" % (acc, rej)
    if age > FIX_FRESH_SEC:
        return "WARN fix %.1fs 전 (수락 %d·거부 %d)" % (age, acc, rej)
    return "LIVE 수락 %d·거부 %d·%.1fs" % (acc, rej, age)


def _tf_word(d):
    tf = d.get("tf") or {}
    if tf.get("fresh"):
        return "LIVE %.1fs" % tf["age_s"] if _num(tf.get("age_s")) else "LIVE"
    if _num(tf.get("age_s")):
        return "LOST map→base %.1fs" % tf["age_s"]
    return "LOST map→base 없음"


def _nav2_word(d):
    n = d.get("nav2") or {}
    total, active = n.get("total", 0), n.get("active", 0)
    if not total:
        return NO_DATA
    if active == total:
        return "OK %d/%d active" % (active, total)
    if active == 0:
        return "FAIL 0/%d — 응답·활성 없음" % total
    return "WARN %d/%d — %s" % (active, total, ",".join(n.get("not_active", [])))


def _node_word(d, node):
    st = (d.get("nav2") or {}).get("nodes", {}).get(node)
    if st is None:
        return NO_DATA
    return "OK active" if st == "active" else ("FAIL unknown" if st == "unknown" else "WARN " + st)


def _gate_word(d):
    g = d.get("gate")
    if g is None:
        return NO_DATA
    recv = g.get("received_age_s")
    if _num(recv) and recv > STALE_AFTER_SEC:
        return "LOST 게이트 상태 끊김 %.0fs" % recv
    src = g.get("source", "?")
    return ("OK source=%s" % src) if src in ("MISSION", "IDLE") else ("WARN source=%s" % src)


def _estop_word(d):
    e = d.get("estop")
    if e is None:
        return NO_DATA
    return "WARN E-STOP 걸림" if e else "OK 해제"


def _robot_state_word(fleet_robot):
    heard = (fleet_robot or {}).get("last_heard_sec")
    if not _num(heard):
        return NO_DATA
    return ("LIVE %.1fs" % heard) if heard <= ROBOT_STATE_LIVE_SEC else ("LOST %.0fs" % heard)


def _battery_word(fleet_robot):
    st = (fleet_robot or {}).get("state") or {}
    pct = st.get("battery_percent")
    return ("%.0f%%" % pct) if _num(pct) else NO_DATA


def _overall(words):
    """가장 나쁜 칸이 전체를 정한다. 🔴 dict 를 그대로 돌면 **키**를 본다 — 첫 판이 그래서 늘 OK 였다."""
    bad = [k for k, w in words.items() if w.startswith(("FAIL", "LOST"))]
    warn = [k for k, w in words.items() if w.startswith(("WARN", "WAIT"))]
    if bad:
        return "FAIL " + ", ".join(bad)
    if warn:
        return "WARN " + ", ".join(warn)
    missing = [k for k, w in words.items() if w == NO_DATA]
    if len(missing) == len(words):
        return NO_DATA
    # 잰 칸은 전부 정상. 못 잰 칸이 있으면 이름으로 밝힌다 — "OK" 한 단어가 빈칸을 덮지 않게
    return "OK" if not missing else "OK (미수신: %s)" % ",".join(missing)


def robot_diagnostics(name, entry, fleet_robot, now, stale_after=STALE_AFTER_SEC):
    fresh, age = diag_freshness(entry, now, stale_after)
    out = {"diag": fresh, "diag_age_s": age}
    base = {"robot_state": _robot_state_word(fleet_robot), "battery": _battery_word(fleet_robot)}
    if fresh != "FRESH":
        # 끊겼거나 받은 적 없다 — 마지막 값을 보여 주지 않는다
        why = NO_DATA if fresh == "NEVER" else "%s (끊김 %.0fs)" % (NO_DATA, age)
        words = {k: why for k in ("pose_fuser", "tf", "nav2", "planner", "controller", "drive_gate", "estop")}
        words.update({k: NO_DATA for k in UNREPORTED})
        words.update(base)
        out.update(words)
        out["overall"] = why
        out["pose_source"] = None
        out["detail"] = None
        return out
    d = entry[0]
    words = {
        "pose_fuser": _pose_fuser_word(d),
        "tf": _tf_word(d),
        "nav2": _nav2_word(d),
        "planner": _node_word(d, "planner_server"),
        "controller": _node_word(d, "controller_server"),
        "drive_gate": _gate_word(d),
        "estop": _estop_word(d),
    }
    words.update({k: NO_DATA for k in UNREPORTED})
    words.update(base)
    out.update(words)
    out["overall"] = _overall({k: v for k, v in words.items() if k not in UNREPORTED})
    out["pose_source"] = d.get("pose_source")
    out["detail"] = d
    return out


def _fleet_block(fleet_robot):
    if not fleet_robot:
        return None
    return {k: fleet_robot.get(k) for k in
            ("drive_mode", "start", "held", "last_heard_sec", "clear_until_idx", "arrival_status")}


def diagnostics(robot_names, diag_by_robot, fleet_status, now, stale_after=STALE_AFTER_SEC):
    fleet_robots = (fleet_status or {}).get("robots") or {}
    robots = {}
    for name in robot_names:
        r = robot_diagnostics(name, diag_by_robot.get(name), fleet_robots.get(name), now, stale_after)
        r["fleet"] = _fleet_block(fleet_robots.get(name))
        robots[name] = r
    return {"generated_at": now, "stale_after_s": stale_after,
            "mission_state": (fleet_status or {}).get("mission_state"), "robots": robots}


def overview(robot_names, diag_by_robot, fleet_status, now, stale_after=STALE_AFTER_SEC):
    diag = diagnostics(robot_names, diag_by_robot, fleet_status, now, stale_after)
    robots = {}
    for name, r in diag["robots"].items():
        detail = r.get("detail") or {}
        agent = detail.get("agent") or {}
        fleet = r.get("fleet") or {}
        robots[name] = {
            "overall": r["overall"],
            "link": r["robot_state"],
            "pose_source": r["pose_source"],
            "nav2": r["nav2"],
            "estop": r["estop"],
            "drive_gate": r["drive_gate"],
            "pose_fuser": r["pose_fuser"],
            "drive_state": DRIVE_NAMES.get(agent.get("drive_state")) if agent else None,
            "held": fleet.get("held"),
            "start": fleet.get("start"),
        }
    warnings = sorted("%s: %s" % (n, r["overall"]) for n, r in robots.items()
                      if r["overall"] != "OK")
    return {"generated_at": now, "mission_state": diag["mission_state"],
            "robots": robots, "warnings": warnings}
