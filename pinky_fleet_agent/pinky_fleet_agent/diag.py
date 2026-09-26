# -*- coding: utf-8 -*-
"""로봇 진단 업링크 `/pinkyN/diag` (R-5) — JSON `std_msgs/String`, 1 Hz. **이 파일이 스키마 정본이다.**

ROS 에 의존하지 않는다. `hybrid_agent_node` 가 재료를 모아 `build()` 에 넘기고 결과를 JSON 으로 낸다.

## 왜 별도 토픽인가
관제 화면이 "어느 구간이 끊겼나" 에 답하려면 로봇 쪽 신호가 올라와야 한다. 그런데 `RobotState.msg` 는
팀11 과 **바이트 동일**해야 해서 늘릴 수 없다. 그래서 우리 소유의 JSON 문자열 토픽을 따로 둔다
(새 msg 타입을 만들면 브리지가 그 타입을 찾아야 한다 — 감사 §2 의 PoseFix 가 그래서 막혔다).

## 정직성 규칙
- 못 잰 값은 `null` 이다. 0·false 로 채우지 않는다.
- 수명주기 서비스가 응답하지 않은 노드는 `inactive` 가 아니라 **`unknown`** 이다.
- 상태 토픽은 **받은 지 몇 초인지**(`received_age_s`)를 같이 싣는다 — 노드가 죽으면 값이 아니라 나이가 말한다.

## 스키마 (`pinky_diag/1`)

    {
      "schema": "pinky_diag/1", "robot": "pinky1", "stamp": <로봇 벽시계 s>,
      "use_sim_time": bool,
      "pose_source": "AMCL" | "PoseFuser" | null,
      "tf": {"fresh": bool, "age_s": float|null},                    # map→base_footprint
      "fix_status": {"accepted": int, "rejected": int, "fix_age_s": float|null,
                     "received_age_s": float, "raw": str} | null,
      "nav2": {"active": int, "total": int, "nodes": {name: state}, "not_active": [name...]},
      "estop": bool | null,                                            # 마지막으로 본 /estop
      "gate": {"source": str, "vx": float, "wz": float, "received_age_s": float} | null,
      "agent": {...}                                                   # route_chain · 단일 목표 상태
    }
"""
import math
import re

SCHEMA = "pinky_diag/1"

# lifecycle_msgs/msg/State.PRIMARY_STATE_* 값
LIFECYCLE_LABELS = {0: "unknown", 1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized"}

# 위치를 PoseFuser 가 낼 때의 8노드. AMCL 이면 amcl 을 더해 9노드 — hybrid_agent_node 파라미터로 바꾼다.
# 출처: launch/hybrid_robot.launch.xml (lifecycle_nodes_nav) + map_server.
DEFAULT_NAV2_NODES = (
    "map_server", "controller_server", "smoother_server", "planner_server",
    "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
)

_FIX_RE = re.compile(r"accepted\s+(\d+)\s+rejected\s+(\d+)\s+age\s+(inf|[0-9.]+)s?")
_GATE_RE = re.compile(r"source=(\w+)\s+vx=(-?[0-9.]+)\s+wz=(-?[0-9.]+)")


def parse_fix_status(text):
    """pose_fuser_node 의 `accepted N rejected M age X.Xs` (또는 `age inf`). 못 읽으면 None."""
    if not isinstance(text, str):
        return None
    m = _FIX_RE.search(text)
    if not m:
        return None
    age = None if m.group(3) == "inf" else float(m.group(3))
    return {"accepted": int(m.group(1)), "rejected": int(m.group(2)), "fix_age_s": age, "raw": text}


def parse_gate_status(text):
    """drive_command_gate 의 `source=MISSION vx=0.10 wz=0.00`. 못 읽으면 None."""
    if not isinstance(text, str):
        return None
    m = _GATE_RE.search(text)
    if not m:
        return None
    return {"source": m.group(1), "vx": float(m.group(2)), "wz": float(m.group(3))}


def nav2_summary(states):
    """{노드: 상태 라벨 또는 None}. None(응답 없음)은 `unknown` 이다 — inactive 가 아니다."""
    nodes = {name: (label if label else "unknown") for name, label in states.items()}
    not_active = sorted(n for n, s in nodes.items() if s != "active")
    return {"active": len(nodes) - len(not_active), "total": len(nodes),
            "nodes": nodes, "not_active": not_active}


def pose_source(nav2, fix):
    """실물은 AMCL, 가상 팜은 PoseFuser. 둘 다 근거가 없으면 None."""
    if nav2 and nav2.get("nodes", {}).get("amcl") == "active":
        return "AMCL"
    if fix is not None:
        return "PoseFuser"
    return None


def _age(now, t):
    return None if t is None else max(0.0, float(now) - float(t))


def build(robot, now, use_sim_time, nav2_states, fix_text=None, fix_time=None,
          gate_text=None, gate_time=None, estop=None, tf_age=None, tf_timeout=2.0, agent=None):
    """진단 한 건. `*_time`·`now` 는 같은 시계(에이전트 시계)여야 한다."""
    fix = parse_fix_status(fix_text) if fix_text is not None else None
    if fix is not None:
        fix["received_age_s"] = _age(now, fix_time)
    gate = parse_gate_status(gate_text) if gate_text is not None else None
    if gate is not None:
        gate["received_age_s"] = _age(now, gate_time)
    nav2 = nav2_summary(nav2_states)
    tf_fresh = tf_age is not None and not math.isinf(tf_age) and tf_age <= tf_timeout
    return {
        "schema": SCHEMA,
        "robot": robot,
        "stamp": float(now),
        "use_sim_time": bool(use_sim_time),
        "pose_source": pose_source(nav2, fix),
        "tf": {"fresh": tf_fresh, "age_s": None if tf_age is None or math.isinf(tf_age) else float(tf_age)},
        "fix_status": fix,
        "nav2": nav2,
        "estop": None if estop is None else bool(estop),
        "gate": gate,
        "agent": agent or {},
    }
