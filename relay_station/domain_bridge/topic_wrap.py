# -*- coding: utf-8 -*-
"""🎁 서비스·액션을 **토픽으로 감싼다** — 도메인 브리지가 나를 수 있게.

## 왜 필요한가

도메인 브리지는 **토픽만** 나른다. 설계 문서가 이유를 직접 적었다:

> "타입 이름과 토픽 이름만으로는 서비스나 클라이언트를 만들 수 없고, 따라서 설정
>  파일로 서비스를 동적으로 브리지할 수 없다."

브리지는 C++ 이고 rclcpp 에는 **제네릭 서비스 API 가 없다.** 토픽은
`create_generic_publisher` 로 타입 **이름만** 알아도 통로를 만들 수 있는데,
서비스는 컴파일 시점에 타입이 필요하다.

⭐ **파이썬은 된다.** `rosidl_runtime_py` 가 런타임에 타입을 찾아 준다:

    get_service("std_srvs/srv/SetBool")   -> 클래스
    set_message_fields(req, {...})        -> dict 로 채우기
    message_to_ordereddict(resp)          -> dict 로 되돌리기

그래서 **로봇 쪽에 이 어댑터를 두면** 설정 한 줄로 아무 서비스나 토픽으로 감쌀 수
있다. 브리지는 토픽만 나르면 되고, 온보드 서비스는 하나도 안 고친다.

## 전화를 편지로 바꿀 때 생기는 진짜 문제

전화(서비스)는 한 번 걸면 한 번이다. 편지(토픽)는 **두 번 올 수 있다.**
LED 는 두 번 켜도 그만이지만, 아래 것들은 아니다:

    문을 연다 · 팔을 움직인다 · 계수기를 올린다

그래서 이 어댑터는 **요청 id 로 중복을 막는다.** 같은 id 가 다시 오면 서비스를
다시 부르지 않고 **그때 그 응답을 다시 낸다.** id 가 없는 요청은 아예 거절한다 —
짝지을 수 없는 요청은 응답을 돌려줄 곳이 없다.

그리고 **응답은 무슨 일이 있어도 낸다.** 실패도 응답이다. 조용히 사라지면
요청한 쪽이 영원히 기다리고, 그건 화면에서 "생각 중" 과 구분이 안 된다.

## ⚠️ 잃는 것 — 감싸면 공짜가 아니다

    · 타입 검사가 **실행 시점**으로 밀린다 (JSON 을 받아 채우다 틀리면 그때 안다)
    · 왕복 시간이 늘어난다 (브리지 홉 + 직렬화)
    · 액션의 **취소가 늦으면 로봇은 그동안 계속 간다** — 주행에서는 안전 문제다
    · 누구나 그 토픽에 쏠 수 있다. 서비스처럼 호출자를 구분하지 않는다

## 쓰는 법

    # 로봇 위에서 (도메인 10)
    python3 topic_wrap.py --config wrap_robot1.yaml

    # 설정
    robot: robot1
    services:
      led:
        type: pinky_interfaces/srv/SetLed
        service: set_led
    actions:
      nav:
        type: nav2_msgs/action/NavigateToPose
        action: navigate_to_pose

    # 그러면 이런 토픽이 생긴다 (브리지가 이것을 나른다)
    robot1/svc/led/request   <- 관제/팀원이 보낸다
    robot1/svc/led/result    -> 로봇이 답한다
    robot1/act/nav/{goal,cancel,feedback,result}

⚠️ 액션 쪽은 **실기 왕복 검증을 못 했다.** 이 레포의 시험 환경에 액션 타입이 하나도
   설치돼 있지 않다(확인함). 서비스 왕복은 실측했다.
"""
import json
import time
import uuid

# ── 사유 코드 — 화면·로그가 같은 말을 쓰도록 여기서 고정한다 ──────────────────────
WHY_NO_ID = "NO_ID"                  # 요청에 id 가 없다 — 짝지을 수 없다
WHY_BAD_JSON = "BAD_JSON"
WHY_BAD_ARGS = "BAD_ARGS"            # 필드가 그 타입과 안 맞는다
WHY_ABSENT = "SERVICE_ABSENT"        # 서버가 없다
WHY_TIMEOUT = "TIMEOUT"
WHY_BUSY = "BUSY"                    # 동시 요청 상한
WHY_FAILED = "CALL_FAILED"
WHY_CANCELLED = "CANCELLED"
WHY_REJECTED = "GOAL_REJECTED"

DEFAULT_DEADLINE_MS = 3000.0
MAX_INFLIGHT = 8
DEDUPE_TTL_S = 60.0


class EnvelopeError(Exception):
    """요청 봉투가 잘못됐다. `why` 는 위 상수 중 하나다."""

    def __init__(self, why, detail=""):
        super().__init__("%s: %s" % (why, detail))
        self.why = why
        self.detail = detail


def new_id():
    return uuid.uuid4().hex[:16]


def parse_request(raw):
    """문자열 → (id, args, deadline_ms). 틀리면 `EnvelopeError`.

    ⭐ id 를 **필수**로 둔다. 없으면 응답을 어디에 붙일지 알 수 없고, 중복도 못 막는다.
       "편의상 없으면 만들어 준다" 로 하면 재전송이 새 요청이 되어 부작용이 두 번 난다.
    """
    try:
        env = json.loads(raw)
    except (ValueError, TypeError) as exc:
        raise EnvelopeError(WHY_BAD_JSON, str(exc))
    if not isinstance(env, dict):
        raise EnvelopeError(WHY_BAD_JSON, "봉투가 객체가 아니다")
    rid = env.get("id")
    if not isinstance(rid, str) or not rid.strip():
        raise EnvelopeError(WHY_NO_ID, "id 는 비어 있지 않은 문자열이어야 한다")
    args = env.get("args", {})
    if not isinstance(args, dict):
        raise EnvelopeError(WHY_BAD_ARGS, "args 는 객체여야 한다")
    try:
        deadline = float(env.get("deadline_ms", DEFAULT_DEADLINE_MS))
    except (TypeError, ValueError):
        raise EnvelopeError(WHY_BAD_ARGS, "deadline_ms 가 숫자가 아니다")
    if not (0 < deadline <= 120000):
        raise EnvelopeError(WHY_BAD_ARGS, "deadline_ms 범위 밖: %r" % deadline)
    return rid.strip(), args, deadline


def ok_result(rid, data, took_ms=None):
    env = {"id": rid, "ok": True, "data": data}
    if took_ms is not None:
        env["tookMs"] = round(took_ms, 1)
    return json.dumps(env, ensure_ascii=False, default=str)


def fail_result(rid, why, detail="", took_ms=None):
    """⭐ 실패도 **반드시 낸다.** 조용히 사라지면 요청자가 영원히 기다린다."""
    env = {"id": rid or "", "ok": False, "why": why, "detail": str(detail)[:400]}
    if took_ms is not None:
        env["tookMs"] = round(took_ms, 1)
    return json.dumps(env, ensure_ascii=False)


class ResultCache:
    """같은 id 가 다시 오면 **다시 부르지 않고** 그때 응답을 다시 낸다.

    ⭐ 토픽은 재전송이 있다. 이것이 없으면 "문을 연다" 가 두 번 실행된다.
    """

    def __init__(self, ttl_s=DEDUPE_TTL_S, clock=time.monotonic):
        self.ttl = ttl_s
        self._clock = clock
        self._done = {}      # id -> (when, payload)
        self._inflight = set()

    def sweep(self):
        now = self._clock()
        for k in [k for k, (t, _) in self._done.items() if now - t > self.ttl]:
            del self._done[k]

    def seen(self, rid):
        """이미 답한 요청이면 그 응답, 아니면 None."""
        self.sweep()
        got = self._done.get(rid)
        return got[1] if got else None

    def is_inflight(self, rid):
        return rid in self._inflight

    def begin(self, rid):
        """새 요청으로 받아들인다. 상한을 넘으면 `EnvelopeError(BUSY)`."""
        if len(self._inflight) >= MAX_INFLIGHT:
            raise EnvelopeError(WHY_BUSY, "동시 요청 %d 개 초과" % MAX_INFLIGHT)
        self._inflight.add(rid)

    def finish(self, rid, payload):
        self._inflight.discard(rid)
        self._done[rid] = (self._clock(), payload)

    @property
    def inflight(self):
        return len(self._inflight)


def topic_names(robot, kind, name, via_team=False):
    """감싼 것이 어떤 토픽 이름을 갖는지 — **한 곳에서만 정한다.**

    ⭐ 브리지 설정(`generate_configs.sh`)과 이 어댑터가 같은 규칙을 써야 한다.
       두 곳에 따로 적으면 조용히 어긋나고, 증상은 "브리지가 안 나른다" 로 보인다.

    `via_team=True` 면 `teleop/` 밑으로 낸다. 벌 2(팀원 주행 벌)는 **teleop 접두어만**
    나르기로 돼 있고 그 규칙을 시험이 고정하고 있다. 팀원에게 이 기능을 열려면
    규칙을 깨는 대신 **그 밑으로 들어간다** — 규칙이 하나로 유지된다.

    ⚠️ 팀원에게 여는 것은 **기본이 아니다.** 설정에 `team: true` 를 적는 행위가
       곧 "이건 팀원도 불러도 된다" 는 결정이다.
    """
    assert kind in ("svc", "act"), kind
    lead = "%s/teleop" % robot if via_team else robot
    base = "%s/%s/%s" % (lead, kind, name)
    if kind == "svc":
        return {"request": base + "/request", "result": base + "/result"}
    return {"goal": base + "/goal", "cancel": base + "/cancel",
            "feedback": base + "/feedback", "result": base + "/result"}


# ══════════════════════════════════════════════════════════════════════════
#  여기부터 ROS 가 필요하다 — 위쪽 봉투 로직은 ROS 없이도 시험할 수 있다.
# ══════════════════════════════════════════════════════════════════════════

def _ros():
    """ROS 를 **늦게** 불러온다 — 봉투 로직 시험이 ROS 없는 환경에서도 돌게."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rosidl_runtime_py import set_message_fields
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_service, get_interface
    return (rclpy, Node, String, set_message_fields,
            message_to_ordereddict, get_service, get_interface)


class WrappedService(object):
    """서비스 하나를 요청/응답 **토픽 쌍**으로 감싼다.

    로봇 안(그 서비스가 사는 도메인)에서 돈다. 브리지는 이 토픽만 나르면 된다.
    """

    def __init__(self, node, robot, name, type_str, service_name,
                 logger=None, via_team=False):
        (_r, _N, String, self._fill, self._to_dict,
         get_service, _gi) = _ros()
        self.node = node
        self.log = logger or node.get_logger()
        self.name = name
        self.topics = topic_names(robot, "svc", name, via_team)
        self.cache = ResultCache()

        self.srv_type = get_service(type_str)      # ⭐ 런타임 해석 — 여기가 요점
        self.client = node.create_client(self.srv_type, service_name)
        self.pub = node.create_publisher(String, "/" + self.topics["result"], 10)
        self.sub = node.create_subscription(
            String, "/" + self.topics["request"], self._on_request, 10)
        self._String = String
        self.log.info("[wrap] %s -> %s (%s)"
                      % (self.topics["request"], service_name, type_str))

    # ---- 내부 ----------------------------------------------------------
    def _emit(self, payload):
        m = self._String()
        m.data = payload
        self.pub.publish(m)

    def _on_request(self, msg):
        t0 = time.monotonic()
        try:
            rid, args, deadline = parse_request(msg.data)
        except EnvelopeError as e:
            # ⭐ id 를 못 읽었어도 응답은 낸다. 어디로 갈지는 몰라도, 화면이
            #    "쐈는데 아무 일도 없다" 대신 이유를 볼 수 있어야 한다.
            self._emit(fail_result(None, e.why, e.detail))
            return

        cached = self.cache.seen(rid)
        if cached is not None:
            self._emit(cached)                 # 재전송 — 다시 부르지 않는다
            return
        if self.cache.is_inflight(rid):
            return                             # 아직 처리 중 — 중복 호출 금지
        try:
            self.cache.begin(rid)
        except EnvelopeError as e:
            self._emit(fail_result(rid, e.why, e.detail))
            return

        def done(payload):
            self.cache.finish(rid, payload)
            self._emit(payload)

        if not self.client.service_is_ready():
            if not self.client.wait_for_service(timeout_sec=min(deadline, 1000) / 1000.0):
                done(fail_result(rid, WHY_ABSENT, self.client.srv_name,
                                 (time.monotonic() - t0) * 1000))
                return
        req = self.srv_type.Request()
        try:
            self._fill(req, args)
        except Exception as exc:                       # noqa: BLE001 - 타입 오류 전부
            done(fail_result(rid, WHY_BAD_ARGS, exc, (time.monotonic() - t0) * 1000))
            return

        future = self.client.call_async(req)

        def on_done(fut):
            took = (time.monotonic() - t0) * 1000
            try:
                resp = fut.result()
            except Exception as exc:                   # noqa: BLE001
                done(fail_result(rid, WHY_FAILED, exc, took))
                return
            done(ok_result(rid, dict(self._to_dict(resp)), took))

        future.add_done_callback(on_done)
        # 마감 시각을 지킨다 — 서비스가 안 돌아와도 요청자를 안 매단다.
        self.node.create_timer(
            deadline / 1000.0,
            lambda: (self.cache.is_inflight(rid)
                     and done(fail_result(rid, WHY_TIMEOUT, self.client.srv_name,
                                          (time.monotonic() - t0) * 1000))))


class WrappedAction(object):
    """액션 하나를 goal/cancel/feedback/result **토픽 넷**으로 감싼다.

    ⚠️ **실기 왕복 검증을 못 했다.** 이 레포의 시험 환경에 액션 타입이 하나도 설치돼
       있지 않다(2026-09-12 확인). 봉투 규약과 취소 경로는 서비스 쪽과 같은 코드를
       쓰지만, 액션 자체는 로봇에서 한 번 돌려 보고 판단해야 한다.

    ⚠️ **취소는 늦는다.** 취소 편지가 브리지를 건너는 동안 로봇은 계속 간다.
       그래서 이것을 **비상 정지로 쓰면 안 된다.** 비상 정지는 벌 2 의
       `teleop_out` 유닛을 끄는 것이고, 그건 바퀴로 가는 길 자체를 끊는다.
    """

    def __init__(self, node, robot, name, type_str, action_name,
                 logger=None, via_team=False):
        from rclpy.action import ActionClient
        (_r, _N, String, self._fill, self._to_dict,
         _gs, get_interface) = _ros()
        self.node = node
        self.log = logger or node.get_logger()
        self.topics = topic_names(robot, "act", name, via_team)
        self.cache = ResultCache()
        self._goals = {}                      # id -> goal handle
        self.act_type = get_interface(type_str)
        self.client = ActionClient(node, self.act_type, action_name)
        self._String = String
        self.pub_result = node.create_publisher(String, "/" + self.topics["result"], 10)
        self.pub_feedback = node.create_publisher(String, "/" + self.topics["feedback"], 10)
        node.create_subscription(String, "/" + self.topics["goal"], self._on_goal, 10)
        node.create_subscription(String, "/" + self.topics["cancel"], self._on_cancel, 10)
        self.log.info("[wrap] %s -> %s (%s)"
                      % (self.topics["goal"], action_name, type_str))

    def _emit(self, pub, payload):
        m = self._String()
        m.data = payload
        pub.publish(m)

    def _on_goal(self, msg):
        t0 = time.monotonic()
        try:
            rid, args, deadline = parse_request(msg.data)
        except EnvelopeError as e:
            self._emit(self.pub_result, fail_result(None, e.why, e.detail))
            return
        cached = self.cache.seen(rid)
        if cached is not None or self.cache.is_inflight(rid):
            if cached is not None:
                self._emit(self.pub_result, cached)
            return
        try:
            self.cache.begin(rid)
        except EnvelopeError as e:
            self._emit(self.pub_result, fail_result(rid, e.why, e.detail))
            return

        def done(payload):
            self.cache.finish(rid, payload)
            self._goals.pop(rid, None)
            self._emit(self.pub_result, payload)

        if not self.client.wait_for_server(timeout_sec=deadline / 1000.0):
            done(fail_result(rid, WHY_ABSENT, "action server", (time.monotonic() - t0) * 1000))
            return
        goal = self.act_type.Goal()
        try:
            self._fill(goal, args)
        except Exception as exc:                       # noqa: BLE001
            done(fail_result(rid, WHY_BAD_ARGS, exc, (time.monotonic() - t0) * 1000))
            return

        def on_feedback(fb):
            self._emit(self.pub_feedback,
                       ok_result(rid, dict(self._to_dict(fb.feedback))))

        send = self.client.send_goal_async(goal, feedback_callback=on_feedback)

        def on_sent(fut):
            handle = fut.result()
            if not handle.accepted:
                done(fail_result(rid, WHY_REJECTED, "", (time.monotonic() - t0) * 1000))
                return
            self._goals[rid] = handle
            res = handle.get_result_async()
            res.add_done_callback(lambda f: done(
                ok_result(rid, dict(self._to_dict(f.result().result)),
                          (time.monotonic() - t0) * 1000)))

        send.add_done_callback(on_sent)

    def _on_cancel(self, msg):
        try:
            rid, _args, _d = parse_request(msg.data)
        except EnvelopeError as e:
            self._emit(self.pub_result, fail_result(None, e.why, e.detail))
            return
        handle = self._goals.get(rid)
        if handle is None:
            self._emit(self.pub_result, fail_result(rid, WHY_FAILED, "그 id 의 목표가 없다"))
            return
        handle.cancel_goal_async()
        self._emit(self.pub_result, fail_result(rid, WHY_CANCELLED, "취소 요청을 보냈다"))


def build_node(config, node_name="topic_wrap"):
    """설정(dict) 하나로 서비스·액션을 전부 감싼 노드를 만든다."""
    rclpy, Node, _S, _f, _t, _gs, _gi = _ros()
    node = Node(node_name)
    robot = config["robot"]
    wrapped = []
    for name, spec in (config.get("services") or {}).items():
        wrapped.append(WrappedService(node, robot, name, spec["type"], spec["service"],
                                      via_team=bool(spec.get("team"))))
    for name, spec in (config.get("actions") or {}).items():
        wrapped.append(WrappedAction(node, robot, name, spec["type"], spec["action"],
                                     via_team=bool(spec.get("team"))))
    if not wrapped:
        raise SystemExit("감쌀 것이 하나도 없다 — 설정을 확인하라 (services/actions)")
    return node, wrapped


def main(argv=None):
    import argparse
    import yaml
    rclpy = __import__("rclpy")
    ap = argparse.ArgumentParser(description="서비스·액션을 토픽으로 감싼다")
    ap.add_argument("--config", required=True)
    args = ap.parse_args(argv)
    with open(args.config, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    rclpy.init()
    node, _wrapped = build_node(cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
