# -*- coding: utf-8 -*-
"""실제 hybrid_agent_node.PinkyAgent 를 ROS 위에서 돌려 레인 규약을 잰다 (가짜 Nav2 서버 · 격리 도메인).

rkd1rjs2/robot_mini_project_pinky 의 relay_station/tests/test_agent_node_loopback.py 를 옮겼다(경로만 바꿨다).
ROS 환경(source /opt/ros/jazzy/setup.bash + install/setup.bash)이 없으면 skip 한다.

`test_route_chain` 은 로직만 잰다. 이 파일은 **hybrid_agent_node 의 배선**
(Route 구독 QoS · LaneStatus 발행 · NavigateToPose 목표 · /estop 발행 자리)을 실제 DDS 위에서 잰다.

⚠️ 격리: ROS_DOMAIN_ID=97 + ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST. 현장망·로봇·브리지에 닿지 않는다.
⚠️ rclpy 전역 초기화가 다른 시험과 섞이지 않게 **별도 프로세스**로 돈다(이 파일을 --scenario 로 다시 부른다).
"""
import json
import os
import subprocess
import sys
import threading
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # pinky_fleet_agent 패키지 디렉터리

WP5 = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
SEQ = 7
ISOLATED_DOMAIN = "97"


def _scenario():
    sys.path.insert(0, REPO)
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from geometry_msgs.msg import Point, TransformStamped
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import Bool, String
    from lifecycle_msgs.srv import GetState
    from pinky_fleet_msgs.msg import RobotState
    from tf2_ros import TransformBroadcaster
    from pinky_fleet_msgs.msg import FleetCommand
    from pinky_lane_msgs.msg import LaneCommand, LaneStatus, Route
    from pinky_fleet_agent.hybrid_agent_node import PinkyAgent

    rclpy.init()
    cb = ReentrantCallbackGroup()
    obs = {"lane_status": [], "estop": [], "goals": [], "cancels": 0, "phase": "init",
           "diag": None, "battery": []}
    lock = threading.Lock()

    class Harness(Node):
        def __init__(self):
            super().__init__("d6_harness")
            self.pose = [0.0, 0.0]
            self.succeed_at = None
            self.stopping = False       # 끝낼 때 가짜 Nav2 의 goal 루프도 끝낸다 — 안 그러면 회귀가 '멈춤' 으로만 보인다
            self.tfb = TransformBroadcaster(self)
            route_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                   reliability=QoSReliabilityPolicy.RELIABLE,
                                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            cmd_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.VOLATILE)
            self.route_pub = self.create_publisher(Route, "/pinky1/route", route_qos)
            self.lane_pub = self.create_publisher(LaneCommand, "/pinky1/lane_command", cmd_qos)
            self.fleet_pub = self.create_publisher(FleetCommand, "/pinky1/command", cmd_qos)
            self.create_subscription(LaneStatus, "/pinky1/lane_status", self._on_ls, cmd_qos, callback_group=cb)
            self.create_subscription(Bool, "/estop", self._on_estop, 10, callback_group=cb)
            # R-5: 진단 재료 — 실제 발행 형식 그대로
            self.fix_pub = self.create_publisher(String, "/pinky1/fix_status", 10)
            self.gate_pub = self.create_publisher(String, "/drive_gate_status", 10)
            self.create_subscription(String, "/pinky1/diag", self._on_diag, 10, callback_group=cb)
            self.create_subscription(RobotState, "/pinky1/state", self._on_state, 10, callback_group=cb)
            # 가짜 수명주기: controller=active(3), planner=inactive(2). 나머지 6개는 서비스가 없다 → unknown
            for node, sid in (("controller_server", 3), ("planner_server", 2)):
                self.create_service(GetState, "/%s/get_state" % node,
                                    lambda req, resp, sid=sid: self._lc(resp, sid), callback_group=cb)
            self.create_timer(0.1, self._tick, callback_group=cb)
            # D7: 에이전트를 거치지 않는 goal (`/robotN/goal_pose` → bt_navigator 우회 경로 흉내)
            self.bypass = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
            ActionServer(self, NavigateToPose, "navigate_to_pose", execute_callback=self._exec,
                         goal_callback=lambda _g: GoalResponse.ACCEPT,
                         cancel_callback=self._cancel, callback_group=cb)

        def _tick(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x, t.transform.translation.y = self.pose
            t.transform.rotation.w = 1.0
            self.tfb.sendTransform(t)
            hb = FleetCommand(); hb.command = FleetCommand.CMD_HEARTBEAT
            self.fleet_pub.publish(hb)
            self.fix_pub.publish(String(data="accepted 2 rejected 0 age 0.3s"))
            self.gate_pub.publish(String(data="source=MISSION vx=0.10 wz=0.00"))

        def _lc(self, resp, sid):
            resp.current_state.id = sid
            resp.current_state.label = {3: "active", 2: "inactive"}[sid]
            return resp

        def _on_diag(self, m):
            with lock:
                obs["diag"] = json.loads(m.data)

        def _on_state(self, m):
            with lock:
                obs["battery"].append(m.battery_percent)

        def _on_ls(self, m):
            with lock:
                obs["lane_status"].append((obs["phase"], m.drive_state, m.route_seq, m.route_idx))

        def _on_estop(self, m):
            with lock:
                obs["estop"].append((obs["phase"], bool(m.data)))

        def _cancel(self, _gh):
            with lock:
                obs["cancels"] += 1
            return CancelResponse.ACCEPT

        def _exec(self, gh):
            p = gh.request.pose.pose.position
            rec = {"phase": obs["phase"], "x": round(p.x, 3), "y": round(p.y, 3), "status": None}
            with lock:
                obs["goals"].append(rec)
            while rclpy.ok() and not self.stopping:
                if gh.is_cancel_requested:
                    gh.canceled(); rec["status"] = "canceled"
                    return NavigateToPose.Result()
                if self.succeed_at and abs(p.x - self.succeed_at[0]) < 1e-6 and abs(p.y - self.succeed_at[1]) < 1e-6:
                    gh.succeed(); rec["status"] = "succeeded"
                    return NavigateToPose.Result()
                time.sleep(0.05)
            return NavigateToPose.Result()

        def lane(self, command, clear=0, seq=SEQ):
            m = LaneCommand(); m.command = command; m.route_seq = seq; m.clear_until_idx = clear
            self.lane_pub.publish(m)

    h = Harness()
    # ⭐ 현장 순서: 코디네이터가 Route 를 **먼저 한 번** 내고, 에이전트는 나중에 뜬다.
    #    TRANSIENT_LOCAL 구독이 아니면 늦게 뜬 에이전트는 경로를 영영 못 받는다 — 그 순서로 잰다.
    r = Route(); r.robot_name = "pinky1"; r.route_seq = SEQ; r.goal_idx = 4
    r.waypoints = [Point(x=x, y=y, z=0.0) for x, y in WP5]
    h.route_pub.publish(r)
    agent = PinkyAgent()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(h); ex.add_node(agent)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()

    def wait(pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            with lock:
                if pred():
                    return True
            time.sleep(0.05)
        return False

    def phase(name):
        with lock:
            obs["phase"] = name

    result = {}
    try:
        # 디스커버리 — 에이전트의 LaneCommand 구독이 보일 때까지
        result["discovered"] = wait(lambda: h.lane_pub.get_subscription_count() >= 1
                                    and h.route_pub.get_subscription_count() >= 1, 15)
        phase("route")
        result["route_status_seen"] = wait(lambda: any(p == "route" and s == SEQ for p, _, s, _ in obs["lane_status"]), 10)

        phase("start")
        h.lane(LaneCommand.CMD_START)
        result["ack_state"] = None
        if wait(lambda: any(p == "start" and d != LaneStatus.DRIVE_IDLE for p, d, _, _ in obs["lane_status"]), 10):
            result["ack_state"] = [d for p, d, _, _ in obs["lane_status"] if p == "start"][-1]

        phase("clear2")
        h.lane(LaneCommand.CMD_CLEARANCE, clear=2)
        wait(lambda: any(g["phase"] == "clear2" for g in obs["goals"]), 10)
        result["goal_clear2"] = [(g["x"], g["y"]) for g in obs["goals"] if g["phase"] == "clear2"]

        phase("clear0")
        c0 = obs["cancels"]
        h.lane(LaneCommand.CMD_CLEARANCE, clear=0)
        result["cancelled_on_clear0"] = wait(lambda: obs["cancels"] > c0, 10)
        time.sleep(0.5)

        phase("clear4")
        h.lane(LaneCommand.CMD_CLEARANCE, clear=4)
        wait(lambda: any(g["phase"] == "clear4" for g in obs["goals"]), 10)
        result["goal_clear4"] = [(g["x"], g["y"]) for g in obs["goals"] if g["phase"] == "clear4"]
        h.pose[:] = [2.0, 0.0]
        h.succeed_at = (2.0, 0.0)
        result["arrived"] = wait(lambda: any(d == LaneStatus.DRIVE_ARRIVED for _, d, _, _ in obs["lane_status"]), 10)
        # ARRIVED 는 Nav2 성공으로 먼저 올 수 있다 — 진행 인덱스는 TF(10 Hz 틱)가 따라올 때까지 기다려 잰다
        wait(lambda: any(i == 4 for _, _, _, i in obs["lane_status"]), 5)
        result["progress_idx_max"] = max(i for _, _, _, i in obs["lane_status"])

        with lock:
            result["estop_before_estop_cmd"] = list(obs["estop"])

        phase("estop")
        h.lane(LaneCommand.CMD_ESTOP, seq=0)
        wait(lambda: any(p == "estop" for p, _ in obs["estop"]), 10)
        phase("resume")
        h.lane(LaneCommand.CMD_RESUME)
        wait(lambda: any(p == "resume" for p, _ in obs["estop"]), 10)
        with lock:
            result["estop_after"] = [e for e in obs["estop"] if e[0] in ("estop", "resume")]

        # ---- D7: 우회로 들어온 goal 도 STOP 이 세운다 (HOLD = 이 서버의 goal 전부 취소) ----
        phase("bypass")
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = 9.0, 9.0
        g.pose.pose.orientation.w = 1.0
        h.bypass.wait_for_server(timeout_sec=5.0)
        h.bypass.send_goal_async(g)
        result["bypass_started"] = wait(lambda: any(x["phase"] == "bypass" for x in obs["goals"]), 10)
        time.sleep(0.3)
        phase("stop")
        t_stop = time.time()
        h.lane(LaneCommand.CMD_STOP)
        ok = wait(lambda: any(x["phase"] == "bypass" and x["status"] == "canceled" for x in obs["goals"]), 5)
        result["bypass_canceled_s"] = round(time.time() - t_stop, 3) if ok else None
        time.sleep(0.5)
        with lock:
            result["estop_in_bypass_stop"] = [e for e in obs["estop"] if e[0] in ("bypass", "stop")]
        phase("resume2")
        h.lane(LaneCommand.CMD_RESUME)

        # ---- R-5: 진단이 재료를 담아 1 Hz 로 나온다 (수명주기 응답이 한 번 돌 시간을 준다) ----
        time.sleep(2.5)
        with lock:
            result["diag"] = obs["diag"]
            b = obs["battery"][-1] if obs["battery"] else None
        result["battery_is_nan"] = (b is not None and b != b)
    finally:
        h.stopping = True
        time.sleep(0.2)
        ex.shutdown()
        agent.destroy_node(); h.destroy_node()
        rclpy.shutdown()
    print("RESULT " + json.dumps(result))


# ---- 관제 검수 후속: D6-1 · ESTOP 재발행 · Nav2 허용 반경 (실제 DDS) -------------------------------

WP21 = [(0.1 * i, 0.0) for i in range(21)]      # 실제 Route 간격 0.10 m
D61_OBSERVE_S = 6.0


def _scenario_d61():
    sys.path.insert(0, REPO)
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from geometry_msgs.msg import Point, TransformStamped
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import Bool, String
    from tf2_ros import TransformBroadcaster
    from pinky_fleet_msgs.msg import FleetCommand
    from pinky_lane_msgs.msg import LaneCommand, Route
    from pinky_fleet_agent.hybrid_agent_node import PinkyAgent

    rclpy.init()
    cb = ReentrantCallbackGroup()
    obs = {"goals": [], "diag": None, "late_estop": []}
    lock = threading.Lock()

    class Harness(Node):
        def __init__(self):
            super().__init__("d61_harness")
            self.tfb = TransformBroadcaster(self)
            q_route = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            q_cmd = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                               reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.VOLATILE)
            self.route_pub = self.create_publisher(Route, "/pinky1/route", q_route)
            self.lane_pub = self.create_publisher(LaneCommand, "/pinky1/lane_command", q_cmd)
            self.fleet_pub = self.create_publisher(FleetCommand, "/pinky1/command", q_cmd)
            self.create_subscription(String, "/pinky1/diag", self._on_diag, 10, callback_group=cb)
            self.create_timer(0.1, self._tick, callback_group=cb)
            # 가짜 Nav2: **받자마자 성공** — 로봇이 목표 0.14 m 앞이고 Nav2 허용이 0.15 m 인 팜과 같은 반응
            ActionServer(self, NavigateToPose, "navigate_to_pose", execute_callback=self._exec,
                         goal_callback=lambda _g: GoalResponse.ACCEPT,
                         cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)

        def _tick(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = 0.86                 # 허가 지점 idx 10 (1.0 m) 0.14 m 앞
            t.transform.rotation.w = 1.0
            self.tfb.sendTransform(t)
            hb = FleetCommand(); hb.command = FleetCommand.CMD_HEARTBEAT
            self.fleet_pub.publish(hb)

        def _on_diag(self, m):
            with lock:
                obs["diag"] = json.loads(m.data)

        def _exec(self, gh):
            with lock:
                obs["goals"].append(time.time())
            gh.succeed()
            return NavigateToPose.Result()

        def lane(self, command, clear=0, seq=SEQ):
            m = LaneCommand(); m.command = command; m.route_seq = seq; m.clear_until_idx = clear
            self.lane_pub.publish(m)

    h = Harness()
    r = Route(); r.robot_name = "pinky1"; r.route_seq = SEQ; r.goal_idx = 20
    r.waypoints = [Point(x=x, y=y, z=0.0) for x, y in WP21]
    h.route_pub.publish(r)
    agent = PinkyAgent()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(h); ex.add_node(agent)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()

    def wait(pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            with lock:
                if pred():
                    return True
            time.sleep(0.05)
        return False

    result = {}
    extra = []
    try:
        result["discovered"] = wait(lambda: h.lane_pub.get_subscription_count() >= 1
                                    and h.route_pub.get_subscription_count() >= 1, 15)
        time.sleep(1.0)                                      # TF 가 에이전트에 닿을 시간
        h.lane(LaneCommand.CMD_START)
        time.sleep(0.3)
        h.lane(LaneCommand.CMD_CLEARANCE, clear=10)
        t0 = time.time()
        time.sleep(D61_OBSERVE_S)
        with lock:
            result["goals_in_window"] = sum(1 for t in obs["goals"] if t >= t0 - 0.5)
            d = obs["diag"] or {}
        result["agent_after"] = d.get("agent")

        # ---- P2: 래치 동안 /estop true 재발행 — 래치 **뒤에** 뜬 구독자도 받는다 ----
        h.lane(LaneCommand.CMD_ESTOP, seq=0)
        time.sleep(1.5)
        late = rclpy.create_node("late_gate")
        late.create_subscription(Bool, "/estop", lambda m: obs["late_estop"].append(bool(m.data)), 10)
        ex.add_node(late); extra.append(late)
        result["late_gate_got_true"] = wait(lambda: True in obs["late_estop"], 4.0)
        h.lane(LaneCommand.CMD_RESUME)

        # ---- D6-1 두 번째 겹: 실제 controller_server 파라미터 서비스에서 허용 반경을 읽는다 ----
        ctrl = rclpy.create_node(
            "controller_server",
            parameter_overrides=[Parameter("general_goal_checker.xy_goal_tolerance", Parameter.Type.DOUBLE, 0.15)],
            automatically_declare_parameters_from_overrides=True)
        ex.add_node(ctrl); extra.append(ctrl)
        wait(lambda: (obs["diag"] or {}).get("agent", {}).get("nav2_xy_goal_tolerance") == 0.15, 8.0)
        with lock:
            result["agent_tol"] = (obs["diag"] or {}).get("agent")
    finally:
        ex.shutdown()
        for n in extra:
            n.destroy_node()
        agent.destroy_node(); h.destroy_node()
        rclpy.shutdown()
    print("RESULT " + json.dumps(result))


# ---- 관제 검수 REVIEW_20260925 §3.2 · 에이전트를 거치지 않는 목표 (실제 DDS) ---------------------------------

def _scenario_bypass():
    """세 줄: ① ESTOP → 우회 goal → RESUME → 취소, /estop false 는 취소 뒤 ② HOLD → 우회 goal → 취소
    ③ 우회 goal 로 달리는 중 하트비트 끊김 → 취소 + /estop true."""
    sys.path.insert(0, REPO)
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import Bool
    from pinky_fleet_msgs.msg import FleetCommand
    from pinky_lane_msgs.msg import LaneCommand
    from pinky_fleet_agent.hybrid_agent_node import PinkyAgent

    rclpy.init()
    cb = ReentrantCallbackGroup()
    lock = threading.Lock()
    obs = {"goals": {}, "estop": []}                # tag -> {"accepted": t, "canceled": t}

    class Harness(Node):
        def __init__(self):
            super().__init__("bypass_harness")
            q = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                           reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
            self.lane_pub = self.create_publisher(LaneCommand, "/pinky1/lane_command", q)
            self.fleet_pub = self.create_publisher(FleetCommand, "/pinky1/command", q)
            self.create_subscription(Bool, "/estop", lambda m: self._es(m), 10, callback_group=cb)
            self.hb_on = True
            self.stopping = False
            self.create_timer(0.1, self._tick, callback_group=cb)
            self.client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
            ActionServer(self, NavigateToPose, "navigate_to_pose", execute_callback=self._exec,
                         goal_callback=lambda _g: GoalResponse.ACCEPT,
                         cancel_callback=self._cancel, callback_group=cb)

        def _cancel(self, gh):
            # 서버가 취소 요청을 **받아들인** 시각 — 실행 고리가 그것을 알아차린 시각(최대 20 ms 늦음)이 아니라
            tag = round(gh.request.pose.pose.position.x, 1)
            with lock:
                obs["goals"].setdefault(tag, {"accepted": None, "canceled": None})["cancel_req"] = time.time()
            return CancelResponse.ACCEPT

        def _es(self, m):
            with lock:
                obs["estop"].append((time.time(), bool(m.data)))

        def _tick(self):
            if self.hb_on:
                hb = FleetCommand(); hb.command = FleetCommand.CMD_HEARTBEAT
                self.fleet_pub.publish(hb)

        def _exec(self, gh):
            tag = round(gh.request.pose.pose.position.x, 1)
            with lock:
                obs["goals"][tag] = {"accepted": time.time(), "canceled": None}
            while rclpy.ok() and not self.stopping:
                if gh.is_cancel_requested:
                    with lock:
                        obs["goals"][tag]["canceled"] = time.time()
                    gh.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.02)
            return NavigateToPose.Result()

        def bypass(self, x):
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"
            g.pose.pose.position.x = float(x)
            g.pose.pose.orientation.w = 1.0
            self.client.send_goal_async(g)

        def lane(self, cmd):
            m = LaneCommand(); m.command = cmd; m.route_seq = 0
            self.lane_pub.publish(m)

        def fleet(self, cmd):
            m = FleetCommand(); m.command = cmd
            self.fleet_pub.publish(m)

    h = Harness()
    agent = PinkyAgent()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(h); ex.add_node(agent)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()

    def wait(pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            with lock:
                if pred():
                    return True
            time.sleep(0.05)
        return False

    def g(tag, key):
        return obs["goals"].get(tag, {}).get(key)

    result = {}
    try:
        result["discovered"] = wait(lambda: h.lane_pub.get_subscription_count() >= 1, 15)
        h.client.wait_for_server(timeout_sec=5.0)
        time.sleep(1.0)                                      # 하트비트로 링크 무장
        # ① ESTOP 중 우회 goal → 래치 중 취소(감시) · 해제 직전 우회 goal → 취소가 /estop false 보다 먼저
        h.lane(LaneCommand.CMD_ESTOP)
        wait(lambda: any(v for _, v in obs["estop"]), 5)
        h.bypass(1.1)
        result["estop_bypass_canceled"] = wait(lambda: g(1.1, "canceled") is not None, 4)
        h.bypass(1.2)
        wait(lambda: g(1.2, "accepted") is not None, 3)
        h.lane(LaneCommand.CMD_RESUME)
        wait(lambda: any((not v) for _, v in obs["estop"]), 5)
        wait(lambda: g(1.2, "canceled") is not None, 3)
        with lock:
            t_false = min((t for t, v in obs["estop"] if not v), default=None)
            result["resume_bypass_canceled"] = g(1.2, "canceled") is not None
            result["cancel_req_vs_estop_false_ms"] = (None if t_false is None or g(1.2, "cancel_req") is None
                                                      else round((t_false - g(1.2, "cancel_req")) * 1000, 1))
            result["cancel_before_estop_false"] = (t_false is not None and g(1.2, "cancel_req") is not None
                                                   and g(1.2, "cancel_req") <= t_false)
        # ② HOLD(FleetCommand STOP) 중 우회 goal → 취소
        for _ in range(3):
            h.fleet(FleetCommand.CMD_STOP)
            time.sleep(0.1)
        h.bypass(2.1)
        result["hold_bypass_canceled"] = wait(lambda: g(2.1, "canceled") is not None, 4)
        h.fleet(FleetCommand.CMD_RESUME)
        h.lane(LaneCommand.CMD_RESUME)
        time.sleep(1.0)
        # ③ 정지 아님 — 우회 goal 로 달리는 중 하트비트 끊김 → 데드맨이 취소 + /estop true
        with lock:
            n_true_before = sum(1 for _, v in obs["estop"] if v)
        h.bypass(3.1)
        wait(lambda: g(3.1, "accepted") is not None, 3)
        time.sleep(0.5)
        result["running_bypass_alive"] = g(3.1, "canceled") is None      # 정지가 아니면 남의 goal 을 안 건드린다
        h.hb_on = False
        result["deadman_canceled"] = wait(lambda: g(3.1, "canceled") is not None, 8)
        with lock:
            result["deadman_estop_true"] = sum(1 for _, v in obs["estop"] if v) > n_true_before
    finally:
        h.stopping = True
        time.sleep(0.2)
        ex.shutdown()
        agent.destroy_node(); h.destroy_node()
        rclpy.shutdown()
    print("RESULT " + json.dumps(result))


# ---- R-7 웹 전환: CMD_SET_MAP 이 진짜 map_server/load_map 서비스로 가고 RobotState.map_name 이 바뀐다 -----------

def _scenario_setmap():
    sys.path.insert(0, REPO)
    import tempfile
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from nav2_msgs.srv import LoadMap
    from pinky_fleet_msgs.msg import FleetCommand, RobotState
    from pinky_fleet_agent.hybrid_agent_node import PinkyAgent

    mapdir = tempfile.mkdtemp()
    open(os.path.join(mapdir, "map4.yaml"), "w").write("image: map4.pgm\nresolution: 0.05\norigin: [-1.175, -0.625, 0.0]\n")
    rclpy.init(args=["--ros-args", "-p", "map_dir:=" + mapdir, "-p", "map_name:=my_map"])
    cb = ReentrantCallbackGroup()
    obs = {"load_urls": [], "map_names": []}
    lock = threading.Lock()

    class Harness(Node):
        def __init__(self):
            super().__init__("setmap_harness")
            q = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                           reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
            self.fleet_pub = self.create_publisher(FleetCommand, "/pinky1/command", q)
            self.create_subscription(RobotState, "/pinky1/state", self._st, 10, callback_group=cb)
            self.create_service(LoadMap, "/map_server/load_map", self._load, callback_group=cb)
            self.create_timer(0.1, self._hb, callback_group=cb)

        def _load(self, req, resp):
            with lock:
                obs["load_urls"].append(req.map_url)
            resp.result = LoadMap.Response.RESULT_SUCCESS
            return resp

        def _st(self, m):
            with lock:
                obs["map_names"].append(m.map_name)

        def _hb(self):
            m = FleetCommand(); m.command = FleetCommand.CMD_HEARTBEAT
            self.fleet_pub.publish(m)

    h = Harness()
    agent = PinkyAgent()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(h); ex.add_node(agent)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()

    def wait(pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            with lock:
                if pred():
                    return True
            time.sleep(0.05)
        return False

    result = {}
    try:
        result["discovered"] = wait(lambda: h.fleet_pub.get_subscription_count() >= 1 and obs["map_names"], 15)
        with lock:
            result["before"] = obs["map_names"][-1] if obs["map_names"] else None
        time.sleep(1.0)
        m = FleetCommand(); m.command = FleetCommand.CMD_SET_MAP; m.map_name = "map4"
        h.fleet_pub.publish(m)
        result["loaded"] = wait(lambda: bool(obs["load_urls"]), 5)
        result["after_ok"] = wait(lambda: obs["map_names"] and obs["map_names"][-1] == "map4", 5)
        with lock:
            result["url"] = obs["load_urls"][-1] if obs["load_urls"] else None
        result["map_load"] = agent._map_load
    finally:
        ex.shutdown()
        agent.destroy_node(); h.destroy_node()
        rclpy.shutdown()
    print("RESULT " + json.dumps(result))


def _run(flag="--scenario"):
    env = dict(os.environ)
    env.pop("CYCLONEDDS_URI", None)
    env.update(ROS_DOMAIN_ID=ISOLATED_DOMAIN, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               PYTHONDONTWRITEBYTECODE="1")
    p = subprocess.run([sys.executable, "-B", os.path.abspath(__file__), flag],
                       capture_output=True, text=True, env=env, timeout=120)
    lines = [l for l in p.stdout.splitlines() if l.startswith("RESULT ")]
    assert lines, "시나리오가 결과를 못 냈다 (rc=%s)\n%s\n%s" % (p.returncode, p.stdout[-1500:], p.stderr[-1500:])
    return json.loads(lines[-1][len("RESULT "):])


if __name__ == "__main__" and "--scenario" in sys.argv:
    _scenario()
    sys.exit(0)
if __name__ == "__main__" and "--scenario-d61" in sys.argv:
    _scenario_d61()
    sys.exit(0)
if __name__ == "__main__" and "--scenario-bypass" in sys.argv:
    _scenario_bypass()
    sys.exit(0)
if __name__ == "__main__" and "--scenario-setmap" in sys.argv:
    _scenario_setmap()
    sys.exit(0)

import pytest  # noqa: E402

try:
    import rclpy  # noqa: F401
    import nav2_msgs.action  # noqa: F401
    import pinky_lane_msgs.msg  # noqa: F401
    _HAVE_ROS = True
except Exception:
    _HAVE_ROS = False


@pytest.fixture(scope="module")
def loop():
    if not _HAVE_ROS:
        pytest.skip("ROS(rclpy·nav2_msgs·pinky_lane_msgs) 환경이 없다 — source install/setup.bash")
    return _run()


def test_디스커버리와_경로_수신(loop):
    assert loop["discovered"], "격리 도메인에서 에이전트 구독이 안 보였다"
    assert loop["route_status_seen"], "Route 를 받고도 LaneStatus 를 안 냈다 (TRANSIENT_LOCAL 구독 확인)"


def test_START_에_LaneStatus_로_ack(loop):
    assert loop["ack_state"] not in (None, 0, 8, 9), loop["ack_state"]


def test_CLEARANCE_가_그_waypoint_로_NavigateToPose_를_보낸다(loop):
    assert loop["goal_clear2"] == [[1.0, 0.0]]


def test_허가_0_은_goal_취소이고_estop_이_아니다(loop):
    assert loop["cancelled_on_clear0"]
    assert [e for e in loop["estop_before_estop_cmd"] if e[1] is True] == [], loop["estop_before_estop_cmd"]


def test_허가_확장_뒤_도착하면_ARRIVED(loop):
    assert loop["goal_clear4"] == [[2.0, 0.0]]
    assert loop["arrived"]
    assert loop["progress_idx_max"] == 4, "B-5: 진행 인덱스가 경로 끝까지 올라야 한다"


def test_ESTOP_은_estop_true__RESUME_은_estop_false(loop):
    assert ["estop", True] in loop["estop_after"]
    assert ["resume", False] in loop["estop_after"]


def test_D7_우회로_들어온_goal_도_STOP_이_1초_안에_세운다(loop):
    """관제 팜은 `/api/robot1/goal` → `/robot1/goal_pose` → bt_navigator 로 에이전트를 거치지 않고 달린다.
    에이전트가 자기 goal 만 취소하면 그 주행은 멈추지 않는다 — 서버의 goal 전부를 취소해야 한다."""
    assert loop["bypass_started"]
    assert loop["bypass_canceled_s"] is not None, "STOP 뒤에도 우회 goal 이 취소되지 않았다"
    assert loop["bypass_canceled_s"] < 1.0, loop["bypass_canceled_s"]
    assert loop["estop_in_bypass_stop"] == [], "STOP 은 HOLD 다 — /estop 을 내면 안 된다"


def test_R5_진단이_실제_재료를_담아_나온다(loop):
    d = loop["diag"]
    assert d and d["schema"] == "pinky_diag/1" and d["robot"] == "pinky1"
    assert d["fix_status"]["accepted"] == 2 and d["fix_status"]["fix_age_s"] == 0.3
    assert d["gate"]["source"] == "MISSION"
    assert d["pose_source"] == "PoseFuser"
    assert d["estop"] is False                        # 마지막 /estop 은 RESUME 의 false


def test_R5_수명주기__응답한_노드는_실제_상태__응답_없는_노드는_unknown(loop):
    n = loop["diag"]["nav2"]
    assert n["total"] == 8 and n["active"] == 1
    assert n["nodes"]["controller_server"] == "active"
    assert n["nodes"]["planner_server"] == "inactive"
    assert sum(1 for s in n["nodes"].values() if s == "unknown") == 6


def test_R5_배터리를_받은_적_없으면_NaN(loop):
    assert loop["battery_is_nan"], "배터리를 받은 적이 없는데 숫자를 냈다 (예전 기본값 95.0)"


# ---- 관제 검수 후속 (실제 DDS) -----------------------------------------------------------------

@pytest.fixture(scope="module")
def d61():
    if not _HAVE_ROS:
        pytest.skip("ROS(rclpy·nav2_msgs·pinky_lane_msgs) 환경이 없다 — source install/setup.bash")
    return _run("--scenario-d61")


def test_D6_1_Nav2_가_즉시_성공해도_같은_목표를_되풀이해_보내지_않는다(d61):
    """관제 리그: 같은 목표 813회 / 811회 succeeded(초당 7~15회).
    이 시나리오를 고치기 전 코드(`ad71041`)로 돌리면 6 s 에 516회다(2026-09-25 중계 실측 — 가짜 Nav2 가
    즉시 답해서 리그보다 빠르다). 고친 코드는 1회."""
    assert d61["discovered"]
    assert 1 <= d61["goals_in_window"] <= 2, d61["goals_in_window"]
    a = d61["agent_after"]
    assert a["reached_idx"] == 10 and a["goals_sent"] <= 2, a


def test_P2_ESTOP_래치_뒤에_뜬_게이트도_estop_true_를_받는다(d61):
    assert d61["late_gate_got_true"], "VOLATILE /estop 을 한 번만 내면 늦게 뜬 게이트는 비상정지를 모른다"


def test_D6_1_controller_server_의_허용_반경을_읽어_도달_반경에_반영한다(d61):
    a = d61["agent_tol"]
    assert a["nav2_xy_goal_tolerance"] == 0.15, a
    assert abs(a["reach_tol"] - 0.17) < 1e-9, a


@pytest.fixture(scope="module")
def bypass():
    if not _HAVE_ROS:
        pytest.skip("ROS(rclpy·nav2_msgs·pinky_lane_msgs) 환경이 없다 — source install/setup.bash")
    return _run("--scenario-bypass")


def test_S2_ESTOP_중_들어온_우회_goal_은_래치_중에_취소된다(bypass):
    assert bypass["discovered"] and bypass["estop_bypass_canceled"]


def test_S2_해제_직전_우회_goal_은_estop_false_보다_먼저_취소된다(bypass):
    assert bypass["resume_bypass_canceled"]
    assert bypass["cancel_before_estop_false"], "게이트가 열린 뒤에 취소됐다 — 그 사이 우회 goal 로 달린다"


def test_S3_HOLD_중_들어온_우회_goal_은_취소된다(bypass):
    assert bypass["hold_bypass_canceled"]


def test_S4_우회_goal_로_달리다_하트비트가_끊기면_취소와_estop_true(bypass):
    assert bypass["running_bypass_alive"], "정지가 아닌데 남의 goal 을 취소했다"
    assert bypass["deadman_canceled"] and bypass["deadman_estop_true"]



def test_R7_CMD_SET_MAP_은_map_server_load_map_으로_가고_보고하는_지도_이름이_바뀐다():
    if not _HAVE_ROS:
        pytest.skip("ROS 환경이 없다")
    r = _run("--scenario-setmap")
    assert r["discovered"] and r["before"] == "my_map"
    assert r["loaded"] and r["url"].endswith("/map4.yaml")
    assert r["after_ok"] and r["map_load"]["result"] == "OK"
