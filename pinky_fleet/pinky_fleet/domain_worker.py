"""도메인 하나를 전담하는 자식 프로세스.

왜 프로세스인가
---------------
rclpy.init() 은 **프로세스당 한 번만** 호출할 수 있다(19강 s24). 로봇 두 대가 서로 다른
ROS_DOMAIN_ID(10, 11)에 있으므로, 하나의 파이썬 프로세스로는 두 대에 동시에 붙을 수 없다.
그래서 19·20강과 똑같이 multiprocessing 으로 도메인마다 프로세스를 나누고, 각 프로세스가
자기 도메인으로 rclpy.init(domain_id=N) 을 호출한다.

백엔드 3종 (검증 단계별로 갈아끼운다)
------------------------------------
  mock      : ROS 없이 시간만 흘려보낸다.        -> L0 (상태머신 로직 검증)
  turtlesim : /turtle1/cmd_vel P제어로 이동.      -> L1 (다중 도메인 배선 검증)
  nav2      : BasicNavigator 로 NavigateToPose.  -> L3/L4 (실제 미션)

세 백엔드 모두 부모와 주고받는 메시지 규약(protocol.py)이 같다. 달라지는 것은
"목표까지 간다"를 무엇으로 구현하느냐 뿐이다 — 20강 s42 의 학습 포인트 그대로다.
"""

from __future__ import annotations

import math
import queue
import time
import traceback

from pinky_fleet import protocol as P
from pinky_fleet.mission_config import Pose2D
from pinky_fleet.pose_utils import (
    distance,
    normalize_deg,
    yaw_deg_from_quaternion,
)


# ---------------------------------------------------------------------------
# 공통 뼈대
# ---------------------------------------------------------------------------
class WorkerBase:
    """부모와의 큐 대화를 담당. 이동 방법은 서브클래스가 구현한다."""

    def __init__(self, robot, cfg, cmd_q, status_q):
        self.robot = robot            # dict: name, domain_id, home
        self.cfg = cfg                # dict: map_frame, localization, feedback_period_sec
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.name = robot['name']
        self.home = Pose2D.from_dict(robot['home'])
        self.map_frame = cfg['map_frame']
        self.feedback_period = float(cfg['feedback_period_sec'])
        self.last_pose = None         # Pose2D

    # ---- 부모로 보내기 -----------------------------------------------------
    def emit(self, event, **payload):
        self.status_q.put(P.msg(self.name, event, **payload))

    def log(self, text):
        self.emit(P.EVT_LOG, text=text)

    # ---- 부모에서 받기 -----------------------------------------------------
    def poll_cmd(self, timeout=0.0):
        try:
            return self.cmd_q.get(timeout=timeout) if timeout else self.cmd_q.get_nowait()
        except queue.Empty:
            return None

    def check_interrupt(self):
        """주행 루프 안에서 취소/종료 명령이 왔는지 본다.

        종료(shutdown)는 여기서 처리하지 않고 큐에 되돌려 놓는다. 주행 루프가 삼켜버리면
        run() 의 메인 루프가 종료 명령을 영영 못 보고 프로세스가 남는다.
        """
        cmd = self.poll_cmd()
        if cmd is None:
            return None
        op = cmd.get('op')
        if op == P.CMD_SHUTDOWN:
            self.cmd_q.put(cmd)   # run() 이 처리하도록 되돌린다
            return P.CMD_SHUTDOWN
        if op == P.CMD_CANCEL:
            return P.CMD_CANCEL
        return None

    # ---- 메인 루프 ---------------------------------------------------------
    def run(self):
        try:
            while True:
                cmd = self.poll_cmd(timeout=0.1)
                if cmd is None:
                    self.idle()
                    continue

                op = cmd.get('op')
                if op == P.CMD_PREPARE:
                    self.do_prepare(cmd)
                elif op == P.CMD_GOTO:
                    self.do_goto(cmd)
                elif op == P.CMD_CANCEL:
                    self.do_cancel()
                elif op == P.CMD_SHUTDOWN:
                    break
                else:
                    self.log(f'알 수 없는 명령: {op}')
        except KeyboardInterrupt:
            pass
        except Exception:
            self.emit(P.EVT_FAILED, reason=traceback.format_exc())
        finally:
            try:
                self.cleanup()
            finally:
                self.emit(P.EVT_BYE)

    # ---- 서브클래스가 채우는 부분 -------------------------------------------
    def idle(self):
        """명령이 없을 때 주기적으로 호출. 위치 갱신/발행용."""

    def do_prepare(self, cmd):
        self.emit(P.EVT_READY)

    def do_goto(self, cmd):
        raise NotImplementedError

    def do_cancel(self):
        pass

    def cleanup(self):
        pass

    # ---- 공통 도우미 -------------------------------------------------------
    def publish_pose(self, pose2d):
        self.last_pose = pose2d
        self.emit(P.EVT_POSE, **pose2d.as_dict())


# ---------------------------------------------------------------------------
# L0: mock — ROS 없이 상태머신만 검증
# ---------------------------------------------------------------------------
class MockWorker(WorkerBase):
    SPEED = 0.5  # m/s 로 가정

    def __init__(self, *args, fail_on=None, hang_on=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.pos = self.home
        self.fail_on = fail_on or []   # 이 tag 의 goto 는 실패시킨다
        self.hang_on = hang_on or []   # 이 tag 의 goto 는 영원히 안 끝난다(타임아웃 검증)

    def do_prepare(self, cmd):
        self.log(f'[mock] Nav2 대기 흉내 (home={self.home})')
        time.sleep(0.2)
        self.publish_pose(self.pos)
        self.emit(P.EVT_READY)

    def do_goto(self, cmd):
        target = Pose2D.from_dict(cmd['pose'])
        tag = cmd.get('tag', '')
        timeout = float(cmd['timeout_sec'])
        total = distance(self.pos.x, self.pos.y, target.x, target.y)
        self.emit(P.EVT_ACCEPTED, tag=tag)

        started = time.time()
        travel = 1e9 if tag in self.hang_on else max(total / self.SPEED, 0.5)
        while True:
            if self.check_interrupt():
                self.emit(P.EVT_CANCELED, tag=tag)
                return
            elapsed = time.time() - started
            if elapsed >= travel:
                break
            if elapsed > timeout:
                self.emit(P.EVT_FAILED, tag=tag, reason=f'타임아웃 {timeout:.0f}s 초과')
                return
            ratio = min(elapsed / travel, 1.0)
            self.publish_pose(Pose2D(
                self.pos.x + (target.x - self.pos.x) * ratio,
                self.pos.y + (target.y - self.pos.y) * ratio,
                target.yaw_deg))
            self.emit(P.EVT_FEEDBACK, tag=tag,
                      distance_remaining=total * (1.0 - ratio), nav_time_sec=elapsed)
            time.sleep(self.feedback_period)

        if tag in self.fail_on:
            self.emit(P.EVT_FAILED, tag=tag, reason='[mock] 강제 실패 주입')
            return
        self.pos = target
        self.publish_pose(self.pos)
        self.emit(P.EVT_ARRIVED, tag=tag)


# ---------------------------------------------------------------------------
# ROS 백엔드 공통 (turtlesim / nav2)
# ---------------------------------------------------------------------------
class RosWorkerBase(WorkerBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        import rclpy
        self.rclpy = rclpy
        # ★ 이 프로세스만의 도메인. 터미널의 ROS_DOMAIN_ID 와 무관하다(19강 s25).
        rclpy.init(args=[], domain_id=int(self.robot['domain_id']))
        self.node = rclpy.create_node(f"fleet_worker_{self.name}")
        self.log(f'rclpy.init(domain_id={self.robot["domain_id"]}) 완료')

    def spin(self, timeout_sec=0.1):
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def cleanup(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# L1: turtlesim — 다중 도메인 배선 검증 (22강 스타일 P 제어)
# ---------------------------------------------------------------------------
class TurtlesimWorker(RosWorkerBase):
    ARRIVE_TOL = 0.25      # m
    ANGLE_TOL_DEG = 8.0
    K_LIN = 1.2
    K_ANG = 4.0
    MAX_LIN = 1.5
    MAX_ANG = 3.0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        from geometry_msgs.msg import Twist
        from turtlesim.msg import Pose as TurtlePose

        self.Twist = Twist
        self.pub = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.node.create_subscription(TurtlePose, '/turtle1/pose', self._on_pose, 10)
        self.pose_seen = False

    def _on_pose(self, data):
        self.last_pose = Pose2D(data.x, data.y, math.degrees(data.theta))
        self.pose_seen = True

    def idle(self):
        self.spin(0.05)

    def stop(self):
        self.pub.publish(self.Twist())

    def do_prepare(self, cmd):
        deadline = time.time() + 10.0
        while not self.pose_seen and time.time() < deadline:
            self.spin(0.1)
        if not self.pose_seen:
            self.emit(P.EVT_FAILED, reason='/turtle1/pose 를 받지 못했습니다. '
                                           'turtlesim 이 같은 도메인에 떠 있는지 확인하세요.')
            return
        self.publish_pose(self.last_pose)
        self.emit(P.EVT_READY)

    def do_goto(self, cmd):
        target = Pose2D.from_dict(cmd['pose'])
        tag = cmd.get('tag', '')
        timeout = float(cmd['timeout_sec'])
        self.emit(P.EVT_ACCEPTED, tag=tag)

        started = time.time()
        last_fb = 0.0
        while True:
            if self.check_interrupt():
                self.stop()
                self.emit(P.EVT_CANCELED, tag=tag)
                return

            self.spin(0.05)
            cur = self.last_pose
            remaining = distance(cur.x, cur.y, target.x, target.y)
            elapsed = time.time() - started

            if remaining < self.ARRIVE_TOL:
                self.stop()
                self.publish_pose(cur)
                self.emit(P.EVT_ARRIVED, tag=tag)
                return
            if elapsed > timeout:
                self.stop()
                self.emit(P.EVT_FAILED, tag=tag, reason=f'타임아웃 {timeout:.0f}s 초과')
                return

            # 방향을 먼저 맞추고 그 다음 전진 (25강 상태별 제어의 축소판)
            heading_err = normalize_deg(
                math.degrees(math.atan2(target.y - cur.y, target.x - cur.x)) - cur.yaw_deg)
            msg = self.Twist()
            if abs(heading_err) > self.ANGLE_TOL_DEG:
                msg.angular.z = max(-self.MAX_ANG,
                                    min(self.MAX_ANG, self.K_ANG * math.radians(heading_err)))
            else:
                msg.linear.x = max(0.0, min(self.MAX_LIN, self.K_LIN * remaining))
                msg.angular.z = self.K_ANG * math.radians(heading_err) * 0.5
            self.pub.publish(msg)

            if elapsed - last_fb >= self.feedback_period:
                last_fb = elapsed
                self.publish_pose(cur)
                self.emit(P.EVT_FEEDBACK, tag=tag,
                          distance_remaining=remaining, nav_time_sec=elapsed)

    def do_cancel(self):
        self.stop()

    def cleanup(self):
        try:
            self.stop()
        except Exception:
            pass
        super().cleanup()


# ---------------------------------------------------------------------------
# L3/L4: nav2 — 실제 미션
# ---------------------------------------------------------------------------
class Nav2Worker(RosWorkerBase):
    """nav2_simple_commander.BasicNavigator 로 NavigateToPose 액션을 쓴다.

    `주피터로 내비게이션 주행하기` p8~p18 의 흐름을 그대로 옮긴 것이다:
      BasicNavigator() -> setInitialPose() -> goToPose()
      -> while not isTaskComplete(): getFeedback() (타임아웃이면 cancelTask())
      -> getResult()
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

        self.TaskResult = TaskResult
        self.nav = BasicNavigator()
        # 현재 위치 표시용 구독 (자료 p11 의 sub_amcl 노드와 같은 역할).
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.amcl_cov = None
        self.loc = self.cfg['localization']

    def _on_amcl(self, m):
        p = m.pose.pose
        self.last_pose = Pose2D(
            p.position.x, p.position.y,
            yaw_deg_from_quaternion(p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w))
        self.amcl_cov = m.pose.covariance

    def idle(self):
        self.spin(0.05)

    # ---- 준비 --------------------------------------------------------------
    def do_prepare(self, cmd):
        import threading

        timeout = float(cmd.get('nav2_activate_timeout_sec', 60.0))
        self.log(f'Nav2 활성화 대기 (최대 {timeout:.0f}s)')

        # waitUntilNav2Active() 는 타임아웃 인자가 없어 영원히 막힐 수 있다.
        # 데몬 스레드로 돌리고 join(timeout) 으로 상한을 건다.
        done = threading.Event()

        def _wait():
            try:
                self.nav.waitUntilNav2Active()
                done.set()
            except Exception:
                pass

        t = threading.Thread(target=_wait, daemon=True)
        t.start()
        t.join(timeout)
        if not done.is_set():
            self.emit(P.EVT_FAILED,
                      reason='Nav2 가 활성화되지 않았습니다. 로봇에서 '
                             'pinky_navigation bringup_launch.xml 이 떠 있는지, '
                             f'ROS_DOMAIN_ID={self.robot["domain_id"]} 가 맞는지 확인하세요.')
            return

        # 출발지를 AMCL 초기 위치로 넣는다 -> RViz 2D Pose Estimate 수동 조작 불필요
        self.log(f'setInitialPose(home={self.home})')
        self.nav.setInitialPose(self._pose_stamped(self.home))

        if self.loc.get('wait_for_convergence', True):
            if not self._wait_for_convergence():
                return
        self.emit(P.EVT_READY)

    def _wait_for_convergence(self) -> bool:
        """AMCL 공분산이 임계값 아래로 내려올 때까지 기다린다.

        setInitialPose 직후 바로 goToPose 를 보내면 위치가 안 잡힌 상태로 출발해
        엉뚱한 경로가 나온다.
        """
        max_xy = float(self.loc['max_xy_std'])
        max_yaw = float(self.loc['max_yaw_std'])
        deadline = time.time() + float(self.loc['convergence_timeout_sec'])
        while time.time() < deadline:
            self.spin(0.1)
            if self.amcl_cov is None:
                continue
            xy = max(math.sqrt(max(self.amcl_cov[0], 0.0)),
                     math.sqrt(max(self.amcl_cov[7], 0.0)))
            yaw = math.sqrt(max(self.amcl_cov[35], 0.0))
            if xy <= max_xy and yaw <= max_yaw:
                self.log(f'AMCL 수렴 (xy_std={xy:.3f}m, yaw_std={yaw:.3f}rad)')
                return True
        self.emit(P.EVT_FAILED,
                  reason='AMCL 이 수렴하지 않았습니다. mission.yaml 의 home 좌표가 '
                         '실제 로봇 위치와 맞는지, 맵이 두 로봇에 동일하게 복사됐는지 '
                         '확인하세요.')
        return False

    def _pose_stamped(self, pose2d):
        from pinky_fleet.pose_utils import pose2d_to_pose_stamped
        return pose2d_to_pose_stamped(
            pose2d, self.map_frame, self.nav.get_clock().now().to_msg())

    # ---- 주행 --------------------------------------------------------------
    def do_goto(self, cmd):
        from rclpy.duration import Duration

        target = Pose2D.from_dict(cmd['pose'])
        tag = cmd.get('tag', '')
        timeout = float(cmd['timeout_sec'])

        if not self.nav.goToPose(self._pose_stamped(target)):
            self.emit(P.EVT_FAILED, tag=tag, reason='goToPose 가 목표를 거부했습니다.')
            return
        self.emit(P.EVT_ACCEPTED, tag=tag)

        last_fb = 0.0
        canceled_by_timeout = False
        while not self.nav.isTaskComplete():
            if self.check_interrupt():
                self.nav.cancelTask()
                continue

            self.spin(0.05)   # amcl_pose 콜백을 돌려 현재 위치 갱신

            fb = self.nav.getFeedback()
            if fb is None:
                continue
            nav_time = Duration.from_msg(fb.navigation_time)
            secs = nav_time.nanoseconds / 1e9
            if secs > timeout and not canceled_by_timeout:
                canceled_by_timeout = True
                self.log(f'타임아웃 {timeout:.0f}s 초과 -> cancelTask()')
                self.nav.cancelTask()
                continue
            if secs - last_fb >= self.feedback_period:
                last_fb = secs
                if self.last_pose:
                    self.publish_pose(self.last_pose)
                self.emit(P.EVT_FEEDBACK, tag=tag,
                          distance_remaining=float(fb.distance_remaining),
                          nav_time_sec=secs)

        result = self.nav.getResult()
        if self.last_pose:
            self.publish_pose(self.last_pose)
        if result == self.TaskResult.SUCCEEDED:
            self.emit(P.EVT_ARRIVED, tag=tag)
        elif result == self.TaskResult.CANCELED:
            if canceled_by_timeout:
                self.emit(P.EVT_FAILED, tag=tag,
                          reason=f'타임아웃 {timeout:.0f}s 초과로 취소')
            else:
                self.emit(P.EVT_CANCELED, tag=tag)
        else:
            self.emit(P.EVT_FAILED, tag=tag, reason=f'Nav2 결과: {result}')

    def do_cancel(self):
        try:
            self.nav.cancelTask()
        except Exception:
            pass

    def cleanup(self):
        try:
            self.nav.cancelTask()
        except Exception:
            pass
        try:
            self.nav.destroy_node()
        except Exception:
            pass
        super().cleanup()


# ---------------------------------------------------------------------------
# multiprocessing 진입점
# ---------------------------------------------------------------------------
BACKENDS = {'mock': MockWorker, 'turtlesim': TurtlesimWorker, 'nav2': Nav2Worker}


def worker_process(robot, cfg, cmd_q, status_q, backend='nav2', fault_inject=None):
    """multiprocessing.Process 의 target. 19강 publisher_process 와 같은 자리."""
    cls = BACKENDS[backend]
    kwargs = {}
    if backend == 'mock' and fault_inject:
        kwargs.update(fault_inject)
    try:
        worker = cls(robot, cfg, cmd_q, status_q, **kwargs)
    except Exception:
        status_q.put(P.msg(robot['name'], P.EVT_FAILED, reason=traceback.format_exc()))
        status_q.put(P.msg(robot['name'], P.EVT_BYE))
        return
    worker.run()
