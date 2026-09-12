"""Pinky Pro 2대 순차 왕복 미션 오케스트레이터 (부모 프로세스).

    목적지 1개 지정 -> 1호기가 목적지 찍고 출발지 복귀 -> 그 다음 2호기가 동일 수행

이 프로세스는 **ROS를 전혀 쓰지 않는다.**
rclpy.init() 은 프로세스당 한 번뿐이라(19강 s24) 도메인마다 프로세스를 나눠야 하는데,
부모가 먼저 rclpy.init() 을 해버리면 자식과 컨텍스트가 꼬인다. 그래서 부모는 순수
파이썬 상태머신으로 남기고, ROS에 붙는 일은 전부 자식 프로세스가 한다:

    자식 A : domain 10  Pinky-1  (domain_worker)
    자식 B : domain 11  Pinky-2  (domain_worker)
    자식 C : domain 20  관제 콘솔 (console_node)   -- 브리지가 중계한 토픽만 본다

프로세스 간 통신은 multiprocessing.Queue 로 한다(서로 도메인이 달라 ROS로는 못 한다).
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import queue
import sys
import time

from pinky_fleet import protocol as P
from pinky_fleet.mission_config import Pose2D, load_mission_config
from pinky_fleet.pose_utils import normalize_deg


# ---------------------------------------------------------------------------
def _fmt(pose: Pose2D) -> str:
    return f'x={pose.x:.3f} y={pose.y:.3f} yaw={pose.yaw_deg:.1f}deg'


class FleetMaster:
    def __init__(self, cfg, args):
        self.cfg = cfg
        self.args = args
        self.backend = args.backend
        self.use_console = (not args.no_console) and (self.backend != 'mock')
        self.state = P.MissionState.INIT

        self.status_q = mp.Queue()
        self.inbox = {}          # robot -> 아직 소비하지 않은 결과 이벤트
        self.cmd_qs = {}
        self.procs = {}
        self.to_console_q = None
        self.from_console_q = None
        self.console_proc = None
        self.goal = None
        self.aborted_reason = ''

    # ---- 상태 ---------------------------------------------------------------
    def set_state(self, state, detail=''):
        self.state = state
        line = f'[STATE] {state.value}' + (f'  {detail}' if detail else '')
        print(line, flush=True)
        if self.to_console_q is not None:
            self.to_console_q.put(
                {'op': P.CON_SET_STATE, 'state': state.value, 'detail': detail})

    # ---- 프로세스 기동 -------------------------------------------------------
    def start(self):
        from pinky_fleet.domain_worker import worker_process

        worker_cfg = {
            'map_frame': self.cfg.map_frame,
            'feedback_period_sec': self.cfg.mission.feedback_period_sec,
            'localization': self.cfg.localization.as_dict(),
        }
        fault = {}
        if self.args.inject_fail:
            fault['fail_on'] = self.args.inject_fail
        if self.args.inject_hang:
            fault['hang_on'] = self.args.inject_hang

        for spec in self.cfg.ordered_robots():
            q = mp.Queue()
            self.cmd_qs[spec.name] = q
            p = mp.Process(
                target=worker_process,
                args=(spec.as_dict(), worker_cfg, q, self.status_q, self.backend, fault),
                name=f'worker_{spec.name}', daemon=True)
            p.start()
            self.procs[spec.name] = p
            print(f'[SPAWN] {spec.name} -> domain {spec.domain_id} '
                  f'(backend={self.backend}, pid={p.pid})', flush=True)

        if self.use_console:
            from pinky_fleet.console_node import config_to_dict, console_process
            self.to_console_q = mp.Queue()
            self.from_console_q = mp.Queue()
            cfg_dict = config_to_dict(self.cfg)
            self.console_proc = mp.Process(
                target=console_process,
                args=(cfg_dict, self.to_console_q, self.from_console_q),
                name='fleet_console', daemon=True)
            self.console_proc.start()
            print(f'[SPAWN] console -> domain {self.cfg.control_domain_id} '
                  f'(pid={self.console_proc.pid})', flush=True)

    def send(self, robot_name, **cmd):
        self.cmd_qs[robot_name].put(cmd)

    def broadcast(self, **cmd):
        for q in self.cmd_qs.values():
            q.put(cmd)

    # ---- 이벤트 대기 ---------------------------------------------------------
    # 워커 두 대가 하나의 status_q 를 공유한다. 준비 단계처럼 두 대가 동시에 진행될 때는
    # 기다리는 로봇보다 다른 로봇의 이벤트가 먼저 도착할 수 있으므로, 결과 이벤트는
    # 버려지지 않도록 로봇별로 쌓아 둔다.
    TERMINAL_EVENTS = (P.EVT_READY, P.EVT_ARRIVED, P.EVT_FAILED, P.EVT_CANCELED, P.EVT_BYE)

    def _pump(self, timeout):
        """status_q 에서 하나 꺼내 출력하고, 결과 이벤트면 보관한다."""
        try:
            m = self.status_q.get(timeout=timeout)
        except queue.Empty:
            return False
        self._trace(m)
        if m['event'] in self.TERMINAL_EVENTS:
            self.inbox.setdefault(m['robot'], []).append(m)
        return True

    def _take(self, robot_name, wanted):
        """보관해 둔 이벤트에서 조건에 맞는 것을 꺼낸다. 없으면 None."""
        for who, msgs in self.inbox.items():
            for i, m in enumerate(msgs):
                hit = (m['event'] == P.EVT_FAILED           # 누구의 실패든 전체 중단 사유
                       or (who == robot_name
                           and m['event'] in tuple(wanted) + (P.EVT_BYE,)))
                if hit:
                    del msgs[i]
                    if m['event'] == P.EVT_BYE:
                        return P.msg(robot_name, P.EVT_FAILED,
                                     reason='워커가 결과를 내지 않고 종료되었습니다.')
                    return m
        return None

    def wait_for(self, robot_name, wanted, timeout):
        """robot_name 이 wanted 중 하나를 보낼 때까지 대기.

        기다리는 대상이 아닌 로봇이 실패하거나 죽어도 즉시 돌려준다 — 한쪽이 무너진 채
        다른 쪽을 계속 주행시키면 안 된다.
        """
        deadline = time.time() + timeout
        while True:
            hit = self._take(robot_name, wanted)
            if hit is not None:
                return hit
            if time.time() >= deadline:
                break
            if not self._pump(0.2) and not self.procs[robot_name].is_alive():
                return P.msg(robot_name, P.EVT_FAILED,
                             reason='워커 프로세스가 예기치 않게 종료되었습니다.')
        return P.msg(robot_name, P.EVT_FAILED,
                     reason=f'부모 대기 타임아웃 ({timeout:.0f}s). '
                            '워커가 응답하지 않습니다.')

    def _trace(self, m):
        ev = m['event']
        who = m['robot']
        if ev == P.EVT_LOG:
            print(f'  [{who}] {m["text"]}', flush=True)
        elif ev == P.EVT_FEEDBACK:
            print(f'  [{who}] 남은거리 {m["distance_remaining"]:.2f} m, '
                  f'경과 {m["nav_time_sec"]:.1f} s', flush=True)
        elif ev == P.EVT_POSE:
            pass  # 위치는 관제 화면(브리지)에서 본다
        elif ev == P.EVT_ACCEPTED:
            print(f'  [{who}] 목표 수락됨', flush=True)

    # ---- 준비 ---------------------------------------------------------------
    def prepare_all(self):
        self.set_state(P.MissionState.WAIT_NAV2, '두 로봇 Nav2 활성화 + 초기 위치 설정')
        for spec in self.cfg.ordered_robots():
            self.send(spec.name, op=P.CMD_PREPARE,
                      nav2_activate_timeout_sec=self.cfg.mission.nav2_activate_timeout_sec)
        for spec in self.cfg.ordered_robots():
            m = self.wait_for(
                spec.name, {P.EVT_READY, P.EVT_FAILED},
                self.cfg.mission.nav2_activate_timeout_sec
                + self.cfg.localization.initial_pose_timeout_sec + 15.0)
            if m['event'] != P.EVT_READY:
                self.abort(f'{m["robot"]} 준비 실패: {m.get("reason", "")}')
                return False
            print(f'  [{spec.name}] 준비 완료', flush=True)
        return True

    # ---- 목적지 확보 ---------------------------------------------------------
    def acquire_goal(self):
        if self.args.goal:
            x, y, yaw = self.args.goal
            self.goal = Pose2D(x, y, normalize_deg(yaw))
            print(f'[GOAL] 인자로 지정됨: {_fmt(self.goal)}', flush=True)
        elif self.use_console:
            need = 2 if self.cfg.goal_input.mode == 'two_click' else 1
            self.set_state(
                P.MissionState.WAIT_GOAL,
                f'RViz "Publish Point" 로 {need}번 클릭'
                + (' (1: 목적지, 2: 바라볼 방향)' if need == 2 else ''))
            while True:
                try:
                    m = self.from_console_q.get(timeout=0.5)
                except queue.Empty:
                    if self.console_proc is not None and not self.console_proc.is_alive():
                        self.abort('관제 콘솔 프로세스가 종료되었습니다.')
                        return False
                    continue
                if m.get('op') == P.CON_GOAL_PICKED:
                    self.goal = Pose2D.from_dict(m['pose'])
                    break
        else:
            self.set_state(P.MissionState.WAIT_GOAL, '터미널 입력')
            raw = input('목적지를 "x y yaw_deg" 로 입력: ').split()
            self.goal = Pose2D(float(raw[0]), float(raw[1]),
                               float(raw[2]) if len(raw) > 2 else 0.0)

        print(f'[GOAL] {_fmt(self.goal)}', flush=True)
        if self.to_console_q is not None:
            self.to_console_q.put({'op': P.CON_SET_GOAL, 'pose': self.goal.as_dict()})

        if not self.args.yes:
            print('  이 좌표로 진행하려면 Enter, 취소하려면 Ctrl-C', flush=True)
            try:
                input()
            except EOFError:
                pass
        return True

    # ---- 한 구간 주행 --------------------------------------------------------
    def leg(self, spec, target, tag, timeout, state, detail):
        self.set_state(state, detail)
        self.send(spec.name, op=P.CMD_GOTO, pose=target.as_dict(),
                  timeout_sec=timeout, tag=tag)
        m = self.wait_for(
            spec.name,
            {P.EVT_ARRIVED, P.EVT_FAILED, P.EVT_CANCELED},
            timeout + 30.0)
        if m['event'] == P.EVT_ARRIVED:
            print(f'  [{spec.name}] 도착: {_fmt(target)}', flush=True)
            return True
        reason = m.get('reason', m['event'])
        self.abort(f'{m["robot"]} {tag} 실패: {reason}')
        return False

    # ---- 미션 ---------------------------------------------------------------
    def run_mission(self):
        for spec in self.cfg.ordered_robots():
            if not self.leg(spec, self.goal, f'{spec.name}:goal',
                            self.cfg.mission.goal_timeout_sec,
                            P.MissionState.GOING_TO_GOAL,
                            f'{spec.name} -> 목적지'):
                return False
            if not self.leg(spec, spec.home, f'{spec.name}:home',
                            self.cfg.mission.home_timeout_sec,
                            P.MissionState.GOING_HOME,
                            f'{spec.name} -> 출발지 복귀'):
                return False
            self.set_state(P.MissionState.SETTLING,
                           f'{spec.name} 복귀 완료, {self.cfg.mission.settle_sec:.1f}s 안정화')
            time.sleep(self.cfg.mission.settle_sec)
        self.set_state(P.MissionState.DONE, '전체 미션 완료')
        return True

    # ---- 종료 ---------------------------------------------------------------
    def abort(self, reason):
        self.aborted_reason = reason
        print(f'[ABORT] {reason}', file=sys.stderr, flush=True)
        self.set_state(P.MissionState.ABORT, reason)
        self.broadcast(op=P.CMD_CANCEL)
        time.sleep(1.0)

    def shutdown(self):
        self.broadcast(op=P.CMD_SHUTDOWN)
        if self.to_console_q is not None:
            self.to_console_q.put({'op': P.CON_SHUTDOWN})
        deadline = time.time() + 5.0
        for p in list(self.procs.values()) + (
                [self.console_proc] if self.console_proc else []):
            p.join(max(0.1, deadline - time.time()))
            if p.is_alive():
                p.terminate()


# ---------------------------------------------------------------------------
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        prog='fleet_master',
        description='Pinky Pro 2대 순차 왕복 관제')
    ap.add_argument('--config', default='', help='mission.yaml 경로 (기본: 패키지 share)')
    ap.add_argument('--backend', default='nav2', choices=['nav2', 'turtlesim', 'mock'],
                    help='nav2=실제 미션, turtlesim=L1 배선검증, mock=L0 로직검증')
    ap.add_argument('--dry-run', action='store_true',
                    help='--backend mock --no-console --yes 와 동일 (L0)')
    ap.add_argument('--no-console', action='store_true',
                    help='관제 도메인 콘솔 프로세스를 띄우지 않는다')
    ap.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'),
                    help='RViz 클릭 대신 목적지를 인자로 지정')
    ap.add_argument('--yes', '-y', action='store_true', help='확인 Enter 생략')
    ap.add_argument('--inject-fail', nargs='*', default=[], metavar='TAG',
                    help='[mock 전용] 해당 tag 구간을 강제 실패 (예: pinky1:home)')
    ap.add_argument('--inject-hang', nargs='*', default=[], metavar='TAG',
                    help='[mock 전용] 해당 tag 구간을 멈춰 타임아웃 검증')
    args = ap.parse_args(argv)
    if args.dry_run:
        args.backend = 'mock'
        args.no_console = True
        args.yes = True
        if not args.goal:
            args.goal = [2.0, 1.0, 90.0]
    return args


def main(argv=None):
    # ★ fork 가 아니라 spawn. 자식이 부모의 메모리(그리고 혹시 있을 DDS 상태)를
    #   물려받지 않고 깨끗한 프로세스에서 rclpy.init(domain_id=) 를 하도록 강제한다.
    try:
        mp.set_start_method('spawn')
    except RuntimeError:
        pass

    args = parse_args(argv)
    cfg = load_mission_config(args.config)
    print(f'[CONFIG] {cfg.source_path}', flush=True)
    robot_desc = ', '.join(
        f'{r.name}(domain {r.domain_id}, home {_fmt(r.home)})' for r in cfg.ordered_robots())
    print(f'[CONFIG] 관제 도메인={cfg.control_domain_id}, 로봇={robot_desc}', flush=True)

    master = FleetMaster(cfg, args)
    ok = False
    try:
        master.start()
        if master.prepare_all() and master.acquire_goal():
            ok = master.run_mission()
    except KeyboardInterrupt:
        master.abort('사용자 Ctrl-C')
    finally:
        master.shutdown()

    if not ok:
        print(f'[RESULT] 실패: {master.aborted_reason}', file=sys.stderr, flush=True)
        return 1
    print('[RESULT] 성공 — 두 로봇 모두 목적지 왕복 완료', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
