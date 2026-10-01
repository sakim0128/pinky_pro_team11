#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""비전 미션 코디네이터 (2026-09-30) — 위치추정 없이 카메라 차선 주행하는 로봇(lane_only)의 시나리오 관제.

RelayFleetCoordinator 를 그대로 물려받아 정지·비상정지·로봇별 정지·재개·제어권·상태 발행·웹 API 는 같고,
경로 배정·구간 예약(위치가 필요하다) 대신 다음을 한다 — 규칙과 계산은 ROS 없는 `pinky_lane_station.vision_mission`:

    시나리오 시작(웹 프리셋·저장 버튼 → {"cmd":"scenario","name":"s1"},
                  웹 직접 설정 → {"cmd":"scenario","custom":{"label":..,"robots":{"pinky1":{"start":"2","goal":"1",..}}}})
      → 로봇마다 JunctionPlan(교차로 고정 동작 · 도착 벽 마커 id/거리) 발행, depart_delay 뒤 LaneCommand START
    10 Hz: LaneCommand CLEARANCE(clear_until_idx 0 = 교차로 대기 · 1 = 통과) — 하트비트 겸용
    LaneStatus JUNCTION_STOP 보고 → 교차로 통행권(먼저 선 로봇, 같은 순간이면 domain_id 작은 쪽).
      시나리오 로봇이 1대면 통행권 없이 로봇이 1 s 정지 뒤 스스로 간다(JunctionPlan.skip_clearance)
    통행권 쥔 로봇이 차선 주행(교차로 뒤 CRUISE)으로 돌아가면 반납 → 다음 로봇 허가
    같은 목적지로 먼저 도착한 로봇이 있으면 뒤 로봇 계획에 arrive_on_obstacle — 그 뒤 장애물 정지가 도착
    로봇이 ARRIVED 를 보고하면 도착, 모두 도착하면 DONE
    위치 표시(천장 카메라 없음): RobotState(frame_id 'odom') + LaneStatus → pinky_lane_station.vision_pose 가 코스
      (vision_course.yaml) 위 위치로 바꾼다. 화면 표시 전용 — /api/fleet/poses (0.2 s 폴링)
    웹에서 만든 시나리오 중 저장한 것은 vision_mission_user.yaml (save_user_scenario / delete_user_scenario)

경로 모드와 다른 점: Route 를 내지 않는다(AUTO_ASSIGN = False — lane_only 로봇은 받아도 무시한다), FleetCommand 는 하트비트만
보낸다(허가 0 에 STOP 을 섞으면 lane_agent 가 START 래치를 푼다).

기동: gateway_web_server.py --vision [vision_mission.yaml]   (기본 pinky_lane_station/config/vision_mission.yaml)
"""

import os
from typing import Any, Dict, Optional

from pinky_fleet_msgs.msg import FleetCommand, RobotState
from pinky_lane_msgs.msg import JunctionPlan, LaneCommand, LaneStatus
from pinky_lane_station.vision_mission import (CUSTOM_NAME, ScenarioRun, VisionConfigError, course_dict,
                                               default_user_path, delete_user_scenario, load_user_scenarios,
                                               load_vision_config, save_user_scenario, scenario_from_dict,
                                               scenario_summary)
from pinky_lane_station.vision_pose import CourseError, PoseTracker, default_course_path, load_vision_course

from .fleet_coordinator import (MISSION_DONE, MISSION_ESTOP, MISSION_IDLE, MISSION_RUNNING, MISSION_STOPPED,
                                ROUTE_QOS, RelayFleetCoordinator, _locked)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_VISION_CONFIG = os.path.join(REPO_ROOT, 'pinky_lane_station', 'config', 'vision_mission.yaml')


class VisionFleetCoordinator(RelayFleetCoordinator):
    AUTO_ASSIGN = False

    def __init__(self, vision_config: Optional[str] = None, user_scenarios: Optional[str] = None):
        super().__init__()
        self.vision_config_path = vision_config or DEFAULT_VISION_CONFIG
        self.vision_cfg = load_vision_config(self.vision_config_path)
        self.vision_user_path = user_scenarios or os.environ.get('PINKY_VISION_USER_SCENARIOS') \
            or default_user_path(self.vision_config_path)
        for err in load_user_scenarios(self.vision_cfg, self.vision_user_path):
            self.get_logger().warn(f"저장 시나리오 건너뜀 — {err}")
        missing = [n for n in self.vision_cfg.robots if n not in self.robots]
        if missing:
            raise VisionConfigError(f'비전 설정의 로봇 {missing} 이 플릿(프로파일 미션)에 없다 — 토픽을 만들 수 없다')
        for name, ctx in self.robots.items():
            if name in self.vision_cfg.robots:
                ctx.domain_id = self.vision_cfg.robots[name]
            ctx.route = None                                    # 경로 모드 흔적을 남기지 않는다
            self.reservation.remove(name)
        if self.mission_state != MISSION_ESTOP:
            self.mission_state = MISSION_IDLE                   # 저장된 RUNNING 으로 되살아나 출발하지 않게
        self.vision_run: Optional[ScenarioRun] = None
        self.vision_scenario: Optional[str] = None
        self.vision_custom = None                               # 마지막 직접 설정(저장 안 한 것) — "주행 시작" 다시 돌리기용
        self.vision_error = ''
        self._vision_seq = int(max([c.route_seq for c in self.robots.values()] + [0])) + 100
        self.plan_pubs = {name: self.create_publisher(JunctionPlan, f'/{name}/junction_plan', ROUTE_QOS)
                          for name in self.robots}
        # 위치 표시 (천장 카메라 없음 — odom + 코스 모양 + 확실한 지점에서 다시 맞추기)
        self.vision_course = None
        self.vision_course_error = ''
        self.vision_pose: Dict[str, PoseTracker] = {}
        course_path = os.environ.get('PINKY_VISION_COURSE') or default_course_path(self.vision_config_path)
        try:
            self.vision_course = load_vision_course(course_path)
            self.vision_course_image = os.path.join(REPO_ROOT, self.vision_course.image_file)
        except (OSError, CourseError, ValueError, KeyError, TypeError) as exc:
            self.vision_course_error = f'{course_path}: {exc}'
            self.vision_course_image = ''
            self.get_logger().warn(f"위치 표시 끔 — 코스 파일 {self.vision_course_error}")
        self.get_logger().info(
            f"VisionFleetCoordinator: {self.vision_config_path} — 시나리오 {list(self.vision_cfg.scenarios)}")

    # ------------------------------------------------------------------ 시나리오

    def _publish_plan(self, name: str) -> None:
        run = self.vision_run
        if run is None or name not in run.seq or name not in self.plan_pubs:
            return
        f = run.plan_fields(name)
        msg = JunctionPlan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_name = f['robot_name']
        msg.seq = int(f['seq'])
        msg.scenario = f['scenario']
        msg.maneuver = f['maneuver']
        msg.step_kind = list(f['step_kind'])
        msg.step_value = [float(v) for v in f['step_value']]
        msg.linear_speed = float(f['linear_speed'])
        msg.angular_speed = float(f['angular_speed'])
        msg.stop_line_count = int(f['stop_line_count'])
        msg.goal_marker_id = int(f['goal_marker_id'])
        msg.arrive_distance = float(f['arrive_distance'])
        msg.skip_clearance = bool(f['skip_clearance'])
        msg.arrive_on_obstacle = bool(f['arrive_on_obstacle'])
        self.plan_pubs[name].publish(msg)

    @_locked
    def start_scenario(self, name: str, custom: Optional[dict] = None) -> bool:
        """프리셋·저장 시나리오(name) 또는 웹 직접 설정(custom dict, 저장 안 함 — 이름 'custom')을 시작."""
        self.vision_error = ''
        if self.estop_latched:
            self.vision_error = 'E-STOP 이 걸려 있다 — 재개 먼저'
            self.get_logger().warn("⛔ scenario refused — E-STOP latched (use fleet resume)")
            return False
        if self.mission_state == MISSION_RUNNING and self.vision_run is not None and not self.vision_run.done:
            self.vision_error = '시나리오가 도는 중이다 — 정지 먼저'
            self.get_logger().warn("⛔ scenario refused — a scenario is running (stop it first)")
            return False
        if custom is not None:
            try:
                scenario = scenario_from_dict(self.vision_cfg, CUSTOM_NAME, custom, 'custom')
            except (VisionConfigError, TypeError, ValueError, AttributeError) as exc:
                self.vision_error = f'직접 설정 거절 — {exc}'
                self.get_logger().error(f"scenario refused — custom: {exc}")
                return False
            name = CUSTOM_NAME
        elif name == CUSTOM_NAME and self.vision_custom is not None:
            scenario = self.vision_custom
        elif name in self.vision_cfg.scenarios:
            scenario = self.vision_cfg.scenarios[name]
        else:
            self.vision_error = f'그런 시나리오가 없다: {name!r}'
            self.get_logger().error(f"scenario refused — unknown {name!r}")
            return False
        missing = [n for n in scenario.robots if n not in self.robots]
        if missing:
            self.vision_error = f'플릿에 없는 로봇 {missing}'
            self.get_logger().error(f"scenario refused — robots {missing} not in fleet")
            return False
        if scenario.source == 'custom':
            self.vision_custom = scenario
        now = self._now()
        run = ScenarioRun(self.vision_cfg, scenario, t0=now, first_seq=self._vision_seq + 1)
        self._vision_seq += len(run.robots) + 1
        self.vision_run, self.vision_scenario = run, name
        for rname, ctx in self.robots.items():
            ctx.arrived = ctx.arrival_confirmed = False
            ctx.clear_until_idx = 0
            ctx.start_acknowledged = ctx.start_gave_up = ctx.start_armed = False
            ctx.start_retry_count = 0
            plan = run.scenario.robots.get(rname)
            if plan is None:
                ctx.start_node = ctx.goal_node = ''
                continue
            ctx.route_seq = run.seq[rname]
            ctx.start_node, ctx.goal_node = plan.start, plan.goal
            self._publish_plan(rname)
        self._start_pose_trackers(run, now)
        self.mission_state = MISSION_RUNNING
        self._save_control_state()
        self.get_logger().info(f"🚦 scenario {name} started: "
                               + ', '.join(f"{n} {p.start}→{p.goal} ({p.direction or p.maneuver}, +{p.depart_delay:.0f}s)"
                                           for n, p in run.scenario.robots.items())
                               + (' — 로봇 1대: 교차로 허가 없이 1 s 정지 뒤 출발' if run.skip_clearance else ''))
        return True

    def _start_pose_trackers(self, run: ScenarioRun, now: float) -> None:
        """시나리오 로봇마다 출발 지점에 위치 추정기를 놓는다 (표시 전용)."""
        self.vision_pose = {}
        if self.vision_course is None:
            return
        for rname, plan in run.scenario.robots.items():
            try:
                tr = PoseTracker(self.vision_course.route(plan.start, plan.goal), now)
            except CourseError as exc:
                self.get_logger().warn(f"[{rname}] 위치 표시 없음 — {exc}")
                continue
            ctx = self.robots.get(rname)
            st = ctx.state if ctx is not None else None
            if st is not None and getattr(st.header, 'frame_id', '') == 'odom':
                tr.feed_odom(st.x, st.y, st.yaw, now)            # 지금 odom 을 출발 자세의 기준으로
            self.vision_pose[rname] = tr

    @_locked
    def vision_poses(self) -> Dict[str, Any]:
        """/api/fleet/poses — 가벼운 위치만 (0.2 s 폴링)."""
        now = self._now()
        robots = {}
        for name, tr in self.vision_pose.items():
            ctx = self.robots.get(name)
            p = tr.pose()
            p['age'] = round(now - ctx.state_time, 2) if ctx is not None and ctx.state is not None else None
            robots[name] = p
        return {'t': now, 'robots': robots, 'error': self.vision_course_error}

    # ------------------------------------------------------------------ 저장 시나리오 (HTTP 쓰레드에서 직접 부른다)

    @_locked
    def save_user_scenario(self, name: str, raw: dict):
        """웹 직접 설정을 이름 붙여 저장 → 프리셋 목록에 뜬다. (ok, 메시지)"""
        self.vision_cfg.user_path = self.vision_user_path
        try:
            scn = save_user_scenario(self.vision_cfg, name, raw)
        except (VisionConfigError, TypeError, ValueError, AttributeError) as exc:
            return False, str(exc)
        except OSError as exc:
            return False, f'파일 쓰기 실패 {self.vision_user_path}: {exc}'
        self.get_logger().info(f"💾 scenario saved: {scn.name} → {self.vision_user_path}")
        return True, scn.name

    @_locked
    def delete_user_scenario(self, name: str):
        self.vision_cfg.user_path = self.vision_user_path
        if self.vision_scenario == name and self.mission_state == MISSION_RUNNING:
            return False, '도는 중인 시나리오는 지울 수 없다'
        try:
            delete_user_scenario(self.vision_cfg, name)
        except VisionConfigError as exc:
            return False, str(exc)
        except OSError as exc:
            return False, f'파일 쓰기 실패 {self.vision_user_path}: {exc}'
        self.get_logger().info(f"🗑 scenario deleted: {name}")
        return True, name

    def start_fleet(self) -> bool:
        """웹 "주행 시작" = 마지막으로 고른 시나리오를 다시 시작. 고른 적이 없으면 거절(시나리오 버튼으로 시작한다)."""
        if not self.vision_scenario:
            self.get_logger().warn("⛔ start refused — vision mode: use a scenario button")
            return False
        return self.start_scenario(self.vision_scenario)

    @_locked
    def resume_fleet(self):
        ok = super().resume_fleet()
        if self.mission_state == MISSION_RUNNING and self.vision_run is not None:
            # 정지(STOP)가 로봇의 START 래치를 풀었다 — 출발한 로봇에 START 를 다시 보낸다(교차로 대기·동작은 로봇이 이어서 한다)
            now = self._now()
            for name in self.vision_run.robots:
                ctx = self.robots.get(name)
                if ctx is not None and not ctx.held and self.vision_run.due(name, now) and not ctx.arrived:
                    self._arm_start(ctx, now)
        return ok

    @_locked
    def resume_robot(self, robot_name: str) -> bool:
        ok = super().resume_robot(robot_name)
        ctx = self.robots.get(robot_name)
        run = self.vision_run
        if (ok and ctx is not None and run is not None and self.mission_state == MISSION_RUNNING
                and robot_name in run.seq and run.due(robot_name, self._now()) and not ctx.arrived):
            self._arm_start(ctx)                                 # 로봇별 정지(STOP)가 풀었던 START 를 다시
        return ok

    # ------------------------------------------------------------------ 콜백

    @_locked
    def _cb_control(self, msg):
        import json
        try:
            data = json.loads(msg.data)
        except Exception:                                        # noqa: BLE001 — 부모가 같은 방식으로 로그를 남긴다
            return super()._cb_control(msg)
        if str(data.get('cmd', '')).lower() != 'scenario':
            return super()._cb_control(msg)
        custom = data.get('custom')
        ok = self.start_scenario(str(data.get('name', '')), custom if isinstance(custom, dict) else None)
        self.control_seq += 1
        self.last_control = {'seq': self.control_seq, 'cmd': 'scenario', 'robot': '',
                             'ok': bool(ok), 'mission_state': self.mission_state, 'error': self.vision_error}

    @_locked
    def _cb_robot_state(self, name: str, msg: RobotState):
        super()._cb_robot_state(name, msg)
        tr = self.vision_pose.get(name)
        if tr is not None and getattr(msg.header, 'frame_id', '') == 'odom':
            tr.feed_odom(msg.x, msg.y, msg.yaw, self._now())

    @_locked
    def _cb_lane_status(self, name: str, msg: LaneStatus):
        ctx = self.robots.get(name)
        if not ctx:
            return
        ctx.lane_status = msg
        ctx.lane_status_time = self._now()
        tr = self.vision_pose.get(name)
        if tr is not None and msg.route_seq == ctx.route_seq:
            tr.feed_status(msg.drive_state, msg.edge_id, getattr(msg, 'state_reason', ''), ctx.lane_status_time)
        if (msg.drive_state == LaneStatus.DRIVE_ESTOP and not self.estop_latched and not self._estop_authority):
            self.estop_latched = True                            # S6 와 같다: 재시작 직후 로봇이 쥔 ESTOP 을 플릿 래치로
            self._pre_estop_state = self.mission_state
            self.mission_state = MISSION_ESTOP
            self.get_logger().warn(f"🚨 [{name}] reports DRIVE_ESTOP — fleet E-STOP latch restored (use fleet resume)")
        self._apply_link_lost_hold(name, ctx, ctx.lane_status_time)
        run = self.vision_run
        if run is None or msg.route_seq != ctx.route_seq or name not in run.seq:
            self._save_control_state()
            return
        if msg.drive_state not in (LaneStatus.DRIVE_IDLE, LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_LINK_LOST):
            if not ctx.start_acknowledged:
                ctx.start_acknowledged = True
                self.get_logger().info(f"[{name}] START acknowledged (drive_state={msg.drive_state})")
        holder_before = run.arbiter.holder
        run.on_status(name, msg.drive_state, msg.route_seq, msg.edge_id, ctx.lane_status_time,
                      getattr(msg, 'state_reason', ''))
        if holder_before == name and run.arbiter.holder != name:
            self.get_logger().info(f"[{name}] 교차로 통과 — 통행권 반납")
        if msg.drive_state == LaneStatus.DRIVE_ARRIVED and not ctx.arrived:
            ctx.arrived = ctx.arrival_confirmed = True
            self.get_logger().info(f"🏁 [{name}] 도착 — {getattr(msg, 'state_reason', '') or run.scenario.robots[name].goal}")
        self._save_control_state()

    # ------------------------------------------------------------------ 10 Hz

    @_locked
    def _loop_tick(self):
        now = self._now()
        self.global_seq += 1
        for ctx in self.robots.values():
            ctx.is_stale = ctx.state is None or (now - ctx.state_time > self.state_timeout_sec)
        run = self.vision_run
        warnings = []
        if run is not None and self.mission_state == MISSION_RUNNING:
            for name in run.tick(now):
                self._publish_plan(name)                         # 통행권 순서 정지선 수 · 같은 목적지 도착 표시
                if run.arrive_on_obstacle.get(name):
                    self.get_logger().info(f"[{name}] 같은 목적지 앞 로봇 도착 — 장애물 정지를 도착으로")
                else:
                    self.get_logger().info(f"[{name}] 교차로 통행권 — 정지선 {run.stop_lines[name]}번째에서 도착")
            for name, ctx in self.robots.items():
                ctx.clear_until_idx = run.clearance(name) if name in run.seq else 0
                if name in run.seq and ctx.is_stale and not ctx.arrived:
                    warnings.append(f"STALE: {name} — 상태 보고 {self.state_timeout_sec:.0f}s 끊김")
            if run.done:
                self.mission_state = MISSION_DONE
                self.get_logger().info(f"🏁 scenario {run.scenario.name} DONE — 모두 도착")
        elif self.mission_state in (MISSION_IDLE, MISSION_ESTOP):
            for ctx in self.robots.values():
                ctx.clear_until_idx = 0
        self.last_warning = ' | '.join(warnings)
        self._save_control_state()
        self._publish_vision_commands(now)

    def _publish_vision_commands(self, now: float) -> None:
        self._retry_resumes(now)
        run = self.vision_run
        for name, ctx in self.robots.items():
            if ctx.held and self.mission_state != MISSION_ESTOP:
                self._send_hold(name, ctx)
                continue
            fleet_pub = self.fleet_cmd_pubs.get(name)
            if fleet_pub:
                hb = FleetCommand()
                hb.command = FleetCommand.CMD_HEARTBEAT              # 하트비트만 — STOP 을 섞으면 lane_agent 가 START 를 푼다
                fleet_pub.publish(hb)
            pub = self.lane_cmd_pubs.get(name)
            if not pub:
                continue
            m = LaneCommand()
            m.route_seq = ctx.route_seq
            m.max_linear_vel = float(self.config.get('defaults', {}).get('max_linear_vel', 0.15))
            m.max_angular_vel = float(self.config.get('defaults', {}).get('max_angular_vel', 1.2))
            if self.mission_state == MISSION_ESTOP:
                m.command = LaneCommand.CMD_ESTOP
            elif self.mission_state in (MISSION_STOPPED, MISSION_DONE):
                m.command = LaneCommand.CMD_STOP
                m.clear_until_idx = ctx.clear_until_idx
            elif self.mission_state == MISSION_RUNNING and run is not None and name in run.seq:
                if not run.due(name, now):
                    m.command = LaneCommand.CMD_HEARTBEAT            # 출발 전 — 로봇은 IDLE 로 기다린다
                elif not ctx.start_acknowledged and not ctx.arrived:
                    if not ctx.start_armed:
                        self._arm_start(ctx, now)
                    if ctx.start_retry_count > 0 and now - ctx.last_start_time >= self.start_retry_interval:
                        ctx.start_retry_count -= 1
                        ctx.last_start_time = now
                        start = LaneCommand()
                        start.command = LaneCommand.CMD_START
                        start.route_seq = ctx.route_seq
                        pub.publish(start)
                        self.get_logger().info(f"[{name}] CMD_START (retries left {ctx.start_retry_count})")
                    elif ctx.start_retry_count <= 0 and not ctx.start_gave_up:
                        ctx.start_gave_up = True
                        self.get_logger().error(f"[{name}] START not acknowledged — robot reports no driving state")
                    m.command = LaneCommand.CMD_CLEARANCE
                    m.clear_until_idx = ctx.clear_until_idx
                else:
                    m.command = LaneCommand.CMD_CLEARANCE
                    m.clear_until_idx = ctx.clear_until_idx
            else:
                m.command = LaneCommand.CMD_HEARTBEAT
            pub.publish(m)

    # ------------------------------------------------------------------ 상태

    @_locked
    def get_fleet_status_dict(self) -> Dict[str, Any]:
        out = super().get_fleet_status_dict()
        now = self._now()
        run = self.vision_run
        out['mode'] = 'vision'
        out['vision'] = {
            'config': self.vision_config_path,
            'user_config': self.vision_user_path,
            'scenarios': [scenario_summary(s) for s in self.vision_cfg.scenarios.values()],
            'course': course_dict(self.vision_cfg),
            'active': self.vision_scenario,
            'error': self.vision_error,
            'run': run.status(now) if run is not None else None,
            'map': dict(self.vision_course.to_dict(), image_url='/api/fleet/vision_map.png')
            if self.vision_course is not None else None,
            'map_error': self.vision_course_error,
        }
        return out
