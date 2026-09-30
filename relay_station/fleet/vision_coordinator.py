#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""비전 미션 코디네이터 (2026-09-30) — 위치추정 없이 카메라 차선 주행하는 로봇(lane_only)의 시나리오 관제.

RelayFleetCoordinator 를 그대로 물려받아 정지·비상정지·로봇별 정지·재개·제어권·상태 발행·웹 API 는 같고,
경로 배정·구간 예약(위치가 필요하다) 대신 다음을 한다 — 규칙과 계산은 ROS 없는 `pinky_lane_station.vision_mission`:

    시나리오 시작(웹 "시나리오 N 시작" → {"cmd":"scenario","name":"s1"})
      → 로봇마다 JunctionPlan(교차로 고정 동작 · 도착 정지선 수) 발행, depart_delay 뒤 LaneCommand START
    10 Hz: LaneCommand CLEARANCE(clear_until_idx 0 = 교차로 대기 · 1 = 통과) — 하트비트 겸용
    LaneStatus JUNCTION_STOP 보고 → 교차로 통행권(먼저 선 로봇, 같은 순간이면 domain_id 작은 쪽)
    통행권 쥔 로봇이 차선 주행(교차로 뒤 CRUISE)으로 돌아가면 반납 → 다음 로봇 허가
    로봇이 ARRIVED 를 보고하면 도착, 모두 도착하면 DONE

경로 모드와 다른 점: Route 를 내지 않는다(AUTO_ASSIGN = False — lane_only 로봇은 받아도 무시한다), FleetCommand 는 하트비트만
보낸다(허가 0 에 STOP 을 섞으면 lane_agent 가 START 래치를 푼다).

기동: gateway_web_server.py --vision [vision_mission.yaml]   (기본 pinky_lane_station/config/vision_mission.yaml)
"""

import os
from typing import Any, Dict, Optional

from pinky_fleet_msgs.msg import FleetCommand
from pinky_lane_msgs.msg import JunctionPlan, LaneCommand, LaneStatus
from pinky_lane_station.vision_mission import ScenarioRun, VisionConfigError, load_vision_config

from .fleet_coordinator import (MISSION_DONE, MISSION_ESTOP, MISSION_IDLE, MISSION_RUNNING, MISSION_STOPPED,
                                ROUTE_QOS, RelayFleetCoordinator, _locked)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_VISION_CONFIG = os.path.join(REPO_ROOT, 'pinky_lane_station', 'config', 'vision_mission.yaml')


class VisionFleetCoordinator(RelayFleetCoordinator):
    AUTO_ASSIGN = False

    def __init__(self, vision_config: Optional[str] = None):
        super().__init__()
        self.vision_config_path = vision_config or DEFAULT_VISION_CONFIG
        self.vision_cfg = load_vision_config(self.vision_config_path)
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
        self._vision_seq = int(max([c.route_seq for c in self.robots.values()] + [0])) + 100
        self.plan_pubs = {name: self.create_publisher(JunctionPlan, f'/{name}/junction_plan', ROUTE_QOS)
                          for name in self.robots}
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
        self.plan_pubs[name].publish(msg)

    @_locked
    def start_scenario(self, name: str) -> bool:
        if self.estop_latched:
            self.get_logger().warn("⛔ scenario refused — E-STOP latched (use fleet resume)")
            return False
        if name not in self.vision_cfg.scenarios:
            self.get_logger().error(f"scenario refused — unknown {name!r}")
            return False
        if self.mission_state == MISSION_RUNNING and self.vision_run is not None and not self.vision_run.done:
            self.get_logger().warn("⛔ scenario refused — a scenario is running (stop it first)")
            return False
        now = self._now()
        run = ScenarioRun(self.vision_cfg, name, t0=now, first_seq=self._vision_seq + 1)
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
        self.mission_state = MISSION_RUNNING
        self._save_control_state()
        self.get_logger().info(f"🚦 scenario {name} started: "
                               + ', '.join(f"{n} {p.start}→{p.goal} ({p.maneuver}, +{p.depart_delay:.0f}s)"
                                           for n, p in run.scenario.robots.items()))
        return True

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
        ok = self.start_scenario(str(data.get('name', '')))
        self.control_seq += 1
        self.last_control = {'seq': self.control_seq, 'cmd': 'scenario', 'robot': '',
                             'ok': bool(ok), 'mission_state': self.mission_state}

    @_locked
    def _cb_lane_status(self, name: str, msg: LaneStatus):
        ctx = self.robots.get(name)
        if not ctx:
            return
        ctx.lane_status = msg
        ctx.lane_status_time = self._now()
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
        run.on_status(name, msg.drive_state, msg.route_seq, msg.edge_id, ctx.lane_status_time)
        if holder_before == name and run.arbiter.holder != name:
            self.get_logger().info(f"[{name}] 교차로 통과 — 통행권 반납")
        if msg.drive_state == LaneStatus.DRIVE_ARRIVED and not ctx.arrived:
            ctx.arrived = ctx.arrival_confirmed = True
            self.get_logger().info(f"🏁 [{name}] 도착 ({run.stop_lines.get(name)}번째 정지선)")
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
                self._publish_plan(name)                         # 통행권 순서로 정해진 정지선 수
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
            'scenarios': [{'name': s.name, 'label': s.label,
                           'robots': {n: {'start': p.start, 'goal': p.goal, 'maneuver': p.maneuver,
                                          'depart_delay': p.depart_delay} for n, p in s.robots.items()}}
                          for s in self.vision_cfg.scenarios.values()],
            'active': self.vision_scenario,
            'run': run.status(now) if run is not None else None,
        }
        return out
