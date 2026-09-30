#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Relay Fleet Coordinator Node (관제 평면 도메인 8).

역할:
1. 기준 경로 배정 (Route: waypoints, edge_ids, junction_idx 등).
2. 구간 배타 점유 및 전방 허가 (Reservation: clear_until_idx, reserve_ahead, release_behind).
3. 10Hz 주기적 주행 허가 하달 (LaneCommand.CMD_CLEARANCE, 하트비트 겸용).
4. 로봇 위치 및 상태 수집 (RobotState / LaneStatus Ingest).
5. 경로 상호 검증 (RouteComparator: nearest_idx, progress, cross_track_error, ROUTE_STATE_MISMATCH).
6. 비전 구역 이벤트 결합 판정 (VisionZoneEvent: ARRIVAL_CONFIRMED vs ARRIVAL_PENDING).
7. 웹 콘솔 및 모니터링 연동 (/fleet/lane/status, /fleet/lane/control).
"""

import functools
import json
import math
import os
import tempfile
import threading
import time
import yaml
from dataclasses import asdict
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Header, String

from pinky_fleet_msgs.msg import FleetCommand, RobotState
from pinky_lane_msgs.msg import LaneCommand, LaneStatus, Route as RouteMsg

from .reservation import DEFAULT_CONFIRM_UPDATES, DEFAULT_UPDATE_PERIOD, Reservation
from pinky_lane_station.road_graph import RoadGraph, Route
from . import profiles as profiles_mod
from .route_comparator import RouteComparator

RELIABLE_10 = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)

ROUTE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

MISSION_IDLE = 'IDLE'
MISSION_ASSIGNED = 'ASSIGNED'
MISSION_RUNNING = 'RUNNING'
MISSION_STOPPED = 'STOPPED'
MISSION_ESTOP = 'ESTOP'
MISSION_DONE = 'DONE'

# 로봇 주행 방식 (D6). 레인 = 팀11 lane_agent(차선 카메라), nav2 = 온보드 PinkyAgent(Nav2 목표 연쇄).
# ⭐ RobotState 는 팀11 과 바이트 동일해야 해서 필드로 구분할 수 없다 — lane_mission.yaml 에서 정한다.
DRIVE_MODE_LANE = 'lane'
DRIVE_MODE_NAV2 = 'nav2'
DRIVE_MODES = (DRIVE_MODE_LANE, DRIVE_MODE_NAV2)

# 제3자 검수 G-3·G-4·G-5 + REVIEW_20260926 G: 멈추는 쪽 제어 상태(비상정지 래치·플릿 정지·완료·로봇별 정지)를 게이트웨이
# 재시작 너머로 남긴다. 예전엔 메모리뿐이라 재시작하면 IDLE→ASSIGNED 로 떠서 FLEET_STOPPED·DONE·ROBOT_HELD 409 가 조용히
# 풀렸고, 래치 복원은 로봇의 DRIVE_ESTOP 보고 하나에만 기댔다(첫 보고 전 창 — G-5). RUNNING 은 남기지 않는다: 재시작하면
# 경로를 다시 배정하고 에이전트는 시작 전으로 돌아가므로 '달리던 중' 을 되살리면 예약 없이 출발시키는 셈이다.
CONTROL_STATE_FILE = 'fleet_control.json'
CONTROL_STATE_VERSION = 1
# 로봇별 정지의 이유 — 상태 API(held_reason)와 화면에 그대로 싣는다
HELD_BY_OPERATOR = '로봇별 정지'
HELD_ESTOP_RESTORED = 'ESTOP 보고 — 재시작 뒤 복원(로봇 재개 필요)'
# REVIEW_20260926 G: 게이트웨이 재기동(10~15 s)은 로봇 데드맨(3 s)을 건드려 LINK_LOST(9)로 래치시킨다. 그 래치는
# LaneCommand RESUME 으로만 풀린다(route_chain `_release(all_latches=True)`) — 화면이 정상처럼 보이면 SET_MAP 이 REFUSED 다.
HELD_LINK_LOST = '링크유실 래치 — 로봇 재개 필요'


def control_state_path() -> str:
    """정지류 제어 상태 파일 — 좌표 프로파일 상태 파일과 같은 폴더(PINKY_RELAY_STATE_DIR 을 따른다)."""
    return os.path.join(os.path.dirname(profiles_mod.state_path()), CONTROL_STATE_FILE)


def _locked(fn):
    """제3자 검수 G-14: HTTP 스레드(assign_conflict·profile_status·get_fleet_status_dict …)가 코디네이터 dict 를
    도는 사이 executor 틱이 같은 dict 를 바꾸면 `dictionary changed size` 로 요청이 떨어질 수 있었다(검수: 코드로 찾음,
    재현은 안 함) — 둘을 한 잠금으로 세운다."""
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        with self._coord_lock:
            return fn(self, *args, **kwargs)
    return wrapper


class FleetRobotContext:
    def __init__(self, name: str, domain_id: int, start_node: str = '', goal_node: str = '',
                 drive_mode: str = DRIVE_MODE_LANE):
        self.name = name
        self.domain_id = domain_id
        if drive_mode not in DRIVE_MODES:
            raise ValueError(f"{name}: drive_mode 는 {DRIVE_MODES} 중 하나여야 한다 (받은 값 {drive_mode!r})")
        self.drive_mode = drive_mode
        self.start_node = start_node
        self.goal_node = goal_node
        self.route: Optional[Route] = None
        self.route_seq: int = 0
        self.state: Optional[RobotState] = None
        self.state_time: float = 0.0
        self.lane_status: Optional[LaneStatus] = None
        self.lane_status_time: float = 0.0
        self.clear_until_idx: int = 0
        self.is_stale: bool = True
        self.is_unlocalized: bool = True
        self.arrived: bool = False
        self.arrival_confirmed: bool = False
        self.vision_zone_state: str = 'NONE'  # 'NONE', 'ENTER', 'PRESENT', 'EXIT'
        self.last_zone_event: Optional[Dict[str, Any]] = None  # {zone_id, event, timestamp}
        self.comparator_result: Optional[Dict[str, Any]] = None
        # Team11 START retry tracking
        self.start_acknowledged: bool = False
        self.start_retry_count: int = 0
        self.last_start_time: float = 0.0
        # 재전송을 다 쓰고도 ack 가 없으면 참. 예전엔 조용히 멈췄다 — 화면이 "왜 안 가나" 에 답하게 드러낸다
        self.start_gave_up: bool = False
        self.start_armed: bool = False                # START 를 한 번이라도 무장했나 — 안 했으면 '포기' 로그를 안 낸다(L7)
        # D7: 로봇별 정지(`/api/pinkyN/stop`). 플릿이 RUNNING 이어도 이 로봇만 STOP 을 계속 받는다.
        self.held: bool = False
        self.held_reason: str = ''                  # 왜 세워 뒀나 — HELD_* (제3자 검수 G-3 · REVIEW_20260926 G)
        # 이 로봇에 LaneCommand RESUME 을 보낸 코디네이터 시각 — 그 사이 날아오던 옛 LINK_LOST 보고로 방금 푼 로봇을 다시
        # 세우지 않는다. LINK_LOST 아닌 보고가 오면 지운다(다음 LINK_LOST 는 새 래치다). 유예(LINK_LOST_RESUME_GRACE_SEC)가
        # 지나도 LINK_LOST 면 RESUME 이 안 닿은 것이다 — 다시 세운다(링크가 끊긴 채 보낸 재개가 래치를 숨기지 않게).
        self.link_lost_released_at: Optional[float] = None
        # 통합 검토 F4·E2E-2: RobotState.nav_status 가 NAV_LINK_LOST 로 이어진 첫 수신 시각(아니면 None)
        self.nav_link_lost_since: Optional[float] = None
        # 통합 검토 F2: RUNNING 중 로봇 재개를 누른 뒤 이 시각까지 '시작 전(IDLE)' 보고가 오면 START 를 다시 무장한다
        self.resume_start_until: Optional[float] = None
        # 로봇 재개 재전송(재기동 직후 DDS 탐색 동안 한 발짜리 RESUME 이 받는 곳 없이 사라졌다) — 이 시각까지, 마지막 보낸 시각
        self.resume_retry_until: Optional[float] = None
        self.resume_retry_since: float = 0.0           # 운영자가 재개를 누른 시각 — 이 뒤의 보고만 '풀렸다' 의 증거다
        self.resume_last_sent: float = 0.0
        self.stall_since: Optional[float] = None    # D6-2 교착 신호 — 요청 없이 허가 지점에 선 시각
        self.stall_warned: bool = False
        self.idle_since: Optional[float] = None     # 통합 검토 F2 — RUNNING 인데 START 확인 뒤 IDLE 을 보고한 시각
        self.idle_warned: bool = False
        # 통합 검토 FLEET-R1: 에이전트가 지금 경로의 도착(DRIVE_ARRIVED)을 처음 말한 시각 — 확인된 포즈가 목표 근처라 받기 전까지
        self.arrive_claim_since: Optional[float] = None
        self.arrive_warned: bool = False
        # 2026-09-29 교차로 규칙: 로봇이 JUNCTION_STOP(정지선·분기 반경) 을 처음 보고한 시각 — 선착순 근거·화면 표시
        self.junction_stop_since: Optional[float] = None


class RelayFleetCoordinator(Node):
    # 플릿 비상정지 래치 — estop_fleet 이 걸고 **resume_fleet 만** 푼다. mission_state 와 따로 둔다:
    # stop_fleet·start_fleet 이 mission_state 를 덮어쓰면 "지금 비상정지 중인가" 가 사라져서, 그 뒤
    # 로봇별 재개가 한 로봇의 ESTOP 래치를 풀었다(관제 검수 P1 후속, 2026-09-25 직렬 검토).
    estop_latched = False
    # S6: 이 코디네이터가 기동 뒤 비상정지를 직접 걸거나 푼 적이 있나. 없으면(재시작 직후) 로봇이 보고하는 ESTOP 이
    #     유일한 근거다 — 메모리에만 있던 래치가 재시작으로 false 가 돼 로봇 재개가 한 대씩 풀던 구멍(관제 검수 §3.2).
    _estop_authority = False
    _pre_estop_state = None
    _pre_stop_state = None      # 플릿 정지 직전 상태 — 재개는 여기로 돌아간다(달리던 플릿만 다시 달린다)
    # R-7 웹 전환: 좌표 프로파일(도로망·미션·화면 지도·로봇 지도 이름 묶음). 목록 파일이 없으면 예전처럼 lane_mission.yaml 하나.
    profiles = {}
    profiles_default = None
    active_profile = None
    profile_note = None
    # R4 (관제 검수 §3.3): 제어 명령을 처리한 결과 — 게이트웨이가 "보냈다" 와 "적용됐다" 를 가를 수 있게
    AUTO_ASSIGN = True      # 기동 때 미션 파일의 start/goal 로 경로를 배정한다 (비전 미션 모드는 끈다)
    control_seq = 0
    last_control = None     # L7: 비상정지 직전 상태 — 재개는 그리로 돌아간다(한 번도 시작 안 한 플릿을 출발시키지 않게)
    # 정지 확인의 증거 — 에이전트(route_chain)가 STOP 을 처리하면 LaneStatus.state_reason 이 이것 중 하나다.
    # 멈춘 상태(IDLE·대기)만으로는 부족하다: STOP 을 받기 전에도, 단일 목표로 달리는 중에도 IDLE 이다.
    STOP_ACK_REASONS = ('LaneCommand STOP', 'FleetCommand STOP')
    HALT_SPEED_MPS = 0.02
    # D6-2: 잡은 엣지 끝(허가 지점)에서 다음 구간 요청도, 막는 로봇도 없이 이만큼 서 있으면 중재가 아니라 교착이다
    STALL_WARN_SEC = 10.0
    HALT_YAW_RPS = 0.05
    # REVIEW_20260926 G: 로봇 재개(LaneCommand RESUME) 뒤 이만큼은 LINK_LOST 보고를 옛 보고로 본다. 에이전트는 RESUME 을
    # 받는 즉시 체인의 link_lost 를 지우고(route_chain `_release`) 보고는 10 Hz 라 보통 0.2 s 안에 바뀐다 — 넉넉히 잡는다.
    LINK_LOST_RESUME_GRACE_SEC = 3.0
    # 로봇 재개 재전송: 게이트웨이 재기동 뒤 새 코디네이터와 에이전트가 서로를 찾는 데 약 3 s 걸린다(실제 프로세스 종단 모의).
    # 그 사이 누른 로봇 재개는 volatile 이라 사라졌고, 유예가 지나 코디네이터가 로봇을 다시 세웠다 — 운영자는 '적용' 을 보았다.
    # 정지·비상정지는 10 Hz 반복이라 괜찮고, START 는 재시도가 있다. RESUME 도 로봇이 래치를 말하는 동안 다시 보낸다.
    RESUME_RETRY_SEC = 6.0
    RESUME_RETRY_INTERVAL_SEC = 1.0
    # 통합 검토 F4: RobotState 의 NAV_LINK_LOST 는 체인 래치만이 아니라 에이전트 LinkWatch 가 '끊겼다' 고 보는 동안에도 나온다
    # (래치 없이 — 멈춰 있던 로봇, E2E-4). 그 상태는 코디네이터의 첫 맥박(10 Hz FleetCommand)에 풀린다. 재기동 직후 그
    # 맥박보다 먼저 닿은 보고로 세우지 않게, 이만큼 이어져야 래치로 본다. LaneStatus DRIVE_LINK_LOST 는 체인 래치 그 자체라 바로.
    NAV_LINK_LOST_CONFIRM_SEC = 3.0
    # 통합 검토 F2: RUNNING 인데 START 를 확인한 Nav2 로봇이 지금 경로를 시작 전(IDLE)으로 이만큼 보고하면 경고한다
    IDLE_IN_RUNNING_WARN_SEC = 3.0
    # 통합 검토 FLEET-R1: 도착 보고는 이만큼(예약의 확인 창 하나 = 2 s) 이어진 뒤에야 본다 — 그래야 확인 창이 로봇이 **선 뒤**의 포즈만
    # 담는다(START 전 옛 좌표계 포즈·방금 튄 포즈가 섞이지 않는다). 그 뒤에도 목표 근처가 아니면 이만큼 가서 경고한다.
    ARRIVE_CONFIRM_SEC = DEFAULT_CONFIRM_UPDATES * DEFAULT_UPDATE_PERIOD
    ARRIVE_UNCONFIRMED_WARN_SEC = 5.0
    # G-14: executor 콜백과 HTTP 스레드의 읽기를 세우는 잠금. __init__ 이 제 것을 만든다 — 이 클래스 값은 __init__ 없이
    # 만든 시험 객체용이다.
    _coord_lock = threading.RLock()
    # G-3·G-4·G-5: 제어 상태 파일 경로. None = 저장하지 않는다(__init__ 없이 만든 시험 객체 — 실물 상태 폴더를 건드리지 않게).
    _control_state_path = None
    _control_state_saved = None     # 마지막으로 쓴 스냅샷 — 바뀔 때만 쓴다
    control_note = None             # 제어 상태 파일을 못 읽어 래치로 떴다 — 화면에 보인다(플릿 재개가 지운다)
    _control_save_error = None      # 마지막 저장이 실패했다 — 다음 저장이 성공하면 지운다

    def __init__(self, config_path: Optional[str] = None):
        super().__init__('relay_fleet_coordinator')
        self._coord_lock = threading.RLock()

        # 1. lane_mission.yaml 로드 (Single Source of Truth)
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        if not config_path:
            # R-7: 좌표 프로파일이 있으면 웹에서 마지막으로 고른 것(없으면 기본)의 미션을 쓴다
            self.profiles_default, self.profiles = profiles_mod.load_profiles()
            if self.profiles:
                name, note = profiles_mod.resolve_active(self.profiles_default, self.profiles)
                prof = self.profiles.get(name)
                if prof is not None and prof.valid:
                    config_path, self.active_profile, self.profile_note = prof.mission_path, name, note
                    if note:
                        self.get_logger().warn(f"profile: {note}")
        if not config_path:
            # 프로파일 목록이 없거나 전부 깨졌을 때 — 기본 프로파일의 미션 파일 그대로
            config_path = os.path.join(pkg_dir, 'config', 'profiles', 'team11_map5', 'lane_mission.yaml')
        self.config_path = config_path

        if os.path.isfile(self.config_path):
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f) or {}
        else:
            self.config = {}

        graph_rel = self.config.get('graph', 'road_graph.yaml')
        if os.path.isabs(graph_rel):
            self.graph_path = graph_rel
        else:
            self.graph_path = os.path.join(os.path.dirname(self.config_path), graph_rel)

        self.graph = RoadGraph.load(self.graph_path)
        self.get_logger().info(f"Loaded RoadGraph from {self.graph_path} ({len(self.graph.nodes)} nodes, {len(self.graph.edges)} edges)")

        # 2. Reservation & RouteComparator 파라미터 적용
        res_cfg = self.config.get('reservation', {})
        self.reservation = Reservation(
            self.graph,
            reserve_ahead=float(res_cfg.get('reserve_ahead', 0.40)),
            release_behind=float(res_cfg.get('release_behind', 0.25)),
            node_stop_margin=float(res_cfg.get('node_stop_margin', 0.20))
        )
        self.comparator = RouteComparator(off_route_threshold=0.35, mismatch_idx_threshold=5)

        # 3. Coordinator 파라미터 적용
        coord_cfg = self.config.get('coordinator', {})
        self.tick_rate = float(coord_cfg.get('tick_rate', 10.0))
        self.state_timeout_sec = float(coord_cfg.get('state_timeout', 2.0))
        self.start_retry_max = int(coord_cfg.get('start_retry_max', 5))
        self.start_retry_interval = float(coord_cfg.get('start_retry_interval', 1.0))
        self.goal_event_timeout = float(coord_cfg.get('goal_event_timeout', 3.0))

        # 4. 로봇 컨텍스트 로드 (lane_mission.yaml 기반 Single Source of Truth)
        self.robots: Dict[str, FleetRobotContext] = {}
        robots_cfg = self.config.get('robots', [])
        if not robots_cfg:
            robots_cfg = [
                {'name': 'pinky1', 'domain_id': 10, 'start': 'BL', 'goal': 'TR'},
                {'name': 'pinky2', 'domain_id': 11, 'start': 'BR', 'goal': 'BL'},
            ]

        for r in robots_cfg:
            r_name = r['name']
            r_domain = int(r.get('domain_id', 10))
            r_start = str(r.get('start', ''))
            r_goal = str(r.get('goal', ''))
            self.robots[r_name] = FleetRobotContext(
                name=r_name,
                domain_id=r_domain,
                start_node=r_start,
                goal_node=r_goal,
                drive_mode=str(r.get('drive_mode', DRIVE_MODE_LANE)),
            )

        # 4. ROS 통신 퍼블리셔 / 서브스크라이버 설정 (도메인 8)
        self.route_pubs: Dict[str, Any] = {}
        self.lane_cmd_pubs: Dict[str, Any] = {}
        self.fleet_cmd_pubs: Dict[str, Any] = {}

        for name, ctx in self.robots.items():
            # 다운링크: 관제(8) -> 로봇 도메인 (브릿지가 중계)
            self.route_pubs[name] = self.create_publisher(RouteMsg, f'/{name}/route', ROUTE_QOS)
            self.lane_cmd_pubs[name] = self.create_publisher(LaneCommand, f'/{name}/lane_command', RELIABLE_10)
            self.fleet_cmd_pubs[name] = self.create_publisher(FleetCommand, f'/{name}/command', RELIABLE_10)

            # 업링크: 로봇 도메인 -> 관제(8)
            self.create_subscription(
                RobotState, f'/{name}/state',
                lambda msg, n=name: self._cb_robot_state(n, msg),
                RELIABLE_10
            )
            self.create_subscription(
                LaneStatus, f'/{name}/lane_status',
                lambda msg, n=name: self._cb_lane_status(n, msg),
                RELIABLE_10
            )

        # 관제 제어 및 상태 토픽
        self.sub_control = self.create_subscription(
            String, '/fleet/lane/control', self._cb_control, 10
        )
        self.pub_status = self.create_publisher(
            String, '/fleet/lane/status', 10
        )

        # 비전 구역 이벤트 수신 (태블릿 연동)
        self.sub_vision_event = self.create_subscription(
            String, '/vision/zone_event', self._cb_vision_zone_event, 10
        )

        # 내부 상태
        self.mission_state = MISSION_IDLE
        self.state_timeout_sec = 1.0
        self.last_warning = ''
        self.global_seq = 0
        # R-D2: (robot_name, camera_id) -> last sequence tracking for duplicate/out-of-order rejection
        self._zone_event_seq: Dict[Tuple[str, str], int] = {}
        self._zone_event_session: Dict[Tuple[str, str], str] = {}

        # 주기적 루프 (10Hz 제어/예약 루프, 2Hz 상태 브로드캐스트)
        self.timer_loop = self.create_timer(0.1, self._loop_tick)  # 10Hz
        self.timer_status = self.create_timer(0.5, self._publish_status)  # 2Hz

        # 초기 기본 경로 자동 생성 및 배정 — 비전 미션 모드(vision_coordinator)는 경로를 내지 않는다(AUTO_ASSIGN = False)
        for name, ctx in self.robots.items():
            if self.AUTO_ASSIGN and ctx.start_node and ctx.goal_node:
                self.assign_route(name, ctx.start_node, ctx.goal_node)

        # G-3·G-4·G-5: 프로파일을 읽고 경로를 배정한 **뒤** 멈추는 쪽 상태를 되살린다 — 배정이 ASSIGNED 로 덮지 않게.
        #     경로는 복원보다 먼저 잡되, 복원 전에 파일을 덮어쓰지 않도록 저장 경로는 여기서야 연다.
        self._control_state_path = control_state_path()
        self._restore_control_state()

        self.get_logger().info(f"RelayFleetCoordinator initialized successfully (profile={self.active_profile}).")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def now(self) -> float:
        """코디네이터 시계 — `hold_confirmation(since=…)` 의 기준(게이트웨이가 정지를 보내기 직전에 읽는다)."""
        return self._now()

    # -------------------------------------------------------------------------
    # 제3자 검수 G-3·G-4·G-5 + REVIEW_20260926 G: 멈추는 쪽 제어 상태를 재시작 너머로
    # -------------------------------------------------------------------------
    def _control_snapshot(self) -> Dict[str, Any]:
        """남길 것: 비상정지 래치(+직전 상태) · 플릿 STOPPED·DONE(+정지 직전 상태) · 로봇별 정지(+이유). RUNNING 은 안 남긴다."""
        latched = bool(self.estop_latched or self.mission_state == MISSION_ESTOP)
        stop_like = self.mission_state if self.mission_state in (MISSION_STOPPED, MISSION_DONE) else None
        return {
            'version': CONTROL_STATE_VERSION,
            'estop_latched': latched,
            'pre_estop_state': self._pre_estop_state,
            'mission_state': stop_like,
            'pre_stop_state': self._pre_stop_state,
            'held': {n: (c.held_reason or HELD_BY_OPERATOR) for n, c in self.robots.items() if c.held},
        }

    def _save_control_state(self) -> None:
        """바뀌었으면 원자적으로 쓴다(임시 파일 + os.replace — 쓰다 꺼져도 반쯤 쓴 파일이 남지 않게).

        ⚠️ 부르는 쪽은 **명령을 내보내기 전에** 부른다. 정지를 내보낸 뒤 쓰기 전에 죽으면 파일은 '안 세웠다' 로 남고,
           재시작 뒤 start 가 세워 둔 로봇에 START 를 보낸다(G-3 이 막으려던 바로 그 길).
        """
        path = self._control_state_path
        if not path:
            return
        snap = self._control_snapshot()
        if snap == self._control_state_saved:
            return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.' + CONTROL_STATE_FILE + '.')
            try:
                with os.fdopen(fd, 'w', encoding='utf-8') as f:
                    json.dump(dict(snap, saved_at=time.time()), f, ensure_ascii=False, sort_keys=True)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp, path)
            except BaseException:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
                raise
        except OSError as exc:
            note = f'제어 상태를 저장하지 못했다 — 재시작하면 정지·래치를 잊을 수 있다 ({exc})'
            if note != self._control_save_error:
                self.get_logger().error(note)
            self._control_save_error = note
            return
        self._control_state_saved = snap
        self._control_save_error = None

    def control_notes(self) -> Optional[str]:
        """제어 상태의 알림(복원 실패·저장 실패) — 상태 API 와 좌표 카드의 '알림' 에 싣는다."""
        return '; '.join(n for n in (self.control_note, self._control_save_error) if n) or None

    def _restore_control_state(self) -> None:
        """__init__ 끝(프로파일·경로 배정 뒤)에서 한 번. 파일이 없으면 처음 뜬 것이다(되살릴 것 없음, S6 은 그대로).

        못 읽거나 깨졌으면 **플릿 비상정지 래치로** 뜬다(fail closed) — 무엇을 세워 뒀는지 모르는데 풀린 채 뜨면 안 된다.
        """
        path = self._control_state_path
        if not path:
            return
        try:
            with open(path, 'rb') as f:
                raw = f.read()
        except FileNotFoundError:
            # 파일 없음이 지금 모습이다 — 바뀌기 전에는 쓰지 않는다. 안 그러면 처음 뜬 코디네이터가 첫 틱에 빈 상태 파일을
            # 만든다(시험·도커가 PINKY_RELAY_STATE_DIR 없이 게이트웨이를 띄우면 실물 상태 폴더에).
            self._control_state_saved = self._control_snapshot()
            return
        except OSError as exc:
            self._restore_failed(f'못 읽었다 ({exc})')
            return
        # 제3자 재검 GW-R1: 바이트로 읽어 **여기서** 푼다. 내용이 무엇이든(UTF-8 이 아닌 파일 — CP949 로 손본 한글 이유 —,
        #     json 이 RecursionError 를 내는 깊은 중첩) '깨졌다' 로 받는다. 예외가 __init__ 밖으로 나가면 게이트웨이는
        #     코디네이터 없이 뜨고(플릿 비상정지·정지 503, LaneCommand 도 안 나감) 파일이 .corrupt 로 안 밀려나 재시작마다 같다.
        #     쓰는 쪽은 UTF-8 만 쓴다 — 다른 인코딩은 누가 손댄 파일이니 믿지 않는다.
        try:
            doc = json.loads(raw.decode('utf-8'))
            if not isinstance(doc, dict) or doc.get('version') != CONTROL_STATE_VERSION:
                raise ValueError(f"version {doc.get('version') if isinstance(doc, dict) else type(doc).__name__}")
            latched, held = doc['estop_latched'], doc['held']
            stop_like, pre_estop, pre_stop = doc['mission_state'], doc['pre_estop_state'], doc['pre_stop_state']
            if not isinstance(latched, bool) or not isinstance(held, dict):
                raise ValueError('estop_latched·held 형식')
            if stop_like not in (None, MISSION_STOPPED, MISSION_DONE):
                raise ValueError(f'mission_state {stop_like!r}')
            if not all(isinstance(n, str) and isinstance(r, str) for n, r in held.items()):
                raise ValueError('held 형식')
        except Exception as exc:        # noqa: BLE001 — 파일 내용 때문에 생성자가 죽는 길은 없어야 한다(fail closed)
            self._restore_failed(f'깨졌다 ({exc})')
            return
        # 직전 상태는 되돌아갈 곳이다. RUNNING 은 버린다 — 재시작 뒤 재개가 예약·START 확인 없이 달리던 플릿을 다시
        # 출발시키지 않게(모르면 출발시키지 않는다 — L7). DONE·STOPPED 만 남기고 나머지는 경로에서 다시 정한다.
        pre_estop = pre_estop if pre_estop in (MISSION_STOPPED, MISSION_DONE) else None
        pre_stop = pre_stop if pre_stop == MISSION_DONE else None
        for name, reason in held.items():
            ctx = self.robots.get(name)
            if ctx is None:
                self.get_logger().warn(f"control state: held robot {name!r} is not in this fleet — ignored")
                continue
            ctx.held, ctx.held_reason = True, reason or HELD_BY_OPERATOR
        if latched:
            self.estop_latched = True
            self.mission_state = MISSION_ESTOP
            self._pre_estop_state = pre_estop
            self._pre_stop_state = pre_stop
            # 파일이 이 코디네이터 자신의 기억이다 — 래치도 세워 둔 로봇도 안다. 로봇의 ESTOP 보고로 더 되살릴 것이 없다.
            # (파일이 '래치 아님' 이면 권한을 주지 않는다 — 파일이 뒤처졌을 수 있으니 S6 이 계속 지킨다.)
            self._estop_authority = True
        elif stop_like is not None:
            self.mission_state = stop_like
            self._pre_stop_state = pre_stop
        if latched or stop_like or held:
            self.get_logger().warn(f"control state restored from {path}: estop_latched={latched} "
                                   f"mission={self.mission_state} held={sorted(held)}")

    def _restore_failed(self, why: str) -> None:
        path = self._control_state_path
        self.control_note = (f'제어 상태 파일을 {why} — 플릿 비상정지 래치로 떴다. 무엇을 세워 뒀는지 모르니 '
                             f'확인 뒤 플릿 재개, 그다음 ESTOP 을 보고하는 로봇은 로봇마다 재개 ({path})')
        self.get_logger().error(self.control_note)
        self.estop_latched = True
        self.mission_state = MISSION_ESTOP
        self._pre_estop_state = self._pre_stop_state = None
        # 권한 없음 — 로봇별 정지를 모르니 ESTOP 을 보고하는 로봇은 S6 대로 HOLD 로 되살린다(플릿 재개가 한꺼번에 풀지 않게)
        self._estop_authority = False
        try:
            os.replace(path, path + '.corrupt')      # 증거는 남기고, 래치 상태를 새로 쓴다(다음 재시작도 래치로)
        except OSError:
            pass
        self._save_control_state()

    def _is_valid_goal_zone_event(self, ctx: FleetRobotContext, ref: Optional[float] = None) -> bool:
        """Goal zone 비전 이벤트의 신선도(freshness) 및 목표 지점 일치성 검증 (P1-2, R-D2).

        ref: 신선도를 재는 기준 시각(없으면 지금). 통합 검토 FLEET-R1 — 도착은 확인된 포즈를 기다려 늦게 받을 수 있다. 그때는
        에이전트가 도착을 **처음 말한 시각**으로 잰다: 예전(보고 즉시 받던 때)에 유효했던 이벤트와 그 뒤에 온 이벤트가 유효하다."""
        if not ctx.last_zone_event:
            return False
        now = self._now()
        ref = now if ref is None else ref
        # Goal freshness는 Relay 수신 시각(received_at)을 기본으로 판단 (R-D2)
        recv_time = ctx.last_zone_event.get('received_at', ctx.last_zone_event.get('timestamp', 0.0))
        if ref - recv_time > self.goal_event_timeout or recv_time > now:
            return False
        zid = str(ctx.last_zone_event.get('zone_id', '')).lower()
        goal = ctx.goal_node.lower() if ctx.goal_node else 'goal'
        # 중계 항목(09-26): 'goal'·'goal_c'·'goal_zone' 별칭은 옛 5노드 경기장(목표가 GOAL_C 하나)의 태블릿 구역 이름이다.
        #     목표가 로봇마다 다른 좌표(map5: TR·BL)에서는 그 별칭이 도착을 확정하면 안 된다 — 구역 이름이 로봇의 목표 노드와
        #     같아야 한다. 프로파일이 없을 때(예전 방식)만 별칭을 받는다. (legacy 프로파일은 2026-09-29 에 지웠다.)
        prof = self.profiles.get(self.active_profile) if self.active_profile else None
        aliases = ('goal', 'goal_c', 'goal_zone') if prof is None else ()
        if zid == goal or zid in aliases:
            evt = ctx.last_zone_event.get('event_type') or ctx.last_zone_event.get('event', '')
            return evt in ('ENTER', 'PRESENT')
        return False

    def _check_mission_done(self):
        """모든 활성 로봇의 도착 및 비전 검증 완료 시 MISSION_DONE 전이 (P2-1)."""
        if self.mission_state != MISSION_RUNNING:
            return
        active_robots = [ctx for ctx in self.robots.values() if ctx.route is not None]
        if not active_robots:
            return
        if all(ctx.arrived and ctx.arrival_confirmed for ctx in active_robots):
            self.mission_state = MISSION_DONE
            self._save_control_state()          # G-4: DONE 도 재시작을 넘긴다
            self.get_logger().info("🏁 All active robots confirmed arrival at Goal! Mission State -> DONE")

    # -------------------------------------------------------------------------
    # 콜백 함수들
    # -------------------------------------------------------------------------
    @_locked
    def _cb_robot_state(self, name: str, msg: RobotState):
        ctx = self.robots.get(name)
        if not ctx:
            return
        ctx.state = msg
        ctx.state_time = self._now()
        ctx.is_stale = False
        ctx.is_unlocalized = not msg.localized
        # 통합 검토 F4·E2E-2: 경로 없는 Nav2 에이전트는 LaneStatus 를 안 낸다 — 그 링크유실 래치는 RobotState 로만 보인다
        if msg.nav_status == RobotState.NAV_LINK_LOST:
            if ctx.nav_link_lost_since is None:
                ctx.nav_link_lost_since = ctx.state_time
        else:
            ctx.nav_link_lost_since = None
        self._apply_link_lost_hold(name, ctx, ctx.state_time)

    def _link_lost_seen(self, ctx: FleetRobotContext) -> bool:
        """로봇이 지금 링크유실을 **말하고 있나**(확인 전 포함) — 로봇 재개가 RESUME 을 보낼지, 재개 유예를 끝낼지 가른다.

        Nav2 로봇만(통합 검토 F3): 팀11 레인 로봇의 LINK_LOST 는 하트비트·LanePath 가 0.9 s 끊기면 들어갔다가 돌아오면 저절로
        나오는 상태이지 래치가 아니다. 그걸로 세우면(세우기 = STOP → 팀11 started=False) 재개해도 START 가 없어 영영 IDLE 이었다.
        LaneStatus DRIVE_LINK_LOST(체인 래치) 또는 RobotState NAV_LINK_LOST(통합 검토 F4 — 경로 없는 에이전트).
        """
        if ctx.drive_mode != DRIVE_MODE_NAV2:
            return False
        ls = ctx.lane_status
        if ls is not None and ls.drive_state == LaneStatus.DRIVE_LINK_LOST:
            return True
        return ctx.state is not None and ctx.state.nav_status == RobotState.NAV_LINK_LOST

    def _link_lost_latched(self, ctx: FleetRobotContext, now: float) -> bool:
        """세울 근거가 되는 링크유실 래치 — LaneStatus 는 바로, RobotState 는 NAV_LINK_LOST_CONFIRM_SEC 이어져야(F4)."""
        if ctx.drive_mode != DRIVE_MODE_NAV2:
            return False                                                    # F3
        ls = ctx.lane_status
        if ls is not None and ls.drive_state == LaneStatus.DRIVE_LINK_LOST:
            return True
        since = ctx.nav_link_lost_since
        return since is not None and now - since >= self.NAV_LINK_LOST_CONFIRM_SEC

    def _apply_link_lost_hold(self, name: str, ctx: FleetRobotContext, now: float) -> None:
        """REVIEW_20260926 G: 링크유실 래치(재기동 중 데드맨)는 LaneCommand RESUME 으로만 풀린다. 코디네이터가 모르면 화면은
        정상인데 START·SET_MAP 을 로봇이 안 받는다 — 로봇별 정지로 세우고 이유를 보인다. 로봇 재개가 푼다.
        재개 뒤 LINK_LOST_RESUME_GRACE_SEC 동안의 보고는 옛 보고로 본다(그 사이 날아오던 것). 둘 다(LaneStatus·RobotState)
        링크유실을 말하지 않을 때만 유예를 끝낸다 — 다른 토픽의 옛 보고가 늦게 닿아 방금 푼 로봇을 다시 세우지 않게."""
        if self._link_lost_latched(ctx, now):
            released = ctx.link_lost_released_at
            fresh_resume = released is not None and now - released < self.LINK_LOST_RESUME_GRACE_SEC
            if not ctx.held and not fresh_resume:
                ctx.held, ctx.held_reason = True, HELD_LINK_LOST
                self.get_logger().warn(f"[{name}] reports DRIVE_LINK_LOST — held until robot resume ({HELD_LINK_LOST})")
        elif not self._link_lost_seen(ctx):
            ctx.link_lost_released_at = None

    @_locked
    def _cb_lane_status(self, name: str, msg: LaneStatus):
        ctx = self.robots.get(name)
        if not ctx:
            return
        ctx.lane_status = msg
        ctx.lane_status_time = self._now()
        if ctx.route is not None and msg.route_seq == ctx.route_seq:
            # 통합 검토 RES-F2: 잡기(다음 엣지 요청)는 에이전트가 믿는 진행도 따른다 — 놓기는 아니다(reservation.note_reported_idx)
            self.reservation.note_reported_idx(name, msg.route_idx)
            # 2026-09-29 교차로 규칙: 정지선(또는 분기 반경) 에 선 로봇은 거리와 무관하게 지금 다음 엣지를 요청한다.
            #     허가는 예약이 선착순(요청 틱 → domain_id) 으로 준다. 로봇은 허가(clear_until > 분기 idx) 전엔 통과하지 않는다.
            if msg.drive_state == LaneStatus.DRIVE_JUNCTION_STOP:
                if ctx.junction_stop_since is None:
                    ctx.junction_stop_since = ctx.lane_status_time
                    eid = self.reservation.request_next_now(name) if self.mission_state == MISSION_RUNNING else ''
                    self.get_logger().info(f"[{name}] JUNCTION_STOP — 다음 엣지 요청 {eid or '(없음)'} (선착순)")
            elif msg.drive_state != LaneStatus.DRIVE_JUNCTION_PASS:
                ctx.junction_stop_since = None

        if (msg.drive_state == LaneStatus.DRIVE_ESTOP and not self.estop_latched
                and not self._estop_authority):
            # S6: 재시작 직후 — 로봇은 아직 ESTOP 래치를 쥐고 있다. 플릿 래치를 되살려 해제를 resume_fleet 한 곳으로 모은다.
            self.estop_latched = True
            # 직렬 검토(L3~L7): 예전엔 IDLE 로 못박았다 — 재시작한 코디네이터도 미션 파일로 배정하고(ASSIGNED),
            #     시작했을 수도 있다(RUNNING). IDLE 로 돌아가면 달리던 플릿이 재개 뒤 예약을 멈춘 채 IDLE 로 섰다.
            self._pre_estop_state = self.mission_state
            self.mission_state = MISSION_ESTOP
            self.get_logger().warn(
                f"🚨 [{name}] reports DRIVE_ESTOP after coordinator restart — fleet E-STOP latch restored (use fleet resume)")
        if (msg.drive_state == LaneStatus.DRIVE_ESTOP and not self._estop_authority and not ctx.held):
            # 재시작으로 로봇별 정지(held)도 잊었다. S7 로 세워 둔 로봇이 ESTOP 래치를 쥔 채 남아 있었을 수 있으므로
            # ESTOP 을 보고하는 로봇은 로봇별 정지로 되살린다 — 플릿 재개가 그 로봇까지 출발시키지 않게(직렬 검토).
            ctx.held, ctx.held_reason = True, HELD_ESTOP_RESTORED
            self.get_logger().warn(f"[{name}] restored as HOLD after restart — use robot resume for it")
        self._apply_link_lost_hold(name, ctx, ctx.lane_status_time)
        self._save_control_state()

        # START ACK 확인: IDLE/ESTOP/LINK_LOST가 아닌 유효 주행 상태이면 START 확인 완료 (P1-4)
        # L5 (관제 검수 REVIEW_20260925 §3.1): Nav2 로봇의 STOP 사유 상태(대기)는 START 응답이 아니다 — 그걸 응답으로
        #     세면 START 보다 늦게 닿은 FleetCommand STOP 이 다시 래치해도 재전송이 없다(답신 §12-5 의 '남은 것').
        stop_reason = ctx.drive_mode == DRIVE_MODE_NAV2 and msg.state_reason in self.STOP_ACK_REASONS
        #     재배정 직후 에이전트가 새 Route 를 받기 전에 보낸 **옛 경로 번호**의 주행 보고는 새 START 의 응답이 아니다
        #     (직렬 검토: 그걸로 ack 가 서서 START 가 안 나가고 로봇이 IDLE 로 섰다). 아래 L5 분기처럼 번호를 본다.
        if (msg.drive_state not in (LaneStatus.DRIVE_IDLE, LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_LINK_LOST)
                and not stop_reason and msg.route_seq == ctx.route_seq):
            if not ctx.start_acknowledged:
                ctx.start_acknowledged = True
                self.get_logger().info(f"[{name}] START acknowledged by LaneStatus (drive_state={msg.drive_state})")
        elif (stop_reason and ctx.start_acknowledged and self.mission_state == MISSION_RUNNING
              and not ctx.held and ctx.route is not None and msg.route_seq == ctx.route_seq):
            # RUNNING 인데 세워 둔 적 없는 로봇이 STOP 사유로 서 있다 → START 재무장(다음 틱부터 재전송)
            self.get_logger().warn(f"[{name}] RUNNING but robot reports '{msg.state_reason}' — re-arming START")
            self._arm_start(ctx)
        if ctx.resume_start_until is not None:
            # 통합 검토 F2: 로봇 재개를 누른 직후의 보고 — 래치가 풀리고 나서야 '시작 전' 인지 보인다(아래 resume_robot)
            if ctx.lane_status_time > ctx.resume_start_until:
                ctx.resume_start_until = None
            else:
                self._rearm_start_if_not_started(name, ctx)

        # 도착 감지
        claim = msg.drive_state == LaneStatus.DRIVE_ARRIVED and msg.route_seq == ctx.route_seq and not ctx.arrived
        if not claim:
            ctx.arrive_claim_since = None
        elif ctx.arrive_claim_since is None:
            ctx.arrive_claim_since = ctx.lane_status_time
        # 🔴 통합 검토 FLEET-R1: 도착은 쥔 엣지·노드를 한꺼번에 놓는다(mark_arrived) — 에이전트의 말만으로 받지 않는다. 그 말은 튄 포즈로
        #    단조로 부푼 RouteFollower 진행에서 나올 수 있다(출발 노드에 선 채 goto 없이 '목표 도착' → 다른 로봇이 그 자리로 들어왔다).
        #    도착을 말한 지 확인 창 하나(ARRIVE_CONFIRM_SEC)가 지나고, 그 창(= 선 뒤의 포즈)의 과반이 목표에서 reserve_ahead 안일 때만
        #    받는다(reservation.goal_gap) — D6-2 불변식 'Nav2 도달 허용 ≤ reserve_ahead' 라 참 도착은 늘 이 안이다. 아니면 엣지를 계속
        #    쥐고 10 Hz 로 다시 오는 보고마다 다시 본다. 오래가면 경고한다(_check_stalls → ARRIVED_NOT_AT_GOAL, 재배정이 푼다).
        if (claim and ctx.lane_status_time - ctx.arrive_claim_since >= self.ARRIVE_CONFIRM_SEC
                and self.reservation.goal_gap(name) <= self.reservation.reserve_ahead):
            ctx.arrived = True
            self.reservation.mark_arrived(name)
            self.get_logger().info(f"[{name}] Onboard reported DRIVE_ARRIVED for route_seq={msg.route_seq}")
            # 신선하고 유효한 Goal Zone 비전 이벤트 결합 확인 (P1-2) — 신선도는 도착을 처음 말한 시각으로(FLEET-R1)
            if self._is_valid_goal_zone_event(ctx, ref=ctx.arrive_claim_since):
                ctx.arrival_confirmed = True
                self.get_logger().info(f"[{name}] ARRIVAL_CONFIRMED by fresh Goal Zone Event!")
                self._check_mission_done()

    @_locked
    def _cb_vision_zone_event(self, msg: String):
        """태블릿 또는 외부에서 수신된 비전 구역 이벤트 JSON 처리 (P1-2, R-D2)."""
        try:
            data = json.loads(msg.data)
            # R-D2: Tablet canonical 우선 파싱, legacy alias 하위 호환
            robot_name = data.get('robot_name') or data.get('robot', '')
            camera_id = data.get('camera_id', '')
            zone_id = data.get('zone_id', '')
            event_type = data.get('event_type') or data.get('event', '')  # 'ENTER', 'PRESENT', 'EXIT'
            confidence = float(data.get('confidence', 1.0))
            source_timestamp = float(data.get('timestamp', 0.0))
            raw_seq = data.get('sequence')
            sequence = int(raw_seq) if raw_seq is not None else None
            session_id = data.get('session_id') or data.get('boot_id')
            reset_flag = bool(data.get('reset', False))

            # duplicate/out-of-order는 (robot_name + camera_id + sequence) 기준으로 필터링
            if sequence is not None and robot_name and camera_id:
                if not hasattr(self, '_zone_event_seq'):
                    self._zone_event_seq = {}
                if not hasattr(self, '_zone_event_session'):
                    self._zone_event_session = {}
                seq_key = (robot_name, camera_id)
                last_session = self._zone_event_session.get(seq_key)

                # 태블릿 프로세스 재시작(seq == 1), 세션 변경, 또는 명시적 reset 시 시퀀스 리셋 허용 (P1)
                if reset_flag or (session_id and last_session and session_id != last_session) or sequence == 1:
                    last_seq = None
                else:
                    last_seq = self._zone_event_seq.get(seq_key)

                if last_seq is not None and sequence <= last_seq:
                    self.get_logger().warn(
                        f"Dropped duplicate/out-of-order zone event for {seq_key}: "
                        f"seq={sequence} <= last_seq={last_seq}"
                    )
                    return
                self._zone_event_seq[seq_key] = sequence
                if session_id:
                    self._zone_event_session[seq_key] = str(session_id)

            ctx = self.robots.get(robot_name)          # 이름은 pinkyN 하나 — robotN 별칭 표는 없다(2026-09-29)

            if ctx:
                recv_now = self._now()
                ctx.vision_zone_state = event_type
                ctx.last_zone_event = {
                    'robot_name': robot_name,
                    'camera_id': camera_id,
                    'zone_id': zone_id,
                    'event_type': event_type,
                    'event': event_type,          # legacy compatibility
                    'confidence': confidence,
                    'sequence': sequence,
                    'session_id': session_id,
                    'source_timestamp': source_timestamp,
                    'received_at': recv_now,
                    'timestamp': recv_now,        # legacy compatibility
                }
                if ctx.arrived and self._is_valid_goal_zone_event(ctx):
                    ctx.arrival_confirmed = True
                    self.get_logger().info(f"[{ctx.name}] VisionZoneEvent confirmed arrival at goal zone: {zone_id}")
                    self._check_mission_done()
        except Exception as e:
            self.get_logger().warn(f"Failed to parse vision zone event: {e}")

    @_locked
    def _cb_control(self, msg: String):
        """String(JSON) 제어 명령 처리."""
        try:
            cmd_data = json.loads(msg.data)
            cmd = cmd_data.get('cmd', '').lower()
            ok = None
            if cmd == 'start':
                ok = self.start_fleet()
            elif cmd == 'stop':
                ok = self.stop_fleet()
            elif cmd == 'estop':
                ok = self.estop_fleet()
            elif cmd == 'resume':
                ok = self.resume_fleet()
            elif cmd == 'stop_robot':
                ok = self.stop_robot(cmd_data.get('robot', ''))
            elif cmd == 'resume_robot':
                ok = self.resume_robot(cmd_data.get('robot', ''))
            elif cmd == 'assign':
                r_name = cmd_data.get('robot', '')
                s_node = cmd_data.get('start', '')
                g_node = cmd_data.get('goal', '')
                ok = self.assign_route(r_name, s_node, g_node)
            elif cmd == 'profile':
                ok = self.switch_profile(str(cmd_data.get('name', '')))
            elif cmd == 'robot_maps':
                ok = self.send_robot_maps()
            elif cmd == 'initial_poses':
                ok = self.send_initial_poses()
            self.control_seq += 1
            self.last_control = {'seq': self.control_seq, 'cmd': cmd, 'robot': cmd_data.get('robot', ''),
                                 'ok': None if ok is None else bool(ok), 'mission_state': self.mission_state}
        except Exception as e:
            self.get_logger().error(f"Error handling control message: {e}")

    # -------------------------------------------------------------------------
    # 핵심 로직 (10Hz 제어 틱)
    # -------------------------------------------------------------------------
    @_locked
    def _loop_tick(self):
        now = self._now()
        self.global_seq += 1

        # 1. 각 로봇 신선도 검사 및 위치 갱신
        for name, ctx in self.robots.items():
            if ctx.state is None or (now - ctx.state_time > self.state_timeout_sec):
                ctx.is_stale = True
            else:
                ctx.is_stale = False

            if not ctx.is_stale and ctx.state and not ctx.is_unlocalized:
                # Reservation 및 Comparator 에 위치 반영
                self.reservation.update_pose(name, ctx.state.x, ctx.state.y)
                if ctx.route:
                    comp = self.comparator.compare(
                        robot_name=name,
                        x=ctx.state.x,
                        y=ctx.state.y,
                        yaw=ctx.state.yaw,
                        route=ctx.route,
                        lane_status=ctx.lane_status,
                        localized=ctx.state.localized,
                        expected_route_seq=ctx.route_seq
                    )
                    ctx.comparator_result = asdict(comp)

        # 2. 주행 실행 중일 때만 신규 예약 진행
        if self.mission_state == MISSION_RUNNING:
            step_results = self.reservation.step(tick=self.global_seq)
            for name, clear_idx in step_results.items():
                ctx = self.robots.get(name)
                if not ctx:
                    continue
                # 신선하지 않거나 비위치추정 상태이면 클리어런스 전진 금지 (안전 홀드)
                if ctx.is_stale or ctx.is_unlocalized:
                    # 안전을 위해 현재 진행 인덱스에서 정지
                    ctx.clear_until_idx = min(ctx.clear_until_idx, self.reservation.robots[name].progress_idx if name in self.reservation.robots else 0)
                else:
                    ctx.clear_until_idx = clear_idx
        elif self.mission_state in (MISSION_IDLE, MISSION_ASSIGNED):
            for ctx in self.robots.values():
                ctx.clear_until_idx = 0
        elif self.mission_state == MISSION_STOPPED:
            # STOP 상태: 현재 위치에서 동결
            pass
        elif self.mission_state == MISSION_ESTOP:
            for ctx in self.robots.values():
                ctx.clear_until_idx = 0

        # 3. 상호 대기 교착(mutual wait) 검사 + 요청 없는 정지(D6-2 교착 신호)
        warnings = []
        deadlocks = self.reservation.mutual_wait()
        if deadlocks:
            warnings.append(f"DEADLOCK_MUTUAL_WAIT: {deadlocks}")
            self.get_logger().warn(f"🚨 {warnings[-1]}")
        warnings += self._check_stalls(now)
        self.last_warning = ' | '.join(warnings)

        # 4. 미션 완료 여부 검사 (P2-1)
        self._check_mission_done()

        # G-3·G-4: 바뀐 것이 있으면 내보내기 전에 남긴다(바뀐 곳마다 이미 쓰지만, 빠뜨린 길이 있어도 한 틱 안에)
        self._save_control_state()

        # 5. 10Hz LaneCommand 하트비트/허가 발행
        self._publish_lane_commands()

    def _check_stalls(self, now: float) -> List[str]:
        """허가 지점에 선 채 **다음 구간 요청도 없고 막는 로봇도 없는** 상태가 STALL_WARN_SEC 이어지면 경고.

        D6-2(관제 검수 REVIEW_20260925 §2): 리그에서 `clearance 대기 (진행 2, 허가 4)`, `waiting_for ''` 로 120 s 섰는데
        화면에는 "pinky2 가 공유구간을 잡아서" 처럼 보였다(관제도 그렇게 읽었다). 막는 이가 없으면 중재가 아니다.
        """
        out = []
        for name, ctx in self.robots.items():
            st = self.reservation.status(name) or {}
            ls = ctx.lane_status
            live = self.mission_state == MISSION_RUNNING and not ctx.held and not ctx.arrived
            idle = self._idle_in_running_warning(name, ctx, ls, live, now)
            if idle:
                out.append(idle)
            unconfirmed = self._arrive_unconfirmed_warning(name, ctx, ls, now)
            if unconfirmed:
                out.append(unconfirmed)
            stalled = (live and st.get('held_edges') and st.get('next_edge')
                       and not st.get('waiting_for') and not st.get('requesting')
                       and ls is not None and ls.drive_state == LaneStatus.DRIVE_WAIT_CLEARANCE)
            # 직렬 검토(L3 후속): 막는 로봇이 **도착해 선** 로봇이면 기다려도 안 풀린다(목표 노드를 계속 잡는다) — 중재가 아니다
            blocker = st.get('blocked_by') or ''
            parked = (live and st.get('waiting_for') and blocker in self.robots and self.robots[blocker].arrived)
            if not (stalled or parked):
                ctx.stall_since, ctx.stall_warned = None, False
                continue
            if ctx.stall_since is None:
                ctx.stall_since = now
            dur = now - ctx.stall_since
            if dur < self.STALL_WARN_SEC:
                continue
            if parked:
                msg = (f"PARKED_BLOCK: {name} — {st['waiting_for']} 를 도착해 선 {blocker} 가 막고 있다 {dur:.0f}s "
                       f"(도착한 로봇은 목표 노드를 계속 잡는다 — 풀리지 않는다, 재배정 필요)")
            else:
                msg = (f"STALL_NO_REQUEST: {name} — {st['held_edges'][-1]} 끝에서 {dur:.0f}s, "
                       f"{st['next_edge']} 요청 없음·막는 로봇 없음 (진행 {st.get('progress_idx')}, 허가 {st.get('clear_until')})")
            out.append(msg)
            if not ctx.stall_warned:
                ctx.stall_warned = True
                self.get_logger().warn(f"🚨 {msg}")
        return out

    def _reports_not_started(self, ctx: FleetRobotContext, ls: Optional[LaneStatus]) -> bool:
        """START 를 확인했던 Nav2 로봇이 **지금 경로 번호로** IDLE 을 보고한다 = 에이전트가 시작 전으로 돌아갔다.

        Nav2 에이전트(route_chain)의 IDLE 은 '경로 없음 또는 started 아님' 뿐이다. RUNNING 중에 그리 되는 길: 에이전트 재시작
        (TRANSIENT_LOCAL Route 를 다시 받는다), 브리지만 재시작해 같은 번호 Route 를 다시 받음(on_route 가 started 를 지운다).
        """
        return (ctx.drive_mode == DRIVE_MODE_NAV2 and ctx.route is not None and ctx.start_acknowledged
                and ls is not None and ls.route_seq == ctx.route_seq and ls.drive_state == LaneStatus.DRIVE_IDLE)

    def _idle_in_running_warning(self, name: str, ctx: FleetRobotContext, ls: Optional[LaneStatus],
                                 live: bool, now: float) -> Optional[str]:
        """통합 검토 F2: RUNNING 인데 시작 전으로 돌아간 로봇은 START 를 다시 받지 못해 조용히 IDLE 로 섰다(화면엔 아무것도 없고
        START 는 확인된 것으로 보였다). 저절로 출발시키지는 않는다 — 운영자가 모르는 출발이다. 경고하고, 로봇 재개가 START 를
        다시 무장한다(resume_robot)."""
        if not (live and self._reports_not_started(ctx, ls)):
            ctx.idle_since, ctx.idle_warned = None, False
            return None
        if ctx.idle_since is None:
            ctx.idle_since = now
        dur = now - ctx.idle_since
        if dur < self.IDLE_IN_RUNNING_WARN_SEC:
            return None
        msg = (f"ROBOT_IDLE_IN_RUNNING: {name} — 로봇 재개 필요 (경로 {ctx.route_seq} 를 시작 전(IDLE)으로 보고한 지 {dur:.0f}s "
               f"— 에이전트·브리지 재시작으로 START 가 지워졌다)")
        if not ctx.idle_warned:
            ctx.idle_warned = True
            self.get_logger().warn(f"🚨 {msg}")
        return msg

    def _arrive_unconfirmed_warning(self, name: str, ctx: FleetRobotContext, ls: Optional[LaneStatus],
                                    now: float) -> Optional[str]:
        """통합 검토 FLEET-R1: 에이전트는 지금 경로의 도착을 말하는데 확인된 포즈는 목표 근처가 아니다 — 받지 않고 엣지를 쥔 채 둔다.
        에이전트는 도착했다고 믿어 스스로 움직이지 않는다(저절로 출발시키지 않는다). 재배정(새 경로)이 푼다."""
        since = ctx.arrive_claim_since                  # 도착이 아닌 보고가 오면 지워진다(_cb_lane_status)
        # 받았거나, 재배정으로 경로 번호가 바뀌었으면(새 보고가 아직 없어도) 더는 그 도착 보고가 아니다
        if since is None or ctx.arrived or ls is None or ls.route_seq != ctx.route_seq:
            ctx.arrive_warned = False
            return None
        dur = now - since
        if dur < self.ARRIVE_UNCONFIRMED_WARN_SEC:
            return None
        msg = (f"ARRIVED_NOT_AT_GOAL: {name} — 재배정 필요 (경로 {ctx.route_seq} 도착 보고 {dur:.0f}s 인데 확인된 위치는 목표 "
               f"{self.reservation.goal_gap(name):.2f} m 앞 — 엣지를 계속 쥔다)")
        if not ctx.arrive_warned:
            ctx.arrive_warned = True
            self.get_logger().warn(f"🚨 {msg}")
        return msg

    def _publish_lane_commands(self):
        now = self._now()
        self._retry_resumes(now)
        for name, ctx in self.robots.items():
            # D7: 로봇별 정지는 플릿 상태보다 앞선다(ESTOP 만 빼고). 경로가 없어도 보낸다 —
            #     에이전트는 STOP 을 route_seq 와 무관하게 받고, 10 Hz 반복이라 하나가 유실돼도 곧 닿는다.
            if ctx.held and self.mission_state != MISSION_ESTOP:
                self._send_hold(name, ctx)
                continue
            # 1. FleetCommand (Merge Gate B Heartbeat & Gate C HOLD/RELEASE for Onboard Nav2 Agent)
            fleet_pub = self.fleet_cmd_pubs.get(name)
            if fleet_pub and ctx.drive_mode == DRIVE_MODE_NAV2:
                # D6: Nav2 로봇의 주행 허가는 LaneCommand(START·CLEARANCE)가 싣는다. FleetCommand 는 하트비트뿐이고,
                #     STOPPED·DONE 에서만 HOLD(STOP) 를 겸한다.
                #     🔴 RUNNING 에서 clear 0 마다 STOP 을 10 Hz 로 보내던 것이 에이전트를 매번 hold+/estop 시켰다.
                #     🔴 ESTOP 에서 CANCEL 을 보내면 에이전트가 레인 임무를 버려 RESUME 뒤에도 출발하지 못한다 —
                #        비상정지는 LaneCommand ESTOP 이 건다.
                fleet_cmd = FleetCommand()
                if self.mission_state in (MISSION_STOPPED, MISSION_DONE):
                    fleet_cmd.command = FleetCommand.CMD_STOP
                else:
                    fleet_cmd.command = FleetCommand.CMD_HEARTBEAT
                fleet_pub.publish(fleet_cmd)
            elif fleet_pub:
                fleet_cmd = FleetCommand()
                if self.mission_state == MISSION_ESTOP:
                    fleet_cmd.command = FleetCommand.CMD_CANCEL
                elif self.mission_state in (MISSION_STOPPED, MISSION_DONE):
                    fleet_cmd.command = FleetCommand.CMD_STOP
                elif self.mission_state == MISSION_RUNNING:
                    if ctx.clear_until_idx == 0:
                        fleet_cmd.command = FleetCommand.CMD_STOP
                    else:
                        fleet_cmd.command = FleetCommand.CMD_RESUME
                else:  # IDLE, ASSIGNED
                    fleet_cmd.command = FleetCommand.CMD_HEARTBEAT
                fleet_pub.publish(fleet_cmd)

            # 2. LaneCommand (Team11 Clearance & Route Progress Tracking)
            if not ctx.route:
                # S5: 경로 없는 로봇도 비상정지 동안은 10 Hz ESTOP 을 받는다 — 예전엔 estop_fleet 의 한 번뿐이라,
                #     그 사이 재시작한 로봇은 다시 래치되지 않았다(관제 검수 §3.2). ESTOP 은 route_seq 와 무관하게 받는다.
                pub = self.lane_cmd_pubs.get(name)
                if pub and self.mission_state == MISSION_ESTOP:
                    m = LaneCommand()
                    m.command = LaneCommand.CMD_ESTOP
                    m.route_seq = 0
                    pub.publish(m)
                continue
            pub = self.lane_cmd_pubs.get(name)
            if not pub:
                continue

            cmd_msg = LaneCommand()
            cmd_msg.route_seq = ctx.route_seq
            cmd_msg.max_linear_vel = 0.15
            cmd_msg.max_angular_vel = 1.2

            if self.mission_state == MISSION_ESTOP:
                cmd_msg.command = LaneCommand.CMD_ESTOP
                cmd_msg.clear_until_idx = 0
            elif self.mission_state in (MISSION_STOPPED, MISSION_DONE):
                cmd_msg.command = LaneCommand.CMD_STOP
                cmd_msg.clear_until_idx = ctx.clear_until_idx
            elif self.mission_state == MISSION_RUNNING:
                # Team11 START retry logic (P1-4): START 미확인 시 재전송 및 clearance 대기
                if not ctx.start_acknowledged:
                    if (ctx.start_retry_count <= 0 and not ctx.start_gave_up and ctx.start_armed
                            and (now - ctx.last_start_time) >= self.start_retry_interval):
                        ctx.start_gave_up = True
                        self.get_logger().error(
                            f"[{name}] START not acknowledged after {self.start_retry_max} retries — "
                            f"no LaneStatus from the robot (drive_mode={ctx.drive_mode})")
                    if ctx.start_retry_count > 0 and (now - ctx.last_start_time) >= self.start_retry_interval:
                        ctx.start_retry_count -= 1
                        ctx.last_start_time = now
                        cmd_msg.command = LaneCommand.CMD_START
                        cmd_msg.clear_until_idx = ctx.clear_until_idx
                        pub.publish(cmd_msg)
                        self.get_logger().info(f"[{name}] Resending CMD_START (retries left: {ctx.start_retry_count})")
                    continue
                cmd_msg.command = LaneCommand.CMD_CLEARANCE
                cmd_msg.clear_until_idx = ctx.clear_until_idx
            else:  # IDLE, ASSIGNED
                cmd_msg.command = LaneCommand.CMD_HEARTBEAT
                cmd_msg.clear_until_idx = 0

            pub.publish(cmd_msg)

    # -------------------------------------------------------------------------
    # 경로 배정 및 제어 액션
    # -------------------------------------------------------------------------
    def _arm_start(self, ctx: FleetRobotContext, now: Optional[float] = None):
        """START 를 (다시) 무장한다 — RUNNING 이고 로봇별 정지가 아니면 다음 틱부터 재전송이 START 를 낸다."""
        now = self._now() if now is None else now
        ctx.start_acknowledged = False
        ctx.start_gave_up = False
        ctx.start_retry_count = self.start_retry_max
        ctx.last_start_time = now - self.start_retry_interval
        ctx.start_armed = True

    def _rearm_start_if_not_started(self, name: str, ctx: FleetRobotContext) -> bool:
        """통합 검토 F2: 로봇 재개(운영자)에 한해 — RUNNING 인데 시작 전(IDLE)을 보고하는 로봇에 START 를 다시 무장한다.
        L5 재무장(STOP 사유)은 IDLE 을 안 봤고, 로봇 재개는 RESUME 만 보내 start_acknowledged 가 남은 채 영영 IDLE 이었다."""
        if (self.mission_state != MISSION_RUNNING or ctx.held or ctx.arrived
                or not self._reports_not_started(ctx, ctx.lane_status)):
            return False
        ctx.resume_start_until = None           # 한 번만 — 무장하면 START 재전송·ack 가 이어받는다
        self.get_logger().warn(f"[{name}] robot resume: RUNNING but robot reports not started (IDLE, "
                               f"route_seq={ctx.route_seq}) — re-arming START")
        self._arm_start(ctx)
        return True

    def _assign_conflict(self, robot_name: str, goal_node: str,
                         start_node: Optional[str] = None) -> Optional[Tuple[str, str]]:
        """(부딪히는 로봇, 이유) 또는 None — 규칙은 Reservation.assign_conflict 한 곳에 있다(ROS 없이 시험한다)."""
        return self.reservation.assign_conflict(robot_name, goal_node, start_node)

    @_locked
    def assign_conflict(self, robot_name: str, goal_node: str, start_node: Optional[str] = None) -> Optional[str]:
        """부딪히는 다른 로봇 이름(없으면 None). 이유는 assign_conflict_why."""
        hit = self._assign_conflict(robot_name, goal_node, start_node)
        return hit[0] if hit else None

    @_locked
    def assign_conflict_why(self, robot_name: str, goal_node: str, start_node: Optional[str] = None) -> str:
        hit = self._assign_conflict(robot_name, goal_node, start_node)
        return hit[1] if hit else ''

    @_locked
    def assign_route(self, robot_name: str, start_node: str, goal_node: str) -> bool:
        ctx = self.robots.get(robot_name)
        if not ctx:
            self.get_logger().error(f"Robot {robot_name} not found in fleet context.")
            return False
        hit = self._assign_conflict(robot_name, goal_node, start_node)
        if hit:
            self.get_logger().error(f"assign refused: {robot_name} {start_node} -> {goal_node} — {hit[1]} (L3)")
            return False

        try:
            route = self.graph.shortest_route(start_node, goal_node, step=0.10)
            ctx.route = route
            ctx.start_node = start_node
            ctx.goal_node = goal_node
            ctx.route_seq += 1
            ctx.arrived = False
            ctx.arrival_confirmed = False
            ctx.clear_until_idx = 0

            # Reservation 등록
            self.reservation.register(robot_name, ctx.domain_id, route)
            # L6: 새 경로는 새 START 가 필요하다(에이전트는 Route 를 받으면 시작 전으로 돌아간다). 예전엔 ESTOP·RUNNING
            #     중 재배정하면 옛 ack 가 남아 START 가 안 나가 IDLE 로 섰다.
            #     무장은 RUNNING 일 때만 — 아니면 옛 ack 만 지운다. 달리지 않는 플릿에 START 가 무장돼 있으면 재개 한 번에
            #     출발했다(직렬 검토). 비상정지·정지 뒤 RUNNING 으로 돌아가면 resume_fleet 가, 처음이면 start_fleet 가 무장한다.
            if self.mission_state == MISSION_RUNNING:
                self._arm_start(ctx)
            else:
                ctx.start_acknowledged = ctx.start_gave_up = ctx.start_armed = False
                ctx.start_retry_count = 0

            # RouteMsg 생성 및 TRANSIENT_LOCAL 발행
            route_msg = self._to_route_msg(robot_name, ctx.route_seq, route)
            if robot_name in self.route_pubs:
                self.route_pubs[robot_name].publish(route_msg)

            self.get_logger().info(
                f"Assigned route to {robot_name}: {start_node} -> {goal_node} "
                f"(seq={ctx.route_seq}, len={route.length:.2f}m, waypoints={len(route.waypoints)})"
            )

            if self.mission_state == MISSION_IDLE:
                self.mission_state = MISSION_ASSIGNED

            return True
        except Exception as e:
            self.get_logger().error(f"Failed to calculate route for {robot_name} ({start_node}->{goal_node}): {e}")
            return False

    def _to_route_msg(self, robot_name: str, route_seq: int, route: Route) -> RouteMsg:
        msg = RouteMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.robot_name = robot_name
        msg.route_seq = route_seq
        msg.waypoints = [Point(x=float(pt[0]), y=float(pt[1]), z=0.0) for pt in route.waypoints]
        msg.edge_ids = list(route.edge_ids)
        msg.edge_forward = list(route.edge_forward)
        msg.edge_end_idx = [int(i) for i in route.edge_end_idx]
        msg.node_ids = list(route.node_ids)
        msg.node_idx = [int(i) for i in route.node_idx]
        msg.crosswalk_idx = [int(i) for i in route.crosswalk_idx]
        msg.junction_idx = [int(i) for i in route.junction_idx]
        msg.goal_idx = int(route.goal_idx)
        msg.length = float(route.length)
        return msg

    # -------------------------------------------------------------------------
    # D7: 로봇별 정지·재개 (`/api/pinkyN/stop` · `/api/pinkyN/resume`)
    # -------------------------------------------------------------------------
    def _resolve(self, robot_name: str) -> Optional[str]:
        """플릿에 있는 로봇 이름이면 그대로, 아니면 None. 별칭 표(robotN→pinkyN)는 없다 — 이름은 pinkyN 하나다(2026-09-29)."""
        return robot_name if robot_name in self.robots else None

    def _send_hold(self, name: str, ctx: FleetRobotContext):
        lane = LaneCommand()
        lane.command = LaneCommand.CMD_STOP
        lane.route_seq = ctx.route_seq
        lane.clear_until_idx = ctx.clear_until_idx
        pub = self.lane_cmd_pubs.get(name)
        if pub:
            pub.publish(lane)
        fleet_pub = self.fleet_cmd_pubs.get(name)
        if fleet_pub:
            fc = FleetCommand()
            fc.command = FleetCommand.CMD_STOP      # 단일 목표(GOTO) 경로도 세운다 — HOLD 이지 ESTOP 이 아니다
            fleet_pub.publish(fc)

    def stop_robot(self, robot_name: str) -> bool:
        name = self._resolve(robot_name)
        if not name:
            self.get_logger().error(f"stop_robot: unknown robot {robot_name!r}")
            return False
        ctx = self.robots[name]
        ctx.held = True
        self._save_control_state()          # G-3: 내보내기 전에 남긴다
        self._send_hold(name, ctx)          # 다음 틱을 기다리지 않는다
        self.get_logger().warn(f"⏸ [{name}] robot STOP (HOLD) — fleet state {self.mission_state} kept")
        return True

    def resume_robot(self, robot_name: str) -> bool:
        name = self._resolve(robot_name)
        if not name:
            self.get_logger().error(f"resume_robot: unknown robot {robot_name!r}")
            return False
        if self.mission_state == MISSION_ESTOP or self.estop_latched:
            # 🔴 관제 검수 P1: 로봇 재개가 LaneCommand RESUME 을 내면 에이전트는 **모든** 래치를 푼다
            #    (`/estop false`). 플릿 비상정지 중에 한 로봇만 풀리면 안 된다 — 비상정지 해제는
            #    `resume_fleet` 한 곳뿐이다. `/api/pinkyN/resume` 과 D8 `/fleet/lane/control` 둘 다 여기를 지난다.
            self.get_logger().warn(f"⛔ [{name}] robot RESUME refused — fleet E-STOP engaged (use fleet resume)")
            return False
        ctx = self.robots[name]
        # REVIEW_20260926 G: 로봇의 링크유실·ESTOP 래치는 LaneCommand RESUME 으로만 풀린다(route_chain `_release(all_latches)`)
        #     — 세운 이유가 그것이거나 로봇이 지금 그렇게 보고하면 아래 R5 분기에서도 RESUME 을 보낸다.
        # 통합 검토 F3·F4: 링크유실은 Nav2 로봇만, 그리고 RobotState NAV_LINK_LOST 도 본다(경로 없는 에이전트는 LaneStatus 가 없다)
        robot_latched = ctx.held_reason in (HELD_LINK_LOST, HELD_ESTOP_RESTORED) or self._link_lost_seen(ctx) or (
            ctx.lane_status is not None and ctx.lane_status.drive_state == LaneStatus.DRIVE_ESTOP)
        ctx.held, ctx.held_reason = False, ''
        self._save_control_state()          # G-3: 내보내기 전에 남긴다
        pub = self.lane_cmd_pubs.get(name)
        if self.mission_state in (MISSION_STOPPED, MISSION_DONE):
            # R5 (관제 검수 §3.3): 플릿이 정지·완료 중이면 로봇별 정지만 푼다. RESUME 을 보내면 에이전트가 STOP 래치를
            # 풀었다가 다음 틱의 플릿 STOP 에 다시 걸려 0.1 s 동안 움직일 수 있었다. 출발은 플릿 재개 때.
            if robot_latched and pub:
                # 단 로봇 래치는 풀어야 한다(안 풀면 SET_MAP REFUSED·START 무시가 그대로 — REVIEW_20260926 G, 화면의 '로봇 재개
                # 필요' 가 거짓이 된다). RESUME 바로 뒤에 같은 발행자로 STOP 을 잇는다 — 에이전트는 이 순서로 받아 STOP 래치를
                # 곧바로 다시 건다. 재시작 뒤라면 새 경로로 시작 전이라 갈 목표도 없다.
                # 🔴 통합 검토 F1: 그래도 RESUME 이 낸 goto 는 해제 문(Nav2 취소 확인) 뒤에 쌓이고, 그 확인이 STOP 보다 먼저
                #    닿으면 goal_is_current 가 통과해(아직 stopped 아님) 플릿 STOPPED 중에 진짜 Nav2 목표가 나갔다 — R5 가 막으려던
                #    바로 그 출발. 그래서 RESUME **앞에** 허가 0 을 준다: 체인은 래치 중에도 CLEARANCE 를 기록하고, RESUME 뒤
                #    평가는 '허가 0 ≤ 도달' 이라 서기만 한다(goto 없음). 진짜 허가는 플릿이 RUNNING 으로 돌아간 뒤 틱이 다시 준다.
                self._publish_resume(name, ctx)
                ctx.resume_retry_since = self._now()
                ctx.resume_retry_until = ctx.resume_retry_since + self.RESUME_RETRY_SEC
            self.get_logger().info(f"▶️ [{name}] robot HOLD cleared — fleet is {self.mission_state}, it moves on fleet resume")
            return True
        if self.mission_state == MISSION_RUNNING:
            # 통합 검토 F2: 시작 전(IDLE)으로 돌아간 로봇은 RESUME 으로는 안 간다 — START 가 필요하다. 지금 IDLE 이면 바로,
            # 래치(링크유실) 중이라 아직 모르면 풀린 뒤 첫 보고가 IDLE 일 때 START 를 다시 무장한다(운영자가 누른 재개에 한해).
            ctx.resume_start_until = self._now() + self.LINK_LOST_RESUME_GRACE_SEC
            self._rearm_start_if_not_started(name, ctx)
        self._publish_resume(name, ctx)
        ctx.resume_retry_since = self._now()
        ctx.resume_retry_until = ctx.resume_retry_since + self.RESUME_RETRY_SEC
        self.get_logger().info(f"▶️ [{name}] robot RESUME")
        return True

    def _publish_resume(self, name: str, ctx: FleetRobotContext) -> None:
        """로봇 재개가 내는 것 — 처음과 재전송이 같은 길을 지난다.

        플릿 STOPPED·DONE(R5)이면 허가 0 → LaneCommand RESUME → STOP(통합 검토 F1: RESUME 이 goto 를 내지 않게, STOP 래치는
        곧바로 다시). 아니면 LaneCommand RESUME + FleetCommand RESUME."""
        now = self._now()
        pub = self.lane_cmd_pubs.get(name)
        lane = LaneCommand()
        lane.command = LaneCommand.CMD_RESUME
        lane.route_seq = ctx.route_seq
        if self.mission_state in (MISSION_STOPPED, MISSION_DONE):
            if pub:
                zero = LaneCommand()
                zero.command = LaneCommand.CMD_CLEARANCE
                zero.route_seq = ctx.route_seq
                zero.clear_until_idx = 0
                pub.publish(zero)
                pub.publish(lane)
                ctx.link_lost_released_at = now
            self._send_hold(name, ctx)
        else:
            if pub:
                pub.publish(lane)
                ctx.link_lost_released_at = now
            fleet_pub = self.fleet_cmd_pubs.get(name)
            if fleet_pub:
                fc = FleetCommand()
                fc.command = FleetCommand.CMD_RESUME
                fleet_pub.publish(fc)
        ctx.resume_last_sent = now

    def _retry_resumes(self, now: float) -> None:
        """운영자가 누른 로봇 재개를, 로봇이 아직 래치(링크유실·ESTOP)를 말하는 동안 RESUME_RETRY_INTERVAL_SEC 마다 다시 보낸다.

        새 움직임을 만들지 않는다 — 운영자가 요청한 재개를 **닿게** 할 뿐이다. 멈추는 것: 로봇이 풀렸다고 말함 · 다시 세움(운영자
        정지·재-세움) · 플릿 비상정지 · 창(RESUME_RETRY_SEC)이 지남. 창이 지나도 래치면 유예 뒤 다시 세운다(안 닿은 것이다)."""
        for name, ctx in self.robots.items():
            until = ctx.resume_retry_until
            if until is None:
                continue
            ls = ctx.lane_status
            still_latched = self._link_lost_seen(ctx) or (ls is not None and ls.drive_state == LaneStatus.DRIVE_ESTOP)
            # '풀렸다' 는 재개 **뒤에** 받은 보고로만 안다. 재기동 직후엔 새 코디네이터가 아직 아무 보고도 못 받았다(DDS 탐색
            # 약 3 s) — 보고가 없다고 풀린 것으로 보면 재전송이 첫 틱에 멈췄다(최종 종단 모의에서 그대로 재현).
            heard = max(ctx.lane_status_time or 0.0, ctx.state_time or 0.0) > ctx.resume_retry_since
            released = heard and not still_latched
            if (now > until or ctx.held or self.estop_latched or self.mission_state == MISSION_ESTOP
                    or released):
                ctx.resume_retry_until = None
                continue
            if now - ctx.resume_last_sent >= self.RESUME_RETRY_INTERVAL_SEC:
                self._publish_resume(name, ctx)
                if self.mission_state == MISSION_RUNNING and ctx.resume_start_until is not None:
                    ctx.resume_start_until = now + self.LINK_LOST_RESUME_GRACE_SEC   # 풀린 뒤 첫 IDLE 에 START 재무장(F2)
                self.get_logger().info(f"▶️ [{name}] robot RESUME re-sent — robot still reports a latch")

    # 멈춰 있음을 뜻하는 LaneStatus 상태 — CRUISE 만 빼고 전부
    _HALTED_DRIVE_STATES = (LaneStatus.DRIVE_IDLE, LaneStatus.DRIVE_WAIT_CLEARANCE, LaneStatus.DRIVE_ARRIVED,
                            LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_LINK_LOST)

    @_locked
    def hold_confirmation(self, robot_name: str, since: float) -> Optional[bool]:
        """정지 명령(`since` 시각) **뒤에** 받은 LaneStatus 로 판정한다.

        True  = 로봇이 **STOP 을 처리했다**(state_reason 이 STOP_ACK_REASONS, 또는 ESTOP·링크유실 래치)
                그리고 멈춰 있다(CRUISE 아님 · 속도 ≈ 0)
        False = 아직 달린다(CRUISE 또는 속도 > HALT_SPEED_MPS)
        None  = 모른다 — 그 뒤 보고가 없거나, 멈춰 있어도 STOP 을 처리했다는 증거가 아직 없다
        관제 검수 P2: 예전 정지 API 는 수신자가 없어도 success:true 였다. 그리고 IDLE·대기만으로 확인하면
        STOP 이 닿기 전에 보낸 보고나 단일 목표로 달리는 로봇(체인은 IDLE)을 '멈췄다' 로 센다(직렬 검토).
        """
        name = self._resolve(robot_name)
        if not name:
            return None
        ctx = self.robots[name]
        st = ctx.lane_status
        if st is None or (ctx.lane_status_time or 0.0) <= since:
            return None
        v, w = float(st.linear_velocity), float(st.angular_velocity)
        if (st.drive_state == LaneStatus.DRIVE_CRUISE
                or (math.isfinite(v) and abs(v) > self.HALT_SPEED_MPS)
                or (math.isfinite(w) and abs(w) > self.HALT_YAW_RPS)):     # 제자리 회전도 달리는 것이다
            return False
        if st.drive_state not in self._HALTED_DRIVE_STATES:
            return None
        if st.drive_state in (LaneStatus.DRIVE_ESTOP, LaneStatus.DRIVE_LINK_LOST):
            return True
        if ctx.drive_mode == DRIVE_MODE_LANE:
            # 팀11 lane_agent 는 STOP 에 사유 문구를 안 남긴다: CMD_STOP → started=False → IDLE '대기'
            # (pinky_pro_team11 ed86db5 lane_driver.py·drive_fsm.py). 그 IDLE 은 START 없이는 안 움직이고,
            # 로봇별 정지 중엔 START 를 안 보낸다 → 레인 로봇은 **IDLE** 이 증거다(대기·도착은 아니다).
            return True if st.drive_state == LaneStatus.DRIVE_IDLE else None
        return True if st.state_reason in self.STOP_ACK_REASONS else None

    # -------------------------------------------------------------------------
    # R-7 웹 전환: 좌표 프로파일 (월요일 실물 전환 — 태블릿 T-11 · 관제 C-M1 과 같은 순간에)
    # -------------------------------------------------------------------------
    @_locked
    def profile_switch_blocker(self) -> Optional[str]:
        """좌표를 지금 바꾸면 안 되는 이유(None = 된다). 달리는 도중 좌표가 바뀌면 경로·예약·지도가 섞인다."""
        if self.estop_latched or self.mission_state == MISSION_ESTOP:
            return '플릿 비상정지 중'
        if self.mission_state not in (MISSION_IDLE, MISSION_ASSIGNED, MISSION_DONE):
            return f'플릿이 {self.mission_state} — IDLE·ASSIGNED·DONE 에서만 바꾼다'
        for name, ctx in self.robots.items():
            st, ls = ctx.state, ctx.lane_status
            moving = st is not None and (abs(st.linear_velocity) > self.HALT_SPEED_MPS
                                         or abs(st.angular_velocity) > self.HALT_YAW_RPS)
            if moving or (ls is not None and ls.drive_state == LaneStatus.DRIVE_CRUISE):
                return f'{name} 가 움직이는 중'
        return None

    def switch_profile(self, name: str) -> bool:
        prof = self.profiles.get(name)
        if prof is None:
            self.get_logger().error(f"profile switch refused: unknown profile {name!r}")
            return False
        if not prof.valid:
            self.get_logger().error(f"profile switch refused: {name} is broken — {'; '.join(prof.problems)}")
            return False
        why = self.profile_switch_blocker()
        if why:
            self.get_logger().error(f"profile switch refused: {why}")
            return False
        if sorted(prof.robot_names()) != sorted(self.robots):
            self.get_logger().error(f"profile switch refused: robots {prof.robot_names()} ≠ {sorted(self.robots)}")
            return False
        cfg = prof.mission
        drive_modes = {r['name']: str(r.get('drive_mode', DRIVE_MODE_LANE)) for r in cfg.get('robots', [])}
        for dm in drive_modes.values():
            if dm not in DRIVE_MODES:
                self.get_logger().error(f"profile switch refused: unknown drive_mode {dm!r}")
                return False
        res_cfg = cfg.get('reservation', {})
        self.config, self.config_path, self.graph_path, self.graph = cfg, prof.mission_path, prof.graph_path, prof.graph
        self.reservation = Reservation(self.graph,
                                       reserve_ahead=float(res_cfg.get('reserve_ahead', 0.40)),
                                       release_behind=float(res_cfg.get('release_behind', 0.25)),
                                       node_stop_margin=float(res_cfg.get('node_stop_margin', 0.20)))
        for r in cfg.get('robots', []):
            ctx = self.robots[r['name']]
            ctx.start_node, ctx.goal_node = str(r.get('start', '')), str(r.get('goal', ''))
            ctx.drive_mode = drive_modes[r['name']]
            ctx.route = None
            ctx.clear_until_idx = 0
            ctx.arrived = ctx.arrival_confirmed = False
            ctx.start_acknowledged = ctx.start_gave_up = ctx.start_armed = False
            ctx.start_retry_count = 0
            ctx.comparator_result = None
            ctx.stall_since, ctx.stall_warned = None, False
        self.mission_state = MISSION_IDLE
        self._pre_estop_state = self._pre_stop_state = None
        self._save_control_state()          # G-4: DONE 에서 바꾸면 남긴 DONE 도 지운다(로봇별 정지는 그대로)
        for rname, ctx in self.robots.items():
            if ctx.start_node and ctx.goal_node:
                self.assign_route(rname, ctx.start_node, ctx.goal_node)
        self.active_profile, self.profile_note = name, None
        try:
            profiles_mod.write_active(name)
        except OSError as exc:
            self.profile_note = f'고른 프로파일을 저장하지 못했다 — 재시작하면 예전 것으로 뜬다 ({exc})'
            self.get_logger().error(self.profile_note)
        self.get_logger().warn(f"🗺 profile → {name}: {prof.label} ({len(self.graph.nodes)} nodes)")
        return True

    def send_robot_maps(self) -> bool:
        """각 로봇에 FleetCommand CMD_SET_MAP(이 프로파일의 로봇 지도 이름). 로봇이 그 지도 파일을 갖고 있어야 한다."""
        prof = self.profiles.get(self.active_profile) if self.active_profile else None
        if prof is None or not prof.robot_map_name:
            self.get_logger().error("robot map switch refused: active profile has no robot_map_name")
            return False
        why = self.profile_switch_blocker()
        if why:
            self.get_logger().error(f"robot map switch refused: {why}")
            return False
        for name, pub in self.fleet_cmd_pubs.items():
            fc = FleetCommand()
            fc.command = FleetCommand.CMD_SET_MAP
            fc.map_name = prof.robot_map_name
            pub.publish(fc)
        self.get_logger().warn(f"🗺 CMD_SET_MAP {prof.robot_map_name} → {list(self.fleet_cmd_pubs)}")
        return True

    def send_initial_poses(self) -> bool:
        """경로가 있는 로봇에 FleetCommand CMD_SET_INITIAL_POSE(출발 노드 · 첫 구간 방향). 로봇이 출발 노드에 놓여 있어야 맞다."""
        why = self.profile_switch_blocker()
        if why:
            self.get_logger().error(f"initial pose refused: {why}")
            return False
        sent = []
        for name, ctx in self.robots.items():
            pub = self.fleet_cmd_pubs.get(name)
            if not pub or ctx.route is None or len(ctx.route.waypoints) < 2:
                continue
            (x0, y0), (x1, y1) = ctx.route.waypoints[0], ctx.route.waypoints[1]
            fc = FleetCommand()
            fc.command = FleetCommand.CMD_SET_INITIAL_POSE
            fc.x, fc.y, fc.yaw = float(x0), float(y0), math.atan2(y1 - y0, x1 - x0)
            pub.publish(fc)
            sent.append(name)
        self.get_logger().warn(f"📍 CMD_SET_INITIAL_POSE (start nodes) → {sent}")
        return bool(sent)

    @_locked
    def profile_status(self) -> Dict[str, Any]:
        prof = self.profiles.get(self.active_profile) if self.active_profile else None
        return {
            'active': self.active_profile,
            'default': self.profiles_default,
            'label': prof.label if prof else None,
            'frame': prof.frame if prof else None,
            'robot_map_name': prof.robot_map_name if prof else None,
            # 제어 상태 복원 실패(래치로 떴다)·저장 실패도 같은 '알림' 칸에 — 전환 카드가 그 칸을 보인다
            'note': '; '.join(n for n in (self.profile_note, self.control_notes()) if n) or None,
            # REVIEW_20260926 G: 세워 둔 로봇과 이유(링크유실 래치면 로봇 재개 전에는 SET_MAP 이 REFUSED 다)
            'held': {n: (c.held_reason or HELD_BY_OPERATOR) for n, c in self.robots.items() if c.held},
            'switch_blocker': self.profile_switch_blocker(),
            # 미션에 있는데 경로를 못 받은 로봇(배정 거절 L3 등) — 전환 화면에서 숨지 않게
            'unassigned': sorted(n for n, c in self.robots.items()
                                 if c.start_node and c.goal_node and c.route is None),
            'available': [p.to_dict() for p in self.profiles.values()],
            # 웹 배정(2026-09-29): 시작·목적지 드롭다운은 지금 도로망의 endpoint 노드로 채운다 — /api/fleet/profiles 가 싣는다
            'graph': self.graph.summary(),
        }

    @_locked
    def robot_map_check(self, name: str) -> Dict[str, Any]:
        prof = self.profiles.get(self.active_profile) if self.active_profile else None
        ctx = self.robots.get(name)
        if prof is None or ctx is None:
            return {'state': 'UNKNOWN', 'detail': '프로파일 없음'}
        st, detail = profiles_mod.robot_map_check(prof, ctx.state)
        return {'state': st, 'detail': detail, 'robot_map_name': getattr(ctx.state, 'map_name', None) if ctx.state else None}

    @_locked
    def last_heard_sec(self, robot_name: str) -> Optional[float]:
        """로봇에게서 마지막으로 들은 지 몇 초인가 (RobotState·LaneStatus 중 최근). 들은 적 없으면 None."""
        name = self._resolve(robot_name)
        if not name:
            return None
        ctx = self.robots[name]
        t = max(ctx.state_time or 0.0, ctx.lane_status_time or 0.0)
        return None if t <= 0.0 else max(0.0, self._now() - t)

    @_locked
    def start_fleet(self) -> bool:
        if self.estop_latched:
            # START 는 ESTOP 래치를 풀지 않는다(에이전트도 그렇다). 여기서 RUNNING 으로 바꾸면 비상정지 상태가
            # 사라져 로봇별 재개가 ESTOP 을 푸는 길이 열린다 — 비상정지 해제는 resume_fleet 한 곳뿐이다.
            self.get_logger().warn("⛔ Fleet START refused — E-STOP latched (use fleet resume)")
            return False
        self.mission_state = MISSION_RUNNING
        self._save_control_state()          # G-4: 남긴 STOPPED·DONE 을 지운다 — START 를 내보내기 전에
        now = self._now()
        for name, ctx in self.robots.items():
            if ctx.route:
                ctx.start_acknowledged = False
                ctx.start_gave_up = False
                ctx.start_retry_count = self.start_retry_max
                ctx.last_start_time = now
                ctx.start_armed = True
                pub = self.lane_cmd_pubs.get(name)
                # 로봇별 정지(held) 중인 로봇에는 START 를 보내지 않는다 — START 는 에이전트의 STOP 래치를
                # 푼다(관제 검수 P3). 보내면 다음 틱 STOP 까지 0.1 s 동안 출발한다. 재개하면 재전송이 START 를 낸다.
                if pub and not ctx.held:
                    start_cmd = LaneCommand()
                    start_cmd.command = LaneCommand.CMD_START
                    start_cmd.route_seq = ctx.route_seq
                    pub.publish(start_cmd)
        # 경로 없는 Nav2 로봇: STOPPED·DONE 동안 받은 FleetCommand STOP 이 에이전트를 세워 두는데 START 는 경로 있는
        # 로봇에만 간다 — 그 로봇은 영영 정지로 남아 게이트웨이가 허락한 목표까지 에이전트가 취소했다(직렬 검토).
        # FleetCommand RESUME 은 STOP 래치만 푼다(ESTOP 은 아니다).
        for name, ctx in self.robots.items():
            fleet_pub = self.fleet_cmd_pubs.get(name)
            if ctx.route or ctx.held or ctx.drive_mode != DRIVE_MODE_NAV2 or not fleet_pub:
                continue
            fc = FleetCommand()
            fc.command = FleetCommand.CMD_RESUME
            fleet_pub.publish(fc)
        self.get_logger().info(f"🚀 Fleet Mission Started (CMD_START dispatched with max {self.start_retry_max} retries)!")
        return True

    @_locked
    def stop_fleet(self) -> bool:
        if self.estop_latched:
            # STOP 은 ESTOP 보다 약하다 — 비상정지 중이면 그대로 둔다(10 Hz ESTOP 을 계속 보낸다).
            self.get_logger().warn("Fleet STOP ignored — E-STOP already latched (stronger)")
            return False
        if self.mission_state != MISSION_STOPPED:
            self._pre_stop_state = self.mission_state
        self.mission_state = MISSION_STOPPED
        self._save_control_state()          # G-4: 내보내기 전에 남긴다
        for name, ctx in self.robots.items():
            pub = self.lane_cmd_pubs.get(name)
            if pub and ctx.route:
                stop_cmd = LaneCommand()
                stop_cmd.command = LaneCommand.CMD_STOP
                stop_cmd.route_seq = ctx.route_seq
                pub.publish(stop_cmd)
        self.get_logger().info("🛑 Fleet Mission Stopped.")
        return True

    @_locked
    def estop_fleet(self):
        if self.mission_state != MISSION_ESTOP:
            self._pre_estop_state = self.mission_state
        self.mission_state = MISSION_ESTOP
        self.estop_latched = True
        self._estop_authority = True
        for name, ctx in self.robots.items():
            pub = self.lane_cmd_pubs.get(name)
            if pub:
                estop_cmd = LaneCommand()
                estop_cmd.command = LaneCommand.CMD_ESTOP
                pub.publish(estop_cmd)
        # G-5: 래치는 로봇 보고가 아니라 이 파일로 되살린다. 비상정지만은 **내보낸 뒤** 남긴다 — 디스크(fsync)가 비상정지 앞에
        #     서면 안 된다(통합 검토 GW-N2). 내보낸 뒤 쓰기 전에 죽는 틈은 로봇이 DRIVE_ESTOP 을 보고해 S6 이 래치를 되살린다.
        #     로봇별 정지·플릿 정지는 그 보고가 없어(G-3) 내보내기 전에 남긴다.
        self._save_control_state()
        self.get_logger().warn("🚨 Fleet Emergency Stop Engaged!")
        return True

    @_locked
    def resume_fleet(self):
        """플릿 비상정지·정지의 유일한 출구. **로봇별 정지(HOLD)는 유지한다**(S7).

        🔴 관제 검수 §3.2 S7: 예전엔 여기서 held 까지 지워, 따로 세워 둔 로봇이 ESTOP 해제 순간 달렸다.
           held 로봇에는 RESUME 을 보내지 않는다 — 그 로봇의 ESTOP 래치는 그대로 남고, 로봇 재개(resume_robot)가 함께 푼다.
        """
        was_estop = self.mission_state == MISSION_ESTOP
        back = self._pre_estop_state if was_estop else self.mission_state
        if back == MISSION_STOPPED:
            back = self._pre_stop_state
        # L7: 재개는 정지·비상정지 **직전 상태**로 돌아간다. 달리던 플릿만 다시 달린다 — 한 번도 시작하지 않은 플릿
        #     (IDLE·ASSIGNED·DONE)을 재개로 출발시키지 않는다. 직렬 검토: 비상정지 경로만 막아, 배정→재개 · 배정→정지→재개 ·
        #     배정→정지→비상정지→재개 가 START 를 내고 로봇이 떠났다. 직전 상태를 모르면 출발시키지 않는 쪽으로.
        if back == MISSION_RUNNING:
            self.mission_state = MISSION_RUNNING
        elif back in (MISSION_IDLE, MISSION_ASSIGNED, MISSION_DONE):
            self.mission_state = back
        else:
            self.mission_state = (MISSION_ASSIGNED if any(c.route is not None for c in self.robots.values())
                                  else MISSION_IDLE)
        self._pre_estop_state = None
        self._pre_stop_state = None
        self.estop_latched = False
        self._estop_authority = True
        self.control_note = None            # 운영자가 확인하고 풀었다 — 복원 실패 알림은 여기까지
        self._save_control_state()          # 내보내기 전에 남긴다(쓰기 전에 죽으면 래치로 되살아난다 — 멈추는 쪽)
        if self.mission_state == MISSION_RUNNING:
            # RUNNING 으로 돌아가면, 경로는 있는데 START 응답이 없는 로봇(비상정지 중 배정 등)은 START 재무장
            for c in self.robots.values():
                if c.route is not None and not c.held and not c.start_acknowledged:
                    self._arm_start(c)
        held = [n for n, c in self.robots.items() if c.held]
        if held:
            self.get_logger().warn(f"Fleet resume keeps robot HOLD for {held} — use robot resume for each")
        for name, ctx in self.robots.items():
            if ctx.held:
                continue
            pub = self.lane_cmd_pubs.get(name)
            if pub:
                res_cmd = LaneCommand()
                res_cmd.command = LaneCommand.CMD_RESUME
                res_cmd.route_seq = ctx.route_seq
                pub.publish(res_cmd)
        self.get_logger().info(f"▶️ Fleet Resumed → {self.mission_state}.")
        return True

    # -------------------------------------------------------------------------
    # 상태 브로드캐스트 (2Hz JSON)
    # -------------------------------------------------------------------------
    @_locked
    def get_fleet_status_dict(self) -> Dict[str, Any]:
        robots_dict = {}
        for name, ctx in self.robots.items():
            res_status = self.reservation.status(name)
            comp = ctx.comparator_result or {}

            # 도착 확정 판정
            arrival_status = "NOT_ARRIVED"
            if ctx.arrived:
                if ctx.arrival_confirmed:
                    arrival_status = "ARRIVAL_CONFIRMED"
                else:
                    arrival_status = "ARRIVAL_PENDING"

            robots_dict[name] = {
                "name": name,
                "domain_id": ctx.domain_id,
                "drive_mode": ctx.drive_mode,
                "held": ctx.held,
                # G-3 · REVIEW_20260926 G: 왜 세워 뒀나 — 운영자 · 재시작 뒤 ESTOP 보고 · 링크유실 래치(로봇 재개 필요)
                "held_reason": (ctx.held_reason or HELD_BY_OPERATOR) if ctx.held else None,
                "last_heard_sec": self.last_heard_sec(name),
                # R-7: 로봇이 보고한 지도가 지금 프로파일의 지도와 같은가
                "map_check": self.robot_map_check(name),
                # D6-2 교착 신호 — 요청 없이 허가 지점에 선 지 몇 초인가(없으면 null)
                "stall_sec": (None if ctx.stall_since is None else round(self._now() - ctx.stall_since, 1)),
                # R-5 ③: START 가 먹었는지 — 코디네이터 안에만 있던 값을 드러낸다
                "start": {
                    "acknowledged": ctx.start_acknowledged,
                    "retries_left": ctx.start_retry_count,
                    "gave_up": ctx.start_gave_up,
                },
                "start_node": ctx.start_node,
                "goal_node": ctx.goal_node,
                "route_seq": ctx.route_seq,
                "clear_until_idx": ctx.clear_until_idx,
                # 2026-09-29 교차로 규칙: 정지선에 선 지 몇 초(허가 대기·통과 중), 아니면 null
                "junction_wait_sec": (None if ctx.junction_stop_since is None
                                      else round(self._now() - ctx.junction_stop_since, 1)),
                "held_edges": res_status.get("held_edges", []),
                "waiting_for": res_status.get("waiting_for", ""),
                "blocked_by": res_status.get("blocked_by", ""),
                "is_stale": ctx.is_stale,
                "is_unlocalized": ctx.is_unlocalized,
                "arrival_status": arrival_status,
                "vision_zone_state": ctx.vision_zone_state,
                "comparator": comp,
                "state": {
                    "x": ctx.state.x if ctx.state else None,
                    "y": ctx.state.y if ctx.state else None,
                    "yaw": ctx.state.yaw if ctx.state else None,
                    "localized": ctx.state.localized if ctx.state else False,
                    "linear_velocity": ctx.state.linear_velocity if ctx.state else 0.0,
                    "angular_velocity": ctx.state.angular_velocity if ctx.state else 0.0,
                    # ⚠️ 못 잰 배터리는 NaN 으로 온다 — JSON 의 NaN 은 브라우저 JSON.parse 를 깨뜨리므로 null 로
                    "battery_percent": (ctx.state.battery_percent
                                        if ctx.state and math.isfinite(ctx.state.battery_percent) else None),
                } if ctx.state else None,
                "lane_status": {
                    "drive_state": ctx.lane_status.drive_state if ctx.lane_status else 0,
                    "route_idx": ctx.lane_status.route_idx if ctx.lane_status else -1,
                    "edge_id": ctx.lane_status.edge_id if ctx.lane_status else "",
                    "state_reason": ctx.lane_status.state_reason if ctx.lane_status else "",
                } if ctx.lane_status else None
            }

        return {
            "mission_state": self.mission_state,
            "estop_latched": bool(self.estop_latched),
            "control_note": self.control_notes(),
            "profile": {k: v for k, v in self.profile_status().items() if k not in ('available', 'graph')},
            "warning": self.last_warning,
            "edge_holders": dict(self.reservation.edge_holder),
            "node_holders": dict(self.reservation.node_holder),
            "robots": robots_dict,
            "timestamp": self._now()
        }

    @_locked
    def _publish_status(self):
        status_data = self.get_fleet_status_dict()
        msg = String()
        msg.data = json.dumps(status_data)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RelayFleetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
