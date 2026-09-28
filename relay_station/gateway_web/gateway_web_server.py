#!/usr/bin/env python3
"""
Field Gateway Web Streaming Server (with Live Gazebo Stream & Nav2 Click-to-Move)
- 현장 태블릿 카메라 스트림 1:N 팬아웃 (/video_feed)
- 현장 관제 화면 스트림 1:N 팬아웃 (/control_feed)
- Gazebo 실시간 3D 탑뷰 카메라 스트림 (/gazebo_feed) - ROS 2 /camera 토픽 30fps
- 로봇 1, 2 온보드 카메라 스트림 (/robot_camera_feed?id=robot1 | robot2)
- 로봇 1, 2 실시간 좌표 및 네비게이션 상태 API (/api/status)
- 로봇 1 실시간 자율주행 목표 전송 API (/api/robot1/goal, /api/robot1/mission, /api/robot1/stop)
- 실시간 로그 API (/api/logs) 및 gateway.log 파일 기록
- 통합 반응형 웹 대시보드 서빙 (/)
"""

import os
import sys
import io
import json
import time
import threading
from urllib.parse import urlparse, parse_qs
import urllib.request
import http.cookiejar
import base64
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import psutil
import math
import signal
import socket

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image as RosImage, CompressedImage
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge

try:
    from pinky_lane_msgs.msg import PoseFix as RosPoseFix
except Exception:
    RosPoseFix = None
try:
    from pinky_lane_msgs.msg import LaneStatus as RosLaneStatus
except Exception:
    RosLaneStatus = None

from stream_ingest import TabletStreamIngest
import ops_view
from vision_path import (BRIDGE_NODE_PREFIX, classify_receivers,  # noqa: F401
                         downstream_contract as vision_downstream_contract)
from mjpeg_serving import (STREAM_KEEPALIVE_SEC, should_send_frame,  # noqa: F401
                           write_mjpeg_frame)
from source_registry import (
    SourceRegistry, Source, PUSH, PULL, ROS, LOCAL, TRUSTED, UNTRUSTED,
    jpeg_only_provider, readiness, clock_alignment, SEVERITY_ORDER,
)
from mjpeg_puller import MjpegPuller, load_pull_sources, DEFAULT_CONFIG_PATH
from local_camera import LocalCameraSource, load_local_sources
from control_renderer import ControlScreenRenderer
import vision_ingest
from tablet_camera_relay_node import TabletCameraRelayNode
from stream_enhancer import StreamEnhancer
from calibration import (CalibrationStore, CalibrationError, load_arena,
                         STATE_SETTLED, CORNERS, project)
import censorship
import drift
import framing
import masks
import vision_world

FLEET_IMPORT_ERROR = None
try:
    import sys
    # 실행본(~/doc/src/field_gateway_relay)은 **파일별 심링크**다(G-B). abspath 는 링크를 안 풀어 그 폴더의 부모
    # (~/doc/src)를 relay 루트로 봤고, `fleet` 이 없어 코디네이터가 **조용히** 빠졌다 — 실물 게이트웨이는 늘
    # FLEET_COORDINATOR_UNAVAILABLE 이었다(2026-09-26 발견). realpath 로 레포의 relay_station 을 본다.
    _RELAY_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    if _RELAY_ROOT not in sys.path:
        sys.path.insert(0, _RELAY_ROOT)
    from fleet.fleet_coordinator import RelayFleetCoordinator
    from fleet.fleet_coordinator import HELD_ESTOP_RESTORED, HELD_LINK_LOST
except Exception as _fleet_exc:                               # noqa: BLE001 — 없어도 게이트웨이는 뜬다. 이유는 남긴다
    RelayFleetCoordinator = None
    HELD_ESTOP_RESTORED = HELD_LINK_LOST = None               # 코디네이터가 없으면 이 이름을 쓰는 곳까지 오지 않는다(503)
    FLEET_IMPORT_ERROR = f'{type(_fleet_exc).__name__}: {_fleet_exc}'


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(SCRIPT_DIR, 'static')
LOG_FILE = os.path.join(SCRIPT_DIR, 'gateway.log')

# 전역 인스턴스
GLOBAL_STREAM_ENHANCER = StreamEnhancer()

# Track R (R-D1, R-D2): 비전 API (PoseFix, ZoneEvent) 공유 키 인증 (Fail-Closed)
# 키는 환경변수로만 준다(RELAY_VISION_API_KEY 또는 RELAY_VISION_TOKEN). 기본값은 없다 —
# 키가 없으면 비전 API 는 전부 401 이다(fail-closed). 원 저장소 판은 기본 키를 코드에 두고, 키가 비면 통과시켰다.
VISION_API_KEY = os.environ.get('RELAY_VISION_API_KEY') or os.environ.get('RELAY_VISION_TOKEN') or ''

# R-D4: Production 경로는 HTTP -> ROS topic -> Coordinator callback 하나로 유지.
# Direct fallback은 명시적 테스트/미니멀 모드(RELAY_ALLOW_DIRECT_FALLBACK=1)에서만 허용.
ALLOW_DIRECT_FALLBACK = os.environ.get('RELAY_ALLOW_DIRECT_FALLBACK', '0').lower() in ('1', 'true', 'yes')

# 주행·관측·캘리브레이션처럼 **현장에서만** 눌러야 하는 명령의 발신 허용 IP.
# ⚠️ 198.51.100.3 은 이 중계의 LAN 주소다 - 현장마다 바뀐다(오늘도 129.150 과 병존).
#    바뀌면 조작자 본인이 막히는 쪽으로 실패한다(fail-closed). 열리는 쪽이 아니다.
_BASE_LOCAL_CONTROL_IPS = ('127.0.0.1', '::1', 'localhost', '198.51.100.3')


def _container_gateway_ips():
    """컨테이너 안일 때만, 기본 게이트웨이(= 호스트)를 로컬로 친다.

    ⭐ 도커에서는 호스트가 172.x.0.1 로 보인다. 조작자가 자기 기계에서 눌러도 막힌다.
    ⭐ 환경변수로 게이트를 넓히지 **않는다** — 현장 장비에서 누가 켜면 그대로 구멍이다.
       컨테이너 여부는 현장 노트북이 만족시킬 수 없는 조건이라 자기제한적이다.
       게다가 복제본은 127.0.0.1 에만 바인드하므로 여기 닿는 것은 호스트 자신뿐이다.
    """
    if not os.path.exists('/.dockerenv'):
        return ()
    out = []
    try:
        with open('/proc/net/route', encoding='ascii') as fh:
            next(fh, None)
            for line in fh:
                cols = line.split()
                if len(cols) < 3 or cols[1] != '00000000':
                    continue
                raw = int(cols[2], 16)
                out.append('.'.join(str((raw >> (8 * i)) & 0xFF) for i in range(4)))
    except (OSError, ValueError, StopIteration):
        return ()
    return tuple(out)


LOCAL_CONTROL_IPS = _BASE_LOCAL_CONTROL_IPS + _container_gateway_ips()

# D7 (2026-09-24): 로봇 정지·재개 경로 → 플릿 로봇 이름. `/api/stop`·`/api/nav/stop` 은 예전처럼 1호기다.
ROBOT_STOP_PATHS = {
    '/api/robot1/stop': 'pinky1', '/api/stop': 'pinky1', '/api/nav/stop': 'pinky1',
    '/api/robot2/stop': 'pinky2',
}
ROBOT_RESUME_PATHS = {'/api/robot1/resume': 'pinky1', '/api/robot2/resume': 'pinky2'}
# 멈추는 플릿 명령 — 원격에서도 받는다
FLEET_STOP_COMMANDS = ('stop', 'estop')
# 정지 응답의 "로봇 소식이 살아 있나" 문턱. 에이전트는 RobotState·LaneStatus 를 10 Hz 로 낸다.
ROBOT_HEARD_RECENTLY_SEC = 2.0
# R4 (관제 검수 §3.3): 움직이게 하는 제어(시작·재개·배정·로봇 재개)는 코디네이터가 **처리한 결과**를 잠깐 기다려 답한다
CONTROL_APPLY_WAIT_SEC = 0.5


def wait_control_result(coord, seq0, cmd, timeout=None):
    """코디네이터가 seq0 뒤에 처리한 이 명령의 결과 {'ok': bool|None, 'mission_state': ...}. 시간 안에 없으면 None."""
    if coord is None:
        return None
    deadline = time.monotonic() + (CONTROL_APPLY_WAIT_SEC if timeout is None else timeout)
    while time.monotonic() < deadline:
        last = coord.last_control
        if last and last.get('seq', 0) > seq0 and last.get('cmd') == cmd:
            return last
        time.sleep(0.02)
    return None


def applied_reply(result, base):
    """(HTTP 코드, 본문). 적용 200 · 거부 409 NOT_APPLIED · 결과 모름 202."""
    body = dict(base, dispatched=True)
    if result is None:
        body.update(success=False, applied=None,
                    message=body.get('message', '') + ' — 코디네이터가 처리했는지 아직 모른다')
        return 202, body
    body.update(applied=result.get('ok'), mission_state=result.get('mission_state'))
    if result.get('ok') is False:
        body.update(success=False, reason='NOT_APPLIED',
                    message='코디네이터가 거부했다(비상정지 래치·배정 충돌 등) — 상태: %s' % result.get('mission_state'))
        return 409, body
    body['success'] = True
    return 200, body


# R-7 웹 전환: 좌표 프로파일 — 움직이는 로봇이 없을 때 로컬에서만 바꾼다(PLAN_20260925_MAP4 §2 '섞인 상태를 만들지 않는다')
PROFILE_COMMANDS = {'/api/fleet/profile': 'profile', '/api/fleet/robot_maps': 'robot_maps',
                    '/api/fleet/initial_poses': 'initial_poses'}


def profile_map_yaml(coord):
    """지금 프로파일의 화면 지도 yaml — 프로파일이 지도를 싣지 않으면 게이트웨이 기본(--map-yaml)."""
    prof = coord.profiles.get(coord.active_profile) if (coord is not None and coord.active_profile) else None
    return (prof.map_yaml if prof is not None and prof.map_yaml else None) or GLOBAL_DEFAULT_MAP_YAML


def sync_renderer_map():
    if GLOBAL_CONTROL is not None:
        path = profile_map_yaml(GLOBAL_FLEET_COORDINATOR)
        if path and path != getattr(GLOBAL_CONTROL, 'map_yaml_path', None):
            src = GLOBAL_CONTROL.set_map(path)
            app_log(f"[Profile] 화면 지도 → {path} (source={src})")


def renderer_map_meta():
    """화면(렌더러)이 **실제로** 읽은 지도의 규격. 예전 `map_meta` 는 하드코딩이라 화면과 다른 좌표계였다(U-11)."""
    c = GLOBAL_CONTROL
    if c is None:
        return None
    src = getattr(c, 'map_source', None)
    prof = getattr(GLOBAL_FLEET_COORDINATOR, 'active_profile', None)
    # 문자열만 싣는다 — /api/status 전체가 이 한 칸 때문에 직렬화에 실패하면 안 된다
    return {'width': int(getattr(c, 'map_w', 0)), 'height': int(getattr(c, 'map_h', 0)),
            'resolution': float(getattr(c, 'resolution', 0.0)),
            'origin': [float(v) for v in list(getattr(c, 'origin', [0.0, 0.0, 0.0]))[:3]],
            'image_url': '/api/fleet/profile_map.png',
            'source': src if isinstance(src, str) else None,
            'synthetic': src == 'synthetic',
            'profile': prof if isinstance(prof, str) else None}


# 정지 응답이 로봇의 멈춤 보고를 기다리는 한도 — LaneStatus 10 Hz 라 보통 0.2 s 안에 온다(HTTP 는 스레드 서버)
STOP_CONFIRM_WAIT_SEC = 1.0
STOP_CONFIRM_POLL_SEC = 0.05
def parse_goal_body(req):
    """(x, y, yaw) 또는 거부 이유(문자열). 움직이는 명령은 모자란 값을 0 으로 채우지 않는다."""
    if not isinstance(req, dict):
        return '본문이 JSON 객체가 아니다'
    if 'x' not in req or 'y' not in req:
        return 'x·y 가 없다'
    out = []
    for key, default in (('x', None), ('y', None), ('yaw', 0.0)):
        v = req.get(key, default)
        if isinstance(v, bool):
            return '%s 가 수가 아니다' % key
        try:
            f = float(v)
        except (TypeError, ValueError):
            return '%s 가 수가 아니다: %r' % (key, v)
        if not math.isfinite(f):
            return '%s 가 유한한 수가 아니다: %r' % (key, v)
        out.append(f)
    return tuple(out)


# 통합 검토 OPS-10: 미션 API 가 받는 값 → /robot1/mission_cmd 로 내는 명령. 옛 내비게이터(robot1_mission_navigator
# `_cb_mission_cmd`)가 실제로 아는 명령만 둔다 — 이 API 는 늘 'mission' + 값을 냈으므로 끝까지 닿는 값은 1·2 뿐이었다
# ('point1'·'mission1' 은 'missionpoint1'·'missionmission1' 이 되어 내비게이터가 버렸다 — 202 로 보냈다고만 했다).
MISSION_COMMANDS = {'1': 'mission1', '2': 'mission2'}


def parse_mission_body(req):
    """(미션 값, None) 또는 (None, 거부 이유). 움직이는 명령은 빠진 값을 기본 미션으로 채우지 않는다."""
    if not isinstance(req, dict):
        return None, '본문이 JSON 객체가 아니다'
    if 'mission' not in req:
        return None, 'mission 이 없다'
    v = req['mission']
    if isinstance(v, bool) or not isinstance(v, (str, int)):
        return None, 'mission 이 글자·정수가 아니다: %r' % (v,)
    m = str(v).strip()
    if m not in MISSION_COMMANDS:
        return None, '모르는 미션 %r — 받는 값: %s (정지는 /api/robot1/stop)' % (m, ', '.join(sorted(MISSION_COMMANDS)))
    return m, None


# S1 (관제 검수 REVIEW_20260925 §3.2): 에이전트를 거치지 않고 bt_navigator 로 곧장 가는 목표·미션 경로.
#   /robot1/goal_pose·mission_cmd → 브리지 → 로봇. 에이전트의 래치·체인을 하나도 안 거치므로 여기서 막는다.
DIRECT_MOTION_PATHS = {'/api/robot1/goal': 'pinky1', '/api/goal': 'pinky1', '/api/robot1/mission': 'pinky1'}


# 통합 검토 OPS-5: 로봇이 래치를 쥐고 있다는 LaneStatus.drive_state — ESTOP(8)·LINK_LOST(9). 에이전트는 그동안 목표를
# 곧바로 취소하고 지도 교체(SET_MAP)를 거부한다.
LATCHED_DRIVE_STATES = ({RosLaneStatus.DRIVE_ESTOP: 'ESTOP 래치', RosLaneStatus.DRIVE_LINK_LOST: '링크유실 래치'}
                        if RosLaneStatus is not None else {8: 'ESTOP 래치', 9: '링크유실 래치'})
# 에이전트 RELEASE_HOLD_REASON('해제 보류 — Nav2 취소 미확인' + 꼬리표)의 머리 — 래치는 풀렸지만 Nav2 취소를 확인하기 전이라
# 문(/estop)이 닫혀 있다. 이 동안 들어온 목표도 에이전트가 취소한다.
RELEASE_HOLD_PREFIX = '해제 보류'
# 통합 검토 UI-R1: 로봇 스스로의 래치 보고를 '지금' 으로 믿는 한도(초). 에이전트는 LaneStatus 를 10 Hz(state_rate)로 낸다 —
# 이보다 오래 보고가 없으면 로봇이 조용한 것(전원·배터리 교체·에이전트 정지)이고, 그 마지막 보고는 로봇 재개로도 바뀌지 않는다.
LATCH_REPORT_FRESH_SEC = 3.0


def latch_report_age(coord, ctx):
    """로봇의 마지막 LaneStatus 가 몇 초 전인가(코디네이터 시계). 모르면 None."""
    try:
        return max(0.0, float(coord.now()) - float(getattr(ctx, 'lane_status_time', 0.0) or 0.0))
    except Exception:
        return None


def robot_latch_report(ctx):
    """로봇이 LaneStatus 로 보고하는 래치·해제 보류(글자). 없으면 None — 보고가 없으면 모르는 것이지 래치가 아니다."""
    ls = getattr(ctx, 'lane_status', None)
    if ls is None:
        return None
    what = LATCHED_DRIVE_STATES.get(ls.drive_state)
    if what:
        return what
    reason = str(getattr(ls, 'state_reason', '') or '')
    return reason if reason.startswith(RELEASE_HOLD_PREFIX) else None


# 우회 이동 거부의 HTTP 코드 — 기본 409(지금은 안 된다). 판정 자체를 못 하는 것은 503(제3자 검수 G-1·G-10).
MOTION_BLOCK_HTTP = {'NO_FLEET_COORDINATOR': 503, 'ROBOT_NOT_IN_FLEET': 503}


def motion_block_reason(coord, robot):
    """에이전트를 우회하는 이동 명령을 지금 받으면 안 되는 이유 (None = 받아도 된다).

    ESTOP 래치·플릿 정지·로봇별 정지 중 목표가 들어가면 RESUME 이 `/estop false` 를 내는 순간 아무도 허가하지 않은
    곳으로 달린다(S2) — 리그에서 ESTOP 중 `/api/robot1/goal` 이 200 을 돌려줬다(S1).
    """
    if coord is None:
        # 제3자 검수 G-1: 예전엔 여기서 None(=받아도 된다)이라 코디네이터가 import·생성에 실패한 게이트웨이(실물이 09-26 까지
        # 그랬다 — REPLY §14-3)에서 우회 목표·미션이 200 이었다. 래치·정지·HOLD 를 확인할 수 없으면 받지 않는다.
        return 'NO_FLEET_COORDINATOR', '거부 — 플릿 코디네이터가 없어 비상정지·정지·로봇별 정지를 확인할 수 없다'
    if coord.estop_latched or coord.mission_state == 'ESTOP':
        return 'FLEET_ESTOP', '거부 — 플릿 비상정지 중이다. 해제는 /api/fleet/resume'
    if coord.mission_state == 'STOPPED':
        return 'FLEET_STOPPED', '거부 — 플릿이 정지 상태다. 재개 뒤에 보낸다'
    if coord.mission_state == 'DONE':
        # 미션 완료 뒤에도 코디네이터는 Nav2 로봇에 STOP 을 10 Hz 로 보낸다 — 에이전트가 이 목표를 곧바로 취소한다
        return 'FLEET_DONE', '거부 — 미션 완료 상태라 코디네이터가 로봇을 세워 두고 있다. 새 배정·시작 뒤에 보낸다'
    robots = getattr(coord, 'robots', None)
    ctx = robots.get(robot) if isinstance(robots, dict) else None
    if ctx is None:
        # 제3자 검수 G-10: 예전엔 'pinky1' 을 못 찾으면(미션이 로봇 이름을 바꾸면) HOLD 검사를 조용히 건너뛰었다.
        return 'ROBOT_NOT_IN_FLEET', '거부 — 플릿 코디네이터에 %s 가 없어 로봇별 정지를 확인할 수 없다' % robot
    if ctx.held:
        why = getattr(ctx, 'held_reason', '') or '로봇별 정지'
        return 'ROBOT_HELD', '거부 — %s 가 로봇별 정지(HOLD: %s) 중이다. /api/%s/resume 뒤에 보낸다' % (
            robot, why, robot.replace('pinky', 'robot'))
    latched = robot_latch_report(ctx)
    age = latch_report_age(coord, ctx) if latched else None
    if latched and age is not None and age > LATCH_REPORT_FRESH_SEC:
        # 통합 검토 UI-R1: 조용한 로봇의 옛 래치 보고로도 우회 목표는 계속 거부한다(움직이는 쪽은 닫힌 채로 — 에이전트가 없으면
        # 래치도 취소도 지키는 쪽이 없다). 다만 '로봇 재개 뒤에' 라고 하지 않는다 — 로봇 재개는 옛 보고를 바꾸지 못한다.
        return 'ROBOT_LATCHED', ('거부 — %s 의 마지막 보고(%.0f s 전): %s — 그 뒤로 보고가 없다(에이전트·링크 확인). '
                                 '로봇이 래치가 풀렸다고 다시 보고해야 받는다' % (robot, age, latched))
    if latched:
        # 통합 검토 OPS-5: 예전엔 로봇이 ESTOP·링크유실 래치나 해제 보류를 보고하는 중에도 목표가 200 이었다 — 에이전트는
        # 그 목표를 곧바로 취소한다(정지·래치·해제 대기 중 남의 goal 은 전부 취소). 받아 놓고 안 가는 것을 성공이라 하지 않는다.
        return 'ROBOT_LATCHED', '거부 — %s 보고: %s — 에이전트가 이 목표를 곧 취소한다. %s' % (
            robot, latched, '해제(Nav2 취소 확인)를 기다린 뒤 보낸다' if latched.startswith(RELEASE_HOLD_PREFIX)
            else '로봇 재개(/api/%s/resume) 뒤에 보낸다' % robot.replace('pinky', 'robot'))
    if coord.mission_state == 'RUNNING' and ctx.route is not None:
        # 제3자 검수 G-6: 플릿 미션이 달리는 동안 경로가 있는 로봇에 우회 목표를 넣으면 예약(공유 구간 중재)을 거치지 않고
        # Nav2 로 간다 — 두 대가 달릴 때 충돌 방지를 비켜 가는 길이다. IDLE·ASSIGNED(시작 전)는 그대로 받는다.
        return 'FLEET_RUNNING', ('거부 — 플릿 미션이 달리는 중이고 %s 에 경로가 있다. 우회 목표는 예약(공유 구간 중재)을 '
                                 '거치지 않는다 — 플릿이 시작 전(IDLE·ASSIGNED)일 때만 받는다' % robot)
    return None


def latched_robots(coord):
    """지금 지도 교체(SET_MAP)를 보내면 에이전트가 거부할 로봇 — [(이름, 이유)] (통합 검토 OPS-3).

    코디네이터가 링크유실·ESTOP 래치로 세워 둔 로봇, 또는 스스로 ESTOP·링크유실 래치를 **지금**(LATCH_REPORT_FRESH_SEC 안)
    보고하는 로봇. 에이전트는 체인 래치(ESTOP·링크유실) 중 SET_MAP 을 REFUSED 한다 — 로봇 재개(LaneCommand RESUME)만 그
    래치를 푼다.
    """
    out = []
    with getattr(coord, '_coord_lock', None) or threading.RLock():
        for name, ctx in coord.robots.items():
            if ctx.held and ctx.held_reason in (HELD_LINK_LOST, HELD_ESTOP_RESTORED):
                out.append((name, ctx.held_reason))
                continue
            ls = getattr(ctx, 'lane_status', None)
            what = LATCHED_DRIVE_STATES.get(ls.drive_state) if ls is not None else None
            age = latch_report_age(coord, ctx) if what else None
            # 통합 검토 UI-R1: 예전엔 옛 보고도 셌다 — ESTOP·링크유실을 마지막으로 보고하고 조용해진 로봇(전원·배터리 교체·
            # 에이전트 정지) 하나 때문에 ②·③ 이 플릿 전체에 409 였고, 로봇 재개는 옛 보고를 못 바꿔 풀 길이 없었다(54bb60b 는
            # 성한 로봇에 보냈다). 조용한 로봇은 어차피 SET_MAP 을 못 받는다 — 막지 않고 보낸다(지도 대조가 그 로봇을 보인다).
            if what and age is not None and age <= LATCH_REPORT_FRESH_SEC:
                out.append((name, '%s 보고 %.1f s 전' % (what, age)))
    return out


def fleet_resume_target(coord):
    """플릿 재개(resume_fleet)를 지금 누르면 갈 상태 — 코디네이터의 규칙(L7)을 읽기만 한다(통합 검토 OPS-6).

    STOPPED 는 정지 직전 상태로 돌아간다: 달리던 플릿(RUNNING)만 다시 달리고, IDLE·ASSIGNED·DONE 은 그대로, 모르면
    (재시작 뒤 복원한 STOPPED 등) 경로가 있으면 ASSIGNED 아니면 IDLE. DONE 은 DONE 에 남는다.
    """
    with getattr(coord, '_coord_lock', None) or threading.RLock():
        back = coord.mission_state
        if back == 'STOPPED':
            back = getattr(coord, '_pre_stop_state', None)
        if back in ('RUNNING', 'IDLE', 'ASSIGNED', 'DONE'):
            return back
        return 'ASSIGNED' if any(c.route is not None for c in coord.robots.values()) else 'IDLE'


def motion_block_reply(blocked):
    """(HTTP 코드, 본문) — 우회 이동 거부."""
    body = {'success': False, 'reason': blocked[0], 'message': blocked[1], 'dispatched': False}
    if blocked[0] == 'NO_FLEET_COORDINATOR':
        body['detail'] = FLEET_IMPORT_ERROR
    return MOTION_BLOCK_HTTP.get(blocked[0], 409), body


# R-5: 진단을 받는 로봇 (브리지 업링크 /pinkyN/diag 가 있는 로봇 — robot1/2 한정)
OPS_ROBOTS = ('pinky1', 'pinky2')


class ObservationSession:
    """관측 세션 (MCV-1C, 계획서 §4.5.4 상태기계의 첫 조각: 켜짐/꺼짐).

    켜지면 로컬 캠을 잡아(hold) 두고, 꺼지면 놓는다 - LED 는 그 결과다.
    /api/safety 는 이 상태로 실물 시점 부족의 심각도를 정한다(꺼짐=info, 켜짐=safety).
    """

    def __init__(self):
        self.active = False
        self.since = 0.0

    def set_active(self, active, cams):
        self.active = bool(active)
        self.since = time.time()
        for cam in (cams or []):
            try:
                cam.hold(self.active)
            except Exception as exc:
                app_log(f"[Observe] hold({self.active}) 실패 {getattr(cam, 'name', cam)}: {exc}")

    def public(self):
        return {"active": self.active, "since": self.since}


GLOBAL_OBSERVATION = ObservationSession()
FLEET_DOMAINS_PATH = os.environ.get(
    "FLEET_DOMAINS_ENV",
    os.path.expanduser("~/pinky_pro/src/pinky_pro_team11/relay_station/configs/fleet_domains.env"),
)


def read_fleet_domains():
    """MCV-0C: 도메인 값을 코드에 박지 않고 단일 진실 공급원에서 읽는다."""
    vals = {}
    try:
        with io.open(FLEET_DOMAINS_PATH, encoding="utf-8") as fh:
            for raw in fh:
                line = raw.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                vals[k.strip()] = v.strip().strip('"').strip("'")
    except Exception:
        pass
    for k in ("RELAY_DOMAIN_ID", "TEAM_DOMAIN_ID",
              "ROBOT1_DOMAIN_ID", "ROBOT2_DOMAIN_ID",
              "ROBOT3_DOMAIN_ID", "ROBOT4_DOMAIN_ID"):
        if k in vals:
            try:
                vals[k] = int(vals[k])
            except ValueError:
                pass
    return vals


GLOBAL_INGEST = None
GLOBAL_CONTROL = None
GLOBAL_DEFAULT_MAP_YAML = None     # 게이트웨이 --map-yaml — 지도를 싣지 않는 프로파일(legacy)의 화면 지도
GLOBAL_ROBOT_CAMERAS = None
GLOBAL_GAZEBO_CAM = None
GLOBAL_NETWORK_MONITOR = None
GLOBAL_REGISTRY = None
# MCV-2A-UI 캘리브레이션. 아레나 명세가 없으면 arena=None 이고 모든 소스가
# UNCALIBRATED 로 남는다 - 목적지 좌표를 모르는데 정착시킬 수는 없다.
GLOBAL_CALIB = CalibrationStore(load_arena())
# MCV-2C 검열을 인정할 구현 목록. 파일이 없으면 **아무도 인정하지 않는다**(fail-closed).
GLOBAL_CENSORSHIP_POLICY = censorship.load_policy()


# G-A(2026-09-09): jpeg 유무로 connected 를 근사하던 지역 구현을 지우고
# source_registry.jpeg_only_provider 로 옮겼다 - 바이트 변화로 판정하고, 테스트된다.
_jpeg_only_provider = jpeg_only_provider
GLOBAL_ROBOT_SUB_NODE = None
GLOBAL_FLEET_COORDINATOR = None
# 연산 노드(태블릿)가 낸 좌표를 받는 자리. 신선도 판정은 이 저장소가 한다 —
# 낡은 좌표를 화면에 계속 보여 주지 않는 것이 목적이다(vision_ingest 독스트링).
GLOBAL_VISION = vision_ingest.VisionPoseStore()
VISION_ROBOT_IDS = ('robot1', 'robot2')
LOG_BUFFER = []
LOG_LOCK = threading.Lock()


def _with_vision_path(report):
    """신선도 보고에 `receivers`·`downstream` 을 덧붙인다.

    ⭐ 신선도("값이 왔나")와 경로("어디까지 갔나")는 **다른 사실**이다. 화면이 둘을 같이
       봐야 "보냈는데 아무도 안 듣는다" 와 "아무도 안 보낸다" 를 가른다.
    ⚠️ 못 잰 자리는 `null` 로 둔다 — 0 으로 채우지 않는다.
    """
    if not isinstance(report, dict):
        return report
    for rid, item in report.items():
        if not isinstance(item, dict):
            continue
        item['downstream'] = vision_downstream_contract(rid)
        item['receivers'] = None
        if GLOBAL_ROBOT_SUB_NODE:
            try:
                item['receivers'] = GLOBAL_ROBOT_SUB_NODE.vision_pose_receivers(rid)
            except Exception:
                item['receivers'] = None
    return report


def app_log(msg):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{timestamp}] {msg}"
    print(line, flush=True)
    with LOG_LOCK:
        LOG_BUFFER.append(line)
        if len(LOG_BUFFER) > 200:
            LOG_BUFFER.pop(0)
        try:
            with open(LOG_FILE, 'a', encoding='utf-8') as f:
                f.write(line + "\n")
        except Exception:
            pass


class GazeboCameraManager:
    """Gazebo 월드 내 탑뷰 카메라(/camera 토픽) 실시간 스트리밍 관리"""
    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = self._make_placeholder()
        self._stamp = 0.0
        self._connected = False

    def _make_placeholder(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (25, 30, 40)
        cv2.putText(img, "GAZEBO LIVE CAMERA", (140, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, "Waiting for /camera topic...", (140, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 1, cv2.LINE_AA)
        _, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return jpeg.tobytes()

    def update_frame(self, cv_img):
        _, jpeg = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        jpeg_bytes = jpeg.tobytes()
        stamp = time.time()
        with self._lock:
            if not self._connected:
                app_log("[GazeboCam] Live /camera stream connected!")
                self._connected = True
            self._jpeg = jpeg_bytes
            self._stamp = stamp
        if GLOBAL_STREAM_ENHANCER and cv_img is not None:
            try:
                GLOBAL_STREAM_ENHANCER.update_gazebo_reference(cv_img)
            except Exception:
                pass

    def get_latest_jpeg(self):
        with self._lock:
            return self._jpeg

    def get_latest(self):
        """(jpeg, stamp, connected) - 콜백 스탬프로 판정한다.

        바이트 변화로 판정하면 정지 장면(같은 JPEG)이 '끊김'으로 오판된다 -
        2026-09-10 실측: /camera Publisher 1 인데 gazebo connected=False.
        콜백이 최근에 왔는지가 진실이다. jpeg 는 그대로 준다(화면은 거짓이 아니다).
        """
        with self._lock:
            jpeg = self._jpeg
            stamp = self._stamp
        connected = stamp > 0.0 and (time.time() - stamp) <= 3.0
        return jpeg, stamp, connected


class RobotCameraManager:
    """로봇 1, 2의 온보드 카메라 프레임을 관리하고 웹 스트리밍용 JPEG 제공"""
    def __init__(self):
        self._lock = threading.Lock()
        self._cameras = {
            'robot1': {
                'topic': '/robot1/camera/image_raw',
                'stamp': 0.0,
                'jpeg': self._make_placeholder('Pinky #1', '/robot1/camera/image_raw'),
                'connected': False
            },
            'robot2': {
                'topic': '/robot2/camera/image_raw',
                'stamp': 0.0,
                'jpeg': self._make_placeholder('Pinky #2', '/robot2/camera/image_raw'),
                'connected': False
            }
        }

    def _make_placeholder(self, robot_name, topic):
        img = np.zeros((360, 640, 3), dtype=np.uint8)
        img[:] = (22, 26, 34)
        cv2.rectangle(img, (40, 40), (600, 320), (45, 55, 70), -1)
        cv2.rectangle(img, (40, 40), (600, 320), (0, 180, 255), 2)
        cv2.putText(img, f"ROBOT ONBOARD CAMERA: {robot_name}", (70, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, "Waiting for Camera Topic...", (70, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"Topic: {topic}", (70, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 195, 210), 1, cv2.LINE_AA)
        _, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return jpeg.tobytes()

    def update_frame(self, robot_id, cv_img):
        _, jpeg = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        jpeg_bytes = jpeg.tobytes()
        stamp = time.time()
        with self._lock:
            if robot_id in self._cameras:
                if not self._cameras[robot_id]['connected']:
                    app_log(f"[RobotCam] First frame received for {robot_id}!")
                self._cameras[robot_id]['jpeg'] = jpeg_bytes
                self._cameras[robot_id]['stamp'] = stamp
                self._cameras[robot_id]['connected'] = True

    def update_compressed(self, robot_id, jpeg_bytes):
        stamp = time.time()
        with self._lock:
            if robot_id in self._cameras:
                if not self._cameras[robot_id]['connected']:
                    app_log(f"[RobotCam] First compressed frame received for {robot_id}!")
                self._cameras[robot_id]['jpeg'] = bytes(jpeg_bytes)
                self._cameras[robot_id]['stamp'] = stamp
                self._cameras[robot_id]['connected'] = True

    def get_latest_jpeg(self, robot_id='robot1'):
        with self._lock:
            cam = self._cameras.get(robot_id, self._cameras['robot1'])
            return cam['jpeg']


class CpuSampler:
    """U-5. CPU 사용률을 **배경에서** 꾸준히 잰다.

    ⭐ 요청 처리 중에 `interval` 을 줘서 재면 그 요청이 그만큼 멈춘다. 그렇다고
       `interval=None` 으로 부르면 "직전 호출 이후" 라서 첫 호출이 0.0 이고, 스레드가
       여럿이면 간격이 0 에 가까워 또 0.0 이 된다. 그래서 **한 스레드가 혼자** 잰다.
    ⭐ 아직 한 번도 못 쟀으면 `None` 이다 — 0 이 아니다. 안 잰 값을 숫자로 쓰지 않는다.
    """

    PERIOD_S = 2.0

    def __init__(self):
        self._v = None
        self._lock = threading.Lock()
        t = threading.Thread(target=self._worker, name="cpu-sampler", daemon=True)
        t.start()

    def _worker(self):
        while True:
            try:
                # interval 을 주면 그 구간을 실제로 재고 온다 - 배경 스레드라 막혀도 된다.
                v = psutil.cpu_percent(interval=self.PERIOD_S)
            except Exception:                     # noqa: BLE001 - 못 재면 모른다로 둔다
                v = None
            with self._lock:
                self._v = v

    def value(self):
        with self._lock:
            return self._v


GLOBAL_CPU = CpuSampler()


class NetworkLatencyMonitor:
    """백그라운드에서 병렬 스레드로 각 장비(로봇, 태블릿, 공유기, 외부망)의 Ping 지연시간 동시 측정"""
    def __init__(self):
        self.targets = {
            # ⭐ 고정 설비만 핑한다. 태블릿·폰은 **들고 다니는** 기기라 주소가
            #    사이트마다 바뀌고, 여기 박아 두면 낡은 주소로 TIMEOUT 을 보고한다.
            #    그 기기들의 도달 여부는 소스 자신이 말한다(activeUrl·receiveFps) -
            #    후보 목록을 시도해 **실제로 붙은 주소**를 아는 쪽이 더 정확하다.
            'router': ('198.51.100.1', '공유기 (Router)'),
            'robot1': ('198.51.100.5', 'Robot #1 (Main)'),
            'robot2': ('198.51.100.6', 'Robot #2 (Sub)'),
            'robot3': ('198.51.100.8', 'Robot #3 (Add)'),
            'internet': ('8.8.8.8', '외부 인터넷 (WAN)')
        }
        self.results = {
            key: {'ip': ip, 'name': name, 'online': False, 'rtt_ms': None}
            for key, (ip, name) in self.targets.items()
        }
        self.lock = threading.Lock()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _ping_one(self, key, ip, name):
        import subprocess, re
        try:
            res = subprocess.run(['ping', '-c', '1', '-W', '1', ip], capture_output=True, text=True, timeout=1.5)
            m = re.search(r'time=([0-9.]+)\s*ms', res.stdout)
            if m:
                return key, {'ip': ip, 'name': name, 'online': True, 'rtt_ms': float(m.group(1))}
        except Exception:
            pass
        return key, {'ip': ip, 'name': name, 'online': False, 'rtt_ms': None}

    def _worker(self):
        from concurrent.futures import ThreadPoolExecutor
        while True:
            current = {}
            with ThreadPoolExecutor(max_workers=6) as executor:
                futures = [executor.submit(self._ping_one, k, ip, name) for k, (ip, name) in self.targets.items()]
                for f in futures:
                    try:
                        k, val = f.result()
                        current[k] = val
                    except Exception:
                        pass
            with self.lock:
                self.results = current
            time.sleep(2.0)

    def get_status(self):
        with self.lock:
            return dict(self.results)


def get_jenkins_status():
    auth_str = base64.b64encode(b'admin:admin1234').decode('utf-8')
    res = {
        'online': False,
        'in_queue': False,
        'last_build_number': None,
        'last_result': 'UNKNOWN',
        'is_building': False,
        'console_log': ''
    }
    try:
        req = urllib.request.Request('http://localhost:8085/job/deploy-pinky-fleet/api/json')
        req.add_header('Authorization', f'Basic {auth_str}')
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            data = json.loads(resp.read().decode('utf-8'))
            res['online'] = True
            res['in_queue'] = data.get('inQueue', False)
            last_build = data.get('lastBuild')
            if last_build:
                res['last_build_number'] = last_build.get('number')
                try:
                    b_req = urllib.request.Request(f"http://localhost:8085/job/deploy-pinky-fleet/{last_build.get('number')}/api/json")
                    b_req.add_header('Authorization', f'Basic {auth_str}')
                    with urllib.request.urlopen(b_req, timeout=1.5) as b_resp:
                        b_data = json.loads(b_resp.read().decode('utf-8'))
                        res['is_building'] = b_data.get('building', False)
                        res['last_result'] = b_data.get('result') or ('BUILDING' if res['is_building'] else 'UNKNOWN')
                except Exception:
                    pass
                
                try:
                    l_req = urllib.request.Request('http://localhost:8085/job/deploy-pinky-fleet/lastBuild/logText/progressiveText?start=0')
                    l_req.add_header('Authorization', f'Basic {auth_str}')
                    with urllib.request.urlopen(l_req, timeout=1.5) as l_resp:
                        full_log = l_resp.read().decode('utf-8', errors='ignore')
                        lines = full_log.strip().split('\n')
                        res['console_log'] = '\n'.join(lines[-35:])
                except Exception:
                    pass
    except Exception as e:
        res['error'] = str(e)
    return res


def trigger_jenkins_build(target, action):
    cj = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(cj))
    auth_str = base64.b64encode(b'admin:admin1234').decode('utf-8')
    try:
        crumb_req = urllib.request.Request('http://localhost:8085/crumbIssuer/api/json')
        crumb_req.add_header('Authorization', f'Basic {auth_str}')
        with opener.open(crumb_req, timeout=2) as resp:
            crumb_data = json.loads(resp.read().decode('utf-8'))
            crumb = crumb_data['crumb']
            field = crumb_data['crumbRequestField']

        url = f'http://localhost:8085/job/deploy-pinky-fleet/buildWithParameters?TARGET_ROBOT={target}&ACTION={action}'
        build_req = urllib.request.Request(url, data=b'')
        build_req.add_header('Authorization', f'Basic {auth_str}')
        build_req.add_header(field, crumb)
        with opener.open(build_req, timeout=3) as resp:
            return True, f"Build triggered (HTTP {resp.status})"
    except Exception as e:
        return False, str(e)


CAMERA_STREAMER_HTML = """<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>📱 현장 휴대폰 카메라 송출기</title>
  <style>
    body { margin: 0; background: #0d1117; color: #c9d1d9; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; display: flex; flex-direction: column; align-items: center; justify-content: space-between; height: 100vh; overflow: hidden; }
    .header { padding: 14px; text-align: center; width: 100%; background: #161b22; border-bottom: 1px solid #30363d; box-sizing: border-box; }
    .header h1 { font-size: 1.1rem; margin: 0; color: #58a6ff; }
    .viewport { flex: 1; width: 100%; position: relative; background: #000; display: flex; align-items: center; justify-content: center; }
    video { width: 100%; height: 100%; object-fit: cover; }
    .overlay { position: absolute; top: 14px; left: 14px; background: rgba(0,0,0,0.7); padding: 6px 14px; border-radius: 20px; font-size: 0.85rem; font-weight: bold; border: 1px solid #30363d; }
    .status-active { color: #3fb950; border-color: #238636; }
    .status-inactive { color: #f85149; }
    .controls { padding: 16px; width: 100%; max-width: 420px; box-sizing: border-box; display: flex; flex-direction: column; gap: 10px; background: #161b22; border-top: 1px solid #30363d; }
    .btn { padding: 14px; border: none; border-radius: 8px; font-size: 1rem; font-weight: bold; cursor: pointer; transition: 0.2s; }
    .btn-primary { background: #238636; color: #fff; }
    .btn-secondary { background: #30363d; color: #c9d1d9; }
    .btn-danger { background: #da3633; color: #fff; }
  </style>
</head>
<body>
  <div class="header">
    <h1>📱 현장 휴대폰 카메라 ➔ 관제 웹 송출기</h1>
  </div>
  <div class="viewport">
    <video id="webcam" autoplay playsinline muted></video>
    <div id="status-tag" class="overlay status-inactive">🔴 송출 대기 중</div>
  </div>
  <div class="controls">
    <button id="btn-toggle" class="btn btn-primary" onclick="toggleStream()">📷 카메라 송출 시작</button>
    <button class="btn btn-secondary" onclick="switchCamera()">🔄 전면/후면 카메라 전환</button>
  </div>
  <canvas id="hidden-canvas" style="display:none;"></canvas>

  <script>
    let stream = null;
    let isStreaming = false;
    let facingMode = 'environment';
    let intervalId = null;
    let sentCount = 0, failCount = 0, inFlight = 0, lastOkAt = 0;
    let wakeLock = null;

    function renderStatus() {
      if (!isStreaming) return;
      const age = lastOkAt ? Math.round((Date.now() - lastOkAt) / 1000) : -1;
      const hidden = document.visibilityState !== 'visible';
      const head = hidden ? '백그라운드 - 송출이 멈출 수 있습니다' : '송출 중';
      statusTag.textContent = head + ' · 전송 ' + sentCount + ' · 실패 ' + failCount
        + (age >= 0 ? ' · 마지막 성공 ' + age + '초 전' : ' · 아직 성공 0');
      statusTag.className = 'overlay ' + ((failCount > 0 && sentCount === 0) ? 'status-inactive' : 'status-active');
    }

    async function acquireWakeLock() {
      try {
        if ('wakeLock' in navigator) {
          wakeLock = await navigator.wakeLock.request('screen');
          wakeLock.addEventListener('release', () => { wakeLock = null; });
        }
      } catch (e) { wakeLock = null; }
    }

    document.addEventListener('visibilitychange', () => {
      if (document.visibilityState === 'visible' && isStreaming) { acquireWakeLock(); }
      renderStatus();
    });

    window.addEventListener('load', () => {
      if (cameraBlockedReason()) {
        statusTag.textContent = '차단됨 - http 주소라 브라우저가 카메라를 막습니다';
        statusTag.className = 'overlay status-inactive';
      }
    });

    const video = document.getElementById('webcam');
    const canvas = document.getElementById('hidden-canvas');
    const ctx = canvas.getContext('2d');
    const statusTag = document.getElementById('status-tag');
    const btnToggle = document.getElementById('btn-toggle');

    function cameraBlockedReason() {
      if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) return null;
      return `이 페이지 주소가 http 라서 브라우저가 카메라 기능 자체를 차단했습니다.
권한 문제가 아닙니다 (isSecureContext=${window.isSecureContext}, origin=${location.origin}).

해결 1) Chrome 에서 chrome://flags/#unsafely-treat-insecure-origin-as-secure 를 열고
        ${location.origin} 를 넣은 뒤 Enabled -> Relaunch
해결 2) 폰 Termux 에서
        socat TCP-LISTEN:8889,fork,reuseaddr TCP:${location.hostname}:8889
        를 띄우고 http://127.0.0.1:8889/camera_streamer 로 열기`;
    }

    async function startCamera() {
      const blocked = cameraBlockedReason();
      if (blocked) { alert(blocked); return false; }
      try {
        if (stream) {
          stream.getTracks().forEach(t => t.stop());
        }
        stream = await navigator.mediaDevices.getUserMedia({
          video: { facingMode: facingMode, width: { ideal: 1280 }, height: { ideal: 720 } },
          audio: false
        });
        video.srcObject = stream;
        await video.play();
        return true;
      } catch (err) {
        alert("카메라를 열지 못했습니다: " + err.name + " - " + err.message);
        return false;
      }
    }

    async function toggleStream() {
      if (isStreaming) {
        stopStream();
      } else {
        const ok = await startCamera();
        if (!ok) return;
        isStreaming = true;
        btnToggle.textContent = "⏹️ 카메라 송출 중지";
        btnToggle.className = "btn btn-danger";
        sentCount = 0; failCount = 0; inFlight = 0; lastOkAt = 0;
        renderStatus();
        acquireWakeLock();

        canvas.width = 640;
        canvas.height = 360;

        intervalId = setInterval(() => {
          if (!isStreaming || video.readyState !== video.HAVE_ENOUGH_DATA) return;
          ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
          if (inFlight >= 3) { failCount++; renderStatus(); return; }
          canvas.toBlob(blob => {
            if (!blob) return;
            inFlight++;
            fetch('/api/camera/upload', {
              method: 'POST',
              headers: { 'Content-Type': 'image/jpeg' },
              body: blob
            }).then(r => { if (r.ok) { sentCount++; lastOkAt = Date.now(); } else { failCount++; } })
              .catch(() => { failCount++; })
              .finally(() => { inFlight--; renderStatus(); });
          }, 'image/jpeg', 0.65);
        }, 66);
      }
    }

    function stopStream() {
      isStreaming = false;
      if (intervalId) clearInterval(intervalId);
      if (stream) {
        stream.getTracks().forEach(t => t.stop());
        stream = null;
      }
      video.srcObject = null;
      btnToggle.textContent = "📷 카메라 송출 시작";
      btnToggle.className = "btn btn-primary";
      statusTag.textContent = "송출 대기 중 · 전송 " + sentCount + " · 실패 " + failCount;
      statusTag.className = "overlay status-inactive";
      if (wakeLock) { try { wakeLock.release(); } catch (e) {} wakeLock = null; }
    }

    async function switchCamera() {
      facingMode = (facingMode === 'environment') ? 'user' : 'environment';
      if (isStreaming) {
        await startCamera();
      }
    }
  </script>
</body>
</html>
"""



# ---- MCV-2A4 드리프트 감시 -------------------------------------------------------

# 판정을 이 간격보다 자주 하지 않는다. 조각 정합 4번이 매 폴링마다 돌면 서빙이 느려지고,
# 느려진 화면은 사람이 안 본다 - 안 보는 감시는 감시가 아니다.
DRIFT_MIN_INTERVAL_S = 5.0
GLOBAL_DRIFT_CACHE = {}


def _decoded_still(source_id):
    """그 소스의 최신 프레임을 디코드해서 준다. 없으면 None."""
    jpeg = _calib_still(source_id)
    if not jpeg:
        return None
    try:
        return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


def _drift_anchor(source_id, receipt):
    """정착한 자리의 조각과 그때의 정합도를 남긴다."""
    try:
        frame = _decoded_still(source_id)
        corners = [c['px'] for c in (receipt or {}).get('corners', [])]
        matrix = (receipt or {}).get('homography')
        if frame is None or len(corners) != 4 or not matrix:
            app_log(f"[Drift] {source_id} 기준을 못 잡았다 (프레임/모서리 없음)")
            return
        ms, _problem = _calib_masks(source_id)
        # ⭐ 마스크에 걸치는 조각은 안 뜬다. 흔들리는 케이블 하나가 MATCH_LOST 로
        #    **진짜 흔들림을 가린다** (classify 는 MATCH_LOST 를 먼저 본다).
        refs = drift.take_patches(frame, corners, masks=ms)
        measured = drift.measure(frame, refs, corners, matrix)
        drift.save_reference(GLOBAL_CALIB.state_dir, source_id, refs, measured, corners)
        GLOBAL_DRIFT_CACHE.pop(source_id, None)
        app_log(f"[Drift] {source_id} 기준 저장")
    except Exception as exc:
        app_log(f"[Drift] {source_id} 기준 저장 실패: {exc}")


def _drift_state(source_id, state):
    """(state, detail) 또는 None. **throttle 한다** - 위 상수의 이유를 보라."""
    now = time.time()
    hit = GLOBAL_DRIFT_CACHE.get(source_id)
    if hit and (now - hit[0]) < DRIFT_MIN_INTERVAL_S:
        return hit[1]
    refs, meta = drift.load_reference(GLOBAL_CALIB.state_dir, source_id)
    if refs is None:
        GLOBAL_DRIFT_CACHE[source_id] = (now, None)
        return None
    frame = _decoded_still(source_id)
    matrix = state.get('homography')
    corners = meta.get('cornersPx') or []
    if frame is None or not matrix or len(corners) != 4:
        got = (drift.STATE_UNKNOWN, {'why': drift.WHY_NO_MEASUREMENT})
    else:
        measured = drift.measure(frame, refs, corners, matrix)
        got = drift.classify(measured, {'scores': meta.get('scores')})
    GLOBAL_DRIFT_CACHE[source_id] = (now, got)
    return got


# ---- MCV-2V 영상 -> 월드 ----------------------------------------------------------

def _vision_build(source_id, res_cm, wall_is_dark=None):
    """정착된 정합에서 평면도를 만들어 var/ 에 남긴다. 산출물 경로를 돌려준다."""
    frame = _decoded_still(source_id)
    if frame is None:
        raise vision_world.VisionWorldError('프레임이 없다 - 소스가 끊겼는지 본다')
    state = _calib_state(source_id)
    box, _raw = _calib_censor(source_id)
    ms, mask_problem = _calib_masks(source_id)
    out = vision_world.build(frame, state, GLOBAL_CALIB.arena, res_cm=res_cm,
                             censor_box=box, wall_is_dark=wall_is_dark, masks=ms)
    if mask_problem:
        # 못 읽은 마스크는 유령 벽을 만든다. 산출물에 그 사실을 적는다.
        out['provenance']['masksProblem'] = mask_problem
    outdir = os.path.join(GLOBAL_CALIB.state_dir, 'vision')
    os.makedirs(outdir, exist_ok=True)
    stem = out['version'].replace(':', '-')
    paths = {}
    for name, data, mode in (('%s.sdf' % stem, out['sdf'], 'w'),
                             ('%s.yaml' % stem, out['yaml'], 'w'),
                             ('%s.pgm' % stem, out['pgm'], 'wb')):
        p = os.path.join(outdir, name)
        with open(p, mode, **({} if mode == 'wb' else {'encoding': 'utf-8'})) as fh:
            fh.write(data)
        paths[name.rsplit('.', 1)[1]] = p
    with io.open(os.path.join(outdir, '%s.json' % stem), 'w', encoding='utf-8') as fh:
        fh.write(json.dumps(out['provenance'], ensure_ascii=False, indent=2))
    return {'version': out['version'], 'paths': paths,
            'provenance': out['provenance']}


# ---- MCV-2C 검열 판정 ----------------------------------------------------------

def _censorship_state(source_id):
    """이 소스의 검열 상태. (state, severity, detail).

    ⭐ 프레임을 **한 장만** 디코드한다. 매 요청마다 전수로 재면 서빙이 느려지고,
       그러면 사람이 이 화면을 안 보게 된다 - 안 보는 경고는 없는 경고다.
    """
    src = GLOBAL_REGISTRY.get(source_id) if GLOBAL_REGISTRY else None
    if not src:
        return None
    info = src.clock_info() or {}
    rules = info.get('processRules')
    box = info.get('censorBox')
    evidence = None
    if box:
        try:
            jpeg, _stamp, connected = src.latest_jpeg()
            if jpeg and connected:
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                                     cv2.IMREAD_COLOR)
                evidence = censorship.blur_evidence(frame, box)
        except Exception as exc:
            app_log(f"[Censor] 증거 측정 실패 {source_id}: {exc}")
    return censorship.classify(rules, box, evidence, GLOBAL_CENSORSHIP_POLICY)


def _merge_censorship(ready):
    """저하 목록에 검열 판정을 얹고 worstSeverity 를 다시 구한다.

    🔴 **시청을 끊지 않는다**(사용자 결정 2026-09-11). 교육장 화면이 비는 쪽이 더 나쁘고,
       게이트웨이는 집행 지점이 아니다. 여기가 하는 일은 **말을 정확히 하는 것**뿐이다.
    """
    if not GLOBAL_REGISTRY:
        return ready
    for sid in GLOBAL_REGISTRY.ids():
        src = GLOBAL_REGISTRY.get(sid)
        # 실물 시점만 본다 - ROS 합성 화면은 검열 대상이 아니다.
        if not src or src.transport not in ('pull', 'push'):
            continue
        got = _censorship_state(sid)
        if not got:
            continue
        state, severity, detail = got
        item = censorship.degradation_for(sid, state, severity, detail)
        if item:
            ready['degradations'].append(item)
    worst = 'ok'
    for d in ready['degradations']:
        if (SEVERITY_ORDER.index(d['severity'])
                > SEVERITY_ORDER.index(worst)):
            worst = d['severity']
    ready['worstSeverity'] = worst
    ready['censorshipPolicy'] = GLOBAL_CENSORSHIP_POLICY.to_dict()
    return ready


# ---- MCV-2A-UI 도우미 ---------------------------------------------------------

# 미리보기 해상도. 1 cm = 이 픽셀. 아레나 270x125 cm 면 1080x500 이 된다.
CALIB_PREVIEW_PX_PER_CM = 4.0


def _calib_frame_facts(source_id):
    """(프레임크기, 발행자세션). **곁표가 잰 값**이지 설정값이 아니다.

    2026-09-10: 폰 /status 는 640x480 이라 했지만 실제 프레임은 1088x1088 이었다.
    영수증에 설정값을 적으면 그 영수증이 거짓말을 한다.
    """
    if not GLOBAL_REGISTRY:
        return None, None
    src = GLOBAL_REGISTRY.get(source_id)
    if not src:
        return None, None
    info = src.clock_info() or {}
    w, h = info.get('frameWidth'), info.get('frameHeight')
    size = (int(w), int(h)) if (w and h) else None
    return size, info.get('publisherSession')


def _calib_censor(source_id):
    """이 소스의 검열 상자. 헤더가 없으면 None - 상자가 꺼져 있다는 뜻이다."""
    if not GLOBAL_REGISTRY:
        return None, None
    src = GLOBAL_REGISTRY.get(source_id)
    if not src:
        return None, None
    info = src.clock_info() or {}
    return info.get('censorBox'), info.get('censorBoxRaw')


def _calib_framing(source_id):
    """MCV-2G. 이 화각이 쓸 만한가. 거치하는 사람이 볼 것이다.

    ⭐ 정착 전에도 답할 수 있는 부분(금지 띠·권장 상자)과, 정착해야 알 수 있는
       부분(점유율·cm/px)을 **섞지 않는다.** 모르는 항목은 None 으로 남는다.
    """
    size, _session = _calib_frame_facts(source_id)
    if not size:
        raise CalibrationError('프레임 크기를 모른다 - 소스가 끊겼는지 본다')
    st = _calib_state(source_id)
    sc = {}
    if GLOBAL_REGISTRY:
        src = GLOBAL_REGISTRY.get(source_id)
        if src:
            sc = src.clock_info() or {}
    return framing.assess(size, corners_px=st.get('corners'),
                          matrix=st.get('homography'), sidecar=sc)


def _calib_masks(source_id):
    """MCV-2M. (masks, problem) — 운영자가 "여긴 바닥이 아니다" 라고 한 자리.

    ⭐ 못 읽었으면 빈 목록이되 **조용하지 않다**. 운영자는 그려 놨는데 평면도에
       유령 벽이 생기면 관측 실패로 읽는다.
    """
    return masks.load(GLOBAL_CALIB.state_dir, source_id)


def _mask_blocking(source_id, points, frame_size):
    """그 점들 중 마스크에 덮이는 첫 (라벨, 점). 없으면 None.

    ⭐ 마스크 안에는 핀을 못 찍는다. "여기는 안 본다" 고 해 놓고 그 자리를 모서리로
       쓰면 그 행렬은 **못 보는 점으로 푼 것**이 된다.
    """
    ms, _problem = _calib_masks(source_id)
    if not ms:
        return None
    for label, px in points:
        m = masks.covering(ms, px, frame_size)
        if m:
            return (m['why'], label)
    return None


def _calib_lens(source_id):
    """이 소스가 지금 쓰는 렌즈. 앱이 X-Camera-Lens 로 준다. 없으면 None."""
    if not GLOBAL_REGISTRY:
        return None
    src = GLOBAL_REGISTRY.get(source_id)
    if not src:
        return None
    return (src.clock_info() or {}).get('cameraLens')


def _calib_state(source_id):
    """지금 조건에서의 정합 상태. 조건이 바뀌었으면 여기서 DRIFT 로 떨어진다."""
    size, session = _calib_frame_facts(source_id)
    lens = _calib_lens(source_id)
    st = GLOBAL_CALIB.state(source_id, frame_size=size, publisher_session=session,
                            lens=lens)
    st['liveLens'] = lens
    st['liveFrameSize'] = list(size) if size else None
    st['livePublisherSession'] = session
    box, raw = _calib_censor(source_id)
    # 지금 프레임에 걸려 있는 상자. 영수증의 censorBox 와 다를 수 있다 - 규칙은 바뀐다.
    st['liveCensorBox'] = box
    st['liveCensorBoxRaw'] = raw

    # MCV-2M. 운영자가 선언한 제외 영역. 검열 상자와 **주인이 다르다**.
    ms, mask_problem = _calib_masks(source_id)
    st['masks'] = ms
    st['masksProblem'] = mask_problem

    # MCV-2A4 - 정착 상태일 때만 본다. 아직 안 찍었으면 움직였는지 물을 것도 없다.
    if st.get('state') == STATE_SETTLED:
        got = _drift_state(source_id, st)
        if got:
            dstate, ddetail = got
            st['drift'] = {'state': dstate, 'detail': ddetail}
            if dstate == drift.STATE_DRIFT:
                # ⭐ 카메라가 움직였으면 그 행렬은 **틀린 것**이다. 무효화 3종이
                #    못 잡는 자리라 여기서 내린다(fail-closed).
                st['state'] = 'DRIFT'
                # ⭐ 사유를 뭉뚱그리지 않는다. 조각을 전부 놓친 것은 "움직였다"가
                #    아니라 "확인할 수 없다" 다 - 모르는 것을 아는 것처럼 안 말한다.
                st['reason'] = (drift.REASON_UNVERIFIABLE
                                if ddetail.get('why') == drift.WHY_ALL_LOST
                                else drift.REASON_CAMERA_MOVED)
    return st


def _calib_still(source_id):
    """소스의 최신 JPEG 한 장. 없으면 None - 낡은 프레임을 정지화면이라 부르지 않는다."""
    if not source_id or not GLOBAL_REGISTRY:
        return None
    src = GLOBAL_REGISTRY.get(source_id)
    if not src:
        return None
    try:
        jpeg, _stamp, connected = src.latest_jpeg()
    except Exception:
        return None
    return jpeg if (jpeg and connected) else None


def _calib_preview(source_id):
    """놓인 네 점으로 위에서 내려다본 그림. (jpeg|None, 이유코드).

    ⭐ 정착 전에도 보여준다 - 그게 이 화면의 목적이다(찍으면서 확인한다).
       다만 **관측으로 내보내지는 않는다.** 미리보기는 사람이 보는 그림이다.
    ⭐ 왜곡 보정을 안 했다. 광각이면 가장자리가 휜 채로 펴진다 - 그래서 영수증에
       undistorted 를 적는다(V-2: 3이 4보다 먼저다).
    """
    if not GLOBAL_CALIB.arena:
        return None, 'NO_ARENA'
    jpeg = _calib_still(source_id)
    if not jpeg:
        return None, 'NO_FRAME'
    st = _calib_state(source_id)
    pts = st.get('corners')
    if not pts or len(pts) != 4:
        return None, 'NO_POINTS'
    arena = GLOBAL_CALIB.arena
    scale = CALIB_PREVIEW_PX_PER_CM
    out_w = int(round(arena.width_cm * scale))
    out_h = int(round(arena.height_cm * scale))
    try:
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return None, 'DECODE_FAILED'
        # 아레나는 좌하단이 원점이고 그림은 좌상단이 원점이다. y 를 뒤집어 놓는다 -
        # 안 뒤집으면 위아래가 거꾸로인 그림을 '정합됐다'고 내놓게 된다.
        targets = arena.corner_targets()
        dst = np.asarray(
            [[targets[c][0] * scale, out_h - targets[c][1] * scale] for c in CORNERS],
            dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(
            np.asarray(pts, dtype=np.float32), dst)
        warped = cv2.warpPerspective(frame, matrix, (out_w, out_h))
        ok, buf = cv2.imencode('.jpg', warped, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return None, 'ENCODE_FAILED'
        return buf.tobytes(), None
    except Exception as exc:
        app_log(f"[Calib] preview failed for {source_id}: {exc}")
        return None, 'FAILED'


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class GatewayRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        if any(k in self.path for k in ('/api/status', '/video_feed', '/control_feed', '/gazebo_feed', '/robot_camera_feed')):
            return
        app_log(f"[HTTP] {self.client_address[0]} - {format % args}")

    def _check_vision_auth(self, parsed_url=None) -> bool:
        """비전 API 인증 검사 (Track R: R-D1, R-D2 Fail-Closed).
        
        지원 방식:
          - Header: X-API-Key: <token>
          - Header: Authorization: Bearer <token>
          - Query Param: ?token=<token>
        """
        if not VISION_API_KEY:
            return False             # 키를 안 줬으면 아무도 못 들어온다 (fail-closed)

        # 1. Header: X-API-Key
        api_key = self.headers.get('X-API-Key')
        if api_key and api_key.strip() == VISION_API_KEY:
            return True

        # 2. Header: Authorization: Bearer <token>
        auth_header = self.headers.get('Authorization', '')
        if auth_header.startswith('Bearer '):
            token = auth_header[7:].strip()
            if token == VISION_API_KEY:
                return True

        # 3. Query Param: ?token=...
        if parsed_url is None:
            parsed_url = urlparse(self.path)
        qs = parse_qs(parsed_url.query)
        q_tokens = qs.get('token')
        if q_tokens and q_tokens[0].strip() == VISION_API_KEY:
            return True

        return False

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-API-Key, Authorization')
        self.end_headers()

    def do_HEAD(self):
        parsed = urlparse(self.path)
        if parsed.path in ('/video_feed', '/control_feed', '/gazebo_feed', '/robot_camera_feed'):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
        elif parsed.path in ('/api/status', '/api/logs', '/api/sources', '/api/safety', '/api/observe', '/api/clock'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
        elif parsed.path.endswith('.png'):
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.end_headers()
        elif parsed.path.endswith(('.jpg', '.jpeg')):
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.end_headers()
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b'{}'
        
        try:
            req_json = json.loads(post_data.decode('utf-8'))
        except Exception:
            req_json = {}

        # 1. 로봇 1호기 & Gazebo 목표 지점 전송 API (/api/robot1/goal, /api/goal)
        if parsed.path in ('/api/robot1/goal', '/api/goal'):
            client_ip = self.client_address[0]
            if client_ip not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote goal dispatch from {client_ip}")
                res = {
                    'success': False,
                    'error': 'Forbidden',
                    'message': '안전 정책: 목표 지정 및 이동 명령은 현장 중계 노트북(로컬)에서만 실행할 수 있습니다.'
                }
                self._send_json(json.dumps(res).encode('utf-8'), code=403)
                return

            blocked = motion_block_reason(GLOBAL_FLEET_COORDINATOR, DIRECT_MOTION_PATHS[parsed.path])
            if blocked:
                code, body = motion_block_reply(blocked)
                self._send_json(json.dumps(body, ensure_ascii=False).encode('utf-8'), code=code)
                return
            # 검수 R-script-1(게이트웨이 쪽): 예전엔 깨진 본문을 {} 로 받아 (0,0,0) 목표를 냈고, NaN·Infinity·1e999 도
            # 그대로 받았다. 움직이는 명령은 본문이 온전한 객체이고 x·y 가 있고 x·y·yaw 가 유한한 수일 때만 낸다.
            goal = parse_goal_body(req_json)             # 깨진 본문은 {} 가 되어 'x·y 가 없다' 로 거부된다
            if isinstance(goal, str):
                self._send_json(json.dumps({'success': False, 'reason': 'BAD_GOAL', 'message': '목표 거부 — ' + goal,
                                            'dispatched': False}, ensure_ascii=False).encode('utf-8'), code=400)
                return
            x, y, yaw = goal
            # 제3자 검수 G-11: 예전엔 ROS 노드가 없어 아무것도 안 냈어도 200 success 였다(보고만 거짓). 못 냈으면 503.
            why = None
            if not GLOBAL_ROBOT_SUB_NODE:
                why = ('NO_ROS_NODE', '목표 불가 — 게이트웨이의 ROS 노드가 없어 아무것도 내지 않았다')
            else:
                try:
                    GLOBAL_ROBOT_SUB_NODE.send_goal(x, y, yaw)
                except Exception as exc:                      # noqa: BLE001 — 발행 실패는 응답으로 말한다
                    why = ('PUBLISH_FAILED', f'목표 불가 — /robot1/goal_pose 발행 실패 ({type(exc).__name__}: {exc})')
            if why:
                self._send_json(json.dumps({'success': False, 'reason': why[0], 'message': why[1], 'dispatched': False,
                                            'goal': [x, y, yaw]}, ensure_ascii=False).encode('utf-8'), code=503)
                return
            # 통합 검토 OPS-5: 예전 문구 'sent to Robot #1 & Gazebo' 는 로봇이 받아 달린다는 뜻으로 읽혔다. 이 응답이 아는 것은
            # 발행했다는 것뿐이다(받는 쪽 수도 모른다) — 달리는지는 로봇 카드·LaneStatus 로 본다.
            res = {'success': True, 'dispatched': True, 'confirmed': None, 'topic': '/robot1/goal_pose',
                   'message': f'목표 ({x:.2f}, {y:.2f}) 를 /robot1/goal_pose 로 발행했다 — 로봇이 받아 달리는지는 '
                              f'이 응답이 확인하지 않는다', 'goal': [x, y, yaw]}
            self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            return

        # 2. 로봇 1호기 미션 실행 API (/api/robot1/mission)
        elif parsed.path == '/api/robot1/mission':
            client_ip = self.client_address[0]
            if client_ip not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote mission trigger from {client_ip}")
                res = {
                    'success': False,
                    'error': 'Forbidden',
                    'message': '안전 정책: 미션 시작 명령은 현장 중계 노트북(로컬)에서만 실행할 수 있습니다.'
                }
                self._send_json(json.dumps(res).encode('utf-8'), code=403)
                return

            blocked = motion_block_reason(GLOBAL_FLEET_COORDINATOR, DIRECT_MOTION_PATHS[parsed.path])
            if blocked:
                code, body = motion_block_reply(blocked)
                self._send_json(json.dumps(body, ensure_ascii=False).encode('utf-8'), code=code)
                return
            # 통합 검토 OPS-10: 예전엔 빈 본문·깨진 본문·mission 없는 본문을 미션 '1'(움직이는 경로)로 받아 냈다 — 목표 API 가
            # 그런 본문을 400 으로 막는 것(R-script-1)과 같은 규칙: 움직이는 명령은 본문이 말한 미션만 낸다.
            mission, bad = parse_mission_body(req_json)     # 깨진 본문은 {} 가 되어 'mission 이 없다' 로 거부된다
            if bad:
                self._send_json(json.dumps({'success': False, 'reason': 'BAD_MISSION', 'dispatched': False,
                                            'allowed': sorted(MISSION_COMMANDS), 'message': '미션 거부 — ' + bad},
                                           ensure_ascii=False).encode('utf-8'), code=400)
                return
            if not GLOBAL_ROBOT_SUB_NODE:
                self._send_json(json.dumps({'success': False, 'reason': 'NO_ROS_NODE', 'mission': mission,
                                            'message': '미션 불가 — 게이트웨이의 ROS 노드가 없다'},
                                           ensure_ascii=False).encode('utf-8'), code=503)
                return
            subs = GLOBAL_ROBOT_SUB_NODE.send_mission(MISSION_COMMANDS[mission])
            # R3 (관제 검수 §3.3): 예전엔 받는 쪽이 0 인 토픽에 보내고 'triggered' 200 이었다. 도메인 8 구독자는
            # 브리지일 수 있다(로봇의 옛 미션 내비게이터가 받는지는 여기서 모른다) — 보냈다고만 말한다.
            self._send_json(json.dumps({
                'success': False, 'dispatched': True, 'confirmed': None, 'mission': mission,
                'topic': '/robot1/mission_cmd', 'd8_subscribers': subs,
                'means': 'DOMAIN_8_SUBSCRIBER_EXISTS_INCLUDING_BRIDGE',
                'message': (f'미션 {mission} 을 보냈다 — 받는 쪽이 있는지 모른다(도메인 8 구독자 {subs}, 브리지 포함)' if subs
                            else f'미션 {mission} 을 보냈다 — 도메인 8 구독자 수를 세지 못했다(모른다)' if subs is None
                            else f'미션 {mission} 을 보냈지만 도메인 8 에 받는 쪽이 0 이다'),
            }, ensure_ascii=False).encode('utf-8'), code=202)
            return

        # 2-B. 관측 세션 켜기/끄기 (MCV-1C). 카메라를 실제로 여는 명령이므로 주행 명령과 같은 로컬 게이트.
        elif parsed.path == '/api/observe':
            client_ip = self.client_address[0]
            if client_ip not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote observe toggle from {client_ip}")
                res = {
                    'success': False,
                    'error': 'Forbidden',
                    'message': '안전 정책: 관측 세션은 현장 중계 노트북(로컬)에서만 켜고 끌 수 있습니다.'
                }
                self._send_json(json.dumps(res).encode('utf-8'), code=403)
                return
            _active = bool(req_json.get('active', False))
            GLOBAL_OBSERVATION.set_active(_active, GLOBAL_LOCAL_CAMS)
            app_log(f"[Observe] session {'ON' if _active else 'OFF'} by {client_ip}")
            res = {'success': True}
            res.update(GLOBAL_OBSERVATION.public())
            self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            return


        # 2-C. 캘리브레이션 (MCV-2A-UI). 관측 좌표계를 정하는 행위라 주행 명령과 같은 게이트다.
        # 2-C. 관측 제외 영역 (MCV-2M). 관측 좌표계를 바꾸는 선언이라 같은 로컬 게이트.
        elif parsed.path == '/api/calibration/masks':
            client_ip = self.client_address[0]
            if client_ip not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote mask edit from {client_ip}")
                self._send_json(json.dumps(
                    {'success': False, 'error': 'Forbidden'},
                    ensure_ascii=False).encode('utf-8'), code=403)
                return
            src = str(req_json.get('src', '')).strip()
            if not src or not GLOBAL_REGISTRY or not GLOBAL_REGISTRY.get(src):
                self._send_json(json.dumps(
                    {'success': False, 'message': '알 수 없는 소스: %s' % src},
                    ensure_ascii=False).encode('utf-8'), code=400)
                return
            try:
                size, _session = _calib_frame_facts(src)
                # ⭐ 지금 정착한 점들을 덮는지 본다. 하나라도 덮으면 **아무것도 안 바꾼다.**
                got = masks.parse(req_json.get('masks'),
                                  state=_calib_state(src), frame_size=size)
                masks.save(GLOBAL_CALIB.state_dir, src, got)
                # 마스크가 바뀌면 흔들림 기준선의 조각 구성이 달라진다 - 캐시를 버린다.
                GLOBAL_DRIFT_CACHE.pop(src, None)
                app_log(f"[Mask] {src} <- {len(got)} 개 ({client_ip})")
                res = {'success': True, 'masks': got,
                       'coveredFraction': masks.union_fraction(got)}
                self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            except masks.MaskError as exc:
                self._send_json(json.dumps(
                    {'success': False, 'message': str(exc)},
                    ensure_ascii=False).encode('utf-8'), code=400)
            return

        # 2-D. 영상 -> 월드 (MCV-2V). 관측 좌표계를 쓰는 산출물이라 같은 로컬 게이트.
        elif parsed.path == '/api/vision/world':
            client_ip = self.client_address[0]
            if client_ip not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote vision build from {client_ip}")
                self._send_json(json.dumps(
                    {'success': False, 'error': 'Forbidden'},
                    ensure_ascii=False).encode('utf-8'), code=403)
                return
            src = str(req_json.get('src', '')).strip()
            if not src or not GLOBAL_REGISTRY or not GLOBAL_REGISTRY.get(src):
                self._send_json(json.dumps(
                    {'success': False, 'message': '알 수 없는 소스: %s' % src},
                    ensure_ascii=False).encode('utf-8'), code=400)
                return
            try:
                res = _vision_build(src, float(req_json.get('resolutionCm', 2.5)),
                                    wall_is_dark=req_json.get('wallIsDark'))
                app_log(f"[Vision] {src} -> {res['version']}")
                res['success'] = True
                self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            except (vision_world.VisionWorldError, CalibrationError) as exc:
                self._send_json(json.dumps(
                    {'success': False, 'message': str(exc)},
                    ensure_ascii=False).encode('utf-8'), code=400)
            return

        elif parsed.path in ('/api/calibration/points', '/api/calibration/settle',
                             '/api/calibration/clear'):
            client_ip = self.client_address[0]
            if client_ip not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote calibration from {client_ip}")
                res = {'success': False, 'error': 'Forbidden',
                       'message': '안전 정책: 캘리브레이션은 현장 중계 노트북(로컬)에서만 가능합니다.'}
                self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'), code=403)
                return
            src = str(req_json.get('src', '')).strip()
            if not src or not GLOBAL_REGISTRY or not GLOBAL_REGISTRY.get(src):
                self._send_json(json.dumps(
                    {'success': False, 'message': '알 수 없는 소스: %s' % src},
                    ensure_ascii=False).encode('utf-8'), code=400)
                return
            try:
                if parsed.path == '/api/calibration/clear':
                    GLOBAL_CALIB.clear(src)
                    # 기준 조각도 함께 지운다 - 남겨 두면 다음 정착 때 옛 기준을 쓴다.
                    drift.clear_reference(GLOBAL_CALIB.state_dir, src)
                    GLOBAL_DRIFT_CACHE.pop(src, None)
                    res = {'success': True}
                    res.update(_calib_state(src))
                elif parsed.path == '/api/calibration/points':
                    # ⭐ 프레임 크기는 요청이 말하는 값이 아니라 **곁표가 잰 값**을 쓴다.
                    #    클라이언트가 보낸 크기를 믿으면 영수증이 거짓이 된다.
                    size, session = _calib_frame_facts(src)
                    box, _raw = _calib_censor(src)
                    lens = _calib_lens(src)
                    # ⭐ 마스크 안에는 핀을 못 찍는다 - 안 보는 점으로 행렬을 풀게 된다.
                    pts = [('%d번 모서리' % (i + 1), tuple(p))
                           for i, p in enumerate(req_json.get('corners') or [])]
                    pts += [('검증점 %s' % (v.get('name')), tuple(v.get('px') or (0, 0)))
                            for v in (req_json.get('verify') or [])]
                    hit = _mask_blocking(src, pts, size)
                    if hit:
                        raise CalibrationError(
                            "마스크 '%s' 가 %s 를 덮는다 - 안 보기로 한 자리에는 "
                            "핀을 찍을 수 없다. 마스크를 줄이거나 카메라를 옮긴다."
                            % hit)
                    st = GLOBAL_CALIB.set_points(
                        src, req_json.get('corners') or [], size,
                        publisher_session=session,
                        verify=req_json.get('verify') or [],
                        undistorted=bool(req_json.get('undistorted')),
                        censor_box=box, lens=lens)
                    res = {'success': True}
                    res.update(st)
                else:
                    rec = GLOBAL_CALIB.settle(src, note=req_json.get('note'))
                    # ⭐ 드리프트 기준선은 **정착 직후**에 잡는다. 나중에 재면 이미
                    #    움직였을 수 있고, 그러면 움직인 상태가 기준이 된다.
                    _drift_anchor(src, rec)
                    app_log(f"[Calib] {src} settled by {client_ip} "
                            f"(arena={rec.get('arenaVersion')}, "
                            f"accuracyMeasured={rec.get('accuracyMeasured')})")
                    res = {'success': True, 'receipt': rec}
                    res.update(_calib_state(src))
                self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            except CalibrationError as exc:
                self._send_json(json.dumps(
                    {'success': False, 'message': str(exc)},
                    ensure_ascii=False).encode('utf-8'), code=400)
            return

        # 3. 로봇 정지 API — D7 (2026-09-24): 플릿 경로로 보낸다.
        #    🔴 예전엔 `/robot1/mission_cmd` "stop" 만 냈다. Nav2 에이전트·DriveCommandGate 는 그 토픽을
        #       안 듣는다 → 주행 중 정지 **무효**(관제 팜 실측). 게다가 D8 구독자 1 은 **브리지**라
        #       "정지 명령 전달 (수신자 1)" 이라는 거짓 성공을 냈다(R-A 와 같은 함정).
        #    ⭐ 멈추는 명령은 어디서든 받는다 — 태블릿 화면의 정지가 403 이면 정지 버튼이 없는 것과 같다.
        elif parsed.path in ROBOT_STOP_PATHS:
            robot = ROBOT_STOP_PATHS[parsed.path]
            if not GLOBAL_ROBOT_SUB_NODE:
                self._send_json(json.dumps({
                    'success': False, 'reason': 'NO_ROS_NODE', 'robot': robot,
                    'message': '게이트웨이의 ROS 노드가 없다'
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return
            coord = GLOBAL_FLEET_COORDINATOR
            since = coord.now() if coord is not None else None
            GLOBAL_ROBOT_SUB_NODE.send_fleet_control({'cmd': 'stop_robot', 'robot': robot})
            legacy = None
            if robot == 'pinky1':
                # 옛 내비게이터(robot1_mission_navigator)용으로 남긴다. 이 수는 **도메인 8** 구독자라
                # 브리지를 센다 — 로봇이 받았다는 뜻이 아니다.
                legacy = {'topic': '/robot1/mission_cmd', 'd8_subscribers': GLOBAL_ROBOT_SUB_NODE.send_mission('stop'),
                          'means': 'DOMAIN_8_SUBSCRIBER_EXISTS_INCLUDING_BRIDGE'}
            if coord is None:
                # 🔴 관제 검수 P2: 예전엔 받을 코디네이터가 없어도 success:true 200 이었다. 아무도 안 세운다.
                self._send_json(json.dumps({
                    'success': False, 'dispatched': True, 'reason': 'NO_FLEET_COORDINATOR', 'robot': robot,
                    'confirmed': None, 'legacy_mission_cmd': legacy, 'detail': FLEET_IMPORT_ERROR,
                    'message': '플릿 코디네이터가 없어 정지 명령을 받을 곳이 없다',
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return
            # 로봇의 **정지 뒤** 보고를 잠깐 기다린다: True = STOP 을 처리했고 멈춰 있다 · False = 아직 달린다 ·
            # None = 모른다(보고 없음, 또는 STOP 처리 증거 없음). 판정 규칙은 코디네이터 hold_confirmation 한 곳에 있다.
            deadline = time.monotonic() + STOP_CONFIRM_WAIT_SEC
            while True:
                confirmed = coord.hold_confirmation(robot, since)
                if confirmed is True or time.monotonic() >= deadline:
                    break
                time.sleep(STOP_CONFIRM_POLL_SEC)
            heard = coord.last_heard_sec(robot)
            recently = None if heard is None else heard <= ROBOT_HEARD_RECENTLY_SEC
            if confirmed is True:
                code, reason, message = 200, None, '정지 확인 — 로봇이 STOP 을 처리했고 멈춰 있다고 보고했다'
            elif confirmed is False:
                code, reason = 202, 'ROBOT_STILL_CRUISING'
                message = '정지 명령을 보냈으나 %.1f s 안에 로봇은 아직 주행 중이라고 보고했다 — 10 Hz 로 계속 보낸다' % STOP_CONFIRM_WAIT_SEC
            else:
                code, reason = 202, 'NO_STOP_ACK'
                message = ('정지 명령을 보냈으나 로봇 소식이 끊겨 있다 — 받았는지 모른다' if recently is not True else
                           '정지 명령을 보냈으나 %.1f s 안에 로봇이 STOP 을 처리했다는 보고가 없다 — 받았는지 아직 모른다' % STOP_CONFIRM_WAIT_SEC)
            self._send_json(json.dumps({
                # success = **멈춘 것을 로봇 보고로 확인했다.** 보내기만 한 것은 dispatched 다.
                'success': confirmed is True,
                'dispatched': True,
                'reason': reason,
                'robot': robot,
                'dispatched_via': 'fleet',              # LaneCommand STOP + FleetCommand STOP, 10 Hz 반복
                'semantics': 'HOLD',                    # goal 취소 — /estop 은 안 건다. 비상정지는 /api/fleet/estop
                'confirmed': confirmed,
                'confirm_wait_sec': STOP_CONFIRM_WAIT_SEC,
                'robot_last_heard_sec': heard,
                'robot_heard_recently': recently,
                'legacy_mission_cmd': legacy,
                'message': message,
            }, ensure_ascii=False).encode('utf-8'), code=code)
            return

        # 3-A. 로봇 재개 API — D7. 움직이게 하는 명령이라 로컬에서만 받는다.
        elif parsed.path in ROBOT_RESUME_PATHS:
            robot = ROBOT_RESUME_PATHS[parsed.path]
            if self.client_address[0] not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote robot resume from {self.client_address[0]}")
                self._send_json(json.dumps({
                    'success': False, 'error': 'Forbidden', 'robot': robot,
                    'message': '안전 정책: 재개(움직이는 명령)는 현장 중계 노트북(로컬)에서만 실행할 수 있습니다.'
                }, ensure_ascii=False).encode('utf-8'), code=403)
                return
            if not GLOBAL_ROBOT_SUB_NODE:
                self._send_json(json.dumps({
                    'success': False, 'reason': 'NO_ROS_NODE', 'robot': robot,
                    'message': '재개 불가 — 게이트웨이의 ROS 노드가 없다'
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return
            if GLOBAL_FLEET_COORDINATOR is None:
                # 제3자 검수 G-2: 재개도 코디네이터가 받는다(`/fleet/lane/control` 구독처는 코디네이터 하나뿐). 받는 쪽 없이
                # 보내고 202 '모른다' 로 답하던 것을 — 모르는 게 아니라 받을 곳이 없다 — 503 으로.
                self._send_json(json.dumps({
                    'success': False, 'reason': 'NO_FLEET_COORDINATOR', 'robot': robot, 'dispatched': False,
                    'detail': FLEET_IMPORT_ERROR, 'message': '재개 불가 — 플릿 코디네이터가 없어 받을 곳이 없다'
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return
            if GLOBAL_FLEET_COORDINATOR.mission_state == 'ESTOP' or GLOBAL_FLEET_COORDINATOR.estop_latched:
                # 관제 검수 P1: 로봇 재개(LaneCommand RESUME)는 에이전트의 모든 래치를 푼다. 플릿 비상정지 중에는
                # 한 로봇만 풀지 않는다 — 코디네이터도 거부하지만 여기서 먼저 말한다.
                self._send_json(json.dumps({
                    'success': False, 'reason': 'FLEET_ESTOP', 'robot': robot, 'dispatched': False,
                    'message': '재개 거부 — 플릿 비상정지 중이다. 비상정지 해제는 /api/fleet/resume 로만 한다'
                }, ensure_ascii=False).encode('utf-8'), code=409)
                return
            coord = GLOBAL_FLEET_COORDINATOR
            seq0 = coord.control_seq if coord is not None else 0
            GLOBAL_ROBOT_SUB_NODE.send_fleet_control({'cmd': 'resume_robot', 'robot': robot})
            code, body = applied_reply(wait_control_result(coord, seq0, 'resume_robot'), {
                'robot': robot, 'dispatched_via': 'fleet', 'confirmed': None,
                'message': '재개 명령을 플릿 경로로 보냈다'})
            if code == 200 and body.get('mission_state') in ('STOPPED', 'DONE'):
                # 통합 검토 OPS-6: 예전엔 늘 '(플릿 재개 때 출발)' 이었다 — DONE 과, 달리지 않던 플릿의 STOPPED 에서는 플릿
                # 재개가 출발시키지 않는다(L7). 플릿 재개가 실제로 갈 상태로 말한다.
                goes = fleet_resume_target(coord)
                body['fleet_resume_goes_to'] = goes
                if goes == 'RUNNING':
                    tail = '플릿 재개(V2 "재시작") 때 달리던 미션으로 출발한다'
                elif goes == 'DONE':
                    tail = '미션 완료(DONE)라 플릿 재개(V2 "재시작")로는 출발하지 않는다 — 다시 달리려면 "주행 시작"'
                else:
                    tail = '플릿 재개(V2 "재시작")는 %s 로 돌아갈 뿐 출발시키지 않는다 — 출발은 "주행 시작"' % goes
                body['message'] = '로봇별 정지를 풀었다 — 플릿이 %s 라 서 있다. %s' % (body['mission_state'], tail)
            self._send_json(json.dumps(body, ensure_ascii=False).encode('utf-8'), code=code)
            return

        # 3-B. 연산 노드(태블릿) 좌표 수신구 (/api/vision/pose)
        #  영상 → 좌표 → **중계** → 관제 평면. 이 엔드포인트가 그 가운데 토막이다.
        #  ⭐ 응답은 '받았다'(accepted)와 '올렸다'(published)를 **따로** 말한다 —
        #     ROS 노드가 없으면 받기는 해도 아무 데도 안 간다. 둘을 한 낱말로 묶으면
        #     정지 API 가 그랬던 것처럼 거짓 초록이 된다.
        #  🔴 **로컬 게이트를 의도적으로 걸지 않는다.** 연산 노드(태블릿)는 원격이라
        #     `LOCAL_CONTROL_IPS` 로 막으면 이 경로가 통째로 죽는다. 대가는 **무인증
        #     좌표 주입**이다 — 지금은 이 토픽의 소비자가 0 이라 주입이 아무것도 바꾸지
        #     못하지만, **소비자가 생기는 순간 인증을 붙여야 한다**(회수 조건).
        #     `tests/test_calibration_http.py` 의 `UNGATED_KNOWN` 에 같은 근거로 등재.
        elif parsed.path == '/api/vision/pose':
            ok, reason, norm = vision_ingest.validate(
                req_json, robot_ids=set(VISION_ROBOT_IDS))
            if not ok:
                self._send_json(json.dumps({
                    'accepted': False, 'reason': reason,
                    'message': '좌표를 받지 않았다 — 빠진 값을 0 으로 채우지 않는다'
                }, ensure_ascii=False).encode('utf-8'), code=400)
                return
            now_ms = int(time.time() * 1000)
            rec = GLOBAL_VISION.accept(norm, now_ms)
            published, subs = False, None
            _recv = None
            if GLOBAL_ROBOT_SUB_NODE:
                published, subs = GLOBAL_ROBOT_SUB_NODE.publish_vision_pose(
                    norm['robotId'], norm['x'], norm['y'], norm['yaw'])
                try:
                    _recv = GLOBAL_ROBOT_SUB_NODE.vision_pose_receivers(norm['robotId'])
                except Exception:
                    _recv = None
            self._send_json(json.dumps({
                'accepted': True,
                # 받았다 / 올렸다 / 수신자가 있다 — 셋을 따로 말한다.
                'published': bool(published),
                'subscribersMeasured': subs is not None,
                # 🔴 `hasReceiver` 는 **도메인 8** 구독자가 있다는 뜻뿐이다. 지금 그 1 은
                #    브리지다 — "로봇이 받는다" 로 읽으면 안 된다. 종단은 아래 둘로 판정한다:
                #    `receivers.consumer`(브리지가 아닌 구독자)와 `downstream`(도메인 N).
                #    2026-09-20 관제·연산 노드 합의: 이 필드를 어느 수락의 근거로도 쓰지 않는다.
                'hasReceiver': bool((subs or 0) > 0),
                'hasReceiverMeans': 'DOMAIN_8_SUBSCRIBER_EXISTS_INCLUDING_BRIDGE',
                'receivers': _recv,
                'downstream': vision_downstream_contract(norm['robotId']),
                'robotId': norm['robotId'],
                'subscribers': subs,
                'topic': '/%s/vision_pose' % norm['robotId'],
                'receivedAtMs': rec['receivedAtMs'],
                'clockOffsetMs': rec['clockOffsetMs'],
                'clockSuspect': rec['clockSuspect'],
                'staleAfterMs': GLOBAL_VISION.stale_after_ms
            }, ensure_ascii=False).encode('utf-8'))
            return

        # 3-B2. 연산 노드(태블릿) canonical PoseFix 수신구 (/api/vision/pose_fix) [Track R: R-D1]
        #  입력: JSON (PoseFix.msg 와 1:1 의미 보존)
        #  검증: frame_id == map, robot_name in {pinky1, pinky2}, finite x/y/yaw, required types
        #  발행: pinky_lane_msgs/msg/PoseFix -> /pinky1/pose_fix 또는 /pinky2/pose_fix
        #  Relay에서 좌표 및 타임스탬프를 재계산하거나 대체하지 않음.
        elif parsed.path == '/api/vision/pose_fix':
            if not self._check_vision_auth(parsed):
                self._send_json(json.dumps({
                    'accepted': False, 'published': False, 'error': 'Unauthorized',
                    'reason': 'INVALID_OR_MISSING_TOKEN',
                    'message': '비전 API 인증 실패: 유효한 X-API-Key 또는 Bearer 토큰이 필요합니다 (401 Unauthorized).'
                }, ensure_ascii=False).encode('utf-8'), code=401)
                return

            ok, reason, norm = vision_ingest.validate_pose_fix(
                req_json, allowed_robots=('pinky1', 'pinky2'))
            if not ok:
                self._send_json(json.dumps({
                    'accepted': False, 'reason': reason,
                    'message': f'PoseFix 유효성 검증 실패: {reason}'
                }, ensure_ascii=False).encode('utf-8'), code=400)
                return

            if not GLOBAL_ROBOT_SUB_NODE:
                self._send_json(json.dumps({
                    'accepted': False, 'published': False, 'reason': 'NO_ROS_NODE',
                    'message': '게이트웨이 ROS 노드가 없어 PoseFix를 발행할 수 없습니다 (503 Service Unavailable).'
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return

            published, subs = GLOBAL_ROBOT_SUB_NODE.publish_pose_fix(norm)
            if not published:
                self._send_json(json.dumps({
                    'accepted': False,
                    'published': False,
                    'reason': 'ROS_PUBLISH_FAILED',
                    'message': f"PoseFix ROS 발행 실패: /{norm['robot_name']}/pose_fix 토픽 발행기가 준비되지 않았습니다."
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return

            self._send_json(json.dumps({
                'accepted': True,
                'published': True,
                'robot_name': norm['robot_name'],
                'topic': f"/{norm['robot_name']}/pose_fix",
                'subscribers': subs,
                'seq': norm['seq']
            }, ensure_ascii=False).encode('utf-8'), code=200)
            return

        # 3-B2. R-7 좌표 프로파일 전환 · 로봇 지도 · 초기 위치 — 로컬에서만, 움직이는 로봇이 없을 때만
        elif parsed.path in PROFILE_COMMANDS:
            cmd = PROFILE_COMMANDS[parsed.path]
            if self.client_address[0] not in LOCAL_CONTROL_IPS:
                self._send_json(json.dumps({'success': False, 'error': 'Forbidden', 'command': cmd,
                                            'message': '좌표 전환은 현장 중계 노트북(로컬)에서만 한다'},
                                           ensure_ascii=False).encode('utf-8'), code=403)
                return
            coord = GLOBAL_FLEET_COORDINATOR
            if coord is None or not GLOBAL_ROBOT_SUB_NODE:
                self._send_json(json.dumps({'success': False, 'reason': 'NO_FLEET_COORDINATOR', 'command': cmd,
                                            'detail': FLEET_IMPORT_ERROR},
                                           ensure_ascii=False).encode('utf-8'), code=503)
                return
            payload = {'cmd': cmd}
            if cmd == 'profile':
                name = str(req_json.get('name', '')).strip()
                prof = coord.profiles.get(name)
                if prof is None:
                    self._send_json(json.dumps({'success': False, 'reason': 'UNKNOWN_PROFILE', 'command': cmd,
                                                'message': '그런 프로파일이 없다: %r' % name},
                                               ensure_ascii=False).encode('utf-8'), code=404)
                    return
                if not prof.valid:
                    self._send_json(json.dumps({'success': False, 'reason': 'BROKEN_PROFILE', 'command': cmd,
                                                'problems': prof.problems,
                                                'message': '프로파일이 깨졌다 — ' + '; '.join(prof.problems)},
                                               ensure_ascii=False).encode('utf-8'), code=409)
                    return
                payload['name'] = name
            why = coord.profile_switch_blocker()
            latched = latched_robots(coord) if cmd in ('robot_maps', 'initial_poses') else []
            if not why and latched:
                # 통합 검토 OPS-3: 예전엔 코디네이터가 '링크유실 래치 — 로봇 재개 필요' 로 세워 둔 로봇이 있어도 ② 지도 교체가
                # 200 이었다 — 에이전트는 래치 중 SET_MAP 을 거부한다(REFUSED). ③ 초기 위치는 ② 뒤의 지도 좌표라 같이 막는다.
                why = '%s — 로봇 재개 먼저(V2 로봇 카드 "로봇 재개")' % ', '.join('%s(%s)' % nr for nr in latched)
            if why:
                self._send_json(json.dumps({'success': False, 'reason': 'NOT_NOW', 'command': cmd,
                                            'latched': [n for n, _ in latched],
                                            'message': '지금은 안 된다 — ' + why},
                                           ensure_ascii=False).encode('utf-8'), code=409)
                return
            seq0 = coord.control_seq
            GLOBAL_ROBOT_SUB_NODE.send_fleet_control(payload)
            code, body = applied_reply(wait_control_result(coord, seq0, cmd),
                                       {'command': cmd, 'payload': payload, 'message': '보냈다'})
            if cmd == 'profile' and code == 200:
                sync_renderer_map()
                body['message'] = '좌표 프로파일 → %s. 다음: 로봇 지도 전환 → 초기 위치 → 태블릿 좌표계 확인' % payload['name']
            elif cmd == 'robot_maps' and code == 200:
                body['message'] = '로봇에 지도 교체를 보냈다 — 로봇별 "지도 대조" 가 MATCH 가 되는지 본다'
            elif cmd == 'initial_poses' and code == 200:
                body['message'] = '출발 노드를 초기 위치로 보냈다 — 로봇이 실제로 출발 노드에 놓여 있어야 맞다'
            body['profile'] = coord.active_profile
            self._send_json(json.dumps(ops_view.json_safe(body), ensure_ascii=False, allow_nan=False).encode('utf-8'),
                            code=code)
            return

        # 3-C. 관제국 멀티로봇 플릿 제어 API (/api/fleet/start, /api/fleet/stop, /api/fleet/estop, /api/fleet/resume, /api/fleet/assign)
        elif parsed.path in ('/api/fleet/start', '/api/fleet/stop', '/api/fleet/estop', '/api/fleet/resume', '/api/fleet/assign'):
            client_ip = self.client_address[0]
            fleet_cmd_name = parsed.path.split('/')[-1]
            # D7: 멈추는 명령(stop·estop)은 어디서든 받는다 — 태블릿에서 연 V2 화면의 일시정지·비상정지가
            #     403 이면 그 버튼은 없는 것과 같다. 움직이는 명령(start·resume·assign)만 로컬 전용.
            if fleet_cmd_name not in FLEET_STOP_COMMANDS and client_ip not in LOCAL_CONTROL_IPS:
                app_log(f"[SECURITY] Blocked remote fleet command from {client_ip}")
                self._send_json(json.dumps({
                    'success': False, 'error': 'Forbidden',
                    'message': '안전 정책: 플릿 제어 명령은 현장 중계 노트북(로컬)에서만 실행할 수 있습니다.'
                }, ensure_ascii=False).encode('utf-8'), code=403)
                return

            cmd = parsed.path.split('/')[-1]
            direct = ALLOW_DIRECT_FALLBACK and GLOBAL_FLEET_COORDINATOR is not None and not GLOBAL_ROBOT_SUB_NODE
            if GLOBAL_FLEET_COORDINATOR is None or not (GLOBAL_ROBOT_SUB_NODE or direct):
                # 제3자 검수 G-2 (관제 P1): 예전엔 코디네이터가 없어도 `/fleet/lane/control` 에 내고 200 success 였다 — 그 토픽의
                # 구독처는 코디네이터 하나뿐이라 **받는 쪽 0 인 비상정지가 성공**으로 보고됐다(실물은 09-26 까지 코디네이터 없이 돌았다).
                # 보내지 않고 503 — 화면(V2·스크립트)은 5xx 를 실패로 보인다.
                why = (FLEET_IMPORT_ERROR if GLOBAL_FLEET_COORDINATOR is None
                       else 'NO_ROS_NODE — 게이트웨이 ROS 노드가 없어 코디네이터에 닿지 못한다')
                self._send_json(json.dumps({
                    'success': False, 'reason': 'NO_FLEET_COORDINATOR', 'command': cmd, 'dispatched': False,
                    'detail': why,
                    'message': '플릿 명령 불가 — 플릿 코디네이터가 없어 %s 를 받을 곳이 없다(보내지 않았다)' % cmd
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return
            if cmd == 'start' and GLOBAL_FLEET_COORDINATOR.estop_latched:
                # 코디네이터도 거부한다(비상정지 해제는 resume 한 곳) — 토픽으로 보내면 그 거부가 응답에 안 보이니 여기서 말한다
                self._send_json(json.dumps({
                    'success': False, 'reason': 'FLEET_ESTOP', 'command': cmd,
                    'message': '시작 거부 — 플릿 비상정지 중이다. 먼저 /api/fleet/resume 으로 해제한다'
                }, ensure_ascii=False).encode('utf-8'), code=409)
                return
            payload = {'cmd': cmd}
            if cmd == 'assign':
                payload['robot'] = str(req_json.get('robot', '')).strip()
                payload['start'] = str(req_json.get('start', '')).strip()
                payload['goal'] = str(req_json.get('goal', '')).strip()
                coord = GLOBAL_FLEET_COORDINATOR
                other = coord.assign_conflict(payload['robot'], payload['goal'], payload['start'] or None)
                if other:
                    # L3: 같은 목표·다른 로봇 경로 위의 목표·다른 로봇이 설 자리를 지나는 경로 — 코디네이터가 거부한다.
                    #     토픽으로 보내면 그 거부가 응답에 안 보인다
                    why = coord.assign_conflict_why(payload['robot'], payload['goal'], payload['start'] or None)
                    if not isinstance(why, str) or not why:
                        why = '%s 가 이미 목표 노드 %s 를 쓴다(도착한 로봇은 목표 노드를 계속 잡는다)' % (other, payload['goal'])
                    self._send_json(json.dumps({
                        'success': False, 'reason': 'ASSIGN_CONFLICT', 'command': cmd, 'payload': payload,
                        'other': str(other), 'message': '배정 거부 — ' + why
                    }, ensure_ascii=False).encode('utf-8'), code=409)
                    return

            if GLOBAL_ROBOT_SUB_NODE:
                coord = GLOBAL_FLEET_COORDINATOR
                seq0 = coord.control_seq
                GLOBAL_ROBOT_SUB_NODE.send_fleet_control(payload)
                base = {'command': cmd, 'dispatched_via': 'ros_topic', 'payload': payload}
                if cmd in ('start', 'resume', 'assign'):
                    # R4: 움직이게 하는 명령은 코디네이터의 처리 결과로 답한다(비상정지와 경합하면 나중에 거부될 수 있었다)
                    code, body = applied_reply(wait_control_result(coord, seq0, cmd), base)
                else:
                    code, body = 200, dict(base, success=True)   # 멈추는 명령은 기다리지 않는다
                self._send_json(json.dumps(body, ensure_ascii=False).encode('utf-8'), code=code)
                return
            else:
                # Direct fallback for minimal/test environments without full ROS_NODE (RELAY_ALLOW_DIRECT_FALLBACK=1 일 때만 — 위 검사)
                if cmd == 'start': GLOBAL_FLEET_COORDINATOR.start_fleet()
                elif cmd == 'stop': GLOBAL_FLEET_COORDINATOR.stop_fleet()
                elif cmd == 'estop': GLOBAL_FLEET_COORDINATOR.estop_fleet()
                elif cmd == 'resume': GLOBAL_FLEET_COORDINATOR.resume_fleet()
                elif cmd == 'assign': GLOBAL_FLEET_COORDINATOR.assign_route(payload['robot'], payload['start'], payload['goal'])
                self._send_json(json.dumps({'success': True, 'command': cmd, 'dispatched_via': 'direct', 'mission_state': GLOBAL_FLEET_COORDINATOR.mission_state}).encode('utf-8'))
                return

        # 3-D. 태블릿 비전 구역 이벤트 수신구 (/api/vision/zone_event)
        elif parsed.path == '/api/vision/zone_event':
            if not self._check_vision_auth(parsed):
                self._send_json(json.dumps({
                    'accepted': False, 'error': 'Unauthorized',
                    'reason': 'INVALID_OR_MISSING_TOKEN',
                    'message': '비전 API 인증 실패: 유효한 X-API-Key 또는 Bearer 토큰이 필요합니다 (401 Unauthorized).'
                }, ensure_ascii=False).encode('utf-8'), code=401)
                return

            ok, reason, norm = vision_ingest.validate_zone_event(
                req_json, allowed_robots=('pinky1', 'pinky2'))
            if not ok:
                self._send_json(json.dumps({
                    'accepted': False, 'reason': reason,
                    'message': f'ZoneEvent 유효성 검증 실패: {reason}'
                }, ensure_ascii=False).encode('utf-8'), code=400)
                return

            req_json = norm
            if GLOBAL_ROBOT_SUB_NODE:
                GLOBAL_ROBOT_SUB_NODE.send_vision_zone_event(req_json)
                self._send_json(json.dumps({'accepted': True, 'event': req_json, 'dispatched_via': 'ros_topic'}).encode('utf-8'))
                return
            elif ALLOW_DIRECT_FALLBACK and GLOBAL_FLEET_COORDINATOR:
                msg_str = String()
                msg_str.data = json.dumps(req_json)
                GLOBAL_FLEET_COORDINATOR._cb_vision_zone_event(msg_str)
                self._send_json(json.dumps({'accepted': True, 'event': req_json, 'dispatched_via': 'direct'}).encode('utf-8'))
                return
            else:
                self._send_json(json.dumps({
                    'accepted': False, 'reason': 'NO_ROS_NODE',
                    'message': '비전 구역 이벤트 수신 불가 — 게이트웨이 ROS 노드가 없습니다 (503 Service Unavailable).'
                }, ensure_ascii=False).encode('utf-8'), code=503)
                return

        # 4. 휴대폰/태블릿 카메라 스트림 URL 동적 변경 API (/api/camera/url)
        elif parsed.path == '/api/camera/url':
            new_url = req_json.get('url', '').strip()
            if new_url and GLOBAL_INGEST:
                ok = GLOBAL_INGEST.update_target_url(new_url)
                res = {'success': ok, 'url': new_url, 'message': f'휴대폰/태블릿 카메라 주소가 {new_url}(으)로 변경되었습니다.'}
                self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            else:
                self._send_json(json.dumps({'success': False, 'message': '유효한 URL이 필요합니다.'}, ensure_ascii=False).encode('utf-8'), code=400)
            return

        # 5. 자체개발 앱 및 모바일 브라우저 직접 프레임 업로드 API (/api/camera/upload, /upload, /video, /camera 등)
        elif parsed.path in ('/api/camera/upload', '/upload', '/camera', '/image', '/video', '/frame', '/shot.jpg'):
            if GLOBAL_INGEST and post_data:
                ok = GLOBAL_INGEST.ingest_direct_frame(post_data)
                self._send_json(json.dumps({'success': ok}).encode('utf-8'))
            else:
                self._send_json(json.dumps({'success': False}).encode('utf-8'), code=400)
            return

        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'404 Endpoint Not Found')

    def _send_json(self, body_bytes, code=200):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(body_bytes)))
        self.end_headers()
        self.wfile.write(body_bytes)

    def do_GET(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)

        # MCV-2A-UI 캘리브레이션 상태 (GET /api/calibration).
        # 문구는 내지 않는다 - 상태 코드와 숫자만. 말은 UI 가 소유한다(R-6).
        if parsed.path == '/api/calibration':
            ids = (GLOBAL_REGISTRY.ids() if GLOBAL_REGISTRY else [])
            body = {
                'arena': (GLOBAL_CALIB.arena.to_dict() if GLOBAL_CALIB.arena else None),
                'landmarks': (GLOBAL_CALIB.arena.landmarks if GLOBAL_CALIB.arena else {}),
                'corners': list(CORNERS),
                'sources': [_calib_state(i) for i in ids],
            }
            self._send_json(json.dumps(body, ensure_ascii=False).encode('utf-8'))
            return

        # 영수증 (MCV-2S3). 덧붙기만 하므로 과거 정착이 전부 남아 있다.
        if parsed.path == '/api/calibration/framing':
            # 읽기만 한다 - 로컬 게이트를 걸지 않는다. 이 값은 관측을 바꾸지 않는다.
            src = (urllib.parse.parse_qs(parsed.query).get('src') or [''])[0].strip()
            if not src or not GLOBAL_REGISTRY or not GLOBAL_REGISTRY.get(src):
                self._send_json(json.dumps(
                    {'success': False, 'message': '알 수 없는 소스: %s' % src},
                    ensure_ascii=False).encode('utf-8'), code=400)
                return
            try:
                res = {'success': True}
                res.update(_calib_framing(src))
                self._send_json(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            except CalibrationError as exc:
                self._send_json(json.dumps(
                    {'success': False, 'message': str(exc)},
                    ensure_ascii=False).encode('utf-8'), code=400)
            return

        if parsed.path == '/api/calibration/receipts':
            src = (query.get('src') or [None])[0]
            limit = int((query.get('limit') or ['20'])[0])
            body = {'receipts': GLOBAL_CALIB.read_receipts(src, limit=limit)}
            self._send_json(json.dumps(body, ensure_ascii=False).encode('utf-8'))
            return

        # 정지 프레임 한 장. 움직이는 스트림 위에 핀을 찍으면 손이 흔들린다.
        if parsed.path == '/api/calibration/still':
            src = (query.get('src') or [None])[0]
            jpeg = _calib_still(src)
            if not jpeg:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return

        # 정합 미리보기 — 지금 놓인 네 점으로 위에서 내려다본 그림을 만든다.
        if parsed.path == '/api/calibration/preview':
            src = (query.get('src') or [None])[0]
            jpeg, why = _calib_preview(src)
            if not jpeg:
                self._send_json(json.dumps({'success': False, 'reason': why},
                                           ensure_ascii=False).encode('utf-8'), code=409)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return

        # 이 인스턴스가 무엇인지 (GET /api/instance).
        # ⭐ 복제본 화면을 찍은 그림이 현장 증거로 오해되면 안 된다.
        #    정합 안 된 그림을 겹치지 않는 것과 같은 이유다 - 화면은 자기가 뭔지 말해야 한다.
        if parsed.path == '/api/instance':
            self._send_json(json.dumps({
                'label': os.environ.get('MCV_INSTANCE_LABEL') or None,
                'host': socket.gethostname(),
                'rosDomainId': os.environ.get('ROS_DOMAIN_ID'),
                'discoveryRange': os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE'),
            }, ensure_ascii=False).encode('utf-8'))
            return

        # MCVA-13 소스 쌍 시각 오차 (GET /api/clock). 추정이 아니라 실측 중앙값의 차다.
        if parsed.path == '/api/clock':
            self._send_json(json.dumps(clock_alignment(GLOBAL_REGISTRY),
                                       ensure_ascii=False).encode('utf-8'))
            return

        # MCV-1C 관측 세션 상태 (GET /api/observe). 켜고 끄는 것은 POST - 로컬 IP 게이트.
        if parsed.path == '/api/observe':
            self._send_json(json.dumps(GLOBAL_OBSERVATION.public(),
                                       ensure_ascii=False).encode('utf-8'))
            return

        # R-6 저하 안내 판정 (GET /api/safety)
        # 문구는 내지 않는다 - 코드와 숫자만. 말은 UI 가 소유한다.
        if parsed.path == '/api/safety':
            _pubs = 0
            if GLOBAL_ROBOT_SUB_NODE:
                try:
                    _link = GLOBAL_ROBOT_SUB_NODE.get_link_status()
                    _pubs = sum(v.get('total_publishers', 0) for v in _link.values())
                except Exception:
                    _pubs = 0
            _pose_msgs = None
            _pose_pubs = None
            if GLOBAL_ROBOT_SUB_NODE:
                try:
                    _pose_msgs = sum(v.get('pose_msgs', 0) for v in _link.values())
                except Exception:
                    _pose_msgs = None          # 못 세면 모르는 것 — fail-closed
                try:
                    # ⭐ pose 발행자만 센다. 하나라도 못 쟀으면 **모르는 것**이다 —
                    #    0 으로 떨어뜨리면 "발행자 없음" 이라는 다른 주장이 된다.
                    _each = [v.get('pose_publishers') for v in _link.values()]
                    _pose_pubs = None if any(c is None for c in _each) else sum(_each)
                except Exception:
                    _pose_pubs = None
            _ready = readiness(GLOBAL_REGISTRY, robot_publishers=_pubs,
                               observing=GLOBAL_OBSERVATION.active,
                               robot_pose_msgs=_pose_msgs,
                               pose_publishers=_pose_pubs)
            _merge_censorship(_ready)
            body = json.dumps(_ready, ensure_ascii=False).encode('utf-8')
            self._send_json(body)
            return

        # 0. MCV-1A 소스 목록 (GET /api/sources)
        if parsed.path == '/api/sources':
            body = json.dumps(
                GLOBAL_REGISTRY.sources_public() if GLOBAL_REGISTRY else {'default': None, 'sources': []},
                ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)
            return

        # 1. 태블릿 및 스마트폰/웹캠 통합 카메라 스트림 (/video_feed, /video, /stream.mjpg)
        if parsed.path in ('/video_feed', '/video', '/video.mjpg', '/stream.mjpg'):
            src_id = query.get('src', [None])[0]
            boost = query.get('boost', ['0'])[0] in ('1', 'true', 'yes')
            watermark = query.get('watermark', ['0'])[0] in ('1', 'true', 'yes')
            mode = query.get('mode', [''])[0]
            if mode in ('augmented', 'all'):
                boost = True
                watermark = True
            elif mode == 'boost':
                boost = True
            elif mode == 'watermark':
                watermark = True
            wm_style = query.get('style', ['blueprint'])[0]

            source = None
            if src_id and GLOBAL_REGISTRY:
                source = GLOBAL_REGISTRY.resolve(src_id)
                if source is None:
                    self.send_error(404, 'unknown source: %s' % src_id)
                    return

            app_log('[Stream] %s -> %s (src=%s, boost=%s, watermark=%s)' %
                    (self.client_address[0], parsed.path, src_id, boost, watermark))
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()

            _prev, _last = None, 0.0
            while True:
                try:
                    if source:
                        jpeg, _, _ = source.latest_jpeg()
                    elif GLOBAL_INGEST:
                        jpeg, _, _ = GLOBAL_INGEST.get_latest_jpeg()
                    else:
                        jpeg = None

                    _now = time.monotonic()
                    if should_send_frame(jpeg, _prev, _last, _now):
                        _src = jpeg   # ⭐ 재인코딩 **전** 원본을 붙든다 — 아래에서 jpeg 이
                                      # 바뀌면 다음 회차 비교가 보강본끼리가 되어 깨진다
                        if (boost or watermark) and GLOBAL_STREAM_ENHANCER:
                            try:
                                cv_img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                                if cv_img is not None:
                                    enhanced = GLOBAL_STREAM_ENHANCER.enhance(
                                        cv_img, enable_boost=boost, enable_watermark=watermark,
                                        watermark_mode=wm_style
                                    )
                                    ok, enc = cv2.imencode('.jpg', enhanced, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                                    if ok:
                                        jpeg = enc.tobytes()
                            except Exception:
                                pass

                        write_mjpeg_frame(self.wfile, jpeg)
                        _prev, _last = _src, _now
                    time.sleep(0.033)
                except (BrokenPipeError, ConnectionResetError):
                    break
            return

        # 1-B. 단일 JPEG 스냅샷 (/shot.jpg, /snapshot.jpg, /frame.jpg)
        elif parsed.path in ('/shot.jpg', '/snapshot.jpg', '/frame.jpg', '/current.jpg', '/image.jpg'):
            jpeg, _, _ = GLOBAL_INGEST.get_latest_jpeg()
            if jpeg is not None:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_error(503, "Camera feed not available yet")
            return

        # 2. 현장 관제 화면 스트림 (/control_feed)
        elif parsed.path == '/control_feed':
            app_log(f"[Stream] Client {self.client_address[0]} connected to /control_feed")
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()

            _prev, _last = None, 0.0
            while True:
                try:
                    jpeg = GLOBAL_CONTROL.get_latest_jpeg()
                    _now = time.monotonic()
                    if should_send_frame(jpeg, _prev, _last, _now):
                        write_mjpeg_frame(self.wfile, jpeg)
                        _prev, _last = jpeg, _now
                    time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    break
            return

        # 3. Gazebo 탑뷰 실시간 3D 카메라 스트림 (/gazebo_feed)
        elif parsed.path == '/gazebo_feed':
            app_log(f"[Stream] Client {self.client_address[0]} connected to /gazebo_feed")
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()

            _prev, _last = None, 0.0
            while True:
                try:
                    jpeg = GLOBAL_GAZEBO_CAM.get_latest_jpeg()
                    _now = time.monotonic()
                    if should_send_frame(jpeg, _prev, _last, _now):
                        write_mjpeg_frame(self.wfile, jpeg)
                        _prev, _last = jpeg, _now
                    time.sleep(0.033)
                except (BrokenPipeError, ConnectionResetError):
                    app_log(f"[Stream] Client {self.client_address[0]} disconnected from /gazebo_feed")
                    break
            return

        # 4. 로봇 온보드 카메라 스트림 (/robot_camera_feed?id=robot1 | robot2)
        elif parsed.path == '/robot_camera_feed':
            robot_id = query.get('id', ['robot1'])[0]
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()

            _prev, _last = None, 0.0
            while True:
                try:
                    jpeg = GLOBAL_ROBOT_CAMERAS.get_latest_jpeg(robot_id)
                    _now = time.monotonic()
                    if should_send_frame(jpeg, _prev, _last, _now):
                        write_mjpeg_frame(self.wfile, jpeg)
                        _prev, _last = jpeg, _now
                    time.sleep(0.066)
                except (BrokenPipeError, ConnectionResetError):
                    break
            return

        # 5. 상태 JSON API (/api/status)
        elif parsed.path == '/api/status':
            _, stamp, is_connected = GLOBAL_INGEST.get_latest_jpeg()
            robot_data = GLOBAL_CONTROL.get_robot_data()
            nav_status = GLOBAL_ROBOT_SUB_NODE.latest_nav_status if GLOBAL_ROBOT_SUB_NODE else {}
            
            cpu_temp = None
            try:
                temps = psutil.sensors_temperatures()
                if 'coretemp' in temps and temps['coretemp']:
                    cpu_temp = round(temps['coretemp'][0].current, 1)
            except Exception:
                pass
            
            mem = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            host_resources = {
                # U-5. ⭐ `psutil.cpu_percent()` 를 interval 없이 부르면 **직전 호출
                #    이후의 델타**다. 첫 호출은 언제나 0.0 이고, 스레드 여럿이 거의 동시에
                #    부르면 간격이 0 에 가까워 또 0.0 이 나온다(이 서버는 ThreadingMixIn).
                #    라이브 실측 2026-09-12: 연속 3회가 `0.0 / 0.0 / 45.7`.
                #    `CPU 0%` 는 한가하다는 뜻이 아니라 **안 쟀다**는 뜻이었다.
                #    이제 배경에서 꾸준히 재고, 아직 못 쟀으면 None 을 낸다(화면이 '측정 불가').
                'cpu_percent': GLOBAL_CPU.value(),
                'cpu_measured': GLOBAL_CPU.value() is not None,
                'cpu_temp': cpu_temp,
                'memory_percent': mem.percent,
                'memory_used_gb': round(mem.used / (1024**3), 1),
                'memory_total_gb': round(mem.total / (1024**3), 1),
                'disk_percent': disk.percent,
                'disk_free_gb': round(disk.free / (1024**3), 1)
            }

            net_status = GLOBAL_NETWORK_MONITOR.get_status() if GLOBAL_NETWORK_MONITOR else {}
            r1_online = net_status.get('robot1', {}).get('online', False)
            r2_online = net_status.get('robot2', {}).get('online', False)
            r1_rtt = net_status.get('robot1', {}).get('rtt_ms', None)
            r2_rtt = net_status.get('robot2', {}).get('rtt_ms', None)

            # MCV-0C: 아래 값들은 주장이 아니라 측정이다.
            relay_domain = os.environ.get("ROS_DOMAIN_ID")
            fleet_env = read_fleet_domains()
            _empty_link = {"linked": False, "total_publishers": 0, "publishers": {}}
            link = GLOBAL_ROBOT_SUB_NODE.get_link_status() if GLOBAL_ROBOT_SUB_NODE else {}
            r1_link = link.get("robot1", _empty_link)
            r2_link = link.get("robot2", _empty_link)
            observed_pubs = r1_link.get("total_publishers", 0) + r2_link.get("total_publishers", 0)
            any_linked = observed_pubs > 0

            fleet_comm = {
                'control_mode': 'ROBOT1_SOLO',
                'relay_domain': relay_domain,
                'relay_domain_source': 'process env ROS_DOMAIN_ID (measured)',
                'cross_talk_guard': 'SHARED_CONTROL_PLANE' if any_linked else 'UNVERIFIED_NO_ROBOT_PUBLISHERS',
                'observed_robot_publishers': observed_pubs,
                'robot1': {
                    'ip': '198.51.100.5',
                    'online': r1_online,
                    'rtt_ms': r1_rtt,
                    'domain': fleet_env.get('ROBOT1_DOMAIN_ID'),
                    'domain_source': 'fleet_domains.env (configured, not observed)',
                    'linked': r1_link.get('linked'),
                    'publishers': r1_link.get('publishers'),
                    'ssh_port': 2205,
                    'nav_state': nav_status.get('state', 'IDLE')
                },
                'robot2': {
                    'ip': '198.51.100.6',
                    'online': r2_online,
                    'rtt_ms': r2_rtt,
                    'domain': fleet_env.get('ROBOT2_DOMAIN_ID'),
                    'domain_source': 'fleet_domains.env (configured, not observed)',
                    'linked': r2_link.get('linked'),
                    'publishers': r2_link.get('publishers'),
                    'ssh_port': 2206,
                    'state': 'LINKED' if r2_link.get('linked') else 'NO_PUBLISHERS'
                },
                'p2p_link': {
                    'status': 'OBSERVED_SHARED_PLANE' if any_linked else 'UNVERIFIED',
                    'physical_lan': 'CONNECTED' if (r1_online and r2_online) else 'PARTIAL',
                    'summary': ('관제 평면(도메인 ' + str(relay_domain) + ')에서 관측된 로봇 발행자 '
                                + str(observed_pubs) + '개. 0 이면 domain_bridge 가 없거나 '
                                '로봇 노드가 떠 있지 않다는 뜻이며, 격리가 증명된 것이 아니다.')
                }
            }

            status = {
                'timestamp': time.time(),
                'fleet_comm': fleet_comm,
                'tablet_camera': {
                    'connected': is_connected,
                    'last_stamp': stamp,
                    'primary_url': GLOBAL_INGEST.primary_url,
                    'current_url': GLOBAL_INGEST.current_url
                },
                'robots': robot_data,
                # 연산 노드 좌표의 **신선도**. 좌표는 신선할 때만 실린다 —
                # 낡으면 키 자체가 없다(vision_ingest.report).
                # ⭐ 신선도(vision_ingest, 순수 로직)에 **경로 사실**을 덧붙인다.
                #    vision_ingest 는 ROS 를 모르므로 여기서 합친다 — 그 모듈을 순수하게 둔다.
                'visionPose': _with_vision_path(
                    GLOBAL_VISION.report(VISION_ROBOT_IDS, int(time.time() * 1000))),
                'robot1_nav': nav_status,
                'discrepancy': GLOBAL_ROBOT_SUB_NODE.get_discrepancy() if GLOBAL_ROBOT_SUB_NODE else {},
                # ⚠️ 이 값들은 **하드코딩**이고 화면(index.html)·렌더러 기본값과
                #    서로 다른 좌표계다(U-11). `source` 는 렌더러가 실제로 읽은 것을
                #    말한다 — `"synthetic"` 이면 지도가 **허구**이고 좌표를 믿으면 안 된다.
                'map_meta': renderer_map_meta(),
                'network_latency': GLOBAL_NETWORK_MONITOR.get_status() if GLOBAL_NETWORK_MONITOR else {},
                'host_resources': host_resources,
                'fleet': GLOBAL_FLEET_COORDINATOR.get_fleet_status_dict() if GLOBAL_FLEET_COORDINATOR else None,
                'gateway': {
                    'port': self.server.server_port,
                    'status': 'OPERATIONAL'
                },
                # U-1 (HANDOFF_20260928 §3-3): 이 요청의 출처가 움직이는 조작을 낼 수 없는 곳이면 화면이 보기 전용으로
                # 그린다(V2 는 이 응답을 state.gateway 로 두고 `state.gateway?.view_only` 를 본다 — 최상위). 18081 원격 보기
                # 경로는 socat 이 127.0.0.2 로 붙어 들어오므로 포트·쿼리와 무관하게 서버가 말한다 — 화면이 숨기는 조작 =
                # 서버가 403 으로 거절할 조작(같은 판정 집합 LOCAL_CONTROL_IPS).
                'view_only': self.client_address[0] not in LOCAL_CONTROL_IPS,
            }
            body = json.dumps(status).encode('utf-8')
            self._send_json(body)
            return

        # 5-B. 플릿 관제 상태 전용 JSON API (/api/fleet/status)
        # 5-A2. R-7 좌표 프로파일 — 목록·현재·로봇별 지도 대조, 화면 지도
        elif parsed.path == '/api/fleet/profiles':
            coord = GLOBAL_FLEET_COORDINATOR
            if coord is None:
                self._send_json(json.dumps({'success': False, 'reason': 'NO_FLEET_COORDINATOR',
                                            'detail': FLEET_IMPORT_ERROR}, ensure_ascii=False).encode('utf-8'), code=503)
                return
            body = dict(coord.profile_status())
            body['mission_state'] = coord.mission_state
            body['robots'] = {n: coord.robot_map_check(n) for n in coord.robots}
            body['screen_map'] = renderer_map_meta()
            self._send_json(json.dumps(ops_view.json_safe(body), ensure_ascii=False, allow_nan=False).encode('utf-8'))
            return
        elif parsed.path == '/api/fleet/profile_map':
            self._send_json(json.dumps(ops_view.json_safe(renderer_map_meta()), ensure_ascii=False,
                                       allow_nan=False).encode('utf-8'))
            return
        elif parsed.path == '/api/fleet/profile_map.png':
            src = getattr(GLOBAL_CONTROL, 'map_source', None) if GLOBAL_CONTROL else None
            img = cv2.imread(src, cv2.IMREAD_GRAYSCALE) if src and src != 'synthetic' and os.path.isfile(src) else None
            ok, png = cv2.imencode('.png', img) if img is not None else (False, None)
            if not ok:
                self.send_response(404)
                self.end_headers()
                return
            data = png.tobytes()
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        elif parsed.path == '/api/fleet/status':
            if GLOBAL_FLEET_COORDINATOR:
                status_dict = GLOBAL_FLEET_COORDINATOR.get_fleet_status_dict()
            else:
                status_dict = {"status": "FLEET_COORDINATOR_UNAVAILABLE", "robots": {}, "detail": FLEET_IMPORT_ERROR}
            self._send_json(json.dumps(status_dict, ensure_ascii=False).encode('utf-8'))
            return

        # 5-C. 관제 화면 진단 API (R-5) — overview·diagnostics 둘만 먼저. 없는 칸은 "미수신".
        elif parsed.path in ('/api/ops/overview', '/api/ops/diagnostics'):
            diag = GLOBAL_ROBOT_SUB_NODE.diag_snapshot() if GLOBAL_ROBOT_SUB_NODE else {}
            fleet = GLOBAL_FLEET_COORDINATOR.get_fleet_status_dict() if GLOBAL_FLEET_COORDINATOR else None
            build = ops_view.overview if parsed.path.endswith('overview') else ops_view.diagnostics
            body = ops_view.json_safe(build(OPS_ROBOTS, diag, fleet, time.time()))
            self._send_json(json.dumps(body, ensure_ascii=False, allow_nan=False).encode('utf-8'))
            return

        # 6. 실시간 로그 JSON API (/api/logs)
        elif parsed.path == '/api/logs':
            with LOG_LOCK:
                logs_copy = list(LOG_BUFFER[-60:])
            body = json.dumps({'logs': logs_copy}).encode('utf-8')
            self._send_json(body)
            return

        # 7. 젠킨스 상태 API (/api/jenkins/status)
        elif parsed.path == '/api/jenkins/status':
            res = get_jenkins_status()
            body = json.dumps(res).encode('utf-8')
            self._send_json(body)
            return

        # 8. 젠킨스 빌드 트리거 API (/api/jenkins/build)
        elif parsed.path == '/api/jenkins/build':
            target = query.get('target', ['robot1'])[0]
            action = query.get('action', ['deploy'])[0]
            ok, msg = trigger_jenkins_build(target, action)
            res = {'success': ok, 'message': msg, 'target': target, 'action': action}
            body = json.dumps(res).encode('utf-8')
            self._send_json(body, code=200 if ok else 500)
            return

        # 8-B. 카메라 스트림 정보 API (/api/camera/url)
        elif parsed.path == '/api/camera/url':
            info = {
                'primary_url': GLOBAL_INGEST.primary_url if GLOBAL_INGEST else '',
                'current_url': GLOBAL_INGEST.current_url if GLOBAL_INGEST else '',
                'connected': GLOBAL_INGEST.is_connected if GLOBAL_INGEST else False,
                'fps': round(GLOBAL_INGEST._fps, 1) if GLOBAL_INGEST else 0.0
            }
            self._send_json(json.dumps(info, ensure_ascii=False).encode('utf-8'))
            return

        # 8-C. 로컬 카메라 자동 스캔 API (/api/camera/scan)
        elif parsed.path == '/api/camera/scan':
            candidates = GLOBAL_INGEST.auto_scan_cameras() if GLOBAL_INGEST else []
            self._send_json(json.dumps({'candidates': candidates}, ensure_ascii=False).encode('utf-8'))
            return

        # 8-D. 모바일 브라우저 카메라 송출 페이지 (/camera_streamer)
        elif parsed.path == '/camera_streamer':
            streamer_html = CAMERA_STREAMER_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Length', str(len(streamer_html)))
            self.end_headers()
            self.wfile.write(streamer_html)
            return

        # 9. 정적 파일 (index.html 등)
        else:
            filename = 'index.html' if parsed.path in ('/', '') else parsed.path.lstrip('/')
            filepath = os.path.join(STATIC_DIR, filename)

            if os.path.exists(filepath) and os.path.isfile(filepath):
                with open(filepath, 'rb') as f:
                    content = f.read()
                
                content_type = 'text/html; charset=utf-8'
                if filepath.endswith('.js'):
                    content_type = 'application/javascript'
                elif filepath.endswith('.css'):
                    content_type = 'text/css'
                elif filepath.endswith('.png'):
                    content_type = 'image/png'
                elif filepath.endswith(('.jpg', '.jpeg')):
                    content_type = 'image/jpeg'

                self.send_response(200)
                self.send_header('Content-Type', content_type)
                if content_type.startswith(('text/html', 'application/javascript', 'text/css')):
                    # 게이트웨이를 새로 올려도 브라우저가 옛 화면을 캐시해 두면 옛 동작을 한다(R-7 브라우저 확인 때 실제로
                    # 겪음) — 화면 코드는 매번 서버에 묻는다(바뀌지 않았으면 그대로 쓴다)
                    self.send_header('Cache-Control', 'no-cache')
                self.send_header('Content-Length', str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b'404 Not Found')


class RobotDataSubscriberNode(Node):
    """로봇 1, 2 위치 및 Gazebo 실시간 카메라 토픽 구독자 & 네비게이션 목표 발행기"""
    def __init__(self, control_renderer, robot_camera_manager, gazebo_camera_manager):
        super().__init__('robot_data_subscriber_node')
        self.control = control_renderer
        self.cam_mgr = robot_camera_manager
        self.gz_cam_mgr = gazebo_camera_manager
        self.bridge = CvBridge()

        # 로봇 1 위치 구독
        self.sub_r1_odom = self.create_subscription(
            Odometry, '/robot1/odom', self._cb_r1_odom, 10)
        self.sub_r1_pose = self.create_subscription(
            PoseStamped, '/robot1/pose', self._cb_r1_pose, 10)

        # Gazebo 시뮬레이션 위치 구독 (/odom)
        self.sub_gz_odom = self.create_subscription(
            Odometry, '/odom', self._cb_gz_odom, 10)
        # 🔴 기본값을 (0,0,0) 으로 두면 **'안 왔다'와 '원점에 있다'가 구분되지 않는다.**
        #    그 둘을 빼면 오차 0.0 이 나오고, 화면은 그걸 '실측 일치 극우수'(초록)로 읽었다
        #    — 못 쟀다는 사실이 가장 좋은 소식으로 둔갑한다 (QA Q-1, 2026-09-12).
        #    None 이면 값으로 표현할 수가 없다. 읽는 곳은 get_discrepancy() 하나뿐이다.
        self.latest_gz_pose = None
        self.latest_r1_pose = None
        # R-1 (2026-09-12). 발행자가 있는 것과 **값이 오는 것**은 다른 사실이다.
        # AMCL `update_min_d: 0.05` 라 정지한 로봇은 pose 를 안 낸다 —
        # 그 동안 발행자는 1 이고, 그걸 '보정 가능' 으로 읽으면 안 된다.
        # ⚠️ `latest_*_pose` 의 기본값 (0,0,0) 은 '안 왔다' 와 '원점에 있다' 가
        #    구분되지 않는다. 이 계수기가 그 구분을 만든다.
        self.pose_msgs = {'robot1': 0, 'robot2': 0}

        # 로봇 2 위치 구독
        self.sub_r2_odom = self.create_subscription(
            Odometry, '/robot2/odom', self._cb_r2_odom, 10)
        self.sub_r2_pose = self.create_subscription(
            PoseStamped, '/robot2/pose', self._cb_r2_pose, 10)

        # 로봇 1 온보드 카메라 구독 (raw & compressed)
        self.sub_r1_cam = self.create_subscription(
            RosImage, '/robot1/camera/image_raw', self._cb_r1_cam, 5)
        self.sub_r1_comp = self.create_subscription(
            CompressedImage, '/robot1/camera/image_raw/compressed', self._cb_r1_comp, qos_profile_sensor_data)

        # 로봇 2 온보드 카메라 구독 (raw & compressed)
        self.sub_r2_cam = self.create_subscription(
            RosImage, '/robot2/camera/image_raw', self._cb_r2_cam, 5)
        self.sub_r2_comp = self.create_subscription(
            CompressedImage, '/robot2/camera/image_raw/compressed', self._cb_r2_comp, qos_profile_sensor_data)

        # Gazebo 3D 실시간 탑뷰 카메라 구독 (/camera 토픽)
        self.sub_gz_cam = self.create_subscription(
            RosImage, '/camera', self._cb_gz_cam, 5)

        # 로봇 1 네비게이션 텔레메트리 구독 (/robot1/nav_status)
        self.sub_nav_status = self.create_subscription(
            String, '/robot1/nav_status', self._cb_nav_status, 10)
        self.latest_nav_status = {}

        # 로봇 1 자율주행 목표 및 미션 명령 발행기
        # ⭐ 관제 평면(도메인 8)에서는 **로봇별 이름으로** 발행한다. 브리지가 해당
        #    로봇 도메인 안에서만 표준 이름(goal_pose)으로 되돌린다. 접두어 없는
        #    `/goal_pose` 는 로봇이 같은 도메인에 들어오는 순간 **4대 전부**에게 간다.
        #    (robot1_control.yaml 의 다운링크 화이트리스트 주석이 막으려던 사고다.)
        self.pub_goal = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.pub_mission_cmd = self.create_publisher(String, '/robot1/mission_cmd', 10)

        # 연산 노드가 낸 좌표를 관제 평면으로 올리는 발행자 (로봇별 이름).
        # ⭐ 이름을 `vision_pose` 로 따로 둔다 — `amcl_pose`(로봇이 스스로 믿는 값)와
        #    `odom`(추측항법)과 **다른 출처**이고, 화면이 셋을 섞으면 어느 것을 보고
        #    있는지 말할 수 없게 된다.
        #
        # 🔴 이름을 **리터럴로** 적는다. 포맷 문자열(`'/%s/vision_pose' % rid`)로 쓰면
        #    두 가지가 깨진다 — 2026-09-19 에 `test_control_topic_naming` 이 잡았다:
        #      1) `grep '/robot1/vision_pose'` 에 안 걸린다. 이 레포는 "검색 가능한
        #         식별자" 를 규율로 두고 있고 토픽 이름도 그 대상이다.
        #      2) 정적 검사가 브리지 설정과 대사할 수 없다. `goal_pose`·`mission_cmd` 도
        #         같은 이유로 리터럴이다(바로 위 두 줄).
        #    ⚠️ 아래 dict 의 키는 `VISION_ROBOT_IDS` 와 **반드시 같아야** 한다.
        #       어긋나면 검증·발행이 서로 다른 로봇 집합을 보게 되므로 즉시 죽인다.
        self.pub_vision_pose = {
            'robot1': self.create_publisher(PoseStamped, '/robot1/vision_pose', 10),
            'robot2': self.create_publisher(PoseStamped, '/robot2/vision_pose', 10),
        }
        assert set(self.pub_vision_pose) == set(VISION_ROBOT_IDS), (
            "VISION_ROBOT_IDS 와 발행자 목록이 어긋났다: %s vs %s"
            % (sorted(VISION_ROBOT_IDS), sorted(self.pub_vision_pose)))

        # Track R (R-D1): Canonical PoseFix 발행자 (/pinky1/pose_fix, /pinky2/pose_fix)
        # QoS: Reliable, Volatile, Depth 5
        self.pub_pose_fix = {}
        if RosPoseFix is not None:
            pose_fix_qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
            self.pub_pose_fix = {
                'pinky1': self.create_publisher(RosPoseFix, '/pinky1/pose_fix', pose_fix_qos),
                'pinky2': self.create_publisher(RosPoseFix, '/pinky2/pose_fix', pose_fix_qos),
            }

        # Gazebo 디지털 트윈 실시간 위치 동기화 워커 (메인 & 서브 2대)
        self._target_gz_r1 = None
        self._target_gz_r2 = None
        self._last_synced_r1 = (None, None, None)
        self._last_synced_r2 = (None, None, None)
        self._gz_sync_thread = threading.Thread(target=self._gz_sync_worker, daemon=True)
        self._gz_sync_thread.start()

        # 관제 플릿 제어 및 비전 구역 이벤트 발행기 (HTTP 쓰레드 분리 및 안전한 ROS 토픽 단일화)
        self.pub_fleet_control = self.create_publisher(String, '/fleet/lane/control', 10)
        # R-5: 로봇 진단 업링크 (/pinkyN/diag, JSON 1 Hz). 받은 시각은 **중계 시계**로 적는다 —
        #      로봇 시계는 기기마다 어긋나 신선도 판정에 못 쓴다.
        self.diag_latest = {}
        self._diag_lock = threading.Lock()
        for _name in OPS_ROBOTS:
            self.create_subscription(String, f'/{_name}/diag',
                                     lambda msg, n=_name: self._cb_diag(n, msg), 10)
        self.pub_vision_zone_event = self.create_publisher(String, '/vision/zone_event', 10)

        app_log("[ROS2] RobotDataSubscriberNode started (/robot1, /odom, /robot2, /camera + Click-Nav Publisher + Fleet)")

    def _cb_diag(self, name, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return                              # 못 읽은 진단은 없는 것 — 지난 값을 덮지도 않는다
        with self._diag_lock:
            self.diag_latest[name] = (d, time.time())

    def diag_snapshot(self):
        with self._diag_lock:
            return dict(self.diag_latest)

    def send_fleet_control(self, cmd_dict: dict) -> None:
        """HTTP 쓰레드에서 관제 토픽으로 비동기 안전 전달."""
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self.pub_fleet_control.publish(msg)

    def send_vision_zone_event(self, event_dict: dict) -> None:
        """HTTP 쓰레드에서 비전 구역 이벤트 토픽으로 비동기 안전 전달."""
        msg = String()
        msg.data = json.dumps(event_dict)
        self.pub_vision_zone_event.publish(msg)

    def get_link_status(self):
        """MCV-0C: 관제 평면에 로봇 발행자가 실제로 있는지 센다.

        토픽이 `ros2 topic list` 에 보이는 것과 발행자가 있는 것은 다른 사실이다.
        구독자만 있어도 목록에는 나오므로, 판정은 count_publishers 로 한다.
        """
        probes = {
            'robot1': ('/robot1/odom', '/robot1/pose', '/robot1/camera/image_raw', '/robot1/camera/image_raw/compressed', '/robot1/scan'),
            'robot2': ('/robot2/odom', '/robot2/pose', '/robot2/camera/image_raw', '/robot2/camera/image_raw/compressed', '/robot2/scan'),
        }
        out = {}
        for name, topics in probes.items():
            counts = {}
            for t in topics:
                try:
                    counts[t] = self.count_publishers(t)
                except Exception:
                    counts[t] = None
            total = sum(c for c in counts.values() if isinstance(c, int))
            # ⭐ pose 발행자를 **따로** 낸다. 합계(total_publishers)를 pose 칸에 쓰면
            #    카메라·scan 발행자가 "pose 발행자 없음" 을 가린다(2026-09-14 실측:
            #    /robot1/pose 는 0 인데 /robot1/scan 1 때문에 publishers=1 이 나왔다).
            # ⚠️ 못 쟀으면 None 이다 — 0 으로 떨어뜨리지 않는다.
            _pose_c = counts.get('/%s/pose' % name)
            # ⭐ 발행자 수 옆에 **실제로 받은 건수**를 같이 낸다. 판정 기준이
            #    topic list -> Publisher count -> 유량 으로 늘어왔는데 여기만 빠져 있었다.
            out[name] = {'publishers': counts, 'total_publishers': total,
                         'pose_publishers': _pose_c if isinstance(_pose_c, int) else None,
                         'linked': total > 0,
                         'pose_msgs': self.pose_msgs.get(name, 0)}
        return out

    def get_discrepancy(self):
        """정합 오차. **안 왔으면 숫자를 만들지 않는다.**

        ⭐ 예전 판은 두 기본값 (0,0,0) 을 빼서 0.0 을 냈다. 그 0.0 이 화면에서
           초록 '실측 일치 극우수' 가 됐다 — 설계서 §2 원칙 1 위반이다.
           `state` 를 먼저 보고, MEASURED 가 아니면 숫자를 쓰지 않는다.
        """
        r1 = self.latest_r1_pose
        gz = self.latest_gz_pose
        if r1 is None or gz is None:
            missing = []
            if r1 is None: missing.append('ROBOT_POSE')
            if gz is None: missing.append('SIM_POSE')
            return {
                'state': 'UNMEASURED',
                'why': '+'.join(missing),
                'r1_real': r1,
                'gz_sim': gz,
            }
        dx = r1['x'] - gz['x']
        dy = r1['y'] - gz['y']
        dist_m = math.hypot(dx, dy)
        dyaw_deg = math.degrees(r1['yaw'] - gz['yaw'])
        while dyaw_deg > 180.0: dyaw_deg -= 360.0
        while dyaw_deg < -180.0: dyaw_deg += 360.0
        return {
            'state': 'MEASURED',
            'dx_m': round(dx, 3),
            'dy_m': round(dy, 3),
            'dist_err_cm': round(dist_m * 100, 1),
            'dyaw_deg': round(dyaw_deg, 1),
            'r1_real': r1,
            'gz_sim': gz
        }

    def _cb_nav_status(self, msg: String):
        try:
            st = json.loads(msg.data)
            self.latest_nav_status = st
            if self.control:
                self.control.set_nav_status(st)
        except Exception:
            pass

    def send_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(msg)
        if self.control:
            self.control.set_goal(x, y)
        app_log(f"🎯 [Gateway] Published /robot1/goal_pose -> (x={x:.2f}, y={y:.2f})")

    def vision_pose_receivers(self, robot_id):
        """도메인 8 의 `robotN/vision_pose` 구독자를 **누구인지까지** 센다.

        🔴 `get_subscription_count()` 는 **수만** 준다. 그 수의 1 이 브리지면
           "로봇이 받는다" 가 아니라 "중간 다리까지 갔다" 다. 2026-09-19 에 그 오해가
           실제로 났고, 연산 노드가 `hasReceiver: true` 를 종단 근거로 쓸 뻔했다.

        ⭐ 그래서 **노드 이름을 함께 돌려준다.** `bridge`/`consumer` 분류는 이름 접두어로
           하는 **편의**이고, 진실은 `nodes` 목록이다 — 소비자는 그것을 봐야 한다.
        ⚠️ 못 셌으면 `measured: False` 이고 수는 **`None`** 이다. 0 이 아니다.
        """
        topic = '/%s/vision_pose' % robot_id
        try:
            infos = self.get_subscriptions_info_by_topic(topic)
        except Exception:
            # 못 쟀다. 판정은 순수 함수에 맡긴다 — 0 으로 떨어뜨리지 않는다.
            return classify_receivers(None, topic=topic)
        names = []
        for info in infos:
            space = info.node_namespace
            if not space.endswith('/'):
                space += '/'
            names.append(space + info.node_name)
        return classify_receivers(names, topic=topic)

    def publish_vision_pose(self, robot_id, x, y, yaw):
        """연산 노드 좌표를 관제 평면에 올리고 **그 순간의 구독자 수**를 돌려준다.

        🔴 `send_mission` 과 같은 이유로 구독자 수를 함께 돌려준다 — 발행은 수신을
           뜻하지 않는다. 호출자가 이 값을 응답에 실어야 "보냈다" 가 검증 가능해진다.
        """
        pub = self.pub_vision_pose.get(robot_id)
        if pub is None:
            return False, None
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
        pub.publish(msg)
        # ⭐ 셋은 **다른 사실**이다: 발행 호출이 돌았다 / 구독자 수를 쟀다 / 수신자가 있다.
        #    하나로 묶으면 같은 커밋의 정지 API 와 정직 기준이 갈린다(정지는 구독자 0 을
        #    실패로 본다). 호출자가 셋을 따로 응답에 실을 수 있게 튜플로 돌려준다.
        try:
            return True, int(pub.get_subscription_count())
        except Exception:
            return True, None

    def publish_pose_fix(self, norm: dict):
        """Canonical PoseFix 메시지를 관제 도메인 8에 발행한다 (Track R: R-D1).

        Relay에서 좌표를 재계산하지 않고 원본 계측치를 보존하여 발행한다.
        반환값: (published: bool, subscribers: Optional[int])
        """
        robot_name = norm.get('robot_name', '')
        pub = self.pub_pose_fix.get(robot_name)
        if pub is None or RosPoseFix is None:
            return False, None

        msg = RosPoseFix()
        msg.header.frame_id = norm.get('frame_id', 'map')
        # P0: 원본 계측 타임스탬프 100% 보존 (Relay 시계 대체 절대 금지)
        msg.header.stamp.sec = int(norm['stamp_sec'])
        msg.header.stamp.nanosec = int(norm['stamp_nanosec'])

        msg.stamp_is_robot_clock = bool(norm.get('stamp_is_robot_clock', False))
        msg.robot_name = robot_name
        msg.seq = int(norm.get('seq', 0))
        msg.x = float(norm['x'])
        msg.y = float(norm['y'])
        msg.yaw = float(norm['yaw'])
        msg.marker_id = int(norm.get('marker_id', 0))
        msg.marker_range = float(norm.get('marker_range', 0.0))
        msg.reproj_error = float(norm.get('reproj_error', 0.0))
        msg.n_markers = int(norm.get('n_markers', 0))
        msg.pipeline_latency = float(norm.get('pipeline_latency', 0.0))

        pub.publish(msg)
        try:
            subs = int(pub.get_subscription_count())
        except Exception:
            subs = None
        return True, subs

    def send_mission(self, cmd_str):
        """미션 명령을 발행하고 **그 순간의 구독자 수**를 돌려준다.

        🔴 발행은 수신을 뜻하지 않는다. 2026-09-17 22:25 실측에서 도메인 10 의
           `/robot1/mission_cmd` 구독자가 **0** 이었다 — 정지를 눌러도 아무 데도
           안 갔는데 API 는 성공을 냈다. 그 거짓을 끊으려면 호출자가 이 값을 봐야 한다.
        """
        msg = String()
        msg.data = cmd_str
        self.pub_mission_cmd.publish(msg)
        try:
            subs = int(self.pub_mission_cmd.get_subscription_count())
        except Exception:
            subs = None        # 못 세면 모르는 것 — 0 으로 치지 않는다
        app_log(f"🚩 [Gateway] Published /robot1/mission_cmd -> {cmd_str} (subscribers={subs})")
        return subs

    def _gz_sync_worker(self):
        """실물 로봇 1번(pinky) 및 로봇 2번(pinky_sub)의 위치를 Gazebo 속 3D 로봇 위치로 실시간 동기화"""
        import math, subprocess
        while rclpy.ok():
            # 1. Main Robot (pinky) 동기화
            if self._target_gz_r1:
                x, y, yaw = self._target_gz_r1
                lx, ly, lyaw = self._last_synced_r1
                if lx is None or abs(x - lx) > 0.005 or abs(y - ly) > 0.005 or abs(yaw - lyaw) > 0.03:
                    qz = math.sin(yaw / 2.0)
                    qw = math.cos(yaw / 2.0)
                    req = f'name: "pinky", position: {{x: {x:.3f}, y: {y:.3f}, z: 0.05}}, orientation: {{z: {qz:.4f}, w: {qw:.4f}}}'
                    cmd = ['gz', 'service', '-s', '/world/pinky_factory/set_pose',
                           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                           '--timeout', '200', '--req', req]
                    try:
                        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=0.4)
                        self._last_synced_r1 = (x, y, yaw)
                    except Exception:
                        pass

            # 2. Sub Robot (pinky_sub) 동기화 (기본 Y 오프셋 +0.4m)
            if False and self._target_gz_r2:  # [Solo Mode: 1번 로봇만 출력 및 동기화]
                x, y, yaw = self._target_gz_r2
                lx, ly, lyaw = self._last_synced_r2
                if lx is None or abs(x - lx) > 0.005 or abs(y - ly) > 0.005 or abs(yaw - lyaw) > 0.03:
                    qz = math.sin(yaw / 2.0)
                    qw = math.cos(yaw / 2.0)
                    req = f'name: "pinky_sub", position: {{x: {x:.3f}, y: {y + 0.4:.3f}, z: 0.05}}, orientation: {{z: {qz:.4f}, w: {qw:.4f}}}'
                    cmd = ['gz', 'service', '-s', '/world/pinky_factory/set_pose',
                           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                           '--timeout', '200', '--req', req]
                    try:
                        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=0.4)
                        self._last_synced_r2 = (x, y, yaw)
                    except Exception:
                        pass

            time.sleep(0.05)

    def _extract_yaw(self, q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _cb_r1_odom(self, msg):
        pos = msg.pose.pose.position
        yaw = self._extract_yaw(msg.pose.pose.orientation)
        self.latest_r1_pose = {'x': round(pos.x, 3), 'y': round(pos.y, 3), 'yaw': round(yaw, 3)}
        self.control.update_robot_pose('robot1', pos.x, pos.y, yaw)
        self._target_gz_r1 = (pos.x, pos.y, yaw)

    def _cb_r1_pose(self, msg):
        self.pose_msgs['robot1'] += 1
        pos = msg.pose.position
        yaw = self._extract_yaw(msg.pose.orientation)
        self.latest_r1_pose = {'x': round(pos.x, 3), 'y': round(pos.y, 3), 'yaw': round(yaw, 3)}
        self.control.update_robot_pose('robot1', pos.x, pos.y, yaw)
        self._target_gz_r1 = (pos.x, pos.y, yaw)

    def _cb_gz_odom(self, msg):
        pos = msg.pose.pose.position
        yaw = self._extract_yaw(msg.pose.pose.orientation)
        self.latest_gz_pose = {'x': round(pos.x, 3), 'y': round(pos.y, 3), 'yaw': round(yaw, 3)}
        self.control.update_robot_pose('gazebo_sim', pos.x, pos.y, yaw)

    def _cb_r2_odom(self, msg):
        pos = msg.pose.pose.position
        yaw = self._extract_yaw(msg.pose.pose.orientation)
        self.control.update_robot_pose('robot2', pos.x, pos.y, yaw)
        # self._target_gz_r2 = (pos.x, pos.y, yaw)  # [Solo Mode]

    def _cb_r2_pose(self, msg):
        self.pose_msgs['robot2'] += 1
        pos = msg.pose.position
        yaw = self._extract_yaw(msg.pose.orientation)
        self.control.update_robot_pose('robot2', pos.x, pos.y, yaw)
        # self._target_gz_r2 = (pos.x, pos.y, yaw)  # [Solo Mode]

    def _cb_r1_cam(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.cam_mgr.update_frame('robot1', cv_img)
        except Exception:
            pass

    def _cb_r1_comp(self, msg):
        try:
            self.cam_mgr.update_compressed('robot1', msg.data)
        except Exception:
            pass

    def _cb_r2_cam(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.cam_mgr.update_frame('robot2', cv_img)
        except Exception:
            pass

    def _cb_r2_comp(self, msg):
        try:
            self.cam_mgr.update_compressed('robot2', msg.data)
        except Exception:
            pass

    def _cb_gz_cam(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.gz_cam_mgr.update_frame(cv_img)
        except Exception:
            pass


def run_ros_spin(executor):
    try:
        executor.spin()
    except Exception:
        pass


def main():
    global GLOBAL_INGEST, GLOBAL_CONTROL, GLOBAL_ROBOT_CAMERAS, GLOBAL_GAZEBO_CAM, GLOBAL_NETWORK_MONITOR, GLOBAL_ROBOT_SUB_NODE

    import argparse
    parser = argparse.ArgumentParser(description="Field Video & Robot Control Web Gateway")
    parser.add_argument('--port', type=int, default=8889, help="Web server port (default: 8889)")
    # ⭐ 기본값을 비운다. 주소를 지어내면 죽은 주소로 영원히 재접속하고, 그 로그가
    #    진짜 접속 실패를 가린다(2026-09-12 실측 5,293줄). 당겨올 소스는
    #    configs/video_sources.json 이 후보 목록까지 들고 제대로 한다.
    parser.add_argument('--tablet-url', type=str, default="",
                        help="레거시 pull 주소. 비우면 pull 안 한다(권장) — "
                             "당겨올 소스는 configs/video_sources.json 에 둔다")
    parser.add_argument('--fallback-url', type=str, default="",
                        help="레거시 pull 폴백 주소. 비우면 안 쓴다")
    parser.add_argument('--map-yaml', type=str, default="$HOME/my_map.yaml", help="Path to my_map.yaml")
    parser.add_argument('--no-camera', action='store_true', help="Disable background camera ingest and relay")
    args = parser.parse_args()

    app_log("================================================================")
    app_log(" 🛰️  Educational Field Gateway Relay Server Starting")
    app_log(f" - Web Port        : {args.port} (0.0.0.0:{args.port})")
    app_log(f" - Primary Stream  : {args.tablet_url or '(없음 — push 전용)'}")
    app_log(f" - Fallback Stream : {args.fallback_url}")
    app_log(f" - Map Config      : {args.map_yaml}")
    app_log(f" - Camera Disabled : {args.no_camera}")
    app_log(f" - Log File        : {LOG_FILE}")
    app_log("================================================================")

    GLOBAL_INGEST = TabletStreamIngest(args.tablet_url, args.fallback_url)
    if not args.no_camera:
        GLOBAL_INGEST.start()
    else:
        app_log("[Camera] Background camera capture loop is DISABLED (--no-camera)")

    GLOBAL_CONTROL = ControlScreenRenderer(args.map_yaml)
    global GLOBAL_DEFAULT_MAP_YAML
    GLOBAL_DEFAULT_MAP_YAML = args.map_yaml
    GLOBAL_ROBOT_CAMERAS = RobotCameraManager()
    GLOBAL_GAZEBO_CAM = GazeboCameraManager()
    GLOBAL_NETWORK_MONITOR = NetworkLatencyMonitor()

    # --- MCV-1A 소스 레지스트리 ---
    # trust: 관측(ArUco)·pose 보정은 trusted 소스만 입력으로 쓴다(결정 1A).
    # 무인증 push(/api/camera/upload)로 들어오는 태블릿/폰 프레임은 untrusted.
    global GLOBAL_REGISTRY
    GLOBAL_REGISTRY = SourceRegistry()
    GLOBAL_REGISTRY.register(Source(
        'tablet', GLOBAL_INGEST.get_latest_jpeg, PUSH, UNTRUSTED,
        label='태블릿/폰 카메라 (앱 push)'), default=True)
    GLOBAL_REGISTRY.register(Source(
        'gazebo', GLOBAL_GAZEBO_CAM.get_latest, ROS, TRUSTED,
        label='Gazebo 가상 천장'))
    GLOBAL_REGISTRY.register(Source(
        'control', _jpeg_only_provider(GLOBAL_CONTROL.get_latest_jpeg), ROS, TRUSTED,
        # ROS transport 라 REAL_VIEWPOINT_TRANSPORTS 에서 이미 빠지지만, 그 목록이
        # 나중에 넓어져도 관제 화면이 **실물 시점으로 조용히 승격되지 않게** 못 박는다.
        label='관제 화면', viewpoint=False))
    GLOBAL_REGISTRY.register(Source(
        'robot1', _jpeg_only_provider(lambda: GLOBAL_ROBOT_CAMERAS.get_latest_jpeg('robot1')), ROS, TRUSTED,
        label='로봇1 온보드'))
    GLOBAL_REGISTRY.register(Source(
        'robot2', _jpeg_only_provider(lambda: GLOBAL_ROBOT_CAMERAS.get_latest_jpeg('robot2')), ROS, TRUSTED,
        label='로봇2 온보드'))

    # pull 소스 — 폰·태블릿은 각자 :18082 로 서빙한다. push 슬롯만 두면 아무도 안 밀 때
    # 레지스트리가 비어 있다(2026-09-09 실측: 기기는 22.8/12.8 fps 인데 레지스트리 fps 0).
    # 주소는 configs/video_sources.json 이 소유한다 — 코드에 박지 않는다.
    # trust=trusted 근거: pull 은 게이트웨이가 출처를 정한다. 누구나 밀어넣는 push 와 다르다.
    # local 소스 - 중계에 직접 붙은 캠. 네트워크를 안 타므로 tailnet 제약을 우회하고,
    # 폰·태블릿과 달리 게이트웨이가 살아 있으면 항상 있다(유일한 ALWAYS 실물 시점).
    global GLOBAL_LOCAL_CAMS
    GLOBAL_LOCAL_CAMS = []
    global GLOBAL_PULLERS
    GLOBAL_PULLERS = []

    # local 소스 - 필요할 때만 연다 (MCV-1C). LED 의 정답은 boot 플래그가 아니라 lazy open 이다.
    # --no-camera 는 "장치를 절대 열지 않는다" 로 의미를 좁힌다. pull 소스는 건드리지 않는다.
    if args.no_camera:
        app_log("[Sources] --no-camera: 로컬 웹캠(/dev/video0)을 열지 않습니다. pull 소스는 그대로 돕니다.")
        _local_specs = []
    else:
        try:
            import json as _json
            with open(DEFAULT_CONFIG_PATH, encoding='utf-8') as _fh:
                _cfg_doc = _json.load(_fh)
            _local_specs = load_local_sources(_cfg_doc)
        except FileNotFoundError:
            _local_specs = []
        except Exception as _exc:
            app_log(f"[Sources] local 설정을 읽지 못했습니다 - local 소스 없이 계속합니다: {_exc}")
            _local_specs = []
    for _spec in _local_specs:
        try:
            _cam = LocalCameraSource(
                device=_spec['device'], name=_spec['id'],
                width=_spec['width'], height=_spec['height'],
                target_fps=_spec['targetFps'], jpeg_quality=_spec['jpegQuality'],
                stale_after_s=_spec['staleAfterSec'],
                lazy=_spec['lazy'], idle_release_s=_spec['idleReleaseSec']).start()
        except Exception as _exc:
            app_log(f"[Sources] local 기동 실패 {_spec['id']}: {_exc}")
            continue
        GLOBAL_LOCAL_CAMS.append(_cam)
        # probe: 상태 조회는 수요가 아니다 - /api/safety 폴링이 카메라를 켜 두면 안 된다.
        GLOBAL_REGISTRY.register(Source(
            _spec['id'], _cam.get_latest_jpeg, LOCAL,
            TRUSTED if _spec['trust'] == 'trusted' else UNTRUSTED,
            label=_spec['label'], stale_after_s=_spec['staleAfterSec'],
            probe=_cam.probe, sidecar=_cam.sidecar,
            viewpoint=_spec.get('viewpoint', True)))
        app_log(f"[Sources] local 등록: {_spec['id']} <- device {_spec['device']} "
                f"({_spec['trust']}, lazy={_spec['lazy']}, idleRelease={_spec['idleReleaseSec']}s)")

    # pull 소스는 네트워크를 당길 뿐 장치를 열지 않는다 - 카메라 플래그와 무관하게 항상 돈다.
    # (09-10 실측: --no-camera 가 pull 까지 끊어 실물 시점이 0 이었다)
    try:
        _pull_specs = load_pull_sources()
    except Exception as _exc:
        app_log(f"[Sources] pull 설정을 읽지 못했습니다 - pull 소스 없이 계속합니다: {_exc}")
        _pull_specs = []
    for _spec in _pull_specs:
        try:
            _puller = MjpegPuller(_spec['urls'], name=_spec['id'],
                                  stale_after_s=_spec['staleAfterSec'],
                                  discover=_spec.get('discover')).start()
        except Exception as _exc:
            app_log(f"[Sources] pull 기동 실패 {_spec['id']}: {_exc}")
            continue
        GLOBAL_PULLERS.append(_puller)
        GLOBAL_REGISTRY.register(Source(
            _spec['id'], _puller.get_latest_jpeg, PULL,
            TRUSTED if _spec['trust'] == 'trusted' else UNTRUSTED,
            label=_spec['label'], stale_after_s=_spec['staleAfterSec'],
            sidecar=_puller.sidecar, viewpoint=_spec.get('viewpoint', True)))
        app_log(f"[Sources] pull 등록: {_spec['id']} <- {_spec['url']} ({_spec['trust']})")

    # 통합 검토 E2E-3: rclpy 기본 신호 처리는 SIGINT·SIGTERM 에 ROS 만 내리고 HTTP(serve_forever)는 그대로 둔다 — 게이트웨이가
    # 얼어붙은 상태(RUNNING·is_stale=False)를 systemd 정지 한도(TimeoutStopSec)까지 계속 내보냈고, 정지·비상정지 POST 는
    # 끊겼다(RCLError). 신호는 아래 _on_stop_signal 이 받아 HTTP 부터 닫는다 — ROS 는 그 뒤에 정리한다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    executor = rclpy.executors.MultiThreadedExecutor()

    if not args.no_camera:
        relay_node = TabletCameraRelayNode(ingest_manager=GLOBAL_INGEST)
        executor.add_node(relay_node)
    else:
        app_log("[ROS2] TabletCameraRelayNode is DISABLED (--no-camera)")

    sub_node = RobotDataSubscriberNode(
        control_renderer=GLOBAL_CONTROL,
        robot_camera_manager=GLOBAL_ROBOT_CAMERAS,
        gazebo_camera_manager=GLOBAL_GAZEBO_CAM
    )
    GLOBAL_ROBOT_SUB_NODE = sub_node
    executor.add_node(sub_node)

    global GLOBAL_FLEET_COORDINATOR, FLEET_IMPORT_ERROR
    if RelayFleetCoordinator:
        try:
            fleet_coord = RelayFleetCoordinator()
            executor.add_node(fleet_coord)
            GLOBAL_FLEET_COORDINATOR = fleet_coord
            sync_renderer_map()           # R-7: 저장된 좌표 프로파일의 화면 지도로
            app_log("[Fleet] RelayFleetCoordinator initialized and added to executor on Domain 8")
        except Exception as e:
            FLEET_IMPORT_ERROR = f'init {type(e).__name__}: {e}'
            app_log(f"[Fleet] Could not initialize RelayFleetCoordinator: {e}")
    else:
        app_log(f"[Fleet] ⚠️ RelayFleetCoordinator 없음 — 플릿 제어·좌표 전환이 503 이다: {FLEET_IMPORT_ERROR}")

    ros_thread = threading.Thread(target=run_ros_spin, args=(executor,), daemon=True)
    ros_thread.start()

    server_address = ('0.0.0.0', args.port)
    httpd = ThreadedHTTPServer(server_address, GatewayRequestHandler)

    app_log("[READY] Gateway Server listening on:")
    app_log(f" - Localhost : http://localhost:{args.port}")
    # ⭐ 아래는 **기록이지 현황이 아니다.** 이 프로세스가 어느 주소로 닿는지 재지 않는다 -
    #    주소는 사이트마다 바뀌고 실제로 두 번 어긋났다(폰 DHCP 재임대, 태블릿 사이트 이동).
    #    지우면 사람이 들어올 길이 없으므로 남기되 **언제 기준인지**를 같이 찍는다.
    #    최신은 docs/NETWORK_AND_PORTS.md, 찾는 법은 `tailscale ping <tailnet-ip>`.
    app_log(" - 아래는 2026-09-10 기준 기록 (지금 닿는다는 뜻이 아니다):")
    app_log(f"     LAN Cable : http://198.51.100.3:{args.port}")
    app_log(f"     Wi-Fi     : http://203.0.113.150:{args.port}")
    app_log(f"     Tailscale : http://100.64.0.81:{args.port}")

    stop_signal = []

    def _on_stop_signal(signum, _frame):
        # serve_forever 를 도는 이 (주) 스레드에서 httpd.shutdown() 을 부르면 서로 기다려 멈춘다 — 다른 스레드에서 부른다
        if not stop_signal:
            stop_signal.append(signum)
            app_log(f"[Shutdown] {signal.Signals(signum).name} — HTTP 를 닫고 ROS 를 정리한다")
            threading.Thread(target=httpd.shutdown, name='httpd-shutdown', daemon=True).start()

    signal.signal(signal.SIGTERM, _on_stop_signal)
    signal.signal(signal.SIGINT, _on_stop_signal)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        app_log("Shutting down Gateway Server...")
    finally:
        app_log("Cleaning up resources...")
        try:
            httpd.server_close()                  # 새 연결을 받지 않는다 — 얼어붙은 상태를 내보내지 않는다
        except Exception:
            pass
        for _p in (globals().get('GLOBAL_PULLERS', None) or []) + \
                  (globals().get('GLOBAL_LOCAL_CAMS', None) or []):
            try:
                _p.stop()
            except Exception:
                pass
        # ROS 는 HTTP 뒤에 내린다. spin 스레드가 끝나기 전에 인터프리터가 내려가면 C++ 쪽이 abort 한다(격리 실측
        # 'terminate called without an active exception') — executor 를 멈추고 그 스레드를 기다린 뒤 컨텍스트를 닫는다.
        try:
            executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        ros_thread.join(timeout=3.0)
        rclpy.try_shutdown()
        if ros_thread.is_alive():
            app_log("[Shutdown] ROS spin 스레드가 3 s 안에 안 끝났다 — 기다리지 않고 끝낸다")
            sys.stdout.flush()
            os._exit(128 + stop_signal[0] if stop_signal else 1)
    if stop_signal:
        # 신호로 끝났음을 종료 코드로 남긴다(128+신호: SIGTERM 143 · SIGINT 130) — 런처 cleanup 이 이 값을 보존하고
        # 유닛의 SuccessExitStatus(0 130 143)가 정상 정지로 친다(T10)
        sys.exit(128 + stop_signal[0])


if __name__ == '__main__':
    main()
