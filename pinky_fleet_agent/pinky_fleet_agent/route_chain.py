# -*- coding: utf-8 -*-
"""Nav2 로봇용 레인 규약 — START 확인 · 허가 지점까지 목표 연쇄 · HOLD 와 ESTOP 의 구분 (D6).

ROS 에 의존하지 않는다. `hybrid_agent_node` 가 ROS 를 이어 붙이고, 이 모듈은 **무엇을 할지**만
동작 목록으로 돌려준다. 그래서 로봇·Nav2 없이 시험할 수 있다.

## 왜 필요한가 (2026-09-24 관제 팜 실측, rkd1rjs2/robot_mini_project_pinky `docs/REQ_20260924_RELAY_SESSION.md` D6)

코디네이터는 레인 로봇(팀11 `lane_agent`)에 맞춰 짜여 있다 — `LaneCommand CMD_START` 를 보내고
`LaneStatus` 로 ack 를 기다린다. Nav2 에이전트는 그 둘을 몰랐다.

- `CMD_START` 를 처리하지 않고 `LaneStatus` 를 내지 않는다 → 코디네이터가 5회 재전송 뒤 포기한다.
- `Route` 를 구독하지 않고 진행 인덱스가 0 에 고정돼 있었다(B-5) → clearance barrier 가 죽어 있었다.
- 대기(clear 0)마다 `/estop true` 를 냈다 → 게이트가 캐시를 버리고 로그가 넘치며,
  **정지와 비상정지의 구분이 사라졌다.**

## 규약

| 입력 | 동작 |
| :--- | :--- |
| Route(seq) | 새 경로. 진행도를 새로 잡고 시작 전(IDLE)으로 돌아간다. 달리던 goal 은 취소. **같은 메시지의 재전달**(번호·waypoint·goal·발행 시각이 전부 같다)은 무시한다 |
| 위치 (START 전) | 기억만 한다 — 진행도에 넣지 않는다 |
| START (seq 일치) | ack — 상태가 IDLE 을 벗어난다(코디네이터는 IDLE·ESTOP·LINK_LOST 가 아닌 상태를 ack 로 본다). **STOP 래치도 푼다**(ESTOP·링크 유실은 그대로). 시작 전→시작이면 진행도를 경로 처음부터 **지금 위치로** 새로 잡는다 |
| CLEARANCE (seq 일치) | `min(clear_until_idx, goal_idx)` 의 waypoint 까지 NavigateToPose |
| 허가 지점 도달 · clear 0 | **HOLD = goal 취소.** `/estop` 은 건드리지 않는다 — 게이트 MISSION 소스가 0.5 s 뒤 0 이 된다 |
| Nav2 succeeded | 그 waypoint 는 **도달 확정**이다. 거리로 다시 따지지 않는다(D6-1) |
| STOP | HOLD 래치(RESUME 으로 해제). `/estop` 은 건드리지 않는다 |
| ESTOP · 링크 유실 | goal 취소 + `/estop true` 래치. **`LaneCommand RESUME` 으로만** 해제 |
| RESUME (`LaneCommand`) | 모든 래치 해제, 래치가 있었으면 `/estop false`, 허가대로 다시 목표 |
| RESUME (`FleetCommand`) | **STOP 래치만** 푼다 — 레인 모드 코디네이터는 이것을 10 Hz 로 보내므로 ESTOP 까지 풀면 비상정지가 즉시 풀린다 |

route_seq 가 다른 START·CLEARANCE 는 무시한다(`Route.msg` 규약). ESTOP·STOP·RESUME 은 seq 와
무관하게 받는다 — 코디네이터의 `estop_fleet` 은 route_seq 를 채우지 않으며, 멈추라는 명령을
번호가 달라서 버리는 쪽이 더 위험하다.

## D6-1 — 같은 목표를 초당 7~15회 다시 보냈다 (관제 리그 2026-09-24, 813회 전송 / 811회 succeeded)

Nav2 의 `xy_goal_tolerance`(팜 0.15 m)가 체인의 `reach_tol`(0.10 m)보다 넓으면, 로봇이 목표에서
0.14 m 에 있을 때 Nav2 는 **즉시 성공**을 돌려준다. 예전엔 성공을 받고도 거리로 다시 따져
"아직 안 닿았다" → 같은 목표를 또 보냈다 → 또 즉시 성공 → 반복이었다. 세 겹으로 막는다:

1. `succeeded` 는 도달 확정이다(`_reached_idx`). 거리 재판정으로 되돌리지 않는다.
2. 도달 판정 반경은 `max(reach_tol, Nav2 xy_goal_tolerance + REACH_MARGIN_M)` 이다
   (`set_nav2_xy_tolerance` — 에이전트가 controller_server 파라미터를 읽어 넣는다).
3. 같은 waypoint 를 `min_resend_s` 안에 다시 내지 않는다. 다른 원인으로 고리가 생겨도 1 Hz 를 못 넘는다.

`goal_is_current()` 는 미뤄 둔 goto(재시도)를 **실행 직전에** 다시 묻는 자리다 — 그 사이 STOP·ESTOP·
새 경로가 왔으면 그 goto 는 버린다(관제 검수 P1: 정지 중 주행).

## 통합 검토 E2E-1 · RES-F1 — START 전 위치가 진행도를 영영 앞으로 밀었다 (월요일 §14-4)

① 프로파일 전환에서 새 Route 가 먼저 오고, 로봇은 ③ 초기 위치 전까지 **옛 좌표계** 위치를 보고한다. 예전엔 그 위치도
단조 추종기(`RouteFollower.update`)에 넣어, 새 경로 위 엉뚱한 곳(9~22)으로 진행도가 올라가 내려오지 않았다.
③ 뒤 출발점에 선 로봇이 START 를 받으면 "이미 멀리 왔다" 로 알고 허가 대기에 영영 서거나(`진행 N, 허가 M`, N>M —
코디네이터는 다음 엣지를 요청하지 않는다) 움직이지 않고 도착을 보고했다(코디네이터가 엣지를 놓는다).

- START 전 위치는 **기억만** 하고 진행도에 넣지 않는다.
- 시작 전→시작(START) 때 진행도를 경로 처음부터 **그 순간의 위치**로 새로 잡는다(`_project_from_start`).
  출발점의 로봇은 0 근처, 코디네이터 재시작 뒤 경로 중간에 선 로봇은 제자리를 찾는다.
  기억한 위치가 `pose_fresh_s` 보다 오래됐으면 쓰지 않는다 — 시작 뒤 첫 위치부터 평소처럼 따라간다.

## 통합 검토 F2 — 도메인 브리지 재기동이 같은 Route 를 다시 줬다

브리지는 Route 를 TRANSIENT_LOCAL 로 잇는다. 브리지만 재기동하면 **같은 메시지**가 다시 닿는데, 예전엔 그것도 새 경로로
받아 `started=False` 로 돌아갔다 — RUNNING 중에 로봇이 말없이 IDLE 로 섰다(코디네이터는 START 를 다시 내지 않는다).
같은 메시지면 무시한다. **발행 시각까지** 같아야 같은 메시지다: 코디네이터가 재시작하면 번호가 1 부터 다시 매겨져
같은 번호·같은 waypoint 의 Route 가 **새 발행 시각**으로 온다. 그것까지 무시하면 옛 코디네이터의 허가와 START 를 쥔 채
남아, 로봇 재개 한 번에 새 코디네이터가 예약하지 않은 구간으로 달린다. 발행 시각을 모르면(None) 새 경로로 본다(멈춤 쪽).

## 동작 목록 형식

    ('goto', idx, x, y, yaw)   # waypoint idx 로 NavigateToPose (yaw 는 경로 진행 방향)
    ('cancel',)                # 달리던 goal 취소
    ('estop', True|False)      # /estop 발행
"""

import time

from .route_follower import RouteFollower

# LaneCommand 상수 — pinky_lane_msgs/msg/LaneCommand.msg 와 같은 값(시험이 대사한다).
CMD_HEARTBEAT = 0
CMD_START = 1
CMD_STOP = 2
CMD_ESTOP = 3
CMD_RESUME = 4
CMD_SET_SPEED = 5
CMD_CLEARANCE = 6

# LaneStatus.drive_state 상수 — pinky_lane_msgs/msg/LaneStatus.msg 와 같은 값(시험이 대사한다).
DRIVE_IDLE = 0
DRIVE_CRUISE = 1
DRIVE_WAIT_CLEARANCE = 2
DRIVE_ARRIVED = 7
DRIVE_ESTOP = 8
DRIVE_LINK_LOST = 9

# 기본값. hybrid_agent_node 가 ROS 파라미터로 바꿀 수 있다.
REACH_TOL_M = 0.10          # 이 안이면 그 waypoint 에 닿은 것으로 본다 (Route 간격 0.10 m)
RESEND_MIN_M = 0.50         # 허가가 이만큼 늘어야 goal 을 다시 보낸다 (10 Hz 선점 폭주 방지)
NEAR_ACTIVE_M = 0.40        # 달리던 목표가 이만큼 가까우면 허가가 조금 늘어도 다시 보낸다
MAX_GOAL_FAILURES = 3       # 같은 목표가 이만큼 연속 실패하면 허가가 바뀔 때까지 다시 안 보낸다
MIN_RESEND_S = 1.0          # 같은 waypoint 를 이 간격 안에 다시 내지 않는다 (D6-1 세 번째 겹)
REACH_MARGIN_M = 0.02       # Nav2 가 "성공" 이라 할 반경보다 이만큼 넓게 본다 (D6-1 두 번째 겹)
POSE_FRESH_S = 1.0          # START 때 진행도를 새로 잡을 위치가 이보다 오래됐으면 쓰지 않는다 (E2E-1)


class RouteChain:
    """레인 규약 상태 기계. 입력마다 동작 목록을 돌려준다."""

    def __init__(self, reach_tol=REACH_TOL_M, resend_min=RESEND_MIN_M,
                 near_active=NEAR_ACTIVE_M, max_failures=MAX_GOAL_FAILURES,
                 min_resend_s=MIN_RESEND_S, clock=time.monotonic, pose_fresh_s=POSE_FRESH_S):
        self.reach_tol = float(reach_tol)
        self.resend_min = float(resend_min)
        self.near_active = float(near_active)
        self.max_failures = int(max_failures)
        self.min_resend_s = float(min_resend_s)
        self.pose_fresh_s = float(pose_fresh_s)
        self.nav2_xy_tol = None         # controller_server 의 xy_goal_tolerance (읽기 전엔 모른다)
        self._clock = clock
        self._pose = None               # (x, y, 받은 시각) — 마지막 위치. START 전엔 기억만 한다 (E2E-1)
        self._route_stamp = None        # 지금 경로 메시지의 발행 시각 — 같은 메시지의 재전달 판별 (F2)

        self.follower = None
        self.route_seq = 0
        self.goal_idx = 0
        self.clear_until_idx = 0
        self.started = False
        self.arrived = False
        self.active_target = None       # 지금 Nav2 가 달리는 waypoint idx (없으면 None)

        # 안전 래치
        self.stopped = False            # STOP → HOLD (RESUME 으로 해제)
        self.estop = False              # ESTOP → /estop true (LaneCommand RESUME 으로만 해제)
        self.link_lost = False          # 링크 유실 → /estop true (LaneCommand RESUME 으로만 해제)

        self._failures = {}             # target idx -> 연속 실패 수
        self._blocked_target = None     # 실패가 쌓여 포기한 목표
        self._reached_idx = -1          # Nav2 가 succeeded 로 확정한 가장 먼 waypoint (D6-1)
        self._last_sent = None          # (idx, 시각) — 같은 waypoint 재전송 간격 (D6-1)
        self.sends = 0                  # 이 경로에서 낸 goto 수 (진단·시험용)
        self.reason = ''

    # ------------------------------------------------------------ 조회

    @property
    def progress_idx(self):
        return self.follower.progress_idx if self.follower else 0

    @property
    def reached_idx(self):
        """거리 투영(progress)과 Nav2 성공 확정 중 먼 쪽."""
        return max(self.progress_idx, self._reached_idx)

    @property
    def effective_reach_tol(self):
        """도달 판정 반경 — Nav2 가 성공이라 할 반경보다 좁으면 D6-1 고리가 생긴다."""
        if self.nav2_xy_tol is None:
            return self.reach_tol
        return max(self.reach_tol, self.nav2_xy_tol + REACH_MARGIN_M)

    @property
    def latched(self):
        """움직이면 안 되는 래치가 걸려 있는가 (단일 목표 GOTO 도 이것을 따른다)."""
        return self.estop or self.link_lost

    def set_nav2_xy_tolerance(self, tol):
        """controller_server 의 goal checker `xy_goal_tolerance` 를 알린다 (모르면 None)."""
        self.nav2_xy_tol = None if tol is None else float(tol)

    def goal_is_current(self, idx, x, y):
        """이 goto 를 **지금** 보내도 되는가.

        미뤄 둔 동작(Nav2 부재·거부 뒤의 재시도)은 다음 틱에 재생된다. 그 사이 STOP·ESTOP·링크 유실·
        새 경로가 왔으면 상태기계는 이미 그 목표를 버렸다 — 재생하면 **정지 중에 주행**한다(관제 검수 P1).
        """
        f = self.follower
        if f is None or not self.started or self.arrived or self.stopped or self.latched:
            return False
        if self.active_target != idx or not (0 <= idx < len(f.points)):
            return False
        px, py = f.points[idx]
        return abs(px - float(x)) < 1e-9 and abs(py - float(y)) < 1e-9

    def drive_state(self):
        if self.estop:
            return DRIVE_ESTOP
        if self.link_lost:
            return DRIVE_LINK_LOST
        if self.follower is None or not self.started:
            return DRIVE_IDLE
        if self.arrived:
            return DRIVE_ARRIVED
        if self.active_target is not None and not self.stopped:
            return DRIVE_CRUISE
        return DRIVE_WAIT_CLEARANCE

    def status(self):
        """LaneStatus 에 싣는 값."""
        return {
            'drive_state': self.drive_state(),
            'state_reason': self.reason,
            'route_seq': self.route_seq,
            'route_idx': self.progress_idx,
            'clear_until_idx': self.clear_until_idx,
        }

    def same_route(self, route_seq, waypoints, goal_idx, stamp):
        """지금 경로와 **같은 메시지**인가 — 번호·waypoint·goal·발행 시각이 전부 같다 (통합 검토 F2).

        발행 시각을 모르면(None) 같다고 하지 않는다. 코디네이터 재시작은 같은 번호·같은 경로를 새 발행 시각으로 다시 준다 —
        그것은 새 경로다(옛 허가·START 를 버려야 한다).
        """
        f = self.follower
        if f is None or stamp is None or stamp != self._route_stamp or int(route_seq) != self.route_seq:
            return False
        pts = [(float(x), float(y)) for x, y in waypoints]
        gi = len(pts) - 1 if goal_idx is None else int(goal_idx)
        return pts == f.points and gi == self.goal_idx

    # ------------------------------------------------------------ 입력

    def on_route(self, route_seq, waypoints, goal_idx, stamp=None):
        """새 경로. `stamp` = 경로 메시지의 발행 시각(비교만 한다 — 모르면 None).

        통합 검토 F2: 브리지 재기동이 다시 준 **같은 메시지**는 무시한다. 예전엔 달리던 로봇이 `started=False` 로
        돌아가 RUNNING 중에 IDLE 로 섰다.
        """
        if self.same_route(route_seq, waypoints, goal_idx, stamp):
            return []
        acts = self._hold('새 경로')
        self.follower = RouteFollower(waypoints, goal_idx=goal_idx)
        self._route_stamp = stamp
        self.route_seq = int(route_seq)
        self.goal_idx = self.follower.goal_idx
        self.clear_until_idx = 0
        self.started = False
        self.arrived = False
        self._failures.clear()
        self._blocked_target = None
        self._reached_idx = -1
        self._last_sent = None
        self.sends = 0
        self.reason = '경로 수신 — START 대기'
        return acts

    def on_lane_command(self, command, route_seq, clear_until_idx):
        command = int(command)
        if command == CMD_ESTOP:
            return self._latch_estop()
        if command == CMD_STOP:
            return self.on_stop('LaneCommand STOP')
        if command == CMD_RESUME:
            return self._release(all_latches=True)
        if command not in (CMD_START, CMD_CLEARANCE):
            return []                                   # HEARTBEAT · SET_SPEED
        if self.follower is None or int(route_seq) != self.route_seq:
            return []                                   # 다른 경로의 명령
        if command == CMD_START:
            if not self.started:
                self.started = True
                self.reason = 'START 확인'
                self._project_from_start()
            # L4 (관제 검수 REVIEW_20260925 §3.1): 운영자의 START 는 실패로 포기한 목표도 다시 시도하게 한다.
            self._failures.clear()
            self._blocked_target = None
            if self.stopped:
                # STOP → START 은 운영자가 다시 가라는 뜻이다. 예전엔 START 가 STOP 래치를 안 풀어
                # 코디네이터(start_fleet 은 RESUME 을 안 보낸다)로는 영영 못 움직였다(관제 검수 P3).
                # ESTOP·링크 유실 래치는 START 로 풀지 않는다 — LaneCommand RESUME 으로만.
                self.stopped = False
                self._last_sent = None                  # 운영자의 출발 — 재전송 간격으로 늦추지 않는다
                self.reason = 'START — STOP 해제'
            return self._evaluate()
        # CMD_CLEARANCE
        new_clear = max(0, int(clear_until_idx))
        if new_clear != self.clear_until_idx:
            self._blocked_target = None                 # 허가가 바뀌면 포기한 목표를 다시 시도
        self.clear_until_idx = new_clear
        return self._evaluate()

    def on_stop(self, why='STOP'):
        """HOLD 래치. `/estop` 은 건드리지 않는다."""
        self.stopped = True
        return self._hold(why)

    def on_fleet_resume(self):
        """FleetCommand RESUME — STOP 래치만 푼다 (ESTOP·링크 유실은 그대로)."""
        if not self.stopped:
            return []
        return self._release(all_latches=False)

    def on_link_lost(self, moving):
        """링크 유실. 움직이는 중(또는 주행 임무 중)일 때만 래치한다."""
        if self.link_lost:
            return []
        if not (moving or (self.started and not self.arrived)):
            return []
        self.link_lost = True
        return self._hold('관제 링크 유실') + [('estop', True)]

    def on_pose(self, x, y):
        self._pose = (float(x), float(y), self._clock())
        if self.follower is None:
            return []
        if not self.started:
            # 통합 검토 E2E-1·RES-F1: START 전 위치는 진행도에 넣지 않는다. ① 뒤 ③ 전의 옛 좌표계 위치가 단조 추종기를
            #     새 경로 멀리까지 밀어 두면, 출발점에 선 로봇이 START 뒤 영영 서거나 움직이지 않고 도착을 보고했다.
            #     진행도는 START 때 그 순간의 위치로 잡는다(_project_from_start).
            return []
        self.follower.update(x, y)
        return self._evaluate()

    def on_goal_result(self, target_idx, outcome):
        """outcome: 'succeeded' | 'aborted' | 'canceled' | 'unavailable'.

        'unavailable' = Nav2 액션 서버가 아직 없다. Nav2 가 목표를 실패한 것이 아니므로 실패로
        세지 않는다 — 세면 Nav2 가 늦게 뜬 것만으로 3번 만에 포기하고, 코디네이터는 같은 허가를
        계속 보내니 영영 다시 시도하지 않는다. 재시도 간격은 `min_resend_s` 가 정한다.
        """
        if target_idx is None or target_idx != self.active_target:
            return []                                   # 이미 버린 목표의 결과
        self.active_target = None
        if outcome == 'succeeded':
            self._failures.pop(target_idx, None)
            # ⭐ D6-1: Nav2 가 성공이라면 그 waypoint 는 닿은 것이다. 거리로 다시 따지면
            #    (Nav2 허용이 더 넓을 때) "아직" 이 되고 같은 목표를 또 낸다 — 813회 고리.
            self._reached_idx = max(self._reached_idx, target_idx)
            if target_idx >= self.goal_idx:
                self.arrived = True
                self.reason = '목표 도착'
                return []
            self.reason = '허가 지점 도착 — clearance 대기'
            return self._evaluate()
        if outcome == 'unavailable':
            acts = self._evaluate()
            if not acts:
                # 재전송 간격이 사유를 '같은 목표 재전송 간격 대기' 로 덮어쓴다 — 진짜 이유를 남긴다(관제 검수 §3.4)
                self.reason = 'Nav2 액션 서버 없음·거부 — 재시도 대기'
            return acts
        if outcome == 'aborted':
            n = self._failures.get(target_idx, 0) + 1
            self._failures[target_idx] = n
            if n >= self.max_failures:
                self._blocked_target = target_idx
                self.reason = 'Nav2 가 목표 %d 를 %d번 연속 실패 — 허가가 바뀔 때까지 대기' % (target_idx, n)
                return []
            return self._evaluate()
        return []                                       # canceled — 우리가 취소한 것

    def clear_failures(self):
        """Nav2 가 새로 active 가 됐다 — 켜지는 중에 쌓인 실패·포기를 지우고 다시 평가한다(L4)."""
        self._failures.clear()
        self._blocked_target = None
        return self._evaluate()

    def abandon(self):
        """단일 목표(FleetCommand GOTO·CANCEL)가 레인 임무를 대신한다."""
        acts = self._hold('레인 임무 중단')
        self.started = False
        return acts

    # ------------------------------------------------------------ 내부

    def _project_from_start(self):
        """통합 검토 E2E-1: 시작 전→시작. 진행도를 경로 처음부터 지금 위치로 새로 잡는다.

        옛 진행도(단일 목표로 딴 데 다녀온 뒤 다시 START 한 경우)와 옛 Nav2 도달 확정을 버리고 위치로 다시 잡는다.
        처음(0)에서 같은 위치로 `update` 를 진행도가 더 늘지 않을 때까지 되풀이한다 — 탐색 창(±search_window)이
        한 번에 그만큼씩 따라가므로 출발점의 로봇은 0 근처에 서고, 경로 중간에 선 로봇도 첫 허가 전에 제자리를 찾는다
        (한 번만 투영하면 창 끝에 걸려 뒤쪽 waypoint 로 되돌아가는 goto 가 나갈 수 있다).
        `RouteFollower` 는 팀11 원본 바이트 그대로라(test_route_chain) 여기서 진행도를 되돌리고 되풀이한다.
        기억한 위치가 오래됐으면(`pose_fresh_s` — 에이전트는 위치 추정 중일 때만 위치를 넣는다) 아무것도 바꾸지 않는다:
        새 경로면 진행도는 이미 0 이고(START 전 위치는 넣지 않았다), 시작 뒤 첫 위치부터 평소처럼 따라간다.
        """
        pose = self._pose
        if pose is None or self._clock() - pose[2] > self.pose_fresh_s:
            return
        f = self.follower
        self._reached_idx = -1
        f.progress_s, f.progress_idx = 0.0, 0
        for _ in range(len(f.points)):
            before = f.progress_s
            f.update(pose[0], pose[1])
            if f.progress_s <= before:
                break

    def _latch_estop(self):
        if self.estop:
            return []
        self.estop = True
        return self._hold('E-STOP') + [('estop', True)]

    def _release(self, all_latches):
        acts = []
        if all_latches:
            if self.estop or self.link_lost:
                acts.append(('estop', False))
            self.estop = False
            self.link_lost = False
        self.stopped = False
        # 운영자가 다시 가라고 했다 — 실패로 포기했던 목표도 다시 시도한다. 재전송 간격으로 늦추지도 않는다
        # (간격은 결과→재전송 고리를 막는 것이지 사람의 재개를 막는 것이 아니다).
        self._failures.clear()
        self._blocked_target = None
        self._last_sent = None
        self.reason = 'RESUME'
        return acts + self._evaluate()

    def _hold(self, why):
        self.reason = why
        if self.active_target is None:
            return []
        self.active_target = None
        return [('cancel',)]

    def _goto(self, idx):
        now = self._clock()
        if (self.active_target is None and self._last_sent is not None and self._last_sent[0] == idx
                and now - self._last_sent[1] < self.min_resend_s):
            # D6-1 세 번째 겹: 같은 waypoint 를 방금 냈다. 다음 평가(on_pose 10 Hz)가 간격 뒤에 다시 낸다.
            # ⚠️ 달리는 목표가 있을 때(허가 축소로 더 가까운 목표로 바꾸는 선점)는 막지 않는다 —
            #    막으면 허가 밖의 옛 목표로 계속 달린다.
            self.reason = '같은 목표 %d 재전송 간격 대기' % idx
            return []
        f = self.follower
        x, y = f.points[idx]
        yaw = f.heading_at(f.cum[idx])
        self.active_target = idx
        self._last_sent = (idx, now)
        self.sends += 1
        return [('goto', idx, x, y, yaw)]

    def _evaluate(self):
        f = self.follower
        if f is None or not self.started or self.arrived:
            return []
        if self.estop or self.link_lost or self.stopped:
            return self._hold(self.reason or 'HOLD')

        limit = min(self.clear_until_idx, self.goal_idx)
        if limit <= self.reached_idx or f.distance_to_idx(limit) <= self.effective_reach_tol:
            # 허가 지점에 닿았거나 허가가 없다 → HOLD (goal 없음)
            if limit >= self.goal_idx and self.active_target is None:
                self.arrived = True
                self.reason = '목표 도착'
                return []
            if self.active_target is not None and self.active_target > limit:
                return self._hold('허가 축소 — %d 에서 대기' % limit)
            if self.active_target is None:
                self.reason = 'clearance 대기 (진행 %d, 허가 %d)' % (f.progress_idx, self.clear_until_idx)
            return []

        if limit == self._blocked_target:
            return []
        cur = self.active_target
        if cur is None or limit < cur:
            self.reason = '진행 → %d' % limit
            return self._goto(limit)
        if limit > cur:
            advance = f.cum[limit] - f.cum[cur]
            if (advance >= self.resend_min or limit >= self.goal_idx
                    or f.distance_to_idx(cur) <= self.near_active):
                self.reason = '허가 확장 → %d' % limit
                return self._goto(limit)
        return []
