"""부모(fleet_master) ↔ 자식(domain_worker / console_node) 사이의 메시지 규약.

rclpy.init() 은 프로세스당 한 번만 가능하므로(19강 s24) 도메인마다 프로세스를 나눈다.
서로 다른 도메인의 프로세스끼리는 ROS로 대화할 수 없으므로, 프로세스 간 통신은
multiprocessing.Queue 로 한다. 여기 정의한 dict 가 그 큐에 흐르는 유일한 형식이다.

모든 메시지는 순수 파이썬 타입만 담는다(pickle 가능해야 하므로 ROS 메시지 객체 금지).
"""

from __future__ import annotations

from enum import Enum


class MissionState(str, Enum):
    """25강 상태기반 제어 응용. 값이 그대로 관제 화면 문자열이 된다."""

    INIT = 'INIT'
    WAIT_NAV2 = 'WAIT_NAV2'
    WAIT_GOAL = 'WAIT_GOAL'
    GOING_TO_GOAL = 'GOING_TO_GOAL'
    GOING_HOME = 'GOING_HOME'
    SETTLING = 'SETTLING'
    DONE = 'DONE'
    ABORT = 'ABORT'


# ---------------------------------------------------------------- 부모 -> 워커
CMD_PREPARE = 'prepare'    # waitUntilNav2Active + setInitialPose(home) + 수렴 대기
CMD_GOTO = 'goto'          # {'pose': {'x','y','yaw_deg'}, 'timeout_sec': float, 'tag': str}
CMD_CANCEL = 'cancel'      # 진행 중 목표 취소
CMD_SHUTDOWN = 'shutdown'  # 노드 정리 후 프로세스 종료

# ---------------------------------------------------------------- 워커 -> 부모
EVT_READY = 'ready'        # prepare 완료 (Nav2 active + 초기위치 수렴)
EVT_ACCEPTED = 'accepted'  # goal 수락됨
EVT_FEEDBACK = 'feedback'  # {'distance_remaining', 'nav_time_sec'}
EVT_ARRIVED = 'arrived'    # 목표 도달 (SUCCEEDED)
EVT_FAILED = 'failed'      # {'reason': str}  FAILED / 타임아웃 / 예외
EVT_CANCELED = 'canceled'
EVT_POSE = 'pose'          # {'x','y','yaw_deg'} 현재 amcl_pose
EVT_LOG = 'log'            # {'text': str}
EVT_BYE = 'bye'            # 프로세스 정상 종료

# ------------------------------------------------------- 부모 <-> 관제 콘솔(도메인 20)
CON_SET_STATE = 'set_state'    # 부모->콘솔 {'state': str, 'detail': str}
CON_SET_GOAL = 'set_goal'      # 부모->콘솔 {'pose': {...}} 목적지 마커 표시
CON_SET_HOMES = 'set_homes'    # 부모->콘솔 {'homes': {name: pose_dict}}
CON_GOAL_PICKED = 'goal_picked'  # 콘솔->부모 {'pose': {...}} RViz 클릭 결과
CON_SHUTDOWN = 'shutdown'


def msg(robot: str, event: str, **payload) -> dict:
    d = {'robot': robot, 'event': event}
    d.update(payload)
    return d
