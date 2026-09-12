""".msg 의 상수 정의를 검사한다.

ROS 없이는 생성된 메시지 타입을 import 할 수 없으므로 .msg 파일을 텍스트로 읽는다.
상수 값이 겹치면 브리지 너머에서 전혀 다른 명령으로 해석되기 때문에, 값 충돌은
런타임에 조용히 터지는 종류의 버그다. 그래서 여기서 막는다.
"""

import os
import re

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MSG_DIR = os.path.join(REPO, 'pinky_fleet_msgs', 'msg')

CONSTANT = re.compile(r'^\s*uint8\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*(?:#.*)?$')


def constants(filename):
    """<상수 이름>: <값> 매핑. 필드 선언(`uint8   command`)은 걸리지 않는다."""
    path = os.path.join(MSG_DIR, filename)
    found = {}
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            match = CONSTANT.match(line)
            if match:
                found[match.group(1)] = int(match.group(2))
    return found


@pytest.fixture(scope='module')
def commands():
    return constants('FleetCommand.msg')


@pytest.fixture(scope='module')
def states():
    return constants('RobotState.msg')


# --- 값이 겹치면 안 된다 ------------------------------------------------

@pytest.mark.parametrize('filename', ['FleetCommand.msg', 'RobotState.msg'])
def test_values_are_unique(filename):
    found = constants(filename)
    assert found, f'{filename} 에서 상수를 하나도 못 찾았다 (파서 확인 필요)'
    duplicates = {
        value: sorted(k for k, v in found.items() if v == value)
        for value in found.values()
        if list(found.values()).count(value) > 1
    }
    assert not duplicates, f'{filename} 상수 값 충돌: {duplicates}'


# --- FleetCommand ------------------------------------------------------

def test_command_set_is_complete(commands):
    assert commands == {
        'CMD_GOTO': 0,
        'CMD_STOP': 1,
        'CMD_RESUME': 2,
        'CMD_CANCEL': 3,
        'CMD_SET_INITIAL_POSE': 4,
        'CMD_SET_SPEED': 5,
        'CMD_SET_MAP': 6,
        'CMD_HEARTBEAT': 7,
    }


def test_heartbeat_exists(commands):
    """관제 PC 생존 신호. 로봇의 데드맨 스위치가 이걸 먹고 산다."""
    assert commands['CMD_HEARTBEAT'] == 7


# --- RobotState --------------------------------------------------------

def test_nav_status_set_is_complete(states):
    assert states == {
        'NAV_IDLE': 0,
        'NAV_ACTIVE': 1,
        'NAV_SUCCEEDED': 2,
        'NAV_ABORTED': 3,
        'NAV_CANCELED': 4,
        'NAV_HOLD': 5,
        'NAV_LINK_LOST': 6,
    }


def test_link_lost_is_distinct_from_canceled(states):
    """사용자가 누른 정지와 관제 연결 끊김은 화면에서 구분되어야 한다."""
    assert states['NAV_LINK_LOST'] != states['NAV_CANCELED']


# --- GUI 표시가 모든 상태를 덮는지 ---------------------------------------

def test_gui_renders_every_nav_status(states):
    """NAV_ 상수를 추가하고 NAV_STATUS_TEXT 갱신을 잊으면 '?' 로 표시된다."""
    gui = os.path.join(REPO, 'pinky_fleet_station', 'pinky_fleet_station', 'gui_node.py')
    with open(gui, encoding='utf-8') as handle:
        source = handle.read()
    table = source.split('NAV_STATUS_TEXT = {', 1)[1].split('}', 1)[0]
    missing = [name for name in states if f'RobotState.{name}' not in table]
    assert not missing, f'gui_node.NAV_STATUS_TEXT 에 빠진 상태: {missing}'


# --- 가짜 로봇이 모르는 명령에 관대한지 -----------------------------------

def test_fake_state_pub_ignores_unknown_commands():
    """시뮬레이션에서도 coordinator 가 하트비트를 보낸다. 가짜 로봇이 죽으면 안 된다."""
    path = os.path.join(
        REPO, 'pinky_fleet_station', 'pinky_fleet_station', 'fake_state_pub.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()
    body = source.split('def _on_command', 1)[1].split('\n    def ', 1)[0]
    # else 절이 있으면 알 수 없는 명령에 무언가 반응한다는 뜻이다.
    assert '\n        else:' not in body, (
        'fake_state_pub._on_command 에 else 가 생겼다. '
        'CMD_HEARTBEAT 같은 모르는 명령을 조용히 무시하는지 확인할 것')
