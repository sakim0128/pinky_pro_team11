"""실행 중인 Nav2 가 정말 nav2_params_fleet.yaml 로 떠 있는지 확인하는 로직.

왜 필요한가. 실기에서 파라미터를 덤프해 보니 controller_server / planner_server /
bt_navigator 의 값이 **upstream nav2_params.yaml 기본값과 글자 하나까지 같았다.**
`bt_navigator.robot_base_frame` 이 base_link 였는데, 우리 파일은 첫 커밋부터 줄곧
base_footprint 다. 즉 Nav2 가 우리 파일이 아닌 다른 파일로 떠 있었다.

이런 일이 생기는 경로는 몇 가지다.

* `pinky_fleet_agent/launch/robot.launch.xml` 대신 `pinky_navigation` 의
  `bringup_launch.xml` 을 직접 띄우고, 에이전트만 따로 올린 경우.
  에이전트가 살아 있으니 관제 화면은 멀쩡해 보인다 — 그래서 아무도 모른다.
* `params_file:=` 로 다른 경로를 넘긴 경우.
* colcon build 를 안 해서 install 쪽 파일이 낡은 경우.

어느 쪽이든 증상은 "파라미터를 고쳤는데 로봇이 안 변한다" 하나로 같고, 실제로
값을 읽어 보기 전까지는 구분이 안 된다. 그래서 에이전트가 기동 직후 몇 개만
직접 읽어 파일과 대조한다.

이 모듈에는 rclpy 가 들어오지 않는다 (테스트가 ROS 없이 돈다). 서비스 호출은
agent_node 가 하고, 여기서는 "무엇을 물어볼지" 와 "답이 맞는지" 만 정한다.
"""

import os

# 확인할 파라미터. (서비스를 부를 노드 이름, YAML 경로, 파라미터 이름)
#
# 고르는 기준:
#   1. 우리 파일과 upstream 기본값이 **다른** 값이어야 한다. 같으면 구분이 안 된다.
#   2. 사람이 rqt 로 만질 일이 거의 없는 값이어야 한다. inflation_radius 처럼
#      튜닝 대상인 값을 넣으면 정상적인 튜닝 중에도 계속 경고가 뜬다.
#
# robot_base_frame 이 가장 확실한 지표다. 문자열이라 우연히 같아질 수 없고,
# rqt 로 바꿀 이유도 없다.
SENTINELS = (
    ('bt_navigator', ('bt_navigator', 'ros__parameters'), 'robot_base_frame'),
    ('controller_server', ('controller_server', 'ros__parameters'),
     'FollowPath.max_lookahead_dist'),
    ('controller_server', ('controller_server', 'ros__parameters'),
     'general_goal_checker.xy_goal_tolerance'),
    ('planner_server', ('planner_server', 'ros__parameters'), 'GridBased.tolerance'),
    ('global_costmap/global_costmap',
     ('global_costmap', 'global_costmap', 'ros__parameters'), 'footprint_padding'),
    ('local_costmap/local_costmap',
     ('local_costmap', 'local_costmap', 'ros__parameters'), 'width'),
)

# 실수(float) 비교 허용 오차. 파라미터는 우리가 쓴 값 그대로 돌아오므로 넉넉할 필요가 없다.
TOLERANCE = 1e-6


def dig(data, path):
    """중첩 dict 에서 경로를 따라 내려간다. 없으면 None."""
    node = data
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node


def expected_values(params, sentinels=SENTINELS):
    """파일에서 읽은 dict → {(노드, 파라미터): 기대값}.

    파라미터 이름의 점(``FollowPath.max_lookahead_dist``)은 YAML 에서는 중첩이다.
    """
    out = {}
    for node, path, name in sentinels:
        value = dig(params, tuple(path) + tuple(name.split('.')))
        if value is not None:
            out[(node, name)] = value
    return out


def same(expected, actual):
    # 파이썬에서 bool 은 int 라, 이 검사가 없으면 True 와 1 이 같은 값으로 통과한다.
    # 한쪽만 bool 이면 타입이 어긋난 것이므로 그 자체가 보고할 만한 차이다.
    if isinstance(expected, bool) != isinstance(actual, bool):
        return False
    if isinstance(expected, bool):
        return expected == actual
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        return abs(float(expected) - float(actual)) <= TOLERANCE
    return expected == actual


def compare(expected, actual):
    """{(노드, 이름): 값} 두 개를 비교해 어긋난 것만 문자열 목록으로 돌려준다.

    actual 에 없는 키(노드가 안 떠 있거나 응답이 없는 경우)는 mismatch 로 치지
    않는다. 그건 '값이 다르다' 가 아니라 '못 읽었다' 라서 따로 보고한다.
    """
    bad = []
    for key in sorted(expected):
        if key not in actual:
            continue
        if not same(expected[key], actual[key]):
            node, name = key
            bad.append(f'{node}.{name}: 파일={expected[key]} 실행중={actual[key]}')
    return bad


def missing(expected, actual):
    return sorted(f'{node}.{name}' for node, name in expected if (node, name) not in actual)


def default_params_path():
    """설치본 기준 nav2_params_fleet.yaml 경로.

    launch 가 넘겨주는 게 정상이지만, 에이전트만 따로 띄우는 경우를 위해 기본값을 둔다.
    소스 트리(``pinky_fleet_agent/pinky_fleet_agent/``)에서 돌 때도 찾을 수 있게
    상대 경로로 올라간다.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(os.path.dirname(here), 'params', 'nav2_params_fleet.yaml')
    return candidate if os.path.isfile(candidate) else ''


def report(bad, unread):
    """사람이 읽을 경고문. 없으면 None.

    로그 한 줄만 보고 무엇을 해야 할지 알 수 있게 원인 후보까지 적는다.
    """
    if not bad and not unread:
        return None
    lines = []
    if bad:
        lines.append('실행 중인 Nav2 파라미터가 nav2_params_fleet.yaml 과 다르다:')
        lines.extend(f'  - {item}' for item in bad)
        lines.append('원인 후보: (1) robot.launch.xml 이 아니라 pinky_navigation 의 '
                     'bringup_launch.xml 을 직접 띄웠다 (2) params_file 로 다른 파일을 '
                     '넘겼다 (3) colcon build 를 안 해 install 쪽 파일이 낡았다 '
                     '(4) rqt 로 바꾼 뒤 파일에 반영하지 않았다.')
    if unread:
        lines.append(f'읽지 못한 파라미터(노드 미기동 가능): {", ".join(unread)}')
    return '\n'.join(lines)
