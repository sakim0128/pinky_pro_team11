#!/usr/bin/env python3
"""실기용 nav2_params_fleet.yaml 에서 시뮬용(로봇별) 파라미터 파일을 만든다.

실기에서는 로봇마다 ROS_DOMAIN_ID 가 갈려 있어서 두 로봇이 똑같이 ``base_footprint``,
``odom``, ``/scan`` 을 써도 충돌하지 않는다. 가제보는 프로세스가 하나라 도메인을 나눌 수
없고, 대신 네임스페이스로 가른다. 그러면 세 가지가 어긋난다.

1. ``pinky_description`` 이 ``frame_prefix`` 를 걸어 TF 프레임이 ``pinky1/odom``,
   ``pinky1/base_footprint`` 가 된다. Nav2 파라미터의 프레임 이름도 같이 바뀌어야 한다.
2. costmap 의 ``topic: /scan`` 은 **절대 경로**라 네임스페이스가 안 붙는다.
   두 로봇이 존재하지도 않는 ``/scan`` 을 구독하게 된다.
3. ``use_sim_time`` 이 True 여야 한다.

규칙은 단순하다. **프레임 값이 ``odom`` 또는 ``base_footprint`` 면 접두사를 붙이고,
``map`` 은 그대로 둔다.** map 프레임은 두 로봇이 공유해야 관제 PC 가 찍은 좌표가 그대로
통한다.

    python3 pinky_fleet_sim/scripts/make_sim_nav2_params.py

실기 파라미터를 고쳤으면 이 스크립트를 다시 돌린다. 안 돌리면 시뮬과 실기가 달라진다
(test_sim_params.py 가 그걸 잡는다).
"""

import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_PKG = os.path.dirname(HERE)
REPO = os.path.dirname(SIM_PKG)

SOURCE = os.path.join(REPO, 'pinky_fleet_agent', 'params', 'nav2_params_fleet.yaml')
OUT_DIR = os.path.join(SIM_PKG, 'params')

ROBOTS = ['pinky1', 'pinky2']

# 접두사를 붙일 프레임 값. map 은 공유하므로 일부러 빠져 있다.
LOCAL_FRAMES = {'odom', 'base_footprint', 'base_link'}

# 프레임 이름을 담는 키. 값이 LOCAL_FRAMES 에 있을 때만 바꾼다.
FRAME_KEYS = {
    'base_frame_id', 'odom_frame_id', 'global_frame_id',
    'robot_base_frame', 'global_frame', 'local_frame',
}

# 네임스페이스가 안 붙는 절대 토픽. 로봇별로 갈라 줘야 한다.
ABSOLUTE_TOPICS = {'/scan': '/{ns}/scan'}


def rewrite(node, namespace):
    """dict/list 를 재귀적으로 훑으며 프레임과 절대 토픽을 고친다."""
    if isinstance(node, dict):
        out = {}
        for key, value in node.items():
            if key == 'use_sim_time':
                out[key] = True
            elif key in FRAME_KEYS and isinstance(value, str) and value in LOCAL_FRAMES:
                out[key] = f'{namespace}/{value}'
            elif (key == 'topic' and isinstance(value, str)
                    and value in ABSOLUTE_TOPICS):
                out[key] = ABSOLUTE_TOPICS[value].format(ns=namespace)
            else:
                out[key] = rewrite(value, namespace)
        return out
    if isinstance(node, list):
        return [rewrite(item, namespace) for item in node]
    return node


def force_sim_time(data):
    """모든 노드의 ros__parameters 에 use_sim_time: True 를 넣는다.

    실기 파라미터에는 use_sim_time 이 아예 없다 (launch 인자로만 결정하도록 upstream 의
    하드코딩을 지웠다). 시뮬에서는 하나라도 빠지면 그 노드만 벽시계를 쓰면서 TF 가
    미래/과거로 어긋나 디버깅이 지독해진다. 그래서 여기서 못박는다.
    """
    count = 0
    for value in data.values():
        if isinstance(value, dict) and 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time'] = True
            count += 1
        elif isinstance(value, dict):
            # costmap 은 한 겹 더 들어가 있다 (local_costmap/local_costmap/ros__parameters)
            count += force_sim_time(value)
    return count


def build(namespace):
    with open(SOURCE, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    data = rewrite(data, namespace)
    force_sim_time(data)
    return data


def main():
    if not os.path.isfile(SOURCE):
        print(f'원본을 찾을 수 없다: {SOURCE}', file=sys.stderr)
        return 1
    os.makedirs(OUT_DIR, exist_ok=True)
    for namespace in ROBOTS:
        data = build(namespace)
        path = os.path.join(OUT_DIR, f'nav2_sim_{namespace}.yaml')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(
                f'# 자동 생성됨 - 손으로 고치지 말 것.\n'
                f'# 원본: pinky_fleet_agent/params/nav2_params_fleet.yaml\n'
                f'# 생성: pinky_fleet_sim/scripts/make_sim_nav2_params.py\n'
                f'# 네임스페이스 {namespace} 용 (프레임 접두사 + /scan 분리 + use_sim_time)\n')
            yaml.safe_dump(data, handle, default_flow_style=False,
                           allow_unicode=True, sort_keys=False)
        print(f'{path} 생성')
    return 0


if __name__ == '__main__':
    sys.exit(main())
