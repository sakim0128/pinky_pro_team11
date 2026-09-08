"""mission.yaml -> domain_bridge 설정 YAML 생성기.

왜 손으로 안 쓰고 생성하는가
---------------------------
관제 도메인(to_domain)이 로봇 수만큼 반복해서 등장한다. 손으로 관리하면 도메인을 바꿀 때
한 군데를 빠뜨리기 쉽다. mission.yaml 한 곳만 고치고 이 스크립트를 다시 돌리면 끝나도록
단일 진실 공급원을 유지한다.

왜 로봇마다 파일(=프로세스)을 나누는가
--------------------------------------
두 로봇 모두 토픽 이름이 그냥 `/amcl_pose` 다. domain_bridge 의 YAML 은 `topics` 아래
**소스 토픽 이름을 키로** 쓰므로, 한 파일에 두 로봇의 amcl_pose 를 넣으려면 같은 키를
두 번 써야 한다. 상류 예제(examples/example_bridge_config.yaml)가 실제로 중복 키를 쓰지만
그건 YAML 표준이 보장하지 않는 동작이고 PyYAML 로는 안전하게 만들 수도 없다.
그래서 **로봇당 YAML 1개 + domain_bridge 프로세스 1개**로 나눈다.
부수 효과로 한 로봇의 브리지가 죽어도 다른 로봇 관제는 살아 있다.

무엇을 중계하는가 (최소 원칙)
-----------------------------
  /map        : 관제 화면 배경. 두 로봇이 같은 맵 파일을 쓰므로 한 대에서만 가져온다.
                QoS reliable + transient_local — 늦게 붙은 RViz도 받아야 한다(21강 s26).
  /amcl_pose  : 로봇 위치. `remap` 으로 /<robot>/amcl_pose 가 되어 두 대가 안 겹친다.

/tf, /scan, /camera 는 일부러 중계하지 않는다.
  - /tf  : 두 로봇의 frame 이름(map/odom/base_link)이 같아 tf 트리가 충돌한다.
           remap 은 토픽 이름만 바꾸고 메시지 안의 frame_id 는 못 바꾼다.
  - /scan, /camera : 고빈도라 WiFi 대역폭을 먹는데 관제 목적에는 불필요하다.
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml

from pinky_fleet.mission_config import load_mission_config


HEADER = """# 이 파일은 자동 생성됩니다. 직접 고치지 마세요.
# 원본: {src}
# 생성: ros2 run pinky_fleet make_bridge_yaml   (빌드 전이면 python3 tools/make_bridge_yaml.py)
"""


def build_bridge_config(cfg, robot) -> dict:
    doc = {
        'name': f'pinky_fleet_bridge_{robot.name}',
        'from_domain': robot.domain_id,
        'to_domain': cfg.control_domain_id,
        'topics': {
            'amcl_pose': {
                'type': 'geometry_msgs/msg/PoseWithCovarianceStamped',
                'remap': f'{robot.name}/amcl_pose',
                'qos': {'reliability': 'reliable', 'durability': 'volatile'},
            },
        },
    }
    if robot.name == cfg.map_from:
        doc['topics']['map'] = {
            'type': 'nav_msgs/msg/OccupancyGrid',
            'qos': {'reliability': 'reliable', 'durability': 'transient_local'},
        }
    return doc


def output_path(out_dir: str, robot_name: str) -> str:
    return os.path.join(out_dir, f'fleet_bridge_{robot_name}.yaml')


def generate(config_path: str = '', out_dir: str = '') -> list:
    cfg = load_mission_config(config_path)
    out_dir = out_dir or os.path.dirname(cfg.source_path)
    os.makedirs(out_dir, exist_ok=True)

    written = []
    for robot in cfg.robots:
        doc = build_bridge_config(cfg, robot)
        path = output_path(out_dir, robot.name)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(HEADER.format(src=cfg.source_path))
            yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=False)
        written.append(path)
    return written


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='make_bridge_yaml',
        description='mission.yaml 로부터 domain_bridge 설정을 생성한다')
    ap.add_argument('--config', default='', help='mission.yaml 경로')
    ap.add_argument('--out-dir', default='', help='출력 폴더 (기본: mission.yaml 과 같은 폴더)')
    args = ap.parse_args(argv)

    for path in generate(args.config, args.out_dir):
        print(f'생성: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
