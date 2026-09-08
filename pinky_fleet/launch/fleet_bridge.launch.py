"""관제 평면 — domain_bridge 를 로봇 수만큼 띄운다.

XML 이 아니라 Python launch 를 쓰는 이유: 로봇 목록이 mission.yaml 에 있어서 개수가 가변이다.
XML 은 반복문이 없어 로봇을 추가할 때마다 launch 파일을 손대야 한다.

실행:  ros2 launch pinky_fleet fleet_bridge.launch.py
      ros2 launch pinky_fleet fleet_bridge.launch.py config:=/경로/mission.yaml
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    from pinky_fleet.make_bridge_yaml import generate
    from pinky_fleet.mission_config import load_mission_config

    config = LaunchConfiguration('config').perform(context)
    cfg = load_mission_config(config)

    # 항상 새로 생성한다 -> mission.yaml 과 브리지 설정이 어긋날 수 없다.
    out_dir = os.path.join(tempfile.gettempdir(), 'pinky_fleet')
    paths = generate(config, out_dir)

    actions = [
        LogInfo(msg=f'[fleet_bridge] mission.yaml = {cfg.source_path}'),
        LogInfo(msg=f'[fleet_bridge] 관제 도메인 = {cfg.control_domain_id}'),
    ]
    for robot, path in zip(cfg.robots, paths):
        actions.append(LogInfo(
            msg=f'[fleet_bridge] {robot.name}: domain {robot.domain_id} '
                f'-> {cfg.control_domain_id}   ({path})'))
        actions.append(Node(
            package='domain_bridge',
            executable='domain_bridge',
            name=f'domain_bridge_{robot.name}',
            arguments=[path],
            output='screen',
            # 브리지 프로세스의 ROS_DOMAIN_ID 는 무의미하다 — YAML 이 도메인을 정한다(21강 s17).
            # 반면 ROS_LOCALHOST_ONLY 는 치명적이다. 1이면 WiFi 너머 로봇을 못 본다.
            additional_env={'ROS_LOCALHOST_ONLY': '0'},
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='',
            description='mission.yaml 경로 (비우면 패키지 share 의 기본값)'),
        OpaqueFunction(function=_setup),
    ])
