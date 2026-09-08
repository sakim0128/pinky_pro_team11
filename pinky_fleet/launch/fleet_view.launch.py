"""관제 화면 — 관제 도메인에서 rviz2 를 띄운다.

ROS_DOMAIN_ID 를 사용자가 export 하지 않아도 되도록, mission.yaml 의 control_domain_id 를
읽어 rviz2 프로세스 환경변수로 직접 넣는다. 도메인을 바꾸려면 mission.yaml 한 곳만 고치면 된다.

실행:  ros2 launch pinky_fleet fleet_view.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    from pinky_fleet.mission_config import load_mission_config

    share = get_package_share_directory('pinky_fleet')
    cfg = load_mission_config(LaunchConfiguration('config').perform(context))
    rviz_cfg = LaunchConfiguration('rviz').perform(context) or \
        os.path.join(share, 'rviz', 'fleet_view.rviz')

    return [
        LogInfo(msg=f'[fleet_view] rviz2 를 도메인 {cfg.control_domain_id} 에서 실행합니다.'),
        LogInfo(msg='[fleet_view] 목적지 지정: 상단 "Publish Point" 버튼 -> 맵 클릭'),
        Node(
            package='rviz2', executable='rviz2', name='fleet_view',
            arguments=['-d', rviz_cfg], output='screen',
            additional_env={
                'ROS_DOMAIN_ID': str(cfg.control_domain_id),
                'ROS_LOCALHOST_ONLY': '0',
            },
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('rviz', default_value=''),
        OpaqueFunction(function=_setup),
    ])
