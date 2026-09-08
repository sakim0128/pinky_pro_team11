"""mission.yaml 로더.

이 모듈은 **ROS를 import하지 않는다.** 부모 프로세스(fleet_master)와 L0 dry-run이
rclpy 없이도 설정을 읽을 수 있어야 하기 때문이다.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import yaml


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_deg: float

    @staticmethod
    def from_dict(d: dict) -> 'Pose2D':
        return Pose2D(float(d['x']), float(d['y']), float(d.get('yaw_deg', 0.0)))

    def as_dict(self) -> dict:
        return {'x': self.x, 'y': self.y, 'yaw_deg': self.yaw_deg}

    def __str__(self) -> str:
        return f'(x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw_deg:.1f}deg)'


@dataclass(frozen=True)
class RobotSpec:
    name: str
    domain_id: int
    home: Pose2D

    def as_dict(self) -> dict:
        return {'name': self.name, 'domain_id': self.domain_id, 'home': self.home.as_dict()}


@dataclass(frozen=True)
class GoalInput:
    mode: str = 'two_click'
    goal_yaw_deg: float = 0.0


@dataclass(frozen=True)
class MissionParams:
    order: list = field(default_factory=list)
    goal_timeout_sec: float = 90.0
    home_timeout_sec: float = 90.0
    nav2_activate_timeout_sec: float = 60.0
    settle_sec: float = 2.0
    feedback_period_sec: float = 1.0


@dataclass(frozen=True)
class LocalizationParams:
    wait_for_convergence: bool = True
    max_xy_std: float = 0.25
    max_yaw_std: float = 0.35
    convergence_timeout_sec: float = 30.0

    def as_dict(self) -> dict:
        return {
            'wait_for_convergence': self.wait_for_convergence,
            'max_xy_std': self.max_xy_std,
            'max_yaw_std': self.max_yaw_std,
            'convergence_timeout_sec': self.convergence_timeout_sec,
        }


@dataclass(frozen=True)
class MissionConfig:
    control_domain_id: int
    map_frame: str
    robots: list
    goal_input: GoalInput
    mission: MissionParams
    localization: LocalizationParams
    map_from: str
    source_path: str = ''

    def robot(self, name: str) -> RobotSpec:
        for r in self.robots:
            if r.name == name:
                return r
        raise KeyError(f"mission.yaml 에 '{name}' 로봇이 없습니다. "
                       f'있는 이름: {[r.name for r in self.robots]}')

    def ordered_robots(self) -> list:
        return [self.robot(n) for n in self.mission.order]


def default_config_path() -> str:
    """설치된 share 디렉터리 → 소스 트리 순으로 mission.yaml 을 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(
            get_package_share_directory('pinky_fleet'), 'config', 'mission.yaml')
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass

    # 소스 트리에서 바로 실행하는 경우 (빌드 전, L0/L1 검증)
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(here, '..', 'config', 'mission.yaml'))
    if os.path.exists(candidate):
        return candidate
    raise FileNotFoundError('mission.yaml 을 찾을 수 없습니다. --config 로 경로를 지정하세요.')


def load_mission_config(path: str = '') -> MissionConfig:
    path = path or default_config_path()
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)

    robots = [
        RobotSpec(str(r['name']), int(r['domain_id']), Pose2D.from_dict(r['home']))
        for r in raw['robots']
    ]

    gi_raw = raw.get('goal_input', {})
    goal_input = GoalInput(
        mode=str(gi_raw.get('mode', 'two_click')),
        goal_yaw_deg=float(gi_raw.get('goal_yaw_deg', 0.0)),
    )
    if goal_input.mode not in ('two_click', 'one_click'):
        raise ValueError(f"goal_input.mode 는 two_click 또는 one_click 이어야 합니다: "
                         f'{goal_input.mode}')

    m_raw = raw.get('mission', {})
    mission = MissionParams(
        order=[str(n) for n in m_raw.get('order', [r.name for r in robots])],
        goal_timeout_sec=float(m_raw.get('goal_timeout_sec', 90.0)),
        home_timeout_sec=float(m_raw.get('home_timeout_sec', 90.0)),
        nav2_activate_timeout_sec=float(m_raw.get('nav2_activate_timeout_sec', 60.0)),
        settle_sec=float(m_raw.get('settle_sec', 2.0)),
        feedback_period_sec=float(m_raw.get('feedback_period_sec', 1.0)),
    )

    l_raw = raw.get('localization', {})
    localization = LocalizationParams(
        wait_for_convergence=bool(l_raw.get('wait_for_convergence', True)),
        max_xy_std=float(l_raw.get('max_xy_std', 0.25)),
        max_yaw_std=float(l_raw.get('max_yaw_std', 0.35)),
        convergence_timeout_sec=float(l_raw.get('convergence_timeout_sec', 30.0)),
    )

    cfg = MissionConfig(
        control_domain_id=int(raw['control_domain_id']),
        map_frame=str(raw.get('map_frame', 'map')),
        robots=robots,
        goal_input=goal_input,
        mission=mission,
        localization=localization,
        map_from=str(raw.get('bridge', {}).get('map_from', robots[0].name)),
        source_path=path,
    )

    # 설정 오류는 로봇이 움직이기 전에 잡는다.
    names = [r.name for r in robots]
    if len(set(names)) != len(names):
        raise ValueError(f'robots.name 이 중복됩니다: {names}')
    domains = [r.domain_id for r in robots] + [cfg.control_domain_id]
    if len(set(domains)) != len(domains):
        raise ValueError(
            f'도메인 ID가 겹칩니다(로봇 + 관제): {domains}. '
            '겹치면 브리지가 자기 자신을 되받아 무한 루프가 됩니다.')
    for n in mission.order:
        cfg.robot(n)  # 존재하지 않으면 KeyError
    cfg.robot(cfg.map_from)
    return cfg
