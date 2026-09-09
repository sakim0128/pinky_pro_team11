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
    # 로봇 쪽 Nav2 가 네임스페이스 아래에서 도는 경우에만 채운다(기본은 비어 있음).
    # preflight 가 노드 목록에서 찾아 알려준다.
    namespace: str = ''

    def as_dict(self) -> dict:
        return {'name': self.name, 'domain_id': self.domain_id,
                'home': self.home.as_dict(), 'namespace': self.namespace}


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
    """초기 위치 확인 파라미터.

    "수렴"을 기다리지 않는다 — amcl 은 초기 위치가 unknown 이면 공분산을
    [0.5^2, 0.5^2, (pi/12)^2] 로 시작하고 정지 중에는 줄지 않는다.
    확인할 것은 "setInitialPose 이후 새로 발행된 pose 가 home 근처인가" 하나다.
    """

    require_initial_pose: bool = True
    max_initial_offset: float = 0.5        # [m]
    initial_pose_timeout_sec: float = 15.0
    warn_xy_std: float = 0.6               # 넘으면 경고만
    warn_yaw_std: float = 0.45

    def as_dict(self) -> dict:
        return {
            'require_initial_pose': self.require_initial_pose,
            'max_initial_offset': self.max_initial_offset,
            'initial_pose_timeout_sec': self.initial_pose_timeout_sec,
            'warn_xy_std': self.warn_xy_std,
            'warn_yaw_std': self.warn_yaw_std,
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
        RobotSpec(str(r['name']), int(r['domain_id']), Pose2D.from_dict(r['home']),
                  str(r.get('namespace', '') or ''))
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
        require_initial_pose=bool(l_raw.get('require_initial_pose', True)),
        max_initial_offset=float(l_raw.get('max_initial_offset', 0.5)),
        initial_pose_timeout_sec=float(l_raw.get('initial_pose_timeout_sec', 15.0)),
        warn_xy_std=float(l_raw.get('warn_xy_std', 0.6)),
        warn_yaw_std=float(l_raw.get('warn_yaw_std', 0.45)),
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
