import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_lane_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        # colcon 은 data_files 의 source 를 패키지 디렉터리 기준 상대경로로만 받는다.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team11',
    maintainer_email='sakim.working@gmail.com',
    description='핑키 프로 관제 PC 차선 추종 패키지',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'graph_editor = pinky_lane_station.graph_editor:main',
            'lane_coordinator_node = pinky_lane_station.lane_coordinator_node:main',
            'lane_pipeline_node = pinky_lane_station.lane_pipeline_node:main',
            'fake_lane_robot = pinky_lane_station.fake_lane_robot:main',
            'bench_detector = pinky_lane_station.bench_detector:main',
        ],
    },
)
