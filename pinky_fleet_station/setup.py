import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_fleet_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        # colcon 은 data_files 의 source 를 패키지 디렉터리 기준 상대경로로만 받는다.
        # 절대경로를 넣으면 빌드가 AssertionError 로 죽는다.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'maps'), glob('config/maps/*.pgm')),
    ],
    package_data={package_name: ['live_static/*']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team11',
    maintainer_email='sakim.working@gmail.com',
    description='핑키 프로 관제 PC 패키지',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coordinator_node = pinky_fleet_station.coordinator_node:main',
            'live_web_node = pinky_fleet_station.live_web_node:main',
            'overhead_tracker_node = pinky_fleet_station.overhead_tracker_node:main',
            'gui_node = pinky_fleet_station.gui_node:main',
            'fake_state_pub = pinky_fleet_station.fake_state_pub:main',
        ],
    },
)
