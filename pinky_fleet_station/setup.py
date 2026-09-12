import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_fleet_station'

# config/ 는 저장소 루트에 두고 사용자가 편집한다. 빌드 시 패키지 share 로 복사해
# launch 파일의 $(find-pkg-share ...) 기본값이 바로 동작하게 한다.
_here = os.path.dirname(os.path.abspath(__file__))
_repo_config = os.path.join(os.path.dirname(_here), 'config')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join(_repo_config, '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team11',
    maintainer_email='sakim.working@gmail.com',
    description='핑키 프로 관제 PC 패키지',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coordinator_node = pinky_fleet_station.coordinator_node:main',
            'gui_node = pinky_fleet_station.gui_node:main',
            'fake_state_pub = pinky_fleet_station.fake_state_pub:main',
        ],
    },
)
