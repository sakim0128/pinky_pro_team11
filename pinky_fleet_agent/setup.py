import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_fleet_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team11',
    maintainer_email='sakim.working@gmail.com',
    description='핑키 프로 로봇측 관제 에이전트',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agent_node = pinky_fleet_agent.agent_node:main',
            'lane_agent_node = pinky_fleet_agent.lane_agent_node:main',
            'camera_node = pinky_fleet_agent.camera_node:main',
        ],
    },
)
