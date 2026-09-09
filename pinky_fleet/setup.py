from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/tools', glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team11',
    maintainer_email='sakim.working@gmail.com',
    description='Pinky Pro dual-robot sequential round-trip fleet control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_master = pinky_fleet.fleet_master:main',
            'preflight = pinky_fleet.preflight:main',
            'make_bridge_yaml = pinky_fleet.make_bridge_yaml:main',
        ],
    },
)
