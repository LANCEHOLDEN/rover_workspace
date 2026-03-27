from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_reader'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lance Holden',
    maintainer_email='lance@rover.local',
    description='Read bagged INS/GPS data and convert to waypoints for rover navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'extract_waypoints = waypoint_reader.extract_waypoints:main',
            'waypoint_follower = waypoint_reader.waypoint_follower:main',
            'odom_to_path = waypoint_reader.odom_to_path:main',
            'ins_localizer = waypoint_reader.ins_localizer:main',
        ],
    },
)
