from setuptools import find_packages, setup

package_name = 'move_5_feet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lanceholden',
    maintainer_email='lth@clemson.edu',
    description='Package to move RoverRobotics Zero3 a specific distance',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        # This allows you to run: ros2 run rover_movement move_5_feet
            'move_5_feet = move_5_feet.move_5_feet:main'
        ],
    },
)
