from setuptools import setup, find_packages

package_name = 'move_forward'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lance',
    maintainer_email='lance@example.com',
    description='Move rover forward using cmd_vel',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'move_forward = move_forward.move_forward:main',
        ],
    },
)

