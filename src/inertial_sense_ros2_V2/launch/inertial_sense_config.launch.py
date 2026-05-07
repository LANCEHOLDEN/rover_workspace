from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('inertial_sense_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Create node with config file
    inertial_sense_node = Node(
        package='inertial_sense_ros2_v2',
        executable='inertial_sense_node_v2',
        name='inertial_sense_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        inertial_sense_node,
    ])
