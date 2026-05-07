from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('inertial_sense_ros2')

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/serial/by-id/usb-Inertial_Sense_uINS-if00',
        description='Serial port for InertialSense device'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='Baud rate for serial communication'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='body',
        description='Frame ID for published messages'
    )

    # Create node
    inertial_sense_node = Node(
        package='inertial_sense_ros2_v2',
        executable='inertial_sense_node_v2',
        name='inertial_sense_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'navigation_dt_ms': 50,
            'stream_INS': True,
            'stream_IMU': False,
            'stream_GPS': True,
            'stream_GPS_info': True,
            'stream_GPS_raw': False,
            'stream_baro': False,
            'stream_mag': False,
            'stream_preint_IMU': False,
            'enable_log': False,
            'RTK_rover': False,
            'RTK_base': False,
            'dual_GNSS': True,
            'RTK_server_IP': 'c5f28b12-5c7d-4f4e-b9b4-8d12517003ba',
            'RTK_server_port': 7777,
            'RTK_correction_type': 'UBLOX',
            'inclination': 1.14878541071,
            'declination': 0.20007290992,
            'dynamic_model': 8,
            'INS_rpy': [0.0, 0.0, 0.0],
            'INS_xyz': [0.0, 0.0, 0.0],
            'GPS_ant1_xyz': [-0.50, 0.0, 0.13],
            'GPS_ant2_xyz': [ 0.60, 0.0, 0.13],
            'GPS_ref_lla': [0.0, 0.0, 0.0],
            'ser1_baud_rate': 921600,
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        inertial_sense_node,
    ])
