from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for InertialSense device'
    )

    inertial_sense_node = Node(
        package='inertial_sense_ros2_v2',
        executable='inertial_sense_node_v2',
        name='inertial_sense_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 921600,
            'frame_id': 'body',
            'navigation_dt_ms': 10,

            # Enable both INS (for orientation in IMU msg header) and raw IMU
            'stream_INS': True,
            'stream_IMU': True,
            'stream_GPS': False,
            'stream_GPS_info': False,
            'stream_GPS_raw': False,
            'stream_baro': False,
            'stream_mag': False,
            'stream_preint_IMU': False,

            # No RTK for raw IMU testing
            'RTK_rover': False,
            'RTK_base': False,
            'dual_GNSS': False,

            # Sensor config
            'declination': 0.20007290992,
            'dynamic_model': 8,
            'INS_rpy': [0.0, 0.0, 0.0],
            'INS_xyz': [0.0, 0.0, 0.0],
            'GPS_ant1_xyz': [0.2054, 0.15298, -0.08959],
            'GPS_ant2_xyz': [-0.11962, -0.13717, -0.08959],
            'GPS_ref_lla': [34.694374, -82.865496, 270.0],
            'ser1_baud_rate': 921600,
            'enable_log': False,
        }]
    )

    return LaunchDescription([
        port_arg,
        inertial_sense_node,
    ])
