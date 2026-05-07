from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for InertialSense device'
    )

    str2str_relay = ExecuteProcess(
        cmd=[
            'str2str',
            '-in',  'ntrip://clemson:geodnet2026@rtk.geodnet.com:2101/AUTO',
            '-out', 'tcpsvr://:2102',
            '-p',   '34.852', '-82.394', '280.0',
            '-n',   '10000',
        ],
        output='screen',
        name='str2str_ntrip_relay',
    )

    # Delay the InertialSense node by 3s to give str2str time to connect
    inertial_sense_node = TimerAction(
        period=10.0,
        actions=[Node(
        package='inertial_sense_ros2_v2',
        executable='inertial_sense_node_v2',
        name='inertial_sense_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 921600,
            'frame_id': 'body',
            'navigation_dt_ms': 10,

            # Data streams
            'stream_INS': True,
            'stream_GPS': True,
            'stream_GPS_info': True,
            'stream_IMU': False,
            'stream_GPS_raw': False,
            'stream_baro': False,
            'stream_mag': False,
            'stream_preint_IMU': False,
            'stream_INL2': True,

            # RTK rover via local str2str relay (keeps VRS alive)
            'RTK_rover': True,
            'RTK_base': False,
            'dual_GNSS': False,
            'RTK_correction_type': 'RTCM3',
            'RTK_server_IP': '127.0.0.1',
            'RTK_server_port': 2102,
            'RTK_mountpoint': '',
            'RTK_username': '',
            'RTK_password': '',

            # Sensor config
            'inclination': 1.14878541071,
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
    )])

    return LaunchDescription([
        port_arg,
        str2str_relay,
        inertial_sense_node,
    ])
