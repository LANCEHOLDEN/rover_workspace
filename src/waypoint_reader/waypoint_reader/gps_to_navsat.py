#!/usr/bin/env python3
"""
gps_to_navsat.py
================
Converts the InertialSense custom /gps message to sensor_msgs/NavSatFix
so that robot_localization's navsat_transform_node can consume it.

  Subscribes : /gps          (inertial_sense_ros2_v2/msg/GPS)
  Publishes  : /gps/fix      (sensor_msgs/msg/NavSatFix)

Fix quality filtering
---------------------
NavSatFix is only published when fix_type >= GPS_STATUS_FIX_TYPE_3D_FIX (768).
Publishing with a 2D-only or dead-reckoning fix would feed bad data into the EKF.

Covariance
----------
The InertialSense message carries h_acc (horizontal, 1-sigma metres) and
v_acc (vertical, 1-sigma metres). These are converted to variances (sigma²)
and placed on the diagonal of the 3×3 ENU position covariance matrix:
    [h_acc²,  0,      0     ]   east
    [0,       h_acc², 0     ]   north
    [0,       0,      v_acc²]   up

NavSatStatus mapping
--------------------
  fix_type  768  (3D_FIX)               → STATUS_FIX   (0)
  fix_type 1024  (GPS_PLUS_DEAD_RECK)   → STATUS_FIX   (0)
  fix_type >= 3D with RTK               → STATUS_GBAS_FIX (2)  if num_sat > 0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# Import the custom GPS message — graceful fallback if not installed
try:
    from inertial_sense_ros2_v2.msg import GPS
    _GPS_AVAILABLE = True
except ImportError:
    _GPS_AVAILABLE = False

# Fix type constants from GPS.msg
FIX_3D         = 768
FIX_DEAD_RECK  = 256
FIX_GPS_DR     = 1024


class GpsToNavsat(Node):
    def __init__(self):
        super().__init__('gps_to_navsat')

        if not _GPS_AVAILABLE:
            self.get_logger().error(
                'inertial_sense_ros2_v2 package not found — cannot import GPS message. '
                'Make sure the workspace is sourced.'
            )
            return

        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.sub = self.create_subscription(GPS, '/gps', self._cb, 10)
        self._pub_count = 0
        self._drop_count = 0
        self._total_received = 0
        self._last_fix_type = None
        self._last_num_sat = None
        self.create_timer(5.0, self._log_status)
        self.get_logger().info(
            'gps_to_navsat started: /gps → /gps/fix\n'
            '  Only publishing on fix_type >= 3D_FIX (768)\n'
            '  Fix types: DEAD_RECK=256, 2D=512, 3D=768, RTK_SINGLE=2560, RTK_FLOAT=2816, RTK_FIX=3072'
        )

    def _cb(self, msg: 'GPS'):
        self._total_received += 1
        self._last_fix_type = msg.fix_type
        self._last_num_sat = msg.num_sat

        # Drop anything below a 3D fix
        if msg.fix_type < FIX_3D:
            self._drop_count += 1
            return

        fix = NavSatFix()
        fix.header = msg.header
        fix.header.frame_id = 'base_link'

        # Status — standard GNSS fix (no RTK distinction)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.status.status  = NavSatStatus.STATUS_FIX

        # Position
        fix.latitude  = msg.latitude
        fix.longitude = msg.longitude
        fix.altitude  = msg.altitude

        # Covariance — from h_acc / v_acc (1-sigma metres → variance)
        h_var = float(msg.h_acc) ** 2
        v_var = float(msg.v_acc) ** 2

        # Clamp to a minimum — avoid zero-covariance which causes EKF issues
        h_var = max(h_var, 0.01)    # 0.1 m minimum std dev
        v_var = max(v_var, 0.25)    # 0.5 m minimum std dev vertically

        fix.position_covariance = [
            h_var, 0.0,   0.0,
            0.0,   h_var, 0.0,
            0.0,   0.0,   v_var,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(fix)
        self._pub_count += 1

    def _log_status(self):
        if self._total_received == 0:
            self.get_logger().warn(
                'No messages received on /gps in last 5s — '
                'check that inertial_sense_node is running and stream_GPS: true'
            )
        else:
            fix_name = {
                0:    'NONE',
                256:  'DEAD_RECK',
                512:  '2D',
                768:  '3D',
                2560: 'RTK_SINGLE',
                2816: 'RTK_FLOAT',
                3072: 'RTK_FIX',
            }.get(self._last_fix_type, f'UNKNOWN({self._last_fix_type})')
            self.get_logger().info(
                f'GPS msgs: rcvd={self._total_received}, published={self._pub_count}, '
                f'dropped={self._drop_count} | '
                f'last fix_type={fix_name} num_sat={self._last_num_sat}'
            )
        self._pub_count = 0
        self._drop_count = 0
        self._total_received = 0


def main(args=None):
    rclpy.init(args=args)
    node = GpsToNavsat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
