#!/usr/bin/env python3
"""
gps_datum_odom.py
=================
Averages the first N GPS samples to establish a local datum, then publishes
every subsequent GPS fix as a local ENU position relative to that datum.

  Subscribes : /gps          (inertial_sense_ros2_v2/msg/GPS)
  Publishes  : /gps_odom     (nav_msgs/Odometry)  frame: gps_datum → base_link

Usage
-----
  ros2 run waypoint_reader gps_datum_odom
  ros2 run waypoint_reader gps_datum_odom --ros-args -p calib_samples:=50
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    from inertial_sense_ros2_v2.msg import GPS
    _GPS_AVAILABLE = True
except ImportError:
    _GPS_AVAILABLE = False

FIX_3D = 768   # minimum fix type to accept

# WGS-84 ellipsoid
_A  = 6378137.0
_E2 = 0.00669437999014


def _geodetic_to_ecef(lat_rad, lon_rad, alt):
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    N = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    x = (N + alt) * cos_lat * math.cos(lon_rad)
    y = (N + alt) * cos_lat * math.sin(lon_rad)
    z = (N * (1.0 - _E2) + alt) * sin_lat
    return x, y, z


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Convert (lat, lon, alt) degrees to ENU metres relative to datum."""
    lat_r  = math.radians(lat)
    lon_r  = math.radians(lon)
    lat0_r = math.radians(lat0)
    lon0_r = math.radians(lon0)

    x,  y,  z  = _geodetic_to_ecef(lat_r,  lon_r,  alt)
    x0, y0, z0 = _geodetic_to_ecef(lat0_r, lon0_r, alt0)

    dx, dy, dz = x - x0, y - y0, z - z0

    sin_lat0, cos_lat0 = math.sin(lat0_r), math.cos(lat0_r)
    sin_lon0, cos_lon0 = math.sin(lon0_r), math.cos(lon0_r)

    east  = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up    =  cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz

    return east, north, up


class GpsDatumOdom(Node):
    def __init__(self):
        super().__init__('gps_datum_odom')

        self.declare_parameter('calib_samples', 30)
        self.declare_parameter('min_fix_type',  FIX_3D)

        self._n_samples   = self.get_parameter('calib_samples').value
        self._min_fix     = self.get_parameter('min_fix_type').value

        self._buf   = []   # calibration buffer: list of (lat, lon, alt)
        self._datum = None  # (lat0, lon0, alt0) once calibrated

        self._pub = self.create_publisher(Odometry, '/gps_odom', 10)

        if not _GPS_AVAILABLE:
            self.get_logger().error(
                'inertial_sense_ros2_v2 not found — cannot import GPS message.'
            )
            return

        self.create_subscription(GPS, '/gps', self._cb, 10)
        self.create_timer(5.0, self._log_status)

        self.get_logger().info(
            f'gps_datum_odom started — collecting {self._n_samples} samples '
            f'(fix_type >= {self._min_fix}) to set datum'
        )

    def _cb(self, msg: 'GPS'):
        if msg.fix_type < self._min_fix:
            return

        if self._datum is None:
            self._buf.append((msg.latitude, msg.longitude, msg.altitude))

            if len(self._buf) < self._n_samples:
                return

            lat0 = sum(p[0] for p in self._buf) / len(self._buf)
            lon0 = sum(p[1] for p in self._buf) / len(self._buf)
            alt0 = sum(p[2] for p in self._buf) / len(self._buf)
            self._datum = (lat0, lon0, alt0)
            self.get_logger().info(
                f'Datum set from {len(self._buf)} samples:\n'
                f'  lat={lat0:.8f}  lon={lon0:.8f}  alt={alt0:.2f} m'
            )
            self._buf = []
            return

        east, north, up = geodetic_to_enu(
            msg.latitude, msg.longitude, msg.altitude,
            *self._datum
        )

        odom = Odometry()
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = 'gps_datum'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = up
        odom.pose.pose.orientation.w = 1.0   # no orientation from GPS

        h_var = max(float(msg.h_acc) ** 2, 0.01)
        v_var = max(float(msg.v_acc) ** 2, 0.25)
        odom.pose.covariance[0]  = h_var   # east
        odom.pose.covariance[7]  = h_var   # north
        odom.pose.covariance[14] = v_var   # up
        odom.pose.covariance[35] = 9999.0  # yaw unknown

        self._pub.publish(odom)

    def _log_status(self):
        if self._datum is None:
            self.get_logger().info(
                f'Collecting datum samples: {len(self._buf)}/{self._n_samples}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GpsDatumOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
