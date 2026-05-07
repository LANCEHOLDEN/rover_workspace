#!/usr/bin/env python3
"""
simple_gps_odom.py
==================
Averages the first N GPS fixes to set a local datum, then converts every
subsequent fix to local ENU (East-North-Up) metres using a flat-Earth
approximation (accurate to ~0.1 m per km — fine for a rover).

Subscribes : /gps          (inertial_sense_ros2_v2/msg/GPS)
Publishes  : /gps_odom     (nav_msgs/Odometry)  frame: odom → base_link

Tune via ROS parameters:
  calib_samples  — how many GPS fixes to average for the datum (default 20)
  min_fix_type   — minimum GPS fix quality to accept (default 768 = 3D fix)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    from inertial_sense_ros2_v2.msg import GPS
    _GPS_OK = True
except ImportError:
    _GPS_OK = False

FIX_3D   = 768
R_EARTH  = 6378137.0   # WGS-84 equatorial radius in metres


def latlon_to_enu(lat, lon, lat0, lon0):
    """
    Flat-Earth approximation: convert (lat, lon) degrees to (east, north) metres
    relative to datum (lat0, lon0).  Accurate to ~0.1 m per km.
    """
    east  = R_EARTH * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    north = R_EARTH * math.radians(lat - lat0)
    return east, north


class SimpleGpsOdom(Node):
    def __init__(self):
        super().__init__('simple_gps_odom')

        self.declare_parameter('calib_samples', 20)
        self.declare_parameter('min_fix_type',  FIX_3D)

        self._n      = self.get_parameter('calib_samples').value
        self._minfix = self.get_parameter('min_fix_type').value
        self._buf    = []       # collects (lat, lon, alt) during calibration
        self._datum  = None     # (lat0, lon0, alt0) once set

        self._pub = self.create_publisher(Odometry, '/gps_odom', 10)

        if not _GPS_OK:
            self.get_logger().error(
                'inertial_sense_ros2_v2 not available — cannot subscribe to /gps'
            )
            return

        self.create_subscription(GPS, '/gps', self._cb, 10)
        self.create_timer(5.0, self._status)
        self.get_logger().info(
            f'simple_gps_odom: collecting {self._n} samples for datum '
            f'(fix_type >= {self._minfix})'
        )

    def _cb(self, msg: 'GPS'):
        if msg.fix_type < self._minfix:
            return

        # --- Phase 1: collect datum samples ---
        if self._datum is None:
            self._buf.append((msg.latitude, msg.longitude, msg.altitude))
            if len(self._buf) < self._n:
                return
            lat0 = sum(p[0] for p in self._buf) / len(self._buf)
            lon0 = sum(p[1] for p in self._buf) / len(self._buf)
            alt0 = sum(p[2] for p in self._buf) / len(self._buf)
            self._datum = (lat0, lon0, alt0)
            self._buf   = []
            self.get_logger().info(
                f'Datum set: lat={lat0:.8f}  lon={lon0:.8f}  alt={alt0:.1f} m'
            )
            return

        # --- Phase 2: publish ENU relative to datum ---
        east, north = latlon_to_enu(
            msg.latitude, msg.longitude,
            self._datum[0], self._datum[1]
        )
        up = msg.altitude - self._datum[2]

        odom = Odometry()
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = 'odom'       # same frame as wheel odometry
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = up
        odom.pose.pose.orientation.w = 1.0  # no heading from GPS alone

        # Covariance from GPS accuracy report (h_acc = 1-sigma metres → variance)
        h_var = max(float(msg.h_acc) ** 2, 0.25)   # minimum 0.5 m std dev
        v_var = max(float(msg.v_acc) ** 2, 1.0)
        odom.pose.covariance[0]  = h_var   # east
        odom.pose.covariance[7]  = h_var   # north
        odom.pose.covariance[14] = v_var   # up
        odom.pose.covariance[35] = 9999.0  # yaw unknown — EKF ignores it

        self._pub.publish(odom)

    def _status(self):
        if self._datum is None:
            self.get_logger().info(
                f'Waiting for datum: {len(self._buf)}/{self._n} samples collected'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGpsOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
