#!/usr/bin/env python3
"""
ins_localizer.py — converts /ins absolute NED position to a local ENU odometry
topic (/ins/local) that the EKF can fuse as an absolute position source.

The InertialSense INS publishes position in NED meters from a fixed GPS origin
(typically the Earth's surface reference, resulting in large absolute values like
x=13345, y=49356). The EKF works in a local odom frame starting at (0,0), so
this node:

  1. Records the first INS message as the NED origin (x0, y0, z0).
  2. Converts every subsequent message to local ENU:
       enu_x = NED_y - NED_y0   (East)
       enu_y = NED_x - NED_x0   (North)
       enu_z = -(NED_z - NED_z0) (Up)
  3. Publishes nav_msgs/Odometry on /ins/local with:
       - Converted local ENU position
       - Position covariance set from GPS accuracy parameter
       - Identity orientation (heading is handled separately by odom1 in EKF)

The EKF outdoor config fuses /ins/local for x, y position only.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class InsLocalizer(Node):
    def __init__(self):
        super().__init__('ins_localizer')

        # GPS position accuracy in meters (1-sigma). Used to set covariance.
        # Standard GPS: ~1.5m, RTK fixed: ~0.02m
        self.declare_parameter('gps_position_sigma', 1.5)
        sigma = self.get_parameter('gps_position_sigma').value
        self._pos_variance = sigma ** 2

        self._origin = None  # (ned_x0, ned_y0, ned_z0)

        self._pub = self.create_publisher(Odometry, '/ins/local', 10)
        self.create_subscription(Odometry, '/ins', self._cb, 10)

        self.get_logger().info(
            f'ins_localizer started — position sigma={sigma:.2f}m '
            f'(variance={self._pos_variance:.4f} m²)'
        )

    def _cb(self, msg: Odometry):
        ned_x = msg.pose.pose.position.x
        ned_y = msg.pose.pose.position.y
        ned_z = msg.pose.pose.position.z

        if self._origin is None:
            self._origin = (ned_x, ned_y, ned_z)
            self.get_logger().info(
                f'NED origin set: x={ned_x:.3f} y={ned_y:.3f} z={ned_z:.3f}'
            )

        x0, y0, z0 = self._origin

        # NED -> local ENU
        enu_x =  (ned_y - y0)   # East
        enu_y =  (ned_x - x0)   # North
        enu_z = -(ned_z - z0)   # Up

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'

        out.pose.pose.position.x = enu_x
        out.pose.pose.position.y = enu_y
        out.pose.pose.position.z = enu_z

        # Identity orientation — EKF config only fuses x,y from this topic
        out.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Set position covariance diagonal (x, y, z, roll, pitch, yaw)
        v = self._pos_variance
        cov = [0.0] * 36
        cov[0]  = v    # x
        cov[7]  = v    # y
        cov[14] = 9.0  # z — less accurate vertically
        cov[21] = 99.0 # roll  — not used
        cov[28] = 99.0 # pitch — not used
        cov[35] = 99.0 # yaw   — not used (handled by odom1)
        out.pose.covariance = cov

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = InsLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
