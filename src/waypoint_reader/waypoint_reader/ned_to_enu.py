#!/usr/bin/env python3
"""
ned_to_enu.py — converts /ins from NED frame to ENU frame for robot_localization.

The InertialSense driver publishes orientation as qn2b (NED-to-body quaternion)
and position in NED coordinates. robot_localization expects ENU (REP-103).

Subscribes:  /ins           (nav_msgs/Odometry, NED)
Publishes:   /ins_enu       (nav_msgs/Odometry, ENU)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry



class NedToEnu(Node):
    def __init__(self):
        super().__init__('ned_to_enu')
        self._pub = self.create_publisher(Odometry, '/ins_enu', 10)
        self.create_subscription(Odometry, '/ins', self._callback, 10)
        self._origin_x = None
        self._origin_y = None
        self._origin_z = None
        self.get_logger().info('ned_to_enu: /ins (NED) -> /ins_enu (ENU), zeroed to first fix')

    def _callback(self, msg: Odometry):
        # Convert position NED -> ENU first, then zero to first received position
        enu_x =  msg.pose.pose.position.y   # East  = NED y
        enu_y =  msg.pose.pose.position.x   # North = NED x
        enu_z = -msg.pose.pose.position.z   # Up    = -NED Down

        if self._origin_x is None:
            self._origin_x = enu_x
            self._origin_y = enu_y
            self._origin_z = enu_z
            self.get_logger().info(
                f'Origin set: ENU ({enu_x:.2f}, {enu_y:.2f}, {enu_z:.2f}) m from GPS ref'
            )

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'

        # Position: zeroed so rover starts at (0,0,0) in odom frame
        out.pose.pose.position.x = enu_x - self._origin_x
        out.pose.pose.position.y = enu_y - self._origin_y
        out.pose.pose.position.z = enu_z - self._origin_z

        # Orientation: extract NED yaw, convert to ENU yaw, build 2D quaternion.
        # NED yaw: clockwise from North. ENU yaw: CCW from East = pi/2 - NED_yaw.
        #
        # The InertialSense is mounted BACKWARD (sensor +x = rover -x).
        # INS_rpy=[0,0,0] means qn2b is in the SENSOR frame — 180° off from rover.
        # We add pi to get the rover's NED yaw before converting to ENU.
        o = msg.pose.pose.orientation
        yaw_ned_sensor = math.atan2(
            2.0 * (o.w * o.z + o.x * o.y),
            1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        )
        yaw_ned = yaw_ned_sensor + math.pi   # backward-mount correction
        yaw_enu = math.pi / 2.0 - yaw_ned
        out.pose.pose.orientation.w = math.cos(yaw_enu / 2.0)
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(yaw_enu / 2.0)

        # Pose covariance — driver leaves all zeros so set defaults.
        # robot_localization rejects measurements with zero diagonal covariance.
        # Index 35 = yaw variance (row 5, col 5 of 6x6 matrix).
        cov = list(msg.pose.covariance)
        if cov[35] == 0.0:
            cov[0]  = 9999.0  # x   — position NOT fused from ins_enu
            cov[7]  = 9999.0  # y   — position NOT fused from ins_enu
            cov[14] = 9999.0  # z
            cov[21] = 9999.0  # roll
            cov[28] = 9999.0  # pitch
            cov[35] = 0.05    # yaw — ~13 deg std dev, used for heading init
        out.pose.covariance = cov

        # Twist: linear velocities pass through unchanged.
        # angular.z negated: INS body z=Down (CW=+), ROS body z=Up (CCW=+).
        out.twist.twist.linear.x =  msg.twist.twist.linear.x
        out.twist.twist.linear.y =  msg.twist.twist.linear.y
        out.twist.twist.linear.z =  msg.twist.twist.linear.z
        out.twist.twist.angular.x =  msg.twist.twist.angular.x
        out.twist.twist.angular.y =  msg.twist.twist.angular.y
        out.twist.twist.angular.z = -msg.twist.twist.angular.z

        # Twist covariance — driver leaves all zeros so set defaults.
        # Index 35 = angular.z variance (vyaw, used by EKF).
        tcov = list(msg.twist.covariance)
        if tcov[35] == 0.0:
            tcov[0]  = 1e6   # vx  (not used)
            tcov[7]  = 1e6   # vy  (not used)
            tcov[14] = 1e6   # vz  (not used)
            tcov[21] = 1e6   # wx  (not used)
            tcov[28] = 1e6   # wy  (not used)
            tcov[35] = 5.0   # wz  (angular.z, used — higher = less trusted by EKF)
        out.twist.covariance = tcov

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NedToEnu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
