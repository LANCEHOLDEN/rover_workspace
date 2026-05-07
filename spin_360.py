#!/usr/bin/env python3
"""
spin_360.py — Commands the rover to spin a target number of degrees in place.
Uses /odometry/wheels yaw feedback to stop at the correct angle.

Usage:
    python3 spin_360.py
    python3 spin_360.py --angular-speed 0.5   # slower
    python3 spin_360.py --degrees 180          # half spin
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Shortest signed difference between two angles (radians)."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class Spin360(Node):
    def __init__(self, degrees, angular_speed):
        super().__init__('spin_360')

        self.target_radians = math.radians(degrees)
        self.angular_speed = angular_speed if degrees >= 0 else -abs(angular_speed)

        self.start_yaw = None
        self.accumulated = 0.0
        self.last_yaw = None
        self.done = False

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry, '/odometry/wheels', self.odom_cb, 10)

        self.get_logger().info(
            f'Spinning {degrees}° at {angular_speed:.2f} rad/s — '
            f'waiting for odometry...'
        )

    def odom_cb(self, msg):
        if self.done:
            return

        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.last_yaw is None:
            self.last_yaw = yaw
            self.get_logger().info(f'Start yaw: {math.degrees(yaw):.1f}°')
            return

        # Accumulate shortest-path yaw delta each step
        delta = angle_diff(yaw, self.last_yaw)
        self.accumulated += delta
        self.last_yaw = yaw

        remaining = self.target_radians - self.accumulated
        self.get_logger().info(
            f'Rotated: {math.degrees(self.accumulated):.1f}°  '
            f'remaining: {math.degrees(remaining):.1f}°',
            throttle_duration_sec=0.5
        )

        if abs(self.accumulated) >= abs(self.target_radians):
            self.pub.publish(Twist())  # stop
            self.done = True
            self.get_logger().info(
                f'Done — rotated {math.degrees(self.accumulated):.1f}°'
            )
            rclpy.shutdown()
            return

        msg_out = Twist()
        msg_out.angular.z = self.angular_speed
        self.pub.publish(msg_out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--degrees', type=float, default=360.0,
                        help='Degrees to rotate (default: 360)')
    parser.add_argument('--angular-speed', type=float, default=1.0,
                        help='Angular speed in rad/s (default: 1.0)')
    args = parser.parse_args()

    rclpy.init()
    node = Spin360(args.degrees, args.angular_speed)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
