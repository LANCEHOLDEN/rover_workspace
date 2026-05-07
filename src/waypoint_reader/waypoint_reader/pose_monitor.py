#!/usr/bin/env python3
"""
pose_monitor.py — prints /ins, /ins_enu, and /odometry/filtered side by side every second.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self._filtered = None
        self._ins     = None
        self._ins_enu = None
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(self, '_filtered', m), 10)
        self.create_subscription(Odometry, '/ins',               lambda m: setattr(self, '_ins',      m), 10)
        self.create_subscription(Odometry, '/ins_enu',           lambda m: setattr(self, '_ins_enu',  m), 10)
        self.create_timer(1.0, self._print)

    def _get_vals(self, msg):
        if msg is None:
            return None
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.degrees(quat_to_yaw(o.x, o.y, o.z, o.w))
        return p.x, p.y, p.z, yaw

    def _fmt(self, v, width=12):
        return f'{v:{width}.4f}' if v is not None else ' ' * (width - 3) + 'N/A'

    def _print(self):
        i  = self._get_vals(self._ins)
        e  = self._get_vals(self._ins_enu)
        f  = self._get_vals(self._filtered)

        W = 14
        print('─' * 72)
        print(f'  {"":12}  {"INS /ins (NED)":>{W}}   {"INS /ins_enu (ENU)":>{W}}   {"EKF /odom/filtered":>{W}}')
        print(f'  {"x (m)":12}  {self._fmt(i[0] if i else None, W)}   {self._fmt(e[0] if e else None, W)}   {self._fmt(f[0] if f else None, W)}')
        print(f'  {"y (m)":12}  {self._fmt(i[1] if i else None, W)}   {self._fmt(e[1] if e else None, W)}   {self._fmt(f[1] if f else None, W)}')
        print(f'  {"z (m)":12}  {self._fmt(i[2] if i else None, W)}   {self._fmt(e[2] if e else None, W)}   {self._fmt(f[2] if f else None, W)}')
        print(f'  {"yaw (deg)":12}  {self._fmt(i[3] if i else None, W)}   {self._fmt(e[3] if e else None, W)}   {self._fmt(f[3] if f else None, W)}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
