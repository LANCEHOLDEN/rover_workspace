#!/usr/bin/env python3
"""
calibrate_wheelbase.py

Commands a fixed angular velocity for a set duration (should be 1 full rotation),
then computes the correction factor for wheel_base.

Usage:
  python3 calibrate_wheelbase.py

Watch the rover. It should complete EXACTLY 1 full rotation (360°).
Enter whether it over-rotated or under-rotated when prompted.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CURRENT_WHEEL_BASE = 0.381  # meters — current value in zero_config.yaml (15 inches)
OMEGA = 0.4                  # rad/s — spin rate (slow enough to be stable)
DURATION = (2 * math.pi) / OMEGA  # seconds for exactly 1 full rotation


class WheelbaseCalibrator(Node):
    def __init__(self):
        super().__init__('wheelbase_calibrator')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._yaw_start = None
        self._yaw_current = None
        self._total_yaw = 0.0
        self._prev_yaw = None
        self.create_subscription(Odometry, '/odometry/wheels', self._odom_cb, 10)

    def _odom_cb(self, msg):
        o = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(
            2*(o.w*o.z + o.x*o.y),
            1 - 2*(o.y*o.y + o.z*o.z)
        ))
        if self._yaw_start is None:
            self._yaw_start = yaw
            self._prev_yaw = yaw
            self.get_logger().info(f'Start yaw: {yaw:.2f}°')
        else:
            delta = yaw - self._prev_yaw
            # Unwrap
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            self._total_yaw += delta
            self._prev_yaw = yaw

    def spin_and_measure(self):
        # Wait for first odometry message
        self.get_logger().info('Waiting for /odometry/wheels...')
        while self._yaw_start is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'Spinning at {OMEGA} rad/s for {DURATION:.1f}s (should be 1 full rotation)...')
        self.get_logger().info('WATCH THE ROVER — count rotations.')

        cmd = Twist()
        cmd.angular.z = OMEGA
        t_start = time.time()

        while time.time() - t_start < DURATION:
            self._pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        self._pub.publish(Twist())
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)

        reported_deg = self._total_yaw
        expected_deg = math.degrees(OMEGA * DURATION)  # = 360°

        print(f'\n{"="*50}')
        print(f'Expected rotation:  {expected_deg:.1f}°')
        print(f'Odometry reported:  {reported_deg:.1f}°')
        print(f'Current wheel_base: {CURRENT_WHEEL_BASE:.4f} m')

        if abs(reported_deg) < 1.0:
            print('ERROR: No odometry movement detected. Is /odometry/wheels publishing?')
            return

        # The odometry formula: omega_odom = (v_right - v_left) / wheel_base
        # If odom reports X° but physical is 360°, wheel_base needs adjustment.
        # correction = reported / expected  (odom over/under reports physical)
        # new_wheel_base = old * (reported / expected)
        # Because: if reported > expected, odom is over-reporting → wheel_base too small → increase it
        correction = reported_deg / expected_deg
        new_wheel_base = CURRENT_WHEEL_BASE * correction

        print(f'\nOdometry correction factor: {correction:.4f}')
        print(f'Suggested wheel_base:       {new_wheel_base:.4f} m')
        print(f'{"="*50}')
        print(f'\nNow answer: did the rover physically complete exactly 1 full rotation?')
        print(f'  (y) Yes — use {new_wheel_base:.4f} m')
        print(f'  (o) Over-rotated (more than 360°) — wheel_base in COMMAND path is too small')
        print(f'  (u) Under-rotated (less than 360°) — wheel_base in COMMAND path is too large')

        ans = input('\nYour answer (y/o/u): ').strip().lower()

        if ans == 'y':
            print(f'\nSet wheel_base: {new_wheel_base:.4f} m in zero_config.yaml')
        elif ans == 'o':
            # Rover over-rotated physically → command wheel_base too small → increase command wheel_base
            # But since both command and odom share the same param, suggest splitting the difference
            print(f'\nPhysical over-rotation: command wheel_base is too small.')
            print(f'Try: {new_wheel_base:.4f} m (corrects odometry) but physical may still over-rotate.')
            print(f'If over-rotation persists after update, increase further toward {new_wheel_base*1.1:.4f} m')
        elif ans == 'u':
            print(f'\nPhysical under-rotation: command wheel_base is too large.')
            print(f'Try: {new_wheel_base:.4f} m (corrects odometry) but physical may still under-rotate.')
            print(f'If under-rotation persists after update, decrease toward {new_wheel_base*0.9:.4f} m')


def main():
    rclpy.init()
    node = WheelbaseCalibrator()
    try:
        node.spin_and_measure()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
