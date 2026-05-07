#!/usr/bin/env python3
"""
yaw_bias_corrector.py
=====================
Measures IMU yaw drift while the rover is stationary, then republishes
/odometry/local with that bias subtracted from the yaw.

Calibration:
  - Collects 2000 yaw samples from /odometry/local only when stationary
    (|vx| < 0.02 m/s and |vyaw| < 0.02 rad/s)
  - Averages them (circular mean) to find the yaw bias
  - After calibration, every published yaw = raw_yaw - bias

Subscribes : /odometry/local        (nav_msgs/Odometry)
Publishes  : /odometry/local_corrected  (nav_msgs/Odometry)

Use odom_source:=local_corrected in bang_waypoint_follower (add a 'local_corrected'
entry to its odom_topics dict), or just remap this node's output to /odometry/local.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry



class YawBiasCorrector(Node):
    def __init__(self):
        super().__init__('yaw_bias_corrector')

        self.declare_parameter('calib_samples', 1000)

        self._n      = self.get_parameter('calib_samples').value
        self._sum    = 0.0
        self._count  = 0
        self._bias   = None   # yaw rate bias (rad/s), set after calibration

        self._pub = self.create_publisher(Odometry, '/odometry/local_corrected', 10)
        self.create_subscription(Odometry, '/odometry/local', self._cb, 10)
        self.create_timer(5.0, self._log)

        self.get_logger().info(
            f'yaw_bias_corrector: collecting first {self._n} samples to compute yaw rate bias'
        )

    def _cb(self, msg: Odometry):
        # --- calibration phase: collect first N yaw rate samples ---
        if self._bias is None:
            self._sum   += msg.twist.twist.angular.z
            self._count += 1

            if self._count >= self._n:
                self._bias = self._sum / self._n
                self.get_logger().info(
                    f'Yaw rate bias calibrated from {self._n} samples: '
                    f'{self._bias:.6f} rad/s ({math.degrees(self._bias):.4f} deg/s)'
                )

        # --- publish corrected odometry ---
        out = Odometry()
        out.header         = msg.header
        out.child_frame_id = msg.child_frame_id
        out.twist          = msg.twist
        out.pose           = msg.pose

        if self._bias is not None:
            out.twist.twist.angular.z = msg.twist.twist.angular.z - self._bias

        self._pub.publish(out)

    def _log(self):
        if self._bias is None:
            self.get_logger().info(
                f'Calibrating yaw bias: {self._count}/{self._n} stationary samples'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YawBiasCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
