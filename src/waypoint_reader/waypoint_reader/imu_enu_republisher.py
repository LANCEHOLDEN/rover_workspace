#!/usr/bin/env python3
"""
imu_enu_republisher.py
======================
Remaps the raw InertialSense /imu (NED frame, backward-mounted) to
/imu/enu (ENU frame) so robot_localization's EKF and navsat_transform_node
receive data in the REP-103 ENU convention they expect.

  Subscribes : /imu      (sensor_msgs/msg/Imu)  — NED, backward-mounted
  Publishes  : /imu/enu  (sensor_msgs/msg/Imu)  — ENU, consistent with wheel odom

Physical mounting / frame transform
------------------------------------
The InertialSense sensor is mounted such that:
    IMU +x → rover -x  (sensor faces backward)
    IMU +z → rover -z  (sensor z-down; rover z-up)
    IMU +y → rover +y  (right-hand rule forces this)

Combined with the NED→ENU convention this gives a 180° rotation about the
rover's Y-axis:
    enu_x = -imu_x
    enu_y = +imu_y
    enu_z = -imu_z

This is applied to BOTH angular_velocity and linear_acceleration.

Orientation
-----------
The InertialSense /imu orientation field is identity with all-zero
covariance (indicating invalid absolute orientation — only raw gyro/accel).
robot_localization detects the all-zero covariance and ignores the
orientation field automatically, so it is passed through unchanged.

Gravity convention
------------------
After the transform the accelerometer z-axis reads +9.81 m/s² when
stationary (ENU: reaction force points up = +z). This is exactly what
robot_localization's imu0_remove_gravitational_acceleration expects.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuEnuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_enu_republisher')
        self.pub = self.create_publisher(Imu, '/imu/enu', 50)
        self.sub = self.create_subscription(Imu, '/imu', self._cb, 50)
        self._count = 0
        self.create_timer(5.0, self._log_status)
        self.get_logger().info(
            'imu_enu_republisher started: /imu (NED, backward) → /imu/enu (ENU)\n'
            '  Transform: (-x, +y, -z) on angular_velocity and linear_acceleration'
        )

    def _cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = 'base_link'   # ENU body frame, same as wheel odom

        # Remap: IMU NED (backward-mounted) → ENU
        #   enu_x = -imu_x,  enu_y = +imu_y,  enu_z = -imu_z
        out.angular_velocity.x = -msg.angular_velocity.x
        out.angular_velocity.y =  msg.angular_velocity.y
        out.angular_velocity.z = -msg.angular_velocity.z

        out.linear_acceleration.x = -msg.linear_acceleration.x
        out.linear_acceleration.y =  msg.linear_acceleration.y
        out.linear_acceleration.z = -msg.linear_acceleration.z

        # Pass through orientation (identity, all-zero cov — robot_localization
        # ignores it automatically because covariance diagonal is zero)
        out.orientation            = msg.orientation
        out.orientation_covariance = msg.orientation_covariance

        # Remap angular velocity covariance (3×3 diagonal, row-major 9 elements)
        # Rows/cols for x and z are negated axes — variance is sign-invariant (σ² > 0)
        # so the covariance values are copied as-is, just reordered if needed.
        # The InertialSense driver reports all-zero covariance (unknown), so copy through.
        out.angular_velocity_covariance    = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)
        self._count += 1

    def _log_status(self):
        self.get_logger().info(f'Republished {self._count} IMU messages in last 5 s')
        self._count = 0


def main(args=None):
    rclpy.init(args=args)
    node = ImuEnuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
