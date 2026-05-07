#!/usr/bin/env python3
"""
Simple Waypoint Follower for Differential Drive Robot
------------------------------------------------------
Uses wheel odometry (/odometry/wheels) for pose feedback.
Controller logic based on:
  https://github.com/zainkhan-afk/Differential-Drive-Robot-Navigation

State machine per waypoint:
  TURN  — stop and turn in place until heading error < heading_tolerance
  DRIVE — drive forward with heading correction until distance < goal_tolerance
"""

import math
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SimpleWaypointFollower(Node):
    def __init__(self):
        super().__init__('simple_waypoint_follower')

        # Parameters
        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('goal_tolerance', 0.3)       # metres
        self.declare_parameter('max_linear_vel', 0.2)       # m/s
        self.declare_parameter('max_angular_vel', 2.0)      # rad/s
        self.declare_parameter('min_angular_vel', 1.0)      # rad/s
        self.declare_parameter('kp_linear', 1.3)
        self.declare_parameter('kd_linear', 0.07)
        self.declare_parameter('kp_angular', 1.3)
        self.declare_parameter('kd_angular', 0.07)
        self.declare_parameter('heading_tolerance', 0.2)    # radians (~12 deg)
        self.declare_parameter('loop', False)

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear     = self.get_parameter('max_linear_vel').value
        self.max_angular    = self.get_parameter('max_angular_vel').value
        self.min_angular    = self.get_parameter('min_angular_vel').value
        self.kp_linear      = self.get_parameter('kp_linear').value
        self.kd_linear      = self.get_parameter('kd_linear').value
        self.kp_angular        = self.get_parameter('kp_angular').value
        self.kd_angular        = self.get_parameter('kd_angular').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.loop              = self.get_parameter('loop').value

        # Load waypoints
        wp_file = self.get_parameter('waypoint_file').value
        self.waypoints = self._load_waypoints(wp_file)
        self.current_wp_idx = 0

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0          # radians, extracted as 2*atan2(qz, qw)
        self.odom_received = False
        self.origin = None      # (x0, y0) so robot frame starts at (0, 0)

        # PD previous errors
        self.prev_dist_error    = 0.0
        self.prev_heading_error = 0.0
        self.last_time = self.get_clock().now()

        # State machine: 'TURN' first, then 'DRIVE'
        self.state = 'TURN'

        self.active = True

        # ROS interfaces
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/wheels', self._odom_callback, 10)

        self.create_timer(0.05, self._control_loop)   # 20 Hz control
        self.create_timer(1.0,  self._log_status)     # 1 Hz status

        self.get_logger().info(
            f'SimpleWaypointFollower ready | {len(self.waypoints)} waypoints | '
            f'tolerance={self.goal_tolerance}m | loop={self.loop}'
        )
        self.get_logger().info('Waiting for /odometry/wheels ...')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _load_waypoints(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            wps = data.get('waypoints', [])
            self.get_logger().info(f'Loaded {len(wps)} waypoints from {filepath}')
            return wps
        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {filepath}')
            return []
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        if self.origin is None:
            self.origin = (raw_x, raw_y)
            self.odom_received = True
            self.get_logger().info(
                f'Odometry received — origin set to ({raw_x:.3f}, {raw_y:.3f})'
            )

        self.x = raw_x - self.origin[0]
        self.y = raw_y - self.origin[1]

        # Flat-ground yaw: valid when roll and pitch are near zero
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = 2.0 * math.atan2(qz, qw)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if not self.active or not self.odom_received or not self.waypoints:
            return

        # All waypoints done
        if self.current_wp_idx >= len(self.waypoints):
            if self.loop:
                self.get_logger().info('Looping back to first waypoint')
                self.current_wp_idx = 0
            else:
                self.get_logger().info('All waypoints reached — stopping.')
                self._stop()
                self.active = False
                return

        wp = self.waypoints[self.current_wp_idx]
        goal_x = wp['x']
        goal_y = wp['y']

        dx = goal_x - self.x
        dy = goal_y - self.y
        dist_error    = math.sqrt(dx * dx + dy * dy)
        desired_yaw   = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - self.yaw)

        # Time delta for derivative term
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        cmd = Twist()

        if self.state == 'TURN':
            # Turn in place until heading is aligned
            if abs(heading_error) < self.heading_tolerance:
                self.get_logger().info('Heading aligned — driving forward')
                self.state = 'DRIVE'
                self.prev_dist_error    = dist_error
                self.prev_heading_error = 0.0
            else:
                d_heading = (heading_error - self.prev_heading_error) / dt
                angular_vel = self.kp_angular * heading_error + self.kd_angular * d_heading
                self.prev_heading_error = heading_error
                cmd.linear.x  = 0.0
                angular_clamped = max(-self.max_angular, min(self.max_angular, angular_vel))
                if abs(angular_clamped) > 0.0:
                    angular_clamped = math.copysign(max(self.min_angular, abs(angular_clamped)), angular_clamped)
                cmd.angular.z = angular_clamped

        elif self.state == 'DRIVE':
            # Reached waypoint
            if dist_error < self.goal_tolerance:
                self.get_logger().info(
                    f'Reached waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} '
                    f'({goal_x:.2f}, {goal_y:.2f})'
                )
                self.current_wp_idx += 1
                self.prev_dist_error    = 0.0
                self.prev_heading_error = 0.0
                self.state = 'TURN'
                self._stop()
                return

            d_dist    = (dist_error    - self.prev_dist_error)    / dt
            d_heading = (heading_error - self.prev_heading_error) / dt

            linear_vel  = self.kp_linear  * dist_error    + self.kd_linear  * d_dist
            angular_vel = self.kp_angular * heading_error + self.kd_angular * d_heading

            self.prev_dist_error    = dist_error
            self.prev_heading_error = heading_error

            cmd.linear.x  = max(0.0, min(self.max_linear, linear_vel))
            angular_clamped = max(-self.max_angular, min(self.max_angular, angular_vel))
            if abs(angular_clamped) > 0.0:
                angular_clamped = math.copysign(max(self.min_angular, abs(angular_clamped)), angular_clamped)
            cmd.angular.z = angular_clamped

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # Logging / stop
    # ------------------------------------------------------------------

    def _log_status(self):
        if not self.active or not self.odom_received:
            return
        if self.current_wp_idx >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_wp_idx]
        dx = wp['x'] - self.x
        dy = wp['y'] - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        heading_err = normalize_angle(math.atan2(dy, dx) - self.yaw)

        self.get_logger().info(
            f'[{self.state}] WP {self.current_wp_idx + 1}/{len(self.waypoints)} | '
            f'dist={dist:.2f}m | heading_err={math.degrees(heading_err):.1f}° | '
            f'pos=({self.x:.2f}, {self.y:.2f}) | yaw={math.degrees(self.yaw):.1f}°'
        )

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
