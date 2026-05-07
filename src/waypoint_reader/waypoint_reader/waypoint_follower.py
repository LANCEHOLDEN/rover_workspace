#!/usr/bin/env python3
"""
Waypoint Follower Node for ROS2
Reads waypoints from YAML file and navigates rover via /cmd_vel
Subscribes to /ins for position feedback
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import yaml


class PIDController:
    """Simple PID controller with output clamping."""

    def __init__(self, kp, ki, kd, min_out, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        # Anti-windup: clamp integral
        max_integral = (self.max_out - self.min_out) / max(self.ki, 0.001)
        self.integral = max(-max_integral, min(max_integral, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.min_out, min(self.max_out, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Declare parameters
        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 2.2)
        self.declare_parameter('heading_threshold', 0.175)  # radians (~10 deg)
        self.declare_parameter('loop', False)  # Loop through waypoints
        self.declare_parameter('reverse', False)  # Run waypoints in reverse
        self.declare_parameter('stuck_timeout', 15.0)  # seconds before declaring stuck
        self.declare_parameter('stuck_distance_threshold', 0.05)  # meters
        self.declare_parameter('min_angular_vel', 1.5)   # minimum angular cmd to overcome stiction
        self.declare_parameter('min_linear_vel', 0.2)    # minimum linear cmd for consistent forward speed
        self.declare_parameter('yaw_smooth_alpha', 0.14)     # EMA for yaw (0=frozen, 1=raw)
        self.declare_parameter('heading_error_smooth_alpha', 0.20)  # EMA for heading error

        # PID gains
        self.declare_parameter('heading_kp', 1.2)
        self.declare_parameter('heading_ki', 0.02)
        self.declare_parameter('heading_kd', 0.6)
        self.declare_parameter('distance_kp', 0.5)
        self.declare_parameter('distance_ki', 0.0)
        self.declare_parameter('distance_kd', 0.05)

        # Get parameters
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.heading_threshold = self.get_parameter('heading_threshold').value
        self.loop = self.get_parameter('loop').value
        self.reverse = self.get_parameter('reverse').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        self.min_angular_vel = self.get_parameter('min_angular_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.yaw_alpha = self.get_parameter('yaw_smooth_alpha').value
        self.heading_error_alpha = self.get_parameter('heading_error_smooth_alpha').value

        # Initialize PID controllers
        self.heading_pid = PIDController(
            kp=self.get_parameter('heading_kp').value,
            ki=self.get_parameter('heading_ki').value,
            kd=self.get_parameter('heading_kd').value,
            min_out=-self.max_angular,
            max_out=self.max_angular
        )
        self.distance_pid = PIDController(
            kp=self.get_parameter('distance_kp').value,
            ki=self.get_parameter('distance_ki').value,
            kd=self.get_parameter('distance_kd').value,
            min_out=0.0,
            max_out=self.max_linear
        )

        # Load waypoints
        waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = self._load_waypoints(waypoint_file)

        if self.reverse:
            self.waypoints = list(reversed(self.waypoints))
            self.get_logger().info('Running waypoints in REVERSE order')

        self.current_wp_idx = 0

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.ins_origin = None        # First INS position for local frame conversion
        self.ins_origin_yaw = None    # Initial heading — waypoints are rotated by this
        self.last_time = self.get_clock().now()
        self.active = True

        # Stuck detection state
        self.last_progress_time = self.get_clock().now()
        self.last_progress_x = 0.0
        self.last_progress_y = 0.0

        # Smoothed state (exponential moving average)
        self.smooth_yaw = None
        self.smooth_heading_error = None

        # Hysteresis state: track whether we're in turn-in-place mode
        # Enter turning when |heading_error| > heading_threshold (0.175 rad)
        # Exit turning only when |heading_error| < heading_exit_threshold (~0.07 rad)
        self.turning = False

        # Braking state: publish zero velocity for N cycles before starting a turn
        # The motor driver does not stop on lin.x=0 alone — explicit brake pulses needed.
        self.braking = False
        self.brake_cycles_remaining = 0
        self.BRAKE_CYCLES = 10  # 10 cycles @ 20 Hz = 0.5s of explicit zero velocity

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/wheels', self._odom_callback, 10)

        # Control loop at 20 Hz
        self.control_timer = self.create_timer(0.05, self._control_loop)

        # Status logging at 1 Hz
        self.log_timer = self.create_timer(1.0, self._log_status)

        self.get_logger().info(f'Waypoint Follower initialized')
        self.get_logger().info(f'  Waypoints loaded: {len(self.waypoints)}')
        self.get_logger().info(f'  Goal tolerance: {self.goal_tolerance}m')
        self.get_logger().info(f'  Max linear vel: {self.max_linear} m/s')
        self.get_logger().info(f'  Max angular vel: {self.max_angular} rad/s')
        self.get_logger().info(f'  Loop mode: {self.loop}')
        self.get_logger().info('Waiting for /odometry/wheels...')

    def _load_waypoints(self, filepath):
        """Load waypoints from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                waypoints = data.get('waypoints', [])
                self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {filepath}')
                return waypoints
        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {filepath}')
            return []
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def _quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _odom_callback(self, msg):
        """Update position and heading from /odometry/wheels."""
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        if self.ins_origin is None:
            self.ins_origin = (raw_x, raw_y)
            raw_yaw_init = self._quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            self.ins_origin_yaw = raw_yaw_init
            cos_h = math.cos(raw_yaw_init)
            sin_h = math.sin(raw_yaw_init)
            for wp in self.waypoints:
                ox, oy = wp['x'], wp['y']
                wp['x'] = ox * cos_h - oy * sin_h
                wp['y'] = ox * sin_h + oy * cos_h
            self.get_logger().info(
                f'Origin set: x={raw_x:.3f}, y={raw_y:.3f}, '
                f'heading={math.degrees(raw_yaw_init):.1f} deg — waypoints rotated'
            )
            self.odom_received = True
            self.get_logger().info('Odometry received, starting navigation')

        self.current_x = raw_x - self.ins_origin[0]
        self.current_y = raw_y - self.ins_origin[1]

        raw_yaw = self._quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        if self.smooth_yaw is None:
            self.smooth_yaw = raw_yaw
        else:
            diff = self._normalize_angle(raw_yaw - self.smooth_yaw)
            self.smooth_yaw = self._normalize_angle(
                self.smooth_yaw + self.yaw_alpha * diff
            )
        self.current_yaw = self.smooth_yaw

    def _control_loop(self):
        """Main control loop - runs at 20 Hz."""
        if not self.active or not self.waypoints or not self.odom_received:
            return

        # Check if all waypoints completed
        if self.current_wp_idx >= len(self.waypoints):
            if self.loop:
                self.get_logger().info('Looping back to first waypoint')
                self.current_wp_idx = 0
                self.heading_pid.reset()
                self.distance_pid.reset()
                self.turning = False
                self.last_progress_time = self.get_clock().now()
                self.last_progress_x = self.current_x
                self.last_progress_y = self.current_y
            else:
                self.get_logger().info('All waypoints reached! Stopping.')
                self._stop_robot()
                self.active = False
                return

        # Get current waypoint
        wp = self.waypoints[self.current_wp_idx]
        target_x = wp['x']
        target_y = wp['y']

        # Calculate errors
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_heading = math.atan2(dy, dx)
        # EKF yaw is ENU convention (CCW = +yaw), standard negative-feedback sign.
        raw_heading_error = self._normalize_angle(target_heading - self.current_yaw)
        if self.smooth_heading_error is None:
            self.smooth_heading_error = raw_heading_error
        else:
            diff = self._normalize_angle(raw_heading_error - self.smooth_heading_error)
            self.smooth_heading_error = self._normalize_angle(
                self.smooth_heading_error + self.heading_error_alpha * diff
            )
        # Use raw error for large turns so the PID reacts immediately.
        # Only apply smoothing during fine heading corrections while driving forward.
        if abs(raw_heading_error) > self.heading_threshold:
            heading_error = raw_heading_error
        else:
            heading_error = self.smooth_heading_error

        # Time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Check if waypoint reached
        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} '
                f'at ({target_x:.2f}, {target_y:.2f})'
            )
            self.current_wp_idx += 1
            self.heading_pid.reset()
            self.distance_pid.reset()
            self.smooth_heading_error = None
            self.turning = True
            self.braking = True
            self.brake_cycles_remaining = self.BRAKE_CYCLES
            # Reset stuck detector for next waypoint
            self.last_progress_time = self.get_clock().now()
            self.last_progress_x = self.current_x
            self.last_progress_y = self.current_y
            return

        # Stuck detection: check if position has progressed since last check
        moved = math.sqrt(
            (self.current_x - self.last_progress_x) ** 2 +
            (self.current_y - self.last_progress_y) ** 2
        )
        if moved >= self.stuck_distance_threshold:
            # Made progress — reset the stuck timer
            self.last_progress_time = current_time
            self.last_progress_x = self.current_x
            self.last_progress_y = self.current_y
        else:
            stuck_elapsed = (current_time - self.last_progress_time).nanoseconds / 1e9
            if stuck_elapsed > self.stuck_timeout:
                self.get_logger().error(
                    f'STUCK DETECTED: no progress (>{self.stuck_distance_threshold}m) '
                    f'in {stuck_elapsed:.1f}s at wp {self.current_wp_idx + 1}/'
                    f'{len(self.waypoints)} — STOPPING to prevent motor damage.'
                )
                self._stop_robot()
                self.active = False
                return

        # Generate velocity command
        cmd = Twist()

        # Hysteresis: enter turn-in-place when error > threshold, exit only when < exit threshold
        # This prevents rapid mode-switching (jitter) when error hovers near threshold.
        heading_exit_threshold = self.heading_threshold * 0.4  # ~0.14 rad (~8 deg)
        was_turning = self.turning
        if abs(heading_error) > self.heading_threshold:
            self.turning = True
        elif abs(heading_error) < heading_exit_threshold:
            self.turning = False

        # When transitioning from drive -> turn, brake first.
        # The motor driver does not stop on lin.x=0 alone — send explicit zeros.
        if self.turning and not was_turning:
            self.braking = True
            self.brake_cycles_remaining = self.BRAKE_CYCLES
            self.heading_pid.reset()

        if self.braking:
            # Publish zero velocity until rover decelerates
            self.brake_cycles_remaining -= 1
            if self.brake_cycles_remaining <= 0:
                self.braking = False
            # cmd is already zero-initialized — just publish it
        elif self.turning:
            # Turn in place — use raw heading error directly (no EMA lag during turns)
            angular = self.heading_pid.compute(raw_heading_error, dt)
            # Apply floor throughout the turn to overcome stiction
            if abs(angular) > 0.01:
                angular = math.copysign(max(abs(angular), self.min_angular_vel), angular)
            cmd.angular.z = angular
            cmd.linear.x = 0.0
        else:
            # Drive forward with heading correction
            linear = self.distance_pid.compute(distance, dt)
            # Apply floor for consistent speed — prevents tapering to a crawl near waypoint
            cmd.linear.x = max(linear, self.min_linear_vel)
            # Suppress angular correction in the final approach — atan2 becomes noisy
            # at close range and causes the rover to veer before waypoint arrival.
            if distance > self.goal_tolerance * 1.5:
                angular = self.heading_pid.compute(heading_error, dt)
                if abs(heading_error) > 0.15 and abs(angular) > 0.01:
                    angular = math.copysign(max(abs(angular), self.min_angular_vel * 0.5), angular)
                cmd.angular.z = angular
            else:
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def _log_status(self):
        """Log status periodically."""
        if not self.active or not self.odom_received:
            return

        if self.current_wp_idx < len(self.waypoints):
            wp = self.waypoints[self.current_wp_idx]
            dx = wp['x'] - self.current_x
            dy = wp['y'] - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)
            target_heading = math.atan2(dy, dx)
            heading_err = self._normalize_angle(target_heading - self.current_yaw)

            self.get_logger().info(
                f'WP {self.current_wp_idx + 1}/{len(self.waypoints)}: '
                f'dist={dist:.2f}m, heading_err={math.degrees(heading_err):.1f} deg | '
                f'pos=({self.current_x:.3f}, {self.current_y:.3f}), '
                f'yaw={math.degrees(self.current_yaw):.1f} deg | '
                f'target=({wp["x"]:.3f}, {wp["y"]:.3f})'
            )

    def _stop_robot(self):
        """Stop the robot by publishing zero velocities."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Robot stopped')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
