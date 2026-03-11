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
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('heading_threshold', 0.3)  # radians (~17 deg)
        self.declare_parameter('loop', False)  # Loop through waypoints
        self.declare_parameter('reverse', False)  # Run waypoints in reverse

        # PID gains
        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 0.1)
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
        self.ins_origin = None  # First INS position for local frame conversion
        self.last_time = self.get_clock().now()
        self.active = True

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ins', self._odom_callback, 10
        )

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
        self.get_logger().info('Waiting for /ins odometry...')

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
        """Update current position from /ins odometry."""
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        # Set origin from first INS message to convert to local frame
        if self.ins_origin is None:
            self.ins_origin = (raw_x, raw_y)
            self.get_logger().info(
                f'INS origin set: x={raw_x:.3f}, y={raw_y:.3f}'
            )

        if not self.odom_received:
            self.get_logger().info('Odometry received, starting navigation')
            self.odom_received = True

        self.current_x = raw_x - self.ins_origin[0]
        self.current_y = raw_y - self.ins_origin[1]
        self.current_yaw = self._quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

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
        heading_error = self._normalize_angle(target_heading - self.current_yaw)

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
            return

        # Generate velocity command
        cmd = Twist()

        if abs(heading_error) > self.heading_threshold:
            # Turn in place first
            cmd.angular.z = self.heading_pid.compute(heading_error, dt)
            cmd.linear.x = 0.0
        else:
            # Drive forward with heading correction
            cmd.linear.x = self.distance_pid.compute(distance, dt)
            cmd.angular.z = self.heading_pid.compute(heading_error, dt)

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
