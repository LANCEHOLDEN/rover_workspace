#!/usr/bin/env python3
"""
PID Waypoint Follower for ROS2
Subscribes to /ins, publishes to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import yaml


class PIDController:
    def __init__(self, kp, ki, kd, min_out, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.min_out, min(self.max_out, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Parameters
        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('heading_threshold', 0.3)  # radians (~17 deg)

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.heading_threshold = self.get_parameter('heading_threshold').value

        # PID controllers
        self.heading_pid = PIDController(
            kp=1.5, ki=0.0, kd=0.1,
            min_out=-self.max_angular, max_out=self.max_angular
        )
        self.distance_pid = PIDController(
            kp=0.5, ki=0.0, kd=0.05,
            min_out=0.0, max_out=self.max_linear
        )

        # Load waypoints
        waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = self.load_waypoints(waypoint_file)
        self.current_waypoint_idx = 0

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_time = self.get_clock().now()
        self.active = True

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ins', self.odom_callback, 10
        )

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Goal tolerance: {self.goal_tolerance}m')
        self.get_logger().info('Waypoint follower started!')

    def load_waypoints(self, filepath):
        """Load waypoints from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        """Update current position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def control_loop(self):
        """Main control loop - runs at 20 Hz."""
        if not self.active or not self.waypoints:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            self.stop_robot()
            self.active = False
            return

        # Get current waypoint
        wp = self.waypoints[self.current_waypoint_idx]
        target_x = wp['x']
        target_y = wp['y']

        # Calculate errors
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - self.current_yaw)

        # Time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Check if waypoint reached
        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}'
            )
            self.current_waypoint_idx += 1
            self.heading_pid.reset()
            self.distance_pid.reset()
            return

        # Generate velocity commands
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

        # Log status periodically
        if int(current_time.nanoseconds / 1e9) % 2 == 0:
            self.get_logger().info(
                f'WP {self.current_waypoint_idx + 1}: dist={distance:.2f}m, '
                f'heading_err={math.degrees(heading_error):.1f}°'
            )

    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
