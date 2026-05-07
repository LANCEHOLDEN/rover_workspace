#!/usr/bin/env python3
"""
bang_waypoint_follower.py
--------------------------
Waypoint follower using the same bang-bang control as spin_360.py and move_5_feet.py.

TURN phase  — spins at a fixed angular speed, accumulating yaw delta (spin_360 style)
              until the heading to the next waypoint is within tolerance.
DRIVE phase — drives straight at a fixed linear speed, tracking euclidean distance
              (move_5_feet style) until within goal_tolerance of the waypoint.

Usage:
    ros2 run waypoint_reader bang_waypoint_follower --ros-args \
        -p waypoint_file:=/path/to/waypoints.yaml \
        -p odom_source:=local   # 'local' (default), 'global', 'wheels', or 'ins'

odom_source options:
    local   — /odometry/local  : EKF fusing wheels + IMU  [recommended]
    global  — /odometry/global : EKF fusing local + GPS   [GPS-anchored, may jump]
    wheels  — /odometry/wheels : raw wheel encoders only
    ins     — /ins             : InertialSense firmware EKF (NED frame)
"""

import math
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Helpers (same as spin_360.py)
# ---------------------------------------------------------------------------

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Shortest signed difference a - b, wrapped to [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class BangWaypointFollower(Node):
    def __init__(self):
        super().__init__('bang_waypoint_follower')

        # Parameters
        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('linear_speed', 0.5)       # m/s
        self.declare_parameter('angular_speed', 1.8)      # rad/s
        self.declare_parameter('goal_tolerance', 0.1)      # metres
        self.declare_parameter('heading_tolerance', 0.15) # radians (~9 deg)
        self.declare_parameter('loop', False)
        self.declare_parameter('odom_source', 'local')  # 'wheels', 'local', 'local_corrected', 'filtered'

        self.linear_speed      = self.get_parameter('linear_speed').value
        self.angular_speed     = self.get_parameter('angular_speed').value
        self.goal_tolerance    = self.get_parameter('goal_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.loop              = self.get_parameter('loop').value
        odom_source            = self.get_parameter('odom_source').value

        odom_topics = {
            'filtered':        '/odometry/filtered',        # GPS-corrected (can jump)
            'local':           '/odometry/local',           # dead reckoning: wheels + IMU + INS rate
            'local_corrected': '/odometry/local_corrected', # dead reckoning with yaw bias removed
            'global':          '/odometry/global',          # old global EKF
            'wheels':          '/odometry/wheels',          # raw wheel encoders only
            'ins':             '/ins',                      # InertialSense firmware EKF (NED frame)
        }
        if odom_source not in odom_topics:
            self.get_logger().warn(
                f'Unknown odom_source "{odom_source}", defaulting to "local"'
            )
            odom_source = 'local'
        self.odom_topic = odom_topics[odom_source]

        # Load waypoints
        wp_file = self.get_parameter('waypoint_file').value
        self.waypoints = self._load_waypoints(wp_file)
        self.wp_idx = 0

        # Robot state — all positions are relative to the initial /odometry/wheels pose
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.wheels_origin = None   # (x, y, yaw) from first /odometry/wheels message
        self.odom_ready    = False  # true once wheels_origin AND nav odom are both received

        self.last_yaw = None

        # State machine
        self.state  = 'TURN'
        self.active = True

        # ROS interfaces
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        # Always read /odometry/wheels first to pin the coordinate origin
        self._wheels_sub = self.create_subscription(
            Odometry, '/odometry/wheels', self._wheels_origin_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, 10)

        self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.create_timer(1.0,  self._log_status)    # 1 Hz status
        self.create_timer(3.0,  self._warn_if_no_odom)

        self.get_logger().info(
            f'BangWaypointFollower ready | {len(self.waypoints)} waypoints | '
            f'odom={self.odom_topic} | '
            f'linear={self.linear_speed} m/s | angular={self.angular_speed} rad/s'
        )

    # ------------------------------------------------------------------
    # Waypoint loading
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
    # Wheel odometry callback — used only to pin the coordinate origin
    # ------------------------------------------------------------------

    def _wheels_origin_cb(self, msg: Odometry):
        if self.wheels_origin is not None:
            return  # origin already set
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.wheels_origin = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              yaw)
        # Rotate waypoints from rover body frame (x=forward, y=left)
        # into the odom frame using the wheel odometry starting heading.
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        for wp in self.waypoints:
            bx, by = wp['x'], wp['y']
            wp['x'] = bx * cos_y - by * sin_y
            wp['y'] = bx * sin_y + by * cos_y
        self.get_logger().info(
            f'Wheels origin locked: ({self.wheels_origin[0]:.3f}, {self.wheels_origin[1]:.3f}) '
            f'yaw={math.degrees(yaw):.1f}° | waypoints rotated to rover nose direction'
        )

    # ------------------------------------------------------------------
    # Navigation odometry callback
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        if self.wheels_origin is None:
            return  # wait until wheel origin is set

        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        ox, oy, _ = self.wheels_origin
        self.x   = raw_x - ox
        self.y   = raw_y - oy
        self.yaw = yaw
        self.last_yaw  = yaw
        self.odom_ready = True

    # ------------------------------------------------------------------
    # Control loop (20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self):
        if not self.active or not self.odom_ready or not self.waypoints:
            return

        # All waypoints done
        if self.wp_idx >= len(self.waypoints):
            if self.loop:
                self.get_logger().info('Looping back to first waypoint')
                self.wp_idx = 0
            else:
                self.get_logger().info('All waypoints reached — stopping.')
                self._stop()
                self.active = False
                return

        wp     = self.waypoints[self.wp_idx]
        goal_x = wp['x']
        goal_y = wp['y']

        dx = goal_x - self.x
        dy = goal_y - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

        # ---- TURN phase --------------------------------
        if self.state == 'TURN':
            # Already on the waypoint — atan2 is undefined at dist≈0, just advance
            if dist < self.goal_tolerance:
                self.get_logger().info(
                    f'Reached WP {self.wp_idx + 1}/{len(self.waypoints)} during TURN '
                    f'(dist={dist:.2f}m) — advancing'
                )
                self._stop()
                self.wp_idx += 1
                self._enter_turn()
                return

            desired_yaw   = math.atan2(dy, dx)
            heading_error = angle_diff(desired_yaw, self.yaw)

            # Aligned — start driving
            if abs(heading_error) < self.heading_tolerance:
                self.get_logger().info(
                    f'Heading aligned ({math.degrees(heading_error):.1f}°) — driving'
                )
                self._enter_drive()
                return

            # Spin toward waypoint heading
            direction = 1.0 if heading_error > 0 else -1.0
            cmd.angular.z = direction * self.angular_speed
            self.cmd_pub.publish(cmd)

        # ---- DRIVE phase (move_5_feet style) ----------------------------
        elif self.state == 'DRIVE':
            if dist < self.goal_tolerance:
                self.get_logger().info(
                    f'Reached WP {self.wp_idx + 1}/{len(self.waypoints)} '
                    f'({goal_x:.2f}, {goal_y:.2f})'
                )
                self._stop()
                self.wp_idx += 1
                self._enter_turn()
                return

            # If heading has drifted too far, stop and re-align before continuing
            desired_yaw   = math.atan2(dy, dx)
            heading_error = angle_diff(desired_yaw, self.yaw)
            if abs(heading_error) > self.heading_tolerance:
                self.get_logger().info(
                    f'Heading drifted ({math.degrees(heading_error):.1f}°) — re-aligning'
                )
                self._stop()
                self._enter_turn()
                return

            cmd.linear.x = self.linear_speed
            self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _enter_turn(self):
        self.state = 'TURN'

    def _enter_drive(self):
        self._stop()
        self.state = 'DRIVE'

    # ------------------------------------------------------------------
    # Logging / stop
    # ------------------------------------------------------------------

    def _log_status(self):
        if not self.active or not self.odom_ready or self.wp_idx >= len(self.waypoints):
            return
        wp = self.waypoints[self.wp_idx]
        dx = wp['x'] - self.x
        dy = wp['y'] - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        desired_yaw   = math.atan2(dy, dx)
        heading_error = angle_diff(desired_yaw, self.yaw)
        self.get_logger().info(
            f'[{self.state}] WP {self.wp_idx + 1}/{len(self.waypoints)} | '
            f'dist={dist:.2f}m | heading_err={math.degrees(heading_error):.1f}° | '
            f'pos=({self.x:.2f}, {self.y:.2f}) | yaw={math.degrees(self.yaw):.1f}°'
        )

    def _warn_if_no_odom(self):
        if not self.odom_ready:
            if self.wheels_origin is None:
                self.get_logger().warn('Waiting for /odometry/wheels to set origin')
            else:
                self.get_logger().warn(
                    f'Wheels origin set but no messages on {self.odom_topic} yet'
                )

    def _stop(self):
        self.cmd_pub.publish(Twist())


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BangWaypointFollower()
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
