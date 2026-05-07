#!/usr/bin/env python3
"""
log_waypoints_odom.py
----------------------
Logs waypoints from an odometry topic for use with bang_waypoint_follower.

All positions are stored relative to the starting position (origin = first
odometry message received), matching exactly how bang_waypoint_follower
interprets waypoints.

Usage:
  python3 log_waypoints_odom.py                          # uses /odometry/global
  python3 log_waypoints_odom.py -o my_waypoints.yaml
  python3 log_waypoints_odom.py --topic /odometry/local

Controls:
  SPACE   -> log current position as a waypoint
  q       -> save and quit
  Ctrl+C  -> save and quit
"""

import sys
import math
import argparse
import threading
import yaml
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomWaypointLogger(Node):
    def __init__(self, output_file, topic):
        super().__init__('odom_waypoint_logger')
        self.output_file = output_file
        self.topic       = topic
        self.waypoints   = []
        self._lock       = threading.Lock()

        self.origin_x  = None
        self.origin_y  = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.ready = False

        self.sub = self.create_subscription(
            Odometry, topic, self._odom_cb, 10)

    def _odom_cb(self, msg):
        with self._lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            if self.origin_x is None:
                self.origin_x = x
                self.origin_y = y
                self.ready = True
                print(f'\n[HOME] Origin locked at ({x:.3f}, {y:.3f}) from {self.topic}')
                print('[READY] Press SPACE to log waypoints\n')

            self.current_x   = x - self.origin_x
            self.current_y   = y - self.origin_y
            self.current_yaw = yaw

    def log_waypoint(self):
        with self._lock:
            if not self.ready:
                print('[WARN] No odometry received yet — is /odometry/wheels publishing?')
                return
            x   = self.current_x
            y   = self.current_y
            yaw = self.current_yaw

        waypoint = {
            'x':   round(x, 4),
            'y':   round(y, 4),
            'yaw': round(yaw, 4),
        }
        self.waypoints.append(waypoint)
        n = len(self.waypoints)
        print(f'  [WP {n:3d}]  x={x:+8.3f} m  y={y:+8.3f} m  yaw={math.degrees(yaw):+6.1f}°')

    def save(self):
        if not self.waypoints:
            print('\n[INFO] No waypoints logged — nothing to save.')
            return

        frame = 'map' if 'global' in self.topic else 'odom'
        output_data = {
            'frame_id':  frame,
            'waypoints': self.waypoints,
        }

        with open(self.output_file, 'w') as f:
            yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)

        print(f'\n[INFO] Saved {len(self.waypoints)} waypoint(s) -> {self.output_file}')


def keyboard_loop(node, stop_event):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not stop_event.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            ch = sys.stdin.read(1)
            if ch == ' ':
                node.log_waypoint()
            elif ch in ('q', 'Q', '\x03'):
                stop_event.set()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    parser = argparse.ArgumentParser(
        description='Log odometry waypoints for bang_waypoint_follower'
    )
    parser.add_argument(
        '-o', '--output', default='logged_waypoints.yaml',
        help='Output YAML file (default: logged_waypoints.yaml)',
    )
    parser.add_argument(
        '--topic', default='/odometry/global',
        help='Odometry topic to log from (default: /odometry/global)',
    )
    args = parser.parse_args()

    rclpy.init()
    node = OdomWaypointLogger(args.output, args.topic)

    stop_event = threading.Event()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('=' * 60)
    print('  Odometry Waypoint Logger')
    print(f'  Output file : {args.output}')

    print(f'  Topic       : {args.topic}')
    print()
    print('  Waiting for first odometry message...')
    print('  (make sure GPS is locked before logging waypoints)')
    print()
    print('  SPACE  ->  log current position')
    print('  q      ->  save and quit')
    print('  Ctrl+C ->  save and quit')
    print('=' * 60)

    try:
        keyboard_loop(node, stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
