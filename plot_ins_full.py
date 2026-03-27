#!/usr/bin/env python3
"""
plot_ins_full.py — live 9-panel plot of /ins (nav_msgs/Odometry) vs time.

Panels:
  Row 1 — Position delta:   Δx (North, m)  Δy (East, m)  Δz (Down, m)
  Row 2 — Linear velocity:  vx (m/s)       vy (m/s)      vz (m/s)
  Row 3 — Attitude:         Roll (deg)     Pitch (deg)   Yaw (deg)

Run while the ROS stack is active:
    python3 ~/rover_workspace/plot_ins_full.py
"""

import math
import threading
import collections

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import matplotlib.animation as animation

WINDOW_SECONDS = 60
MAX_POINTS     = 6000   # 100 Hz * 60 s


def quat_to_euler(x, y, z, w):
    """Convert quaternion to (roll, pitch, yaw) in degrees."""
    # Roll (x-axis)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    # Pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    # Yaw (z-axis)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))

    return roll, pitch, yaw


class InsListener(Node):
    def __init__(self):
        super().__init__('ins_full_plotter')
        self.lock   = threading.Lock()
        self.origin = None
        self.t0     = None

        keys = ['t', 'dx', 'dy', 'dz', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']
        self.data = {k: collections.deque(maxlen=MAX_POINTS) for k in keys}

        self.create_subscription(Odometry, '/ins', self._cb, 10)
        self.get_logger().info('Subscribed to /ins — waiting for data...')

    def _cb(self, msg):
        pos   = msg.pose.pose.position
        ori   = msg.pose.pose.orientation
        vel   = msg.twist.twist.linear
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        roll, pitch, yaw = quat_to_euler(ori.x, ori.y, ori.z, ori.w)

        with self.lock:
            if self.origin is None:
                self.origin = (pos.x, pos.y, pos.z)
                self.t0     = stamp
                self.get_logger().info(
                    f'Origin set — x={pos.x:.3f}  y={pos.y:.3f}  z={pos.z:.3f}')

            t = stamp - self.t0
            self.data['t'].append(t)
            self.data['dx'].append(pos.x - self.origin[0])
            self.data['dy'].append(pos.y - self.origin[1])
            self.data['dz'].append(pos.z - self.origin[2])
            self.data['vx'].append(vel.x)
            self.data['vy'].append(vel.y)
            self.data['vz'].append(vel.z)
            self.data['roll'].append(roll)
            self.data['pitch'].append(pitch)
            self.data['yaw'].append(yaw)

    def snapshot(self):
        with self.lock:
            return {k: list(v) for k, v in self.data.items()}


def main():
    rclpy.init()
    node = InsListener()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 3 rows x 3 columns
    fig, axes = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
    fig.suptitle('Live /ins Data vs Time  (nav_msgs/Odometry)', fontsize=13)

    #  (row, col)  channel_key   label                        color
    layout = [
        ((0, 0), 'dx',    'Δx — North  (m)',          'tab:blue'),
        ((0, 1), 'dy',    'Δy — East   (m)',          'tab:orange'),
        ((0, 2), 'dz',    'Δz — Down   (m)',          'tab:green'),
        ((1, 0), 'vx',    'vx — N vel  (m/s)',        'tab:red'),
        ((1, 1), 'vy',    'vy — E vel  (m/s)',        'tab:purple'),
        ((1, 2), 'vz',    'vz — D vel  (m/s)',        'tab:brown'),
        ((2, 0), 'roll',  'Roll   (deg)',              'tab:pink'),
        ((2, 1), 'pitch', 'Pitch  (deg)',              'tab:gray'),
        ((2, 2), 'yaw',   'Yaw    (deg)',              'tab:cyan'),
    ]

    lines   = {}
    ax_map  = {}
    ch_map  = {}

    for (r, c), key, label, color in layout:
        ax = axes[r][c]
        line, = ax.plot([], [], color=color, linewidth=0.9)
        ax.set_ylabel(label, fontsize=8)
        ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
        ax.grid(True, alpha=0.3)
        if r == 2:
            ax.set_xlabel('Time (s)')
        lines[key]  = line
        ax_map[key] = ax
        ch_map[key] = key

    def update(_frame):
        snap = node.snapshot()
        t = snap['t']
        if not t:
            return list(lines.values())

        tmin = max(0.0, t[-1] - WINDOW_SECONDS)

        for key, line in lines.items():
            d = snap[key]
            line.set_data(t, d)
            ax = ax_map[key]
            if d:
                lo, hi = min(d), max(d)
                pad = max(abs(lo), abs(hi)) * 0.1 + 0.01
                ax.set_ylim(lo - pad, hi + pad)
            ax.set_xlim(tmin, t[-1] + 0.5)

        return list(lines.values())

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
