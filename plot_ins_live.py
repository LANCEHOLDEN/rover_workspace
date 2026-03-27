#!/usr/bin/env python3
"""
plot_ins_live.py — live plot of /ins x, y, z position change vs time.

Run while the ROS stack is active:
    python3 ~/rover_workspace/plot_ins_live.py
"""

import math
import threading
import collections

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import matplotlib.animation as animation

WINDOW_SECONDS = 60     # how many seconds of history to show
MAX_POINTS = 6000       # max samples kept (100 Hz * 60 s)


class InsListener(Node):
    def __init__(self):
        super().__init__('ins_live_plotter')
        self.lock = threading.Lock()
        self.t  = collections.deque(maxlen=MAX_POINTS)
        self.dx = collections.deque(maxlen=MAX_POINTS)
        self.dy = collections.deque(maxlen=MAX_POINTS)
        self.dz = collections.deque(maxlen=MAX_POINTS)
        self.origin = None
        self.t0 = None

        self.create_subscription(Odometry, '/ins', self._cb, 10)
        self.get_logger().info('Subscribed to /ins — waiting for data...')

    def _cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self.lock:
            if self.origin is None:
                self.origin = (x, y, z)
                self.t0 = stamp
                self.get_logger().info(f'Origin set: x={x:.3f} y={y:.3f} z={z:.3f}')

            self.t.append(stamp - self.t0)
            self.dx.append(x - self.origin[0])
            self.dy.append(y - self.origin[1])
            self.dz.append(z - self.origin[2])

    def snapshot(self):
        with self.lock:
            return list(self.t), list(self.dx), list(self.dy), list(self.dz)


def main():
    rclpy.init()
    node = InsListener()

    # Spin ROS in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    fig.suptitle('Live /ins Position Change vs Time', fontsize=13)

    colors = ['tab:blue', 'tab:orange', 'tab:green']
    labels = ['Δx — Forward/Back (NED North, m)',
              'Δy — Left/Right   (NED East, m)',
              'Δz — Vertical     (NED Down, m)']
    lines = []
    for ax, color, label in zip(axes, colors, labels):
        line, = ax.plot([], [], color=color, linewidth=0.9)
        ax.set_ylabel(label, fontsize=9)
        ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
        ax.grid(True, alpha=0.3)
        lines.append(line)

    axes[2].set_xlabel('Time (s)')

    def update(_frame):
        t, dx, dy, dz = node.snapshot()
        if not t:
            return lines

        tmin = max(0.0, t[-1] - WINDOW_SECONDS)
        for line, data in zip(lines, [dx, dy, dz]):
            line.set_data(t, data)

        for ax, data in zip(axes, [dx, dy, dz]):
            if data:
                pad = max(abs(min(data)), abs(max(data))) * 0.1 + 0.01
                ax.set_ylim(min(data) - pad, max(data) + pad)
            ax.set_xlim(tmin, t[-1] + 0.5)

        return lines

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
