#!/usr/bin/env python3
"""
plot_imu_live.py — live plot of /imu angular velocity and linear acceleration vs time.

Run while the ROS stack is active:
    python3 ~/rover_workspace/plot_imu_live.py
"""

import threading
import collections

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt
import matplotlib.animation as animation

WINDOW_SECONDS = 30
MAX_POINTS = 3000   # 100 Hz * 30 s


class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_live_plotter')
        self.lock = threading.Lock()
        self.t0 = None

        self.t   = collections.deque(maxlen=MAX_POINTS)
        self.wx  = collections.deque(maxlen=MAX_POINTS)
        self.wy  = collections.deque(maxlen=MAX_POINTS)
        self.wz  = collections.deque(maxlen=MAX_POINTS)
        self.ax  = collections.deque(maxlen=MAX_POINTS)
        self.ay  = collections.deque(maxlen=MAX_POINTS)
        self.az  = collections.deque(maxlen=MAX_POINTS)

        self.create_subscription(Imu, '/imu', self._cb, 10)
        self.get_logger().info('Subscribed to /imu — waiting for data...')

    def _cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.t0 is None:
                self.t0 = stamp
            self.t.append(stamp - self.t0)
            self.wx.append(msg.angular_velocity.x)
            self.wy.append(msg.angular_velocity.y)
            self.wz.append(msg.angular_velocity.z)
            self.ax.append(msg.linear_acceleration.x)
            self.ay.append(msg.linear_acceleration.y)
            self.az.append(msg.linear_acceleration.z)

    def snapshot(self):
        with self.lock:
            return (list(self.t),
                    list(self.wx), list(self.wy), list(self.wz),
                    list(self.ax), list(self.ay), list(self.az))


def main():
    rclpy.init()
    node = ImuListener()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = plt.subplots(6, 1, figsize=(12, 11), sharex=True)
    fig.suptitle('Live /imu Data vs Time', fontsize=13)

    configs = [
        ('ω x — Roll rate  (rad/s)',  'tab:blue'),
        ('ω y — Pitch rate (rad/s)',  'tab:orange'),
        ('ω z — Yaw rate   (rad/s)',  'tab:red'),
        ('a x — Forward accel (m/s²)', 'tab:green'),
        ('a y — Lateral accel (m/s²)', 'tab:purple'),
        ('a z — Vertical accel (m/s²)','tab:brown'),
    ]

    lines = []
    for ax, (label, color) in zip(axes, configs):
        line, = ax.plot([], [], color=color, linewidth=0.8)
        ax.set_ylabel(label, fontsize=8)
        ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
        ax.grid(True, alpha=0.3)
        lines.append(line)

    axes[-1].set_xlabel('Time (s)')

    def update(_frame):
        snap = node.snapshot()
        t = snap[0]
        if not t:
            return lines

        channels = snap[1:]
        tmin = max(0.0, t[-1] - WINDOW_SECONDS)

        for line, data in zip(lines, channels):
            line.set_data(t, data)

        for ax, data in zip(axes, channels):
            if data:
                lo, hi = min(data), max(data)
                pad = max(abs(lo), abs(hi)) * 0.1 + 0.01
                ax.set_ylim(lo - pad, hi + pad)
            ax.set_xlim(tmin, t[-1] + 0.5)

        return lines

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
