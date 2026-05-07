#!/usr/bin/env python3
"""
record_imu_drift.py
====================
Records /imu and /imu_odom simultaneously, then plots whether position drift
is proportional to acceleration magnitude — diagnosing residual gravity leakage.

Usage:
    python3 ~/rover_workspace/record_imu_drift.py

  Keep the rover STATIONARY the whole time.
  Press Ctrl+C to stop recording and show the analysis plots.

What this measures
------------------
If drift ∝ acceleration the root cause is un-removed acceleration (residual
gravity from a bad frame transform or wrong gravity magnitude).

  drift_speed  = |velocity|   from /imu_odom twist
  accel_mag    = |a_corrected| which is inferred from d(velocity)/dt

Four plots are produced:

  1. Position x, y vs time  — shows the raw drift trajectory
  2. Speed (|vel|) vs time   — how fast the position is drifting
  3. Raw acc magnitude from /imu vs time  — total specific force
  4. Scatter: speed vs raw acc magnitude  — if linear → drift ∝ accel
     A fitted regression line and R² are shown.
"""

import math
import threading
import collections
import signal
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ---------------------------------------------------------------------------
# Tuneable constants
# ---------------------------------------------------------------------------
MAX_POINTS   = 30_000    # ~5 min at 100 Hz
ALIGN_WINDOW = 0.02      # seconds — match /imu and /imu_odom timestamps within this


# ---------------------------------------------------------------------------
# ROS2 listener node
# ---------------------------------------------------------------------------

class DriftRecorder(Node):
    def __init__(self):
        super().__init__('imu_drift_recorder')
        self.lock = threading.Lock()
        self.t0   = None

        # /imu raw samples
        self.imu_t   = collections.deque(maxlen=MAX_POINTS)
        self.imu_ax  = collections.deque(maxlen=MAX_POINTS)
        self.imu_ay  = collections.deque(maxlen=MAX_POINTS)
        self.imu_az  = collections.deque(maxlen=MAX_POINTS)
        self.imu_wx  = collections.deque(maxlen=MAX_POINTS)
        self.imu_wy  = collections.deque(maxlen=MAX_POINTS)
        self.imu_wz  = collections.deque(maxlen=MAX_POINTS)

        # /imu_odom samples
        self.odom_t  = collections.deque(maxlen=MAX_POINTS)
        self.odom_x  = collections.deque(maxlen=MAX_POINTS)
        self.odom_y  = collections.deque(maxlen=MAX_POINTS)
        self.odom_z  = collections.deque(maxlen=MAX_POINTS)
        self.odom_vx = collections.deque(maxlen=MAX_POINTS)
        self.odom_vy = collections.deque(maxlen=MAX_POINTS)
        self.odom_vz = collections.deque(maxlen=MAX_POINTS)

        self.create_subscription(Imu,      '/imu',      self._imu_cb,  50)
        self.create_subscription(Odometry, '/imu_odom', self._odom_cb, 50)
        self.get_logger().info(
            'Recording /imu + /imu_odom — keep rover STATIONARY\n'
            'Press Ctrl+C to stop and show analysis.'
        )

    def _stamp(self, msg):
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        with self.lock:
            if self.t0 is None:
                self.t0 = t
        return t - self.t0

    def _imu_cb(self, msg):
        t = self._stamp(msg)
        with self.lock:
            self.imu_t.append(t)
            self.imu_ax.append(msg.linear_acceleration.x)
            self.imu_ay.append(msg.linear_acceleration.y)
            self.imu_az.append(msg.linear_acceleration.z)
            self.imu_wx.append(msg.angular_velocity.x)
            self.imu_wy.append(msg.angular_velocity.y)
            self.imu_wz.append(msg.angular_velocity.z)

    def _odom_cb(self, msg):
        t = self._stamp(msg)
        with self.lock:
            self.odom_t.append(t)
            self.odom_x.append(msg.pose.pose.position.x)
            self.odom_y.append(msg.pose.pose.position.y)
            self.odom_z.append(msg.pose.pose.position.z)
            self.odom_vx.append(msg.twist.twist.linear.x)
            self.odom_vy.append(msg.twist.twist.linear.y)
            self.odom_vz.append(msg.twist.twist.linear.z)

    def snapshot(self):
        with self.lock:
            return {
                'imu_t':   np.array(self.imu_t),
                'imu_ax':  np.array(self.imu_ax),
                'imu_ay':  np.array(self.imu_ay),
                'imu_az':  np.array(self.imu_az),
                'imu_wx':  np.array(self.imu_wx),
                'imu_wy':  np.array(self.imu_wy),
                'imu_wz':  np.array(self.imu_wz),
                'odom_t':  np.array(self.odom_t),
                'odom_x':  np.array(self.odom_x),
                'odom_y':  np.array(self.odom_y),
                'odom_z':  np.array(self.odom_z),
                'odom_vx': np.array(self.odom_vx),
                'odom_vy': np.array(self.odom_vy),
                'odom_vz': np.array(self.odom_vz),
            }


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def analyse_and_plot(data):
    imu_t  = data['imu_t']
    odom_t = data['odom_t']

    if len(imu_t) < 10 or len(odom_t) < 10:
        print('Not enough data collected to analyse.')
        return

    # ---- Raw IMU magnitudes ------------------------------------------------
    acc_mag  = np.sqrt(data['imu_ax']**2 + data['imu_ay']**2 + data['imu_az']**2)
    omega_mag = np.sqrt(data['imu_wx']**2 + data['imu_wy']**2 + data['imu_wz']**2)

    # Estimated gravity: median of acc_mag (robust to motion spikes)
    g_est = np.median(acc_mag)

    # Gravity-corrected acceleration magnitude (should be ~0 when stationary)
    # We can't do the full rotation here, but we can look at the residual
    # total magnitude after subtracting g_est from the z-component (after imu_to_enu)
    # imu_to_enu: x→-x, y→+y, z→-z
    ax_enu = -data['imu_ax']
    ay_enu =  data['imu_ay']
    az_enu = -data['imu_az']
    # Subtract estimated gravity from z (approximate — ignores attitude)
    az_corr = az_enu - g_est
    acc_corr_mag = np.sqrt(ax_enu**2 + ay_enu**2 + az_corr**2)

    # ---- Drift speed from /imu_odom twist ----------------------------------
    drift_speed = np.sqrt(data['odom_vx']**2 + data['odom_vy']**2 + data['odom_vz']**2)

    # ---- Position drift distance from origin --------------------------------
    drift_dist = np.sqrt(data['odom_x']**2 + data['odom_y']**2)

    # ---- Interpolate acc_corr_mag onto odom timestamps for scatter ----------
    # Only keep timestamps where both signals overlap
    t_min = max(imu_t[0],  odom_t[0])
    t_max = min(imu_t[-1], odom_t[-1])
    mask_imu  = (imu_t  >= t_min) & (imu_t  <= t_max)
    mask_odom = (odom_t >= t_min) & (odom_t <= t_max)

    imu_t_common   = imu_t[mask_imu]
    acc_corr_common = acc_corr_mag[mask_imu]
    odom_t_common  = odom_t[mask_odom]
    speed_common   = drift_speed[mask_odom]

    # Resample acc onto odom timestamps
    if len(imu_t_common) > 1 and len(odom_t_common) > 1:
        acc_at_odom = np.interp(odom_t_common, imu_t_common, acc_corr_common)
    else:
        acc_at_odom = np.zeros_like(speed_common)

    # ---- Linear regression: speed ~ m * acc + c ----------------------------
    if len(acc_at_odom) > 2:
        coeffs = np.polyfit(acc_at_odom, speed_common, 1)
        slope, intercept = coeffs
        fit_x = np.linspace(acc_at_odom.min(), acc_at_odom.max(), 100)
        fit_y = slope * fit_x + intercept
        # R² correlation
        residuals  = speed_common - np.polyval(coeffs, acc_at_odom)
        ss_res     = np.sum(residuals**2)
        ss_tot     = np.sum((speed_common - speed_common.mean())**2)
        r_squared  = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
    else:
        fit_x = fit_y = np.array([])
        slope = intercept = r_squared = 0.0

    # ---- Print summary ------------------------------------------------------
    duration = odom_t[-1] - odom_t[0] if len(odom_t) > 1 else 0
    print('\n' + '='*60)
    print('IMU DRIFT ANALYSIS SUMMARY')
    print('='*60)
    print(f'  Recording duration  : {duration:.1f} s')
    print(f'  IMU samples         : {len(imu_t)}')
    print(f'  Odom samples        : {len(odom_t)}')
    print(f'  Estimated gravity   : {g_est:.5f} m/s²')
    print(f'  Max position drift  : {drift_dist.max():.4f} m')
    print(f'  Max drift speed     : {drift_speed.max():.4f} m/s')
    print(f'  Mean acc_corr mag   : {acc_corr_mag.mean():.4f} m/s²')
    print(f'  Max  acc_corr mag   : {acc_corr_mag.max():.4f} m/s²')
    print()
    print('  Regression: drift_speed = {:.4f} * acc_corr + {:.4f}'.format(slope, intercept))
    print(f'  R²                  : {r_squared:.4f}')
    if r_squared > 0.7:
        print('  VERDICT: HIGH correlation — drift IS proportional to acceleration.')
        print('           Root cause: residual un-removed gravity component.')
        print('           Fix: improve frame transform or gravity calibration.')
    elif r_squared > 0.3:
        print('  VERDICT: MODERATE correlation — drift partially tracks acceleration.')
        print('           Likely a mix of residual gravity AND sensor bias drift.')
    else:
        print('  VERDICT: LOW correlation — drift does NOT scale with acceleration.')
        print('           Root cause is sensor bias / gyro drift, not gravity leakage.')
    print('='*60 + '\n')

    # ---- Plot ---------------------------------------------------------------
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('IMU Drift vs Acceleration Analysis', fontsize=13, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # 1. Position drift trajectory (XY)
    ax_xy = fig.add_subplot(gs[0, 0])
    sc = ax_xy.scatter(data['odom_x'], data['odom_y'],
                       c=odom_t - odom_t[0], cmap='viridis', s=2)
    ax_xy.set_xlabel('x drift (m)')
    ax_xy.set_ylabel('y drift (m)')
    ax_xy.set_title('Position drift trajectory\n(colour = time, start = origin)')
    ax_xy.axhline(0, color='gray', lw=0.5, ls='--')
    ax_xy.axvline(0, color='gray', lw=0.5, ls='--')
    ax_xy.set_aspect('equal', adjustable='datalim')
    plt.colorbar(sc, ax=ax_xy, label='time (s)', fraction=0.046, pad=0.04)

    # 2. Position drift distance vs time
    ax_dist = fig.add_subplot(gs[0, 1])
    ax_dist.plot(odom_t - odom_t[0], drift_dist, color='tab:blue', lw=0.8)
    ax_dist.set_xlabel('Time (s)')
    ax_dist.set_ylabel('|drift| (m)')
    ax_dist.set_title('Total position drift distance vs time')
    ax_dist.grid(True, alpha=0.3)

    # 3. Drift speed vs time
    ax_spd = fig.add_subplot(gs[1, 0])
    ax_spd.plot(odom_t - odom_t[0], drift_speed, color='tab:orange', lw=0.8)
    ax_spd.set_xlabel('Time (s)')
    ax_spd.set_ylabel('|velocity| (m/s)')
    ax_spd.set_title('Drift speed vs time  (from /imu_odom twist)')
    ax_spd.grid(True, alpha=0.3)

    # 4. Corrected acceleration magnitude vs time
    ax_acc = fig.add_subplot(gs[1, 1])
    ax_acc.plot(imu_t - imu_t[0], acc_corr_mag, color='tab:red', lw=0.6, alpha=0.7,
                label='|a_corr|')
    ax_acc.axhline(0.25, color='gray', lw=0.8, ls='--', label='ZUPT threshold 0.25')
    ax_acc.set_xlabel('Time (s)')
    ax_acc.set_ylabel('|a_corrected| (m/s²)')
    ax_acc.set_title('Gravity-corrected accel magnitude vs time\n(should be ~0 when stationary)')
    ax_acc.legend(fontsize=8)
    ax_acc.grid(True, alpha=0.3)

    # 5. Raw IMU axes vs time (stacked)
    ax_raw = fig.add_subplot(gs[2, 0])
    ax_raw.plot(imu_t - imu_t[0], ax_enu, lw=0.6, alpha=0.8, label='x (ENU)', color='tab:blue')
    ax_raw.plot(imu_t - imu_t[0], ay_enu, lw=0.6, alpha=0.8, label='y (ENU)', color='tab:orange')
    ax_raw.plot(imu_t - imu_t[0], az_corr, lw=0.6, alpha=0.8, label='z − g (ENU)', color='tab:green')
    ax_raw.axhline(0, color='gray', lw=0.5, ls='--')
    ax_raw.set_xlabel('Time (s)')
    ax_raw.set_ylabel('Accel (m/s²)')
    ax_raw.set_title('Per-axis corrected acceleration (ENU body)')
    ax_raw.legend(fontsize=8)
    ax_raw.grid(True, alpha=0.3)

    # 6. Scatter: drift speed vs acc_corr_mag  ← the key proportionality test
    ax_scat = fig.add_subplot(gs[2, 1])
    ax_scat.scatter(acc_at_odom, speed_common, s=2, alpha=0.4, color='tab:purple',
                    label='samples')
    if len(fit_x) > 0:
        ax_scat.plot(fit_x, fit_y, color='black', lw=1.5,
                     label=f'fit  slope={slope:.3f}  R²={r_squared:.3f}')
    ax_scat.set_xlabel('|a_corrected| (m/s²)  from /imu')
    ax_scat.set_ylabel('drift speed (m/s)  from /imu_odom')
    ax_scat.set_title('Proportionality test\n(linear → drift caused by residual acceleration)')
    ax_scat.legend(fontsize=8)
    ax_scat.grid(True, alpha=0.3)

    plt.savefig('/home/lanceholden/rover_workspace/imu_drift_analysis.png', dpi=150,
                bbox_inches='tight')
    print('Plot saved to ~/rover_workspace/imu_drift_analysis.png')
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = DriftRecorder()

    stop_event = threading.Event()

    def handle_sigint(sig, frame):
        print('\nStopped recording. Running analysis...')
        stop_event.set()

    signal.signal(signal.SIGINT, handle_sigint)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('Recording... Press Ctrl+C to stop.')
    stop_event.wait()

    data = node.snapshot()
    node.destroy_node()
    rclpy.shutdown()

    analyse_and_plot(data)


if __name__ == '__main__':
    main()
