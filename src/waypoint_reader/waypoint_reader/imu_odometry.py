#!/usr/bin/env python3
"""
imu_odometry.py
===============
Integrates raw IMU data (sensor_msgs/Imu) into a nav_msgs/Odometry stream
for side-by-side visualisation in RViz against wheel odometry and the
hardware INS EKF output.

  Subscribes : /imu        (sensor_msgs/msg/Imu)
  Publishes  : /imu_odom   (nav_msgs/msg/Odometry)   frame: odom → imu_base_link
  Broadcasts : TF          odom → imu_base_link

Coordinate frame alignment
--------------------------
The InertialSense driver publishes IMU data in the sensor's NED body frame:
    IMU +x = North   (sensor forward)
    IMU +y = East    (sensor right)
    IMU +z = Down    (into ground)

The rover wheel odometry lives in ENU:
    Wheel +x = East  (rover East)
    Wheel +y = North (rover North)
    Wheel +z = Up

Additionally, the sensor is physically mounted such that:
    IMU +x  faces  wheel odom −x   (sensor is backward on the rover)
    IMU +z  faces  wheel odom −z   (sensor z-down = ground, ENU z-up = sky)

This is a 180° rotation about the rover Y-axis:
    enu_x = −imu_x
    enu_y = +imu_y
    enu_z = −imu_z

Startup gravity calibration
-----------------------------
On startup the node collects CALIB_SAMPLES IMU readings (assuming the rover
is stationary) to measure the actual gravity vector in the sensor body frame.
This handles two real-world problems that cause fast backward drift:

  1. Gravity magnitude — the sensor may read ~9.97 m/s² instead of the
     theoretical 9.80665, leaving a residual that prevents ZUPT from firing.
  2. Sensor tilt — a few degrees of tilt projects gravity into the X/Y body
     axes.  If uncorrected, even a 1° tilt produces ~0.17 m/s² of false
     acceleration that integrates into metres of drift in seconds.

The calibration measures the mean gravity vector, then computes the
quaternion that rotates it to world +Z.  Integration starts from that
initial orientation so gravity cancels exactly.

Drift mitigation
-----------------
1. Startup gravity calibration (described above)
2. ZUPT — when both |ω| < zupt_gyro_thresh AND |a_corr| < zupt_accel_thresh
   the rover is assumed stationary and velocity is zeroed.

Parameters
----------
imu_topic          (str)   default: /imu
odom_topic         (str)   default: /imu_odom
child_frame_id     (str)   default: imu_base_link
calib_samples      (int)   default: 100   — samples averaged for gravity cal
zupt_gyro_thresh   (float) default: 0.02  rad/s
zupt_accel_thresh  (float) default: 0.25  m/s²  (raised from 0.15 to tolerate
                                                  sensor noise after calibration)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ---------------------------------------------------------------------------
# Frame transform:  IMU NED (mounted backward/flipped) → ENU body
# ---------------------------------------------------------------------------

def imu_to_enu(x, y, z):
    """Remap from InertialSense IMU frame to ENU wheel-odom frame.

    Mounting: IMU +x → −ENU_x  (backward),  IMU +z → −ENU_z  (z-down → z-up).
    Right-hand rule forces IMU +y → +ENU_y.
    Equivalent to 180° rotation about the Y-axis (det = +1, proper rotation).
    """
    return (-x, y, -z)


# ---------------------------------------------------------------------------
# Quaternion helpers
# ---------------------------------------------------------------------------

def _quat_mult(q1, q2):
    """Hamilton product q1 ⊗ q2.  Both as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def _quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def _quat_from_two_vectors(a, b):
    """Shortest-arc quaternion (x,y,z,w) that rotates unit vector a onto unit vector b."""
    ax, ay, az = a
    bx, by, bz = b
    dot = ax*bx + ay*by + az*bz
    # cross product a × b  →  rotation axis scaled by sin(θ)
    cx = ay*bz - az*by
    cy = az*bx - ax*bz
    cz = ax*by - ay*bx
    # Using the half-angle identity: w = 1 + cos(θ),  xyz = cross(a,b)
    # then normalise.  Degenerates when a ≈ −b (handled below).
    w = 1.0 + dot
    norm = math.sqrt(cx*cx + cy*cy + cz*cz + w*w)
    if norm < 1e-10:
        if dot > 0.0:
            return (0.0, 0.0, 0.0, 1.0)   # same direction → identity
        # Opposite direction → 180° about any perpendicular axis
        if abs(ax) < 0.9:
            px, py, pz = 1.0 - ax*ax, -ax*ay, -ax*az
        else:
            px, py, pz = -ay*ax, 1.0 - ay*ay, -ay*az
        pn = math.sqrt(px*px + py*py + pz*pz)
        return (px/pn, py/pn, pz/pn, 0.0)
    return (cx/norm, cy/norm, cz/norm, w/norm)


def _integrate_quat(q, omega, dt):
    """Rotate q by angular velocity omega (rad/s) over dt using exact angle-axis."""
    wx, wy, wz = omega
    rate = math.sqrt(wx*wx + wy*wy + wz*wz)
    theta = rate * dt
    if theta < 1e-12:
        return q
    half = theta / 2.0
    s = math.sin(half) / rate
    dq = (wx*s, wy*s, wz*s, math.cos(half))
    return _quat_normalize(_quat_mult(q, dq))


def _rotate_vec(q, v):
    """Rotate 3-vector v by unit quaternion q=(x,y,z,w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    wx = (1.0 - 2.0*(qy*qy + qz*qz))*vx + 2.0*(qx*qy - qw*qz)*vy + 2.0*(qx*qz + qw*qy)*vz
    wy = 2.0*(qx*qy + qw*qz)*vx + (1.0 - 2.0*(qx*qx + qz*qz))*vy + 2.0*(qy*qz - qw*qx)*vz
    wz = 2.0*(qx*qz - qw*qy)*vx + (1.0 - 2.0*(qx*qx + qy*qy))*vz + 2.0*(qy*qz + qw*qx)*vy
    return wx, wy, wz


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ImuOdometry(Node):

    CALIB_OMEGA_THRESH = 0.05   # rad/s — reject calibration sample if moving

    def __init__(self):
        super().__init__('imu_odometry')

        self.declare_parameter('imu_topic',         '/imu')
        self.declare_parameter('odom_topic',        '/imu_odom')
        self.declare_parameter('child_frame_id',    'imu_base_link')
        self.declare_parameter('calib_samples',     100)
        self.declare_parameter('zupt_gyro_thresh',  0.02)   # rad/s
        self.declare_parameter('zupt_accel_thresh', 0.25)   # m/s²  — raised from 0.15

        imu_topic           = self.get_parameter('imu_topic').value
        odom_topic          = self.get_parameter('odom_topic').value
        self.child_fid      = self.get_parameter('child_frame_id').value
        self.calib_samples  = self.get_parameter('calib_samples').value
        self.zupt_gyro      = self.get_parameter('zupt_gyro_thresh').value
        self.zupt_accel     = self.get_parameter('zupt_accel_thresh').value

        # Gravity state — overwritten after calibration
        self.gravity   = 9.80665    # magnitude (m/s²), replaced by measured value
        self.g_world   = (0.0, 0.0, 9.80665)  # world-frame gravity vector

        # Calibration buffer
        self._calib_buf  = []       # list of (acc_enu, omega_enu) tuples
        self._calibrated = False

        # Integration state
        self.pos    = [0.0, 0.0, 0.0]
        self.vel    = [0.0, 0.0, 0.0]
        self.q      = (0.0, 0.0, 0.0, 1.0)   # identity
        self.last_t = None

        # Diagnostics
        self._zupt_count = 0
        self._msg_count  = 0

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 50)
        self.tf_br    = TransformBroadcaster(self)
        self.imu_sub  = self.create_subscription(Imu, imu_topic, self._imu_cb, 50)

        self.create_timer(5.0, self._log_status)
        # Publish identity TF immediately so RViz message filter doesn't queue-fill
        # while waiting for the calibration phase to complete.
        self._identity_tf_timer = self.create_timer(0.1, self._publish_identity_tf_once)

        self.get_logger().info(
            f'imu_odometry started — collecting {self.calib_samples} samples '
            f'for gravity calibration (keep rover stationary)\n'
            f'  {imu_topic} → {odom_topic}  (TF: odom → {self.child_fid})\n'
            f'  ZUPT: |ω| < {self.zupt_gyro} rad/s  AND  |a| < {self.zupt_accel} m/s²'
        )

    # ------------------------------------------------------------------
    # One-shot identity TF to prevent RViz message-filter queue overflow
    # ------------------------------------------------------------------

    def _publish_identity_tf_once(self):
        tf = TransformStamped()
        tf.header.stamp    = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = self.child_fid
        tf.transform.rotation.w = 1.0
        self.tf_br.sendTransform(tf)
        self.destroy_timer(self._identity_tf_timer)

    # ------------------------------------------------------------------
    # Startup gravity calibration
    # ------------------------------------------------------------------

    def _try_calibrate(self, acc_enu, omega_enu):
        """Collect samples while stationary; compute gravity vector when ready.

        Returns True once calibration is complete.
        """
        ox, oy, oz = omega_enu
        omega_mag = math.sqrt(ox*ox + oy*oy + oz*oz)
        if omega_mag > self.CALIB_OMEGA_THRESH:
            # Rover is moving — discard buffer and wait for a still period
            if self._calib_buf:
                self.get_logger().warn(
                    f'Rover moved during gravity calibration '
                    f'(|ω|={omega_mag:.3f}), restarting collection.'
                )
                self._calib_buf = []
            return False

        self._calib_buf.append(acc_enu)

        if len(self._calib_buf) < self.calib_samples:
            return False

        # Compute mean gravity vector in ENU body frame
        n   = len(self._calib_buf)
        gx  = sum(a[0] for a in self._calib_buf) / n
        gy  = sum(a[1] for a in self._calib_buf) / n
        gz  = sum(a[2] for a in self._calib_buf) / n
        mag = math.sqrt(gx*gx + gy*gy + gz*gz)

        if mag < 1.0:
            self.get_logger().error(
                f'Gravity calibration failed: measured magnitude {mag:.3f} m/s² '
                f'is too small.  Check sensor connection.'
            )
            self._calib_buf = []
            return False

        self.gravity = mag
        self.g_world = (0.0, 0.0, mag)

        # Initial orientation: quaternion that rotates the measured gravity
        # vector to world +Z (straight up).  This corrects for sensor tilt so
        # gravity cancels exactly on the first integration step.
        g_unit = (gx/mag, gy/mag, gz/mag)
        self.q = _quat_from_two_vectors(g_unit, (0.0, 0.0, 1.0))

        self._calibrated = True
        self.get_logger().info(
            f'Gravity calibration complete:\n'
            f'  measured g body = ({gx:.4f}, {gy:.4f}, {gz:.4f}) m/s²\n'
            f'  magnitude       = {mag:.5f} m/s²  '
            f'(theoretical 9.80665, diff={mag-9.80665:+.4f})\n'
            f'  tilt correction = ({math.degrees(math.asin(max(-1,min(1,gx/mag)))):.2f}°x, '
            f'{math.degrees(math.asin(max(-1,min(1,gy/mag)))):.2f}°y)\n'
            f'  Integration starting — gravity will now cancel cleanly.'
        )
        return True

    # ------------------------------------------------------------------
    # IMU callback
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        stamp = msg.header.stamp
        t     = stamp.sec + stamp.nanosec * 1e-9

        if self.last_t is None:
            self.last_t = t
            return

        dt = t - self.last_t
        self.last_t = t
        self._msg_count += 1

        if dt <= 0.0 or dt > 0.5:
            return

        # ---- Remap from IMU frame (NED, backward-mounted) to ENU ------------
        wx, wy, wz = imu_to_enu(
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        ax, ay, az = imu_to_enu(
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )

        # ---- Startup calibration (first CALIB_SAMPLES stationary frames) ----
        if not self._calibrated:
            if not self._try_calibrate((ax, ay, az), (wx, wy, wz)):
                return   # still collecting — don't integrate yet
            # q was set to the tilt-correcting quaternion by _try_calibrate

        # ---- Attitude integration -------------------------------------------
        self.q = _integrate_quat(self.q, (wx, wy, wz), dt)

        # ---- Rotate body acceleration to world frame ------------------------
        ax_w, ay_w, az_w = _rotate_vec(self.q, (ax, ay, az))

        # ---- Gravity removal ------------------------------------------------
        # Subtract the world-frame gravity vector measured at startup.
        # Using the calibrated magnitude (not 9.80665) eliminates the residual
        # that was preventing ZUPT from firing.
        gx_w, gy_w, gz_w = self.g_world
        ax_w -= gx_w
        ay_w -= gy_w
        az_w -= gz_w

        # ---- ZUPT -----------------------------------------------------------
        omega_mag = math.sqrt(wx*wx + wy*wy + wz*wz)
        accel_mag = math.sqrt(ax_w*ax_w + ay_w*ay_w + az_w*az_w)
        is_static = omega_mag < self.zupt_gyro and accel_mag < self.zupt_accel

        if is_static:
            self.vel[0] = 0.0
            self.vel[1] = 0.0
            self.vel[2] = 0.0
            self._zupt_count += 1
        else:
            self.vel[0] += ax_w * dt
            self.vel[1] += ay_w * dt
            self.vel[2] += az_w * dt

        # ---- Integrate velocity → position ----------------------------------
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt
        self.pos[2] += self.vel[2] * dt

        self._publish(stamp)

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _publish(self, stamp):
        qx, qy, qz, qw = self.q

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = self.child_fid
        odom.pose.pose.position.x    = self.pos[0]
        odom.pose.pose.position.y    = self.pos[1]
        odom.pose.pose.position.z    = self.pos[2]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = self.vel[0]
        odom.twist.twist.linear.y    = self.vel[1]
        odom.twist.twist.linear.z    = self.vel[2]
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = self.child_fid
        tf.transform.translation.x = self.pos[0]
        tf.transform.translation.y = self.pos[1]
        tf.transform.translation.z = self.pos[2]
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

    # ------------------------------------------------------------------
    # Status log every 5 s
    # ------------------------------------------------------------------

    def _log_status(self):
        if not self._calibrated:
            self.get_logger().info(
                f'Gravity calibration: {len(self._calib_buf)}/{self.calib_samples} samples'
            )
            return
        if self._msg_count == 0:
            self.get_logger().warn('No /imu messages in last 5 s')
            return

        qx, qy, qz, qw = self.q
        yaw_deg = math.degrees(math.atan2(
            2.0*(qw*qz + qx*qy),
            1.0 - 2.0*(qy*qy + qz*qz)
        ))
        self.get_logger().info(
            f'pos=({self.pos[0]:.3f}, {self.pos[1]:.3f}, {self.pos[2]:.3f}) m  '
            f'vel=({self.vel[0]:.3f}, {self.vel[1]:.3f}) m/s  '
            f'yaw={yaw_deg:.1f}°  zupt={self._zupt_count}  '
            f'g={self.gravity:.4f} m/s²  msgs={self._msg_count}'
        )
        self._msg_count = 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
