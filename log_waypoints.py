#!/usr/bin/env python3
"""
Waypoint Logger - subscribe to /ins (ECEF Odometry) and /gps topics.
Press SPACE to log the current position as a waypoint.
All waypoints are localized to a home point captured at startup.

Topics:
  /ins  -> nav_msgs/msg/Odometry       (ECEF x, y, z, yaw)
  /gps  -> inertial_sense_ros2/msg/GPS (latitude, longitude, altitude)

Coordinate frames saved per waypoint:
  x, y, z, yaw  ->  ECEF delta from home_ins  (compatible with waypoint_follower)
  east, north, up -> GPS-derived local ENU from home_gps (proper ECEF->ENU rotation)

Usage:
  source /home/lanceholden/rover_workspace/install/setup.bash
  python3 log_waypoints.py
  python3 log_waypoints.py -o my_waypoints.yaml

Controls:
  SPACE   -> log current waypoint
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

try:
    from inertial_sense_ros2.msg import GPS as GPSMsg
    HAS_GPS_MSG = True
except ImportError:
    HAS_GPS_MSG = False


# ---------------------------------------------------------------------------
# Coordinate helpers  (same math used in gps_to_rviz.py)
# ---------------------------------------------------------------------------

def quaternion_to_yaw(x, y, z, w):
    """Quaternion -> yaw (radians)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    """Geodetic (deg, deg, m) -> ECEF (m)."""
    a  = 6378137.0          # WGS-84 semi-major axis
    e2 = 0.00669437999014   # WGS-84 first eccentricity squared
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return x, y, z


def ecef_delta_to_enu(dx, dy, dz, lat0_deg, lon0_deg):
    """
    Rotate an ECEF delta vector (dx, dy, dz) into local ENU
    at the reference geodetic point (lat0, lon0).
    """
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    east  = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = (-math.sin(lat0) * math.cos(lon0) * dx
             - math.sin(lat0) * math.sin(lon0) * dy
             + math.cos(lat0) * dz)
    up    = (math.cos(lat0) * math.cos(lon0) * dx
             + math.cos(lat0) * math.sin(lon0) * dy
             + math.sin(lat0) * dz)
    return east, north, up


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

GPS_FIX_3D = 768   # fix_type value for a valid 3D GPS fix


class WaypointLogger(Node):
    def __init__(self, output_file: str):
        super().__init__('waypoint_logger')
        self.output_file = output_file
        self.waypoints   = []
        self._lock       = threading.Lock()

        # --- Latest raw data from each topic ---
        self.latest_ins = None   # dict: raw x, y, z, yaw (ECEF)
        self.latest_gps = None   # dict: latitude, longitude, altitude, fix_type, num_sat

        # --- Home point (set from first valid messages) ---
        self.home_ins = None     # (x0, y0, z0)  raw ECEF from /ins
        self.home_gps = None     # (lat0, lon0, alt0) + precomputed ECEF tuple
        self._home_locked = False  # True once both homes are set

        # Subscribe to /ins
        self.ins_sub = self.create_subscription(
            Odometry, '/ins', self._ins_callback, 10)

        # Subscribe to /gps if message type is available
        if HAS_GPS_MSG:
            self.gps_sub = self.create_subscription(
                GPSMsg, '/gps', self._gps_callback, 10)
        else:
            print("[WARN] inertial_sense_ros2 not found — /gps will not be subscribed.")
            print("       Run: source /home/lanceholden/rover_workspace/install/setup.bash")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _ins_callback(self, msg: Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        with self._lock:
            self.latest_ins = {
                'x':   msg.pose.pose.position.x,
                'y':   msg.pose.pose.position.y,
                'z':   msg.pose.pose.position.z,
                'yaw': quaternion_to_yaw(qx, qy, qz, qw),
            }
            # Lock in the INS home point on the very first message
            if self.home_ins is None:
                self.home_ins = (
                    self.latest_ins['x'],
                    self.latest_ins['y'],
                    self.latest_ins['z'],
                )
                self._check_home_ready()

    def _gps_callback(self, msg):
        with self._lock:
            self.latest_gps = {
                'latitude':  msg.latitude,
                'longitude': msg.longitude,
                'altitude':  msg.altitude,
                'fix_type':  int(msg.fix_type),
                'num_sat':   int(msg.num_sat),
            }
            # Lock in the GPS home point on the first valid 3D fix
            if self.home_gps is None and msg.fix_type >= GPS_FIX_3D:
                lat0, lon0, alt0 = msg.latitude, msg.longitude, msg.altitude
                self.home_gps = {
                    'latitude':  lat0,
                    'longitude': lon0,
                    'altitude':  alt0,
                    'ecef':      lla_to_ecef(lat0, lon0, alt0),
                }
                self._check_home_ready()

    def _check_home_ready(self):
        """Print a ready message once both home points are established."""
        if self._home_locked:
            return
        ins_ready = self.home_ins is not None
        gps_ready = self.home_gps is not None or not HAS_GPS_MSG
        if ins_ready and gps_ready:
            self._home_locked = True
            hi = self.home_ins
            print(f"\n[HOME] INS origin locked  -> ECEF ({hi[0]:.2f}, {hi[1]:.2f}, {hi[2]:.2f})")
            if self.home_gps:
                hg = self.home_gps
                print(f"[HOME] GPS origin locked  -> {hg['latitude']:.7f} lat, "
                      f"{hg['longitude']:.7f} lon, {hg['altitude']:.2f}m alt")
            print("[READY] Press SPACE to log waypoints\n")

    # -----------------------------------------------------------------------
    # Waypoint logging
    # -----------------------------------------------------------------------

    def log_waypoint(self):
        """Capture the latest data, localize to home, and append a waypoint."""
        with self._lock:
            ins = dict(self.latest_ins) if self.latest_ins else None
            gps = dict(self.latest_gps) if self.latest_gps else None
            home_ins = self.home_ins
            home_gps = dict(self.home_gps) if self.home_gps else None

        if ins is None and gps is None:
            print("[WARN] No data received yet — is the rover publishing?")
            return

        waypoint = {}

        # --- INS: localize to home (ECEF delta) ---
        if ins is not None:
            if home_ins is not None:
                lx = ins['x'] - home_ins[0]
                ly = ins['y'] - home_ins[1]
                lz = ins['z'] - home_ins[2]
            else:
                # Home not set yet — store raw (unusual)
                lx, ly, lz = ins['x'], ins['y'], ins['z']

            waypoint['x']   = float(lx)
            waypoint['y']   = float(ly)
            waypoint['z']   = float(lz)
            waypoint['yaw'] = float(ins['yaw'])

        # --- GPS: convert to local ENU relative to home_gps ---
        if gps is not None:
            waypoint['latitude']  = float(gps['latitude'])
            waypoint['longitude'] = float(gps['longitude'])
            waypoint['altitude']  = float(gps['altitude'])
            waypoint['fix_type']  = gps['fix_type']
            waypoint['num_sat']   = gps['num_sat']

            if home_gps is not None and gps['fix_type'] >= GPS_FIX_3D:
                cx, cy, cz = lla_to_ecef(gps['latitude'], gps['longitude'], gps['altitude'])
                hx, hy, hz = home_gps['ecef']
                east, north, up = ecef_delta_to_enu(
                    cx - hx, cy - hy, cz - hz,
                    home_gps['latitude'], home_gps['longitude']
                )
                waypoint['east']  = float(east)
                waypoint['north'] = float(north)
                waypoint['up']    = float(up)

        self.waypoints.append(waypoint)
        n = len(self.waypoints)

        # --- Pretty-print confirmation ---
        line_parts = [f"[WP {n:3d}]"]
        if 'x' in waypoint:
            line_parts.append(
                f"local ({waypoint['x']:+8.3f}, {waypoint['y']:+8.3f}, {waypoint['z']:+7.3f})m  "
                f"yaw={math.degrees(waypoint['yaw']):+6.1f}°"
            )
        if 'east' in waypoint:
            fix_str = "3D" if gps['fix_type'] >= GPS_FIX_3D else f"fix={gps['fix_type']}"
            line_parts.append(
                f"ENU ({waypoint['east']:+8.3f}E, {waypoint['north']:+8.3f}N, {waypoint['up']:+6.3f}U)m  "
                f"[{fix_str}, {gps['num_sat']} sats]"
            )
        elif gps is not None:
            line_parts.append(
                f"lat={waypoint['latitude']:.7f}  lon={waypoint['longitude']:.7f}  "
                f"(waiting for 3D fix)"
            )
        print("  ".join(line_parts))

    # -----------------------------------------------------------------------
    # Save
    # -----------------------------------------------------------------------

    def save(self):
        """Write all logged waypoints and the home point to the output YAML file."""
        if not self.waypoints:
            print("\n[INFO] No waypoints logged — nothing to save.")
            return

        # Build home_point metadata block
        home_block = {}
        if self.home_ins:
            home_block['ins_ecef_x'] = float(self.home_ins[0])
            home_block['ins_ecef_y'] = float(self.home_ins[1])
            home_block['ins_ecef_z'] = float(self.home_ins[2])
        if self.home_gps:
            home_block['latitude']  = float(self.home_gps['latitude'])
            home_block['longitude'] = float(self.home_gps['longitude'])
            home_block['altitude']  = float(self.home_gps['altitude'])

        output_data = {
            'frame_id':   'local_enu',
            'home_point': home_block if home_block else None,
            'waypoints':  self.waypoints,
        }

        with open(self.output_file, 'w') as f:
            yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)

        print(f"\n[INFO] Saved {len(self.waypoints)} waypoint(s) -> {self.output_file}")


# ---------------------------------------------------------------------------
# Keyboard loop (main thread; ROS spins in background)
# ---------------------------------------------------------------------------

def keyboard_loop(node: WaypointLogger, stop_event: threading.Event):
    """SPACE = log waypoint, q / Ctrl+C = quit."""
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
            elif ch in ('q', 'Q', '\x03'):   # q or Ctrl+C
                stop_event.set()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Log localized waypoints from /ins (ECEF) and /gps on SPACE key press',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '-o', '--output', default='logged_waypoints.yaml',
        help='Output YAML file (default: logged_waypoints.yaml)',
    )
    args = parser.parse_args()

    rclpy.init()
    node = WaypointLogger(args.output)

    stop_event = threading.Event()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("=" * 70)
    print("  Waypoint Logger  (home-point localization enabled)")
    print(f"  Output file : {args.output}")
    print(f"  /ins        : nav_msgs/Odometry  -> local x/y/z (ECEF delta from home)")
    if HAS_GPS_MSG:
        print(f"  /gps        : inertial_sense_ros2/GPS -> ENU from home GPS fix")
    print()
    print("  Waiting for home point lock (first INS msg + first GPS 3D fix)...")
    print()
    print("  SPACE  ->  log current position as a waypoint")
    print("  q      ->  save and quit")
    print("  Ctrl+C ->  save and quit")
    print("=" * 70)

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
