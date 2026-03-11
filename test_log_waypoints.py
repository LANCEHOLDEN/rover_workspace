#!/usr/bin/env python3
"""
Offline test for log_waypoints.py coordinate logic.

Reads the latest bag SQLite directly, simulates pressing SPACE at 5 evenly-spaced
points in time, and saves the result to test_logged_waypoints.yaml.
No ROS runtime or keyboard needed.

Also validates sensor data quality so we can tell if a bag is usable.
"""

import math
import struct
import sqlite3
import yaml
import sys
import os

BAG_DIR = os.path.join(
    os.path.dirname(__file__),
    "src/bag_files/rosbag2_2026_02_23-15_40_48"
)
OUTPUT_FILE = os.path.join(os.path.dirname(__file__), "test_logged_waypoints.yaml")
NUM_WAYPOINTS = 5   # simulated space-bar presses

GPS_FIX_3D = 768

# Sanity bounds for valid sensor data
MAX_VALID_LAT  = 90.0
MAX_VALID_LON  = 180.0
MAX_ECEF_NORM  = 7e6   # ECEF magnitude should be ~6.37M for Earth surface
MIN_ECEF_NORM  = 6e6


# ---------------------------------------------------------------------------
# Coordinate helpers  (same as log_waypoints.py)
# ---------------------------------------------------------------------------

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    a  = 6378137.0
    e2 = 0.00669437999014
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return x, y, z


def ecef_delta_to_enu(dx, dy, dz, lat0_deg, lon0_deg):
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


def is_valid_quaternion(qx, qy, qz, qw, tol=0.05):
    """Unit quaternion check: |q| should be ~1."""
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return abs(norm - 1.0) < tol


def is_valid_ecef(x, y, z):
    """ECEF position on Earth's surface: magnitude ~6.37M m."""
    r = math.sqrt(x*x + y*y + z*z)
    return MIN_ECEF_NORM < r < MAX_ECEF_NORM


def is_valid_lla(lat, lon, alt):
    return (abs(lat) <= MAX_VALID_LAT and abs(lon) <= MAX_VALID_LON
            and -500 < alt < 9000)


# ---------------------------------------------------------------------------
# CDR parsers  — GPS.msg field layout (128 bytes total):
#   [0-3]    CDR header
#   [4-11]   stamp (sec + nanosec)
#   [12-15]  frame_id_len
#   [16-20]  frame_id "body\0"
#   [21]     int8 num_sat   (NO pre-alignment for int8)
#   [22-23]  padding to 4-byte boundary
#   [24-27]  uint32 fix_type
#   [28-31]  int32 cno
#   [32-39]  float64 latitude   (already 8-byte aligned)
#   [40-47]  float64 longitude
#   [48-55]  float64 altitude
#   [56-79]  Vector3 pos_ecef (3 x float64)
#   [80-103] Vector3 vel_ecef (3 x float64)
#   [104-123] float32 h_msl, h_acc, v_acc, s_acc, p_dop
# ---------------------------------------------------------------------------

def parse_odometry_cdr(data):
    offset = 4
    offset += 8   # stamp
    fl = struct.unpack_from('<I', data, offset)[0]; offset += 4 + fl
    offset = (offset + 3) & ~3
    cl = struct.unpack_from('<I', data, offset)[0]; offset += 4 + cl
    offset = (offset + 7) & ~7
    x, y, z = struct.unpack_from('<ddd', data, offset)
    offset += 24
    qx, qy, qz, qw = struct.unpack_from('<dddd', data, offset)
    return x, y, z, qx, qy, qz, qw


def parse_gps_cdr(data):
    offset = 4    # CDR header
    offset += 4   # stamp.sec
    offset += 4   # stamp.nanosec
    fl = struct.unpack_from('<I', data, offset)[0]
    offset += 4 + fl                 # frame_id (includes null terminator)
    # int8 num_sat — no alignment needed before int8
    num_sat = struct.unpack_from('<b', data, offset)[0]
    offset += 1
    offset = (offset + 3) & ~3      # align to 4 for uint32
    fix_type = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    offset += 4                     # int32 cno (now at offset 32, 8-aligned)
    latitude, longitude, altitude = struct.unpack_from('<ddd', data, offset)
    offset += 24
    ecef_x, ecef_y, ecef_z = struct.unpack_from('<ddd', data, offset)
    return fix_type, latitude, longitude, altitude, num_sat, ecef_x, ecef_y, ecef_z


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------

def main():
    db_path = None
    for f in os.listdir(BAG_DIR):
        if f.endswith('.db3'):
            db_path = os.path.join(BAG_DIR, f)
            break

    if not db_path:
        print(f"ERROR: no .db3 file found in {BAG_DIR}")
        return 1

    print(f"Bag : {db_path}")

    conn   = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT id, name FROM topics")
    topics = {row[1]: row[0] for row in cursor.fetchall()}
    print(f"Topics in bag: {list(topics.keys())}")

    ins_id = topics.get('/ins')
    gps_id = topics.get('/gps')

    if ins_id is None:
        print("ERROR: /ins topic not found in bag")
        return 1

    cursor.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
    )
    rows = cursor.fetchall()
    conn.close()

    print(f"Total messages : {len(rows)}")

    # ---- First pass: collect and validate all parsed messages ------------
    ins_messages  = []
    gps_messages  = []
    ins_bad  = 0
    gps_bad  = 0
    gps_fix_hist = {}

    for topic_id, ts, data in rows:
        if topic_id == ins_id:
            try:
                x, y, z, qx, qy, qz, qw = parse_odometry_cdr(data)
                valid_pos  = is_valid_ecef(x, y, z)
                valid_quat = is_valid_quaternion(qx, qy, qz, qw)
                ins_messages.append((ts, x, y, z, qx, qy, qz, qw, valid_pos and valid_quat))
                if not (valid_pos and valid_quat):
                    ins_bad += 1
            except Exception:
                ins_bad += 1

        elif gps_id and topic_id == gps_id:
            try:
                ft, lat, lon, alt, ns, ex, ey, ez = parse_gps_cdr(data)
                gps_fix_hist[ft] = gps_fix_hist.get(ft, 0) + 1
                valid = is_valid_lla(lat, lon, alt)
                gps_messages.append((ts, ft, lat, lon, alt, ns, ex, ey, ez, valid))
                if not valid:
                    gps_bad += 1
            except Exception:
                gps_bad += 1

    ins_good = len(ins_messages) - ins_bad
    gps_good = len(gps_messages) - gps_bad

    print(f"\n=== Sensor data quality ===")
    print(f"  INS  : {len(ins_messages)} msgs  |  valid={ins_good}  bad={ins_bad}")
    print(f"  GPS  : {len(gps_messages)} msgs  |  valid={gps_good}  bad={gps_bad}")
    print(f"  GPS fix_type distribution: { {k:v for k,v in sorted(gps_fix_hist.items())} }")

    if not ins_messages:
        print("ERROR: no /ins messages parsed")
        return 1

    # ---- Diagnose what's wrong if data is bad ----------------------------
    if ins_good == 0:
        print("\n[DIAG] INS: all positions are zero/garbage.")
        print("       The InertialSense sensor had not initialized its EKF when")
        print("       this bag was recorded. Quaternions are also invalid (not unit).")

    fix_3d_count = gps_fix_hist.get(GPS_FIX_3D, 0) + sum(
        v for k, v in gps_fix_hist.items() if k > GPS_FIX_3D)
    if gps_good == 0:
        if fix_3d_count > 0:
            print(f"\n[DIAG] GPS: {fix_3d_count} messages claim fix_type>=768 (3D) but")
            print("       lat/lon values are out-of-range — sensor was publishing")
            print("       uninitialized memory (common on cold start).")
        else:
            print(f"\n[DIAG] GPS: no 3D fix acquired in this recording.")

    # ---- Set home points -------------------------------------------------
    home_ins = (ins_messages[0][1], ins_messages[0][2], ins_messages[0][3])
    print(f"\n[HOME] INS origin (raw ECEF) -> ({home_ins[0]:.3f}, {home_ins[1]:.3f}, {home_ins[2]:.3f})")

    home_gps = None
    for ts, ft, lat, lon, alt, ns, ex, ey, ez, valid in gps_messages:
        if ft >= GPS_FIX_3D and valid:
            home_gps = {'latitude': lat, 'longitude': lon, 'altitude': alt,
                        'ecef': lla_to_ecef(lat, lon, alt)}
            print(f"[HOME] GPS origin -> {lat:.7f} lat, {lon:.7f} lon, {alt:.2f}m alt")
            break

    if home_gps is None:
        print("[HOME] GPS origin -> NOT available (no valid 3D fix in this bag)")

    # ---- Build timestamp -> latest GPS lookup ----------------------------
    gps_by_time = {}
    latest_gps_entry = None
    gi = 0
    for ts, x, y, z, qx, qy, qz, qw, _ in ins_messages:
        while gi < len(gps_messages) and gps_messages[gi][0] <= ts:
            latest_gps_entry = gps_messages[gi]
            gi += 1
        gps_by_time[ts] = latest_gps_entry

    # ---- Simulate NUM_WAYPOINTS SPACE presses ----------------------------
    step    = max(1, len(ins_messages) // (NUM_WAYPOINTS + 1))
    indices = [step * (i + 1) for i in range(NUM_WAYPOINTS)
               if step * (i + 1) < len(ins_messages)]

    print(f"\n--- Simulating {len(indices)} SPACE presses "
          f"(~every {step} INS msgs) ---\n")
    print(f"{'WP':>4}  {'local_x':>10} {'local_y':>10} {'local_z':>9}  "
          f"{'yaw_deg':>8}  {'east':>9} {'north':>9} {'up':>7}  notes")
    print("-" * 95)

    waypoints = []
    for n, idx in enumerate(indices, 1):
        ts, x, y, z, qx, qy, qz, qw, ins_valid = ins_messages[idx]
        yaw = quaternion_to_yaw(qx, qy, qz, qw)

        lx = x - home_ins[0]
        ly = y - home_ins[1]
        lz = z - home_ins[2]

        wp = {'x': float(lx), 'y': float(ly), 'z': float(lz), 'yaw': float(yaw)}

        notes = "INS_UNINIT" if not ins_valid else ""

        east = north = up = None
        gps_entry = gps_by_time.get(ts)
        if gps_entry and home_gps:
            ts2, ft, lat, lon, alt, ns, ex, ey, ez, gps_valid = gps_entry
            if ft >= GPS_FIX_3D and gps_valid:
                cx, cy, cz = lla_to_ecef(lat, lon, alt)
                hx, hy, hz = home_gps['ecef']
                east, north, up = ecef_delta_to_enu(
                    cx-hx, cy-hy, cz-hz,
                    home_gps['latitude'], home_gps['longitude'])
                wp.update({'latitude': float(lat), 'longitude': float(lon),
                           'altitude': float(alt), 'east': float(east),
                           'north': float(north), 'up': float(up), 'num_sat': int(ns)})
            elif not gps_valid:
                notes += " GPS_UNINIT"

        e_s = f"{east:+9.3f}" if east is not None else "      N/A"
        n_s = f"{north:+9.3f}" if north is not None else "      N/A"
        u_s = f"{up:+7.3f}" if up is not None else "    N/A"
        print(f"{n:>4}  {lx:>+10.3f} {ly:>+10.3f} {lz:>+9.3f}  "
              f"{math.degrees(yaw):>+8.2f}  {e_s} {n_s} {u_s}  {notes}")
        waypoints.append(wp)

    # ---- Save YAML -------------------------------------------------------
    home_block = {'ins_ecef_x': float(home_ins[0]), 'ins_ecef_y': float(home_ins[1]),
                  'ins_ecef_z': float(home_ins[2])}
    if home_gps:
        home_block.update({'latitude': float(home_gps['latitude']),
                           'longitude': float(home_gps['longitude']),
                           'altitude': float(home_gps['altitude'])})

    with open(OUTPUT_FILE, 'w') as f:
        yaml.dump({'frame_id': 'local_enu', 'home_point': home_block,
                   'waypoints': waypoints},
                  f, default_flow_style=False, sort_keys=False)

    print(f"\nSaved {len(waypoints)} waypoints -> {OUTPUT_FILE}")

    # ---- Final verdict ---------------------------------------------------
    print("\n=== Test result ===")
    if ins_good == 0 and gps_good == 0:
        print("FAIL — Sensor data was uninitialized in this bag.")
        print("       The coordinate math and CDR parsing are correct.")
        print("       Record a new bag outdoors with the sensor fully initialized")
        print("       (wait for solid GPS lock + INS convergence) then re-run.")
    else:
        print("PASS — Valid sensor data found. Check test_logged_waypoints.yaml.")

    return 0


if __name__ == '__main__':
    sys.exit(main())
