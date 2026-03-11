#!/usr/bin/env python3
"""
Extract waypoints from ROS2 bag files containing /ins and /gps data.
Converts to local waypoints and saves to YAML for use with waypoint_follower.
"""

import argparse
import yaml
import math
import sqlite3
import os
import struct

# Try to use rosbags if available, otherwise use raw SQLite parsing
try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import get_typestore, Stores
    TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
    USE_ROSBAGS = True
except ImportError:
    USE_ROSBAGS = False
    TYPESTORE = None


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (radians)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """
    Convert geodetic coordinates (lat, lon, alt) to local ENU (East-North-Up).
    Uses a simple approximation valid for short distances.

    Args:
        lat, lon, alt: Current position in degrees and meters
        lat0, lon0, alt0: Reference origin in degrees and meters

    Returns:
        (east, north, up) in meters
    """
    EARTH_RADIUS = 6378137.0  # WGS84 equatorial radius

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    dlat = lat_rad - lat0_rad
    dlon = lon_rad - lon0_rad

    # Approximate conversion to meters
    north = dlat * EARTH_RADIUS
    east = dlon * EARTH_RADIUS * math.cos(lat0_rad)
    up = alt - alt0

    return east, north, up


def parse_odometry_cdr(data):
    """
    Parse nav_msgs/msg/Odometry from CDR serialization.
    This is a simplified parser for the specific fields we need.
    """
    # CDR format for Odometry - we need pose.pose.position and orientation
    # Skip CDR header (4 bytes) and header fields
    offset = 4  # CDR encapsulation header

    # Skip std_msgs/Header: stamp (8 bytes) + frame_id string
    offset += 8  # stamp
    frame_id_len = struct.unpack_from('<I', data, offset)[0]
    offset += 4 + frame_id_len
    # Align to 4 bytes
    offset = (offset + 3) & ~3

    # Skip child_frame_id string
    child_frame_id_len = struct.unpack_from('<I', data, offset)[0]
    offset += 4 + child_frame_id_len
    # Align to 8 bytes for doubles
    offset = (offset + 7) & ~7

    # Now we're at PoseWithCovariance.pose (Pose)
    # Pose = Point (3 doubles) + Quaternion (4 doubles)
    x, y, z = struct.unpack_from('<ddd', data, offset)
    offset += 24
    qx, qy, qz, qw = struct.unpack_from('<dddd', data, offset)

    return x, y, z, qx, qy, qz, qw


def parse_gps_cdr(data):
    """
    Parse inertial_sense_ros2/msg/GPS from CDR serialization.
    """
    offset = 4  # CDR encapsulation header

    # Skip Header: stamp (8 bytes) + frame_id string
    offset += 8
    frame_id_len = struct.unpack_from('<I', data, offset)[0]
    offset += 4 + frame_id_len
    offset = (offset + 3) & ~3

    # num_sat (int8) + padding + fix_type (uint32) + cno (int32)
    num_sat = struct.unpack_from('<b', data, offset)[0]
    offset += 1
    offset = (offset + 3) & ~3  # align to 4
    fix_type = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    cno = struct.unpack_from('<i', data, offset)[0]
    offset += 4

    # Align to 8 for doubles
    offset = (offset + 7) & ~7

    # latitude, longitude, altitude (3 doubles)
    latitude, longitude, altitude = struct.unpack_from('<ddd', data, offset)

    return fix_type, latitude, longitude, altitude


class WaypointExtractor:
    def __init__(self, bag_path, distance_threshold=1.0, use_gps=False, use_ins=True,
                 smooth_window=50):
        self.bag_path = bag_path
        self.distance_threshold = distance_threshold
        self.use_gps = use_gps
        self.use_ins = use_ins
        self.smooth_window = smooth_window

        self.waypoints = []
        self.gps_origin = None  # (lat, lon, alt) reference point
        self.ins_origin = None  # (x, y, z) first INS position for local frame
        self.last_x = None
        self.last_y = None
        self.position_buffer = []  # Buffer for smoothing

        self.ins_count = 0
        self.gps_count = 0

    def extract(self):
        """Extract waypoints from the bag file."""
        print(f"Reading bag: {self.bag_path}")
        print(f"Sources: INS={self.use_ins}, GPS={self.use_gps}")
        print(f"Distance threshold: {self.distance_threshold}m")
        print(f"Smoothing window: {self.smooth_window} samples")
        print("-" * 50)

        if USE_ROSBAGS:
            self._extract_with_rosbags()
        else:
            print("(Using native SQLite reader)")
            self._extract_with_sqlite()

        self._print_summary()
        return self.waypoints

    def _extract_with_rosbags(self):
        """Extract using rosbags library."""
        with Reader(self.bag_path) as reader:
            messages = []
            for connection, timestamp, rawdata in reader.messages():
                messages.append((timestamp, connection, rawdata))

            messages.sort(key=lambda x: x[0])

            for timestamp, connection, rawdata in messages:
                if connection.topic == '/ins' and self.use_ins:
                    self._process_ins_rosbags(rawdata, connection.msgtype)
                elif connection.topic == '/gps' and self.use_gps:
                    self._process_gps_rosbags(rawdata, connection.msgtype)

    def _extract_with_sqlite(self):
        """Extract using native SQLite reader."""
        # Find the .db3 file
        db_path = None
        for f in os.listdir(self.bag_path):
            if f.endswith('.db3'):
                db_path = os.path.join(self.bag_path, f)
                break

        if not db_path:
            print(f"ERROR: No .db3 file found in {self.bag_path}")
            return

        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Get topic info
        cursor.execute("SELECT id, name, type FROM topics")
        topics = {row[1]: (row[0], row[2]) for row in cursor.fetchall()}

        # Read messages sorted by timestamp
        cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")

        ins_topic_id = topics.get('/ins', (None, None))[0]
        gps_topic_id = topics.get('/gps', (None, None))[0]

        for topic_id, timestamp, data in cursor.fetchall():
            if topic_id == ins_topic_id and self.use_ins:
                self._process_ins_sqlite(data)
            elif topic_id == gps_topic_id and self.use_gps:
                self._process_gps_sqlite(data)

        conn.close()

    def _process_ins_rosbags(self, rawdata, msgtype):
        """Process INS (Odometry) message using rosbags."""
        msg = TYPESTORE.deserialize_cdr(rawdata, msgtype)
        self.ins_count += 1

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = quaternion_to_yaw(qx, qy, qz, qw)

        self._add_waypoint(x, y, z, yaw, source='ins')

    def _process_ins_sqlite(self, data):
        """Process INS (Odometry) message from raw CDR data."""
        try:
            x, y, z, qx, qy, qz, qw = parse_odometry_cdr(data)
            self.ins_count += 1
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            self._add_waypoint(x, y, z, yaw, source='ins')
        except Exception as e:
            pass  # Skip malformed messages

    def _process_gps_rosbags(self, rawdata, msgtype):
        """Process GPS message using rosbags."""
        try:
            msg = TYPESTORE.deserialize_cdr(rawdata, msgtype)
        except Exception:
            # Custom message type not in typestore, fall back to SQLite parser
            self._process_gps_sqlite(rawdata)
            return

        self.gps_count += 1

        GPS_STATUS_FIX_TYPE_3D_FIX = 768
        if msg.fix_type < GPS_STATUS_FIX_TYPE_3D_FIX:
            return

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        self._process_gps_coords(lat, lon, alt)

    def _process_gps_sqlite(self, data):
        """Process GPS message from raw CDR data."""
        try:
            fix_type, lat, lon, alt = parse_gps_cdr(data)
            self.gps_count += 1

            GPS_STATUS_FIX_TYPE_3D_FIX = 768
            if fix_type < GPS_STATUS_FIX_TYPE_3D_FIX:
                return

            self._process_gps_coords(lat, lon, alt)
        except Exception as e:
            pass  # Skip malformed messages

    def _process_gps_coords(self, lat, lon, alt):
        """Process GPS coordinates and convert to local frame."""
        if self.gps_origin is None:
            self.gps_origin = (lat, lon, alt)
            print(f"GPS origin set: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m")

        east, north, up = geodetic_to_enu(lat, lon, alt, *self.gps_origin)

        yaw = 0.0
        if len(self.waypoints) > 0:
            last_wp = self.waypoints[-1]
            dx = east - last_wp['x']
            dy = north - last_wp['y']
            if math.sqrt(dx*dx + dy*dy) > 0.1:
                yaw = math.atan2(dy, dx)

        self._add_waypoint(east, north, up, yaw, source='gps')

    def _add_waypoint(self, x, y, z, yaw, source='unknown'):
        """Add a waypoint if it exceeds the distance threshold. Uses smoothing."""
        # Convert to local frame by subtracting the first position
        if self.ins_origin is None:
            self.ins_origin = (x, y, z)
            print(f"INS origin set: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        local_x = x - self.ins_origin[0]
        local_y = y - self.ins_origin[1]
        local_z = z - self.ins_origin[2]

        # Add to smoothing buffer
        self.position_buffer.append((local_x, local_y, local_z, yaw))
        if len(self.position_buffer) > self.smooth_window:
            self.position_buffer.pop(0)

        # Don't emit waypoints until the buffer is full (except for the very first one)
        if len(self.position_buffer) < self.smooth_window and self.last_x is not None:
            return

        # Compute the average position from the buffer
        avg_x = sum(p[0] for p in self.position_buffer) / len(self.position_buffer)
        avg_y = sum(p[1] for p in self.position_buffer) / len(self.position_buffer)
        avg_z = sum(p[2] for p in self.position_buffer) / len(self.position_buffer)
        # Use the most recent yaw (averaging angles is problematic)
        avg_yaw = self.position_buffer[-1][3]

        if self.last_x is None:
            # First waypoint
            self.waypoints.append({
                'x': float(avg_x),
                'y': float(avg_y),
                'z': float(avg_z),
                'yaw': float(avg_yaw),
                'source': source
            })
            self.last_x, self.last_y = avg_x, avg_y
            print(f"WP {len(self.waypoints):3d}: x={avg_x:8.3f}, y={avg_y:8.3f}, "
                  f"yaw={math.degrees(avg_yaw):6.1f} deg [{source}]")
        else:
            dist = math.sqrt((avg_x - self.last_x)**2 + (avg_y - self.last_y)**2)
            if dist >= self.distance_threshold:
                self.waypoints.append({
                    'x': float(avg_x),
                    'y': float(avg_y),
                    'z': float(avg_z),
                    'yaw': float(avg_yaw),
                    'source': source
                })
                self.last_x, self.last_y = avg_x, avg_y
                print(f"WP {len(self.waypoints):3d}: x={avg_x:8.3f}, y={avg_y:8.3f}, "
                      f"yaw={math.degrees(avg_yaw):6.1f} deg [{source}]")

    def _print_summary(self):
        """Print extraction summary."""
        print("-" * 50)
        print("Summary:")
        print(f"  INS messages processed: {self.ins_count}")
        print(f"  GPS messages processed: {self.gps_count}")
        print(f"  Waypoints extracted: {len(self.waypoints)}")

        if len(self.waypoints) >= 2:
            total_dist = 0.0
            for i in range(1, len(self.waypoints)):
                dx = self.waypoints[i]['x'] - self.waypoints[i-1]['x']
                dy = self.waypoints[i]['y'] - self.waypoints[i-1]['y']
                total_dist += math.sqrt(dx*dx + dy*dy)
            print(f"  Total path length: {total_dist:.2f}m")

        if len(self.waypoints) == 0:
            print("WARNING: No waypoints extracted!")
            print("  - Check if the rover moved during recording")
            print("  - Try reducing the distance threshold (-d)")
            print("  - Verify the bag contains /ins or /gps topics")

    def save(self, output_file):
        """Save waypoints to YAML file."""
        if len(self.waypoints) == 0:
            print("No waypoints to save.")
            return False

        # Remove source field for cleaner output (optional)
        waypoints_clean = []
        for wp in self.waypoints:
            waypoints_clean.append({
                'x': wp['x'],
                'y': wp['y'],
                'z': wp['z'],
                'yaw': wp['yaw']
            })

        output_data = {
            'waypoints': waypoints_clean,
            'frame_id': 'odom',
            'distance_threshold': self.distance_threshold,
            'gps_origin': {
                'latitude': self.gps_origin[0] if self.gps_origin else None,
                'longitude': self.gps_origin[1] if self.gps_origin else None,
                'altitude': self.gps_origin[2] if self.gps_origin else None
            } if self.use_gps and self.gps_origin else None
        }

        with open(output_file, 'w') as f:
            yaml.dump(output_data, f, default_flow_style=False)

        print(f"Saved to: {output_file}")
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Extract waypoints from ROS2 bag with INS/GPS data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Extract from INS data (default)
  ros2 run waypoint_reader extract_waypoints ./bag_files/data3

  # Extract from GPS data
  ros2 run waypoint_reader extract_waypoints ./bag_files/data3 --gps

  # Use both INS and GPS, save to custom file
  ros2 run waypoint_reader extract_waypoints ./bag_files/data3 --ins --gps -o my_waypoints.yaml

  # Denser waypoints (0.5m apart)
  ros2 run waypoint_reader extract_waypoints ./bag_files/data3 -d 0.5
        '''
    )

    parser.add_argument('bag_path', help='Path to ROS2 bag folder')
    parser.add_argument('-o', '--output', default='waypoints.yaml',
                        help='Output YAML file (default: waypoints.yaml)')
    parser.add_argument('-d', '--distance', type=float, default=1.0,
                        help='Min distance between waypoints in meters (default: 1.0)')
    parser.add_argument('--ins', action='store_true', default=True,
                        help='Use INS/Odometry data (default: True)')
    parser.add_argument('--no-ins', action='store_true',
                        help='Disable INS/Odometry data')
    parser.add_argument('--gps', action='store_true',
                        help='Use GPS data (converts to local ENU coordinates)')
    parser.add_argument('-s', '--smooth', type=int, default=50,
                        help='Smoothing window size in samples (default: 50, 0 to disable)')

    args = parser.parse_args()

    use_ins = args.ins and not args.no_ins

    if not use_ins and not args.gps:
        print("Error: Must enable at least one data source (--ins or --gps)")
        return 1

    extractor = WaypointExtractor(
        args.bag_path,
        distance_threshold=args.distance,
        use_gps=args.gps,
        use_ins=use_ins,
        smooth_window=max(args.smooth, 1)
    )

    extractor.extract()
    extractor.save(args.output)

    return 0


if __name__ == '__main__':
    exit(main())
