#!/usr/bin/env python3
"""
Extract waypoints from a ROS2 bag file containing /ins (Odometry) data.
Saves waypoints to a YAML file for use with waypoint_follower.py
"""

import argparse
import yaml
import math
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (radians)."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def extract_waypoints(bag_path, output_file, distance_threshold=1.0):
    """
    Extract waypoints from bag file.

    Args:
        bag_path: Path to the bag folder
        output_file: Output YAML file path
        distance_threshold: Minimum distance (meters) between waypoints
    """
    waypoints = []
    last_x, last_y = None, None
    msg_count = 0

    print(f"Reading bag: {bag_path}")

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/ins':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                msg_count += 1

                # Extract position from Odometry message
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = msg.pose.pose.position.z

                # Get yaw from quaternion
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                yaw = quaternion_to_yaw(qx, qy, qz, qw)

                # Check distance from last waypoint
                if last_x is None:
                    # First waypoint
                    waypoints.append({
                        'x': float(x),
                        'y': float(y),
                        'z': float(z),
                        'yaw': float(yaw)
                    })
                    last_x, last_y = x, y
                    print(f"Waypoint 1: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°")
                else:
                    dist = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                    if dist >= distance_threshold:
                        waypoints.append({
                            'x': float(x),
                            'y': float(y),
                            'z': float(z),
                            'yaw': float(yaw)
                        })
                        last_x, last_y = x, y
                        print(f"Waypoint {len(waypoints)}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°")

    print(f"\n--- Summary ---")
    print(f"Processed {msg_count} messages from /ins")
    print(f"Extracted {len(waypoints)} waypoints (threshold: {distance_threshold}m)")

    if len(waypoints) == 0:
        print("WARNING: No waypoints extracted! Check if the rover moved during recording.")
        return []

    # Save to YAML
    output_data = {
        'waypoints': waypoints,
        'frame_id': 'body',
        'distance_threshold': distance_threshold
    }

    with open(output_file, 'w') as f:
        yaml.dump(output_data, f, default_flow_style=False)

    print(f"Saved to: {output_file}")
    return waypoints


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract waypoints from ROS2 bag')
    parser.add_argument('bag_path', help='Path to bag folder (e.g., ./data3)')
    parser.add_argument('-o', '--output', default='waypoints.yaml', help='Output file')
    parser.add_argument('-d', '--distance', type=float, default=1.0,
                        help='Min distance between waypoints in meters (default: 1.0)')

    args = parser.parse_args()
    extract_waypoints(args.bag_path, args.output, args.distance)
