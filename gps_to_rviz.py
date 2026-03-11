#!/usr/bin/env python3
"""
Converts InertialSense GPS (lat/lon/alt) to local ENU frame for RViz2.
- Publishes /gps_path (nav_msgs/Path) - position trail
- Publishes /gps_pose (geometry_msgs/PoseStamped) - current position
- Publishes /rtk_markers (visualization_msgs/MarkerArray) - RTK status overlay
- Broadcasts TF map->body combining GPS position + INS orientation
"""
import rclpy
from rclpy.node import Node
from inertial_sense_ros2.msg import GPS, RTKInfo, RTKRel
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import math


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    a = 6378137.0
    e2 = 0.00669437999014
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return x, y, z


def ecef_to_enu(dx, dy, dz, lat0_deg, lon0_deg):
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    e = -math.sin(lon0) * dx + math.cos(lon0) * dy
    n = (-math.sin(lat0) * math.cos(lon0) * dx
         - math.sin(lat0) * math.sin(lon0) * dy
         + math.cos(lat0) * dz)
    u = (math.cos(lat0) * math.cos(lon0) * dx
         + math.cos(lat0) * math.sin(lon0) * dy
         + math.sin(lat0) * dz)
    return e, n, u


# AR ratio thresholds for RTK fix quality
AR_FIXED_THRESHOLD = 3.0    # >= this → RTK Fixed (cm level)
AR_FLOAT_THRESHOLD = 1.1    # >= this → RTK Float (dm level)


def rtk_fix_label(ar_ratio, differential_age):
    """Return human-readable RTK fix status string."""
    if differential_age > 5.0:
        return 'RTK STALE'
    if ar_ratio >= AR_FIXED_THRESHOLD:
        return f'RTK FIXED  AR={ar_ratio:.2f}'
    if ar_ratio >= AR_FLOAT_THRESHOLD:
        return f'RTK FLOAT  AR={ar_ratio:.2f}'
    return f'NO RTK  AR={ar_ratio:.2f}'


def rtk_color(ar_ratio, differential_age):
    """Return (r, g, b) color for current RTK quality."""
    if differential_age > 5.0:
        return (0.5, 0.5, 0.5)   # grey  — stale corrections
    if ar_ratio >= AR_FIXED_THRESHOLD:
        return (0.0, 1.0, 0.2)   # bright green — RTK Fixed
    if ar_ratio >= AR_FLOAT_THRESHOLD:
        return (1.0, 0.8, 0.0)   # amber — RTK Float
    return (1.0, 0.2, 0.2)       # red   — no RTK


class GpsToRviz(Node):
    # Fix type constants from GPS.msg (GPS_STATUS_FIX_* in data_sets.h)
    VALID_FIX_TYPES = {
        0x0300,   # FIX_3D
        0x0400,   # FIX_GPS_PLUS_DEAD_RECK
        0x0800,   # FIX_DGPS
        0x0900,   # FIX_SBAS
        0x0A00,   # FIX_RTK_SINGLE
        0x0B00,   # FIX_RTK_FLOAT
        0x0C00,   # FIX_RTK_FIX
    }

    def __init__(self):
        super().__init__('gps_to_rviz')

        self.origin_lla = None
        self.origin_ecef = None
        self.ins_orientation = None  # latest quaternion from /ins

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.current_enu = None       # latest ENU position from GPS
        self.last_path_enu = None     # last position appended to path
        self.min_move_dist = 0.5      # meters — ignore drift below this

        # RTK state
        self.ar_ratio = 0.0
        self.differential_age = 0.0
        self.distance_to_base = 0.0
        self.cycle_slip_count = 0
        self.base_enu = None          # base station in local ENU (set once origin known)
        self.base_lla = None          # base station lat/lon/alt

        # Subscribers
        self.gps_sub = self.create_subscription(GPS, '/gps', self.gps_cb, 10)
        self.ins_sub = self.create_subscription(Odometry, '/ins', self.ins_cb, 10)
        self.rtk_rel_sub = self.create_subscription(RTKRel, '/RTK/rel', self.rtk_rel_cb, 10)
        self.rtk_info_sub = self.create_subscription(RTKInfo, '/RTK/info', self.rtk_info_cb, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/gps_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/gps_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/rtk_markers', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Broadcast TF at 50Hz so RViz2 message filter doesn't overflow
        self.create_timer(0.02, self.tf_timer_cb)
        # Publish RTK status markers at 2Hz
        self.create_timer(0.5, self.publish_rtk_markers)

        self.get_logger().info('Waiting for valid GPS fix...')

    # ------------------------------------------------------------------ #
    #  RTK callbacks                                                       #
    # ------------------------------------------------------------------ #

    def rtk_rel_cb(self, msg):
        self.ar_ratio = msg.ar_ratio
        self.differential_age = msg.differential_age
        self.distance_to_base = msg.distance_to_base

    def rtk_info_cb(self, msg):
        self.cycle_slip_count = msg.cycle_slip_count
        if msg.base_lla[0] != 0.0 and self.base_lla is None:
            self.base_lla = (msg.base_lla[0], msg.base_lla[1], msg.base_lla[2])
            self.get_logger().info(
                f'Base station: {self.base_lla[0]:.6f} lat, '
                f'{self.base_lla[1]:.6f} lon, {self.base_lla[2]:.1f}m alt')
            # Compute base ENU once we have an origin
            self._update_base_enu()

    def _update_base_enu(self):
        """Recompute base station ENU whenever origin or base_lla is set."""
        if self.origin_ecef is None or self.base_lla is None:
            return
        bx, by, bz = lla_to_ecef(*self.base_lla)
        dx = bx - self.origin_ecef[0]
        dy = by - self.origin_ecef[1]
        dz = bz - self.origin_ecef[2]
        self.base_enu = ecef_to_enu(dx, dy, dz, *self.origin_lla[:2])

    # ------------------------------------------------------------------ #
    #  TF + INS                                                            #
    # ------------------------------------------------------------------ #

    def tf_timer_cb(self):
        """Broadcast map->body TF at 50Hz so RViz2 message filter stays happy."""
        if self.current_enu is None:
            return
        from geometry_msgs.msg import Quaternion
        east, north, up = self.current_enu
        orient = self.ins_orientation if self.ins_orientation is not None \
            else Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'body'
        tf.transform.translation.x = east
        tf.transform.translation.y = north
        tf.transform.translation.z = up
        tf.transform.rotation = orient
        self.tf_broadcaster.sendTransform(tf)

    def ins_cb(self, msg):
        self.ins_orientation = msg.pose.pose.orientation

    # ------------------------------------------------------------------ #
    #  GPS callback                                                        #
    # ------------------------------------------------------------------ #

    def gps_cb(self, msg):
        if msg.fix_type not in self.VALID_FIX_TYPES or msg.num_sat < 4:
            self.get_logger().info(
                f'No fix yet — fix_type={msg.fix_type}, sats={msg.num_sat}',
                throttle_duration_sec=5.0)
            return

        if self.origin_lla is None:
            self.origin_lla = (msg.latitude, msg.longitude, msg.altitude)
            self.origin_ecef = lla_to_ecef(*self.origin_lla)
            self.get_logger().info(
                f'Origin set: {msg.latitude:.6f} lat, '
                f'{msg.longitude:.6f} lon, {msg.altitude:.1f}m alt')
            self._update_base_enu()

        # Convert to local ENU
        cx, cy, cz = lla_to_ecef(msg.latitude, msg.longitude, msg.altitude)
        dx = cx - self.origin_ecef[0]
        dy = cy - self.origin_ecef[1]
        dz = cz - self.origin_ecef[2]
        east, north, up = ecef_to_enu(dx, dy, dz, *self.origin_lla[:2])
        self.current_enu = (east, north, up)

        stamp = msg.header.stamp

        # Use INS orientation if available, else identity
        if self.ins_orientation is not None:
            orient = self.ins_orientation
        else:
            from geometry_msgs.msg import Quaternion
            orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Publish current pose
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = up
        pose.pose.orientation = orient
        self.pose_pub.publish(pose)

        # Only append to path if moved more than min_move_dist
        if self.last_path_enu is None or math.sqrt(
                (east - self.last_path_enu[0]) ** 2 +
                (north - self.last_path_enu[1]) ** 2) >= self.min_move_dist:
            self.path.header.stamp = stamp
            self.path.poses.append(pose)
            self.last_path_enu = (east, north, up)
            self.path_pub.publish(self.path)

        rtk_label = rtk_fix_label(self.ar_ratio, self.differential_age)
        self.get_logger().info(
            f'E={east:.2f}m  N={north:.2f}m  U={up:.2f}m  '
            f'sats={msg.num_sat}  hacc={msg.h_acc:.2f}m  '
            f'[{rtk_label}  age={self.differential_age:.1f}s  '
            f'base={self.distance_to_base/1000:.1f}km]',
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ #
    #  RTK marker publisher                                                #
    # ------------------------------------------------------------------ #

    def publish_rtk_markers(self):
        if self.current_enu is None:
            return

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        east, north, up = self.current_enu
        r, g, b = rtk_color(self.ar_ratio, self.differential_age)

        # --- Marker 0: colored sphere at rover position (fix quality) ---
        rover_sphere = Marker()
        rover_sphere.header.stamp = now
        rover_sphere.header.frame_id = 'map'
        rover_sphere.ns = 'rtk'
        rover_sphere.id = 0
        rover_sphere.type = Marker.SPHERE
        rover_sphere.action = Marker.ADD
        rover_sphere.pose.position.x = east
        rover_sphere.pose.position.y = north
        rover_sphere.pose.position.z = up + 0.5   # float above ground
        rover_sphere.pose.orientation.w = 1.0
        rover_sphere.scale.x = 0.4
        rover_sphere.scale.y = 0.4
        rover_sphere.scale.z = 0.4
        rover_sphere.color.r = r
        rover_sphere.color.g = g
        rover_sphere.color.b = b
        rover_sphere.color.a = 1.0
        markers.markers.append(rover_sphere)

        # --- Marker 1: RTK status text above rover ---
        label = rtk_fix_label(self.ar_ratio, self.differential_age)
        status_text = Marker()
        status_text.header.stamp = now
        status_text.header.frame_id = 'map'
        status_text.ns = 'rtk'
        status_text.id = 1
        status_text.type = Marker.TEXT_VIEW_FACING
        status_text.action = Marker.ADD
        status_text.pose.position.x = east
        status_text.pose.position.y = north
        status_text.pose.position.z = up + 1.2
        status_text.pose.orientation.w = 1.0
        status_text.scale.z = 0.35        # text height
        status_text.color.r = r
        status_text.color.g = g
        status_text.color.b = b
        status_text.color.a = 1.0
        status_text.text = (
            f'{label}\n'
            f'age={self.differential_age:.1f}s  '
            f'base={self.distance_to_base/1000:.1f}km\n'
            f'slips={self.cycle_slip_count}'
        )
        markers.markers.append(status_text)

        # --- Marker 2: base station sphere (if known) ---
        if self.base_enu is not None:
            be, bn, bu = self.base_enu
            base_marker = Marker()
            base_marker.header.stamp = now
            base_marker.header.frame_id = 'map'
            base_marker.ns = 'rtk'
            base_marker.id = 2
            base_marker.type = Marker.CYLINDER
            base_marker.action = Marker.ADD
            base_marker.pose.position.x = be
            base_marker.pose.position.y = bn
            base_marker.pose.position.z = bu
            base_marker.pose.orientation.w = 1.0
            base_marker.scale.x = 2.0
            base_marker.scale.y = 2.0
            base_marker.scale.z = 0.3
            base_marker.color.r = 0.2
            base_marker.color.g = 0.4
            base_marker.color.b = 1.0
            base_marker.color.a = 0.8
            markers.markers.append(base_marker)

            # label for base station
            base_text = Marker()
            base_text.header.stamp = now
            base_text.header.frame_id = 'map'
            base_text.ns = 'rtk'
            base_text.id = 3
            base_text.type = Marker.TEXT_VIEW_FACING
            base_text.action = Marker.ADD
            base_text.pose.position.x = be
            base_text.pose.position.y = bn
            base_text.pose.position.z = bu + 1.0
            base_text.pose.orientation.w = 1.0
            base_text.scale.z = 0.4
            base_text.color.r = 0.6
            base_text.color.g = 0.8
            base_text.color.b = 1.0
            base_text.color.a = 1.0
            base_text.text = (
                f'RTK BASE\n'
                f'{self.base_lla[0]:.5f}, {self.base_lla[1]:.5f}\n'
                f'{self.distance_to_base/1000:.1f}km away'
            )
            markers.markers.append(base_text)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = GpsToRviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
