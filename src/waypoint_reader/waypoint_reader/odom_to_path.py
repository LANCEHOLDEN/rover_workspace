#!/usr/bin/env python3
"""
odom_to_path.py — converts Odometry topics to nav_msgs/Path for RViz visualization.

Subscribes:
  /odometry/filtered  -> publishes /path/filtered  (red)
  /odometry/wheels    -> publishes /path/wheels     (blue)
  /ins                -> publishes /path/ins         (green)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')

        self.declare_parameter('max_poses', 500)
        max_poses = self.get_parameter('max_poses').value

        self._paths = {}
        self._pubs = {}
        self._max_poses = max_poses

        sources = [
            ('/odometry/filtered', '/path/filtered'),
            ('/odometry/wheels',   '/path/wheels'),
            ('/ins',               '/path/ins'),
        ]

        for odom_topic, path_topic in sources:
            self._paths[odom_topic] = Path()
            self._paths[odom_topic].header.frame_id = 'odom'
            self._pubs[odom_topic] = self.create_publisher(Path, path_topic, 10)
            self.create_subscription(
                Odometry, odom_topic,
                lambda msg, t=odom_topic: self._callback(msg, t),
                10
            )

        self.get_logger().info('odom_to_path publishing:')
        for _, pt in sources:
            self.get_logger().info(f'  {pt}')

    def _callback(self, msg: Odometry, topic: str):
        path = self._paths[topic]
        path.header.stamp = msg.header.stamp
        path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        path.poses.append(pose)
        if len(path.poses) > self._max_poses:
            path.poses.pop(0)

        self._pubs[topic].publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
