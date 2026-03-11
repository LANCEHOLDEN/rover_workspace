#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.twist = Twist()

        self.distance = 0.5  # meters
        self.speed = 0.5     # m/s
        self.duration = self.distance / self.speed  # seconds

        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'Moving forward {self.distance} m at {self.speed} m/s'
        )

    def timer_callback(self):
        elapsed_time = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        if elapsed_time < self.duration:
            self.twist.linear.x = self.speed
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)
        else:
            # Stop robot
            self.twist.linear.x = 0.0
            self.publisher_.publish(self.twist)

            self.get_logger().info('Reached 0.5 meters forward — stopping.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MoveForward()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

