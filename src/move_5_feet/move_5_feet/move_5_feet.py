import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveFiveFeet(Node):
    def __init__(self):
        super().__init__('move_five_feet_node')
        
        # Adjust topic names if your Zero3 uses different ones (e.g., /wheel/odometry)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odometry/wheels', self.odom_callback, 10)
        
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_distance = 1.524  # 5 feet in meters
        self.is_moving = True

        # Timer to publish commands at 10Hz
        self.timer = self.create_timer(0.1, self.move_robot)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Set starting position on the first message received
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f"Starting at: ({self.start_x}, {self.start_y})")

    def move_robot(self):
        if self.start_x is None or not self.is_moving:
            return

        # Calculate Euclidean distance
        distance_moved = math.sqrt(
            (self.current_x - self.start_x)**2 + 
            (self.current_y - self.start_y)**2
        )

        msg = Twist()
        if distance_moved < self.target_distance:
            msg.linear.x = 0.2  # Move forward at 0.2 m/s
            self.get_logger().info(f"Moved: {distance_moved:.2f}m")
        else:
            msg.linear.x = 0.0  # Stop
            self.is_moving = False
            self.get_logger().info("Target reached! Stopping.")
          

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveFiveFeet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
