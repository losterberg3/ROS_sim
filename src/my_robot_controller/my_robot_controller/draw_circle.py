#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Subscriber(Node):
    
    def __init__(self):
        super().__init__("subber")
        self.create_subscription(Twist, "/cmd_vel", self.call_back, 10)

    def call_back(self, msg: Twist):
        self.get_logger().info(str(msg.linear.x))
  

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()