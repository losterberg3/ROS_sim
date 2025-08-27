#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub = self.create_publisher(Twist)
        self.get_logger().info("Draw circle node has been started")
        

def main(args=None):
    rclpy.init(args=args)
    rclpy.shutdown()
