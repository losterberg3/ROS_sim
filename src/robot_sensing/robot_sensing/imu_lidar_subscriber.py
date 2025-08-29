#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu

class imu_lidar_subscriber(Node):

    def __init__(self):
        super().__init__("IMUPLUSLIDAR")
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.call_back_lidar, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.call_back_imu, 10)

    def call_back_lidar(self, msg: LaserScan):
        self.get_logger().info(f"Received lidar: {msg.scan_time}")


    def call_back_imu(self, msg: Imu):
        self.get_logger().info(f"Received imu: {msg.orientation.z}")
        


def main(args = None):
    rclpy.init(args = args)
    node = imu_lidar_subscriber()
    rclpy.spin(node)
    rclpy._shutdown()

    


if __name__ == '__main__':
    main()
