#!/usr/bin/env python3

import rclpy
import math
import numpy
import csv
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

class imu_lidar_subscriber(Node):

    def __init__(self):
        super().__init__("IMUPLUSLIDAR")
        
        self.orientation_q = [0, 0, 0, 1]
        self.queues = []

        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.call_back_lidar, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.call_back_imu, 10)
        self.imu_sub = self.create_subscription(Odometry, "/odom", self.call_back_odom, 10)

    def call_back_imu(self, msg: Imu):
        self.latest_imu = msg
        queue = {
            "wx": self.latest_imu.angular_velocity.x,
            "wy": self.latest_imu.angular_velocity.y,
            "wz": self.latest_imu.angular_velocity.z,
            "t": self.latest_imu.header.stamp.sec + 1e-9 * self.latest_imu.header.stamp.nanosec
        }
        self.queues.append(queue)

    def call_back_lidar(self, msg: LaserScan):
        self.latest_lidar = msg
        
        lidar_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        
        while len(self.queues) > 0 and self.queues[0]["t"] < lidar_time:
            if len(self.queues) > 1:
                dt = self.queues[1]["t"] - self.queues[0]["t"]
                self.queues.pop(0)
            else:
                dt = lidar_time - self.queues[0]["t"]
                self.queues[0]["t"] = lidar_time

            wx, wy, wz = self.queues[0]["wx"], self.queues[0]["wy"], self.queues[0]["wz"]

            angle = math.sqrt(wx**2 + wy**2 + wz**2) * dt
            
            if angle > 0:
                axis = [wx/angle, wy/angle, wz/angle]
                dq = [
                    axis[0]*math.sin(angle/2),
                    axis[1]*math.sin(angle/2),
                    axis[2]*math.sin(angle/2),
                    math.cos(angle/2)
                ]
                
                self.orientation_q = self.quaternion_multiply(self.orientation_q, dq)

        self.fused_imuandlidar()

    def fused_imuandlidar(self):
        angle = 0
        self.rangesxy = []
        for range in self.latest_lidar.ranges:
            rangexy = [
                range * math.cos(angle),
                range * math.sin(angle),
                0
            ]
            self.rangesxy.append(rangexy)
            angle += 2 * math.pi / 360

        r = Rotation.from_quat(self.orientation_q)
        self.rotated_points = r.apply(self.rangesxy)

        self.apply_odom()

    def call_back_odom(self, msg: Odometry):
        self.latest_odom = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        #self.get_logger().info(f"{msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec}")
        
    def apply_odom(self):
        self.world_points = numpy.array(self.rotated_points) + numpy.array(self.latest_odom)

        with open('world_points.csv', 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for point in self.world_points:
                writer.writerow([point[0], point[1], point[2]])
              
    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        return [x, y, z, w]

def main(args = None):
    rclpy.init(args = args)
    node = imu_lidar_subscriber()
    rclpy.spin(node)
    rclpy._shutdown()

if __name__ == '__main__':
    main()
