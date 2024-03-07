#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
#import time
import logging

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.pub_2 = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.sub_1 = self.create_subscription(LaserScan, '/scan', self.callback_laserscan, 1)
        self.sub_2 = self.create_subscription(Odometry, '/ego_racecar/odom', self.callback_odometry, 1)
        self.lidar_info_local = LaserScan()
        self.odometry_info_local = Odometry()

        self.logger = self.get_logger()
        self.logger.set_level(logging.DEBUG)  # Set the log level to DEBUG or another desired level
        self.logger.info("SafetyNode initialized") 

    def callback_laserscan(self, lidar_info):
        self.lidar_info_local = lidar_info
        self.publish()

    def callback_odometry(self, odometry_info):
        self.odometry_info_local = odometry_info

    def publish(self):
        TTC_threshold = 1
        v_x = self.odometry_info_local.twist.twist.linear.x
        v_y = self.odometry_info_local.twist.twist.linear.y
        self.logger.debug(f"Vx: {v_x}")
        self.logger.debug(f"Vy: {v_y}")
        self.logger.debug(f"# of lidar ranges: {len(self.lidar_info_local.ranges)}")
        self.logger.debug(f"Vx: {v_x}")
        self.logger.debug(f"angle min: {self.lidar_info_local.angle_min}")
        self.logger.debug(f"angle max: {self.lidar_info_local.angle_max }")
        
        self.angles = np.arange(self.lidar_info_local.angle_min, self.lidar_info_local.angle_max, self.lidar_info_local.angle_increment)
        self.ranges = np.array(self.lidar_info_local.ranges)
        self.range_rates = v_x * np.cos(self.angles)
        
        # Calculate TTC values while filtering out invalid values (NaN, Inf, division by zero)
        valid_mask = (self.range_rates > 0) & ~np.isnan(self.ranges) & ~np.isinf(self.ranges)
        valid_ttcs = np.divide(self.ranges[valid_mask], self.range_rates[valid_mask])
        
        if len(valid_ttcs) > 0:
            self.min_ttc = np.min(valid_ttcs)
            self.logger.debug(f"min_ttc {self.min_ttc}")
            if(self.min_ttc<= TTC_threshold):
                    ackermann_drive_result = AckermannDriveStamped()
                    ackermann_drive_result.drive.speed = 0.0
                    self.pub_2.publish(ackermann_drive_result)
                    self.logger.debug(f"called inside min_TTC<TTC Threshold: {ackermann_drive_result}")
        else:
            self.min_ttc = float('inf')  # or any other suitable value for no valid data
            
def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()