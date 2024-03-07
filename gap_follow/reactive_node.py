#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32
import math
import numpy as np
import logging

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_follow_gap')

        # Topics & Subscriptions, Publishers
        lidar_scan_topic = '/scan'
        drive_topic = '/drive'
        angle_topic = '/angle'

        self.lidar_sub = self.create_subscription(LaserScan, lidar_scan_topic, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.angle_pub = self.create_publisher(Float32, angle_topic, 10)

        self.logger = self.get_logger()
        self.logger.set_level(logging.DEBUG)  # Set the log level to DEBUG or another desired level
        self.logger.info("Reactive wall_follow initialized") 

    def preprocess_lidar(self, scan_msg):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1. Setting each value to the mean over some window
            2. Rejecting high values (e.g., > 3m)
        """
        ranges_raw = np.array(scan_msg.ranges)

        half_window = 2
        mvg_window = 2 * half_window + 1  # window for moving average
        # append np.nan values so that the resulting moving average array has the same length
        ranges = np.append(np.append(np.array([np.nan] * half_window), ranges_raw), np.array([np.nan] * half_window))
        ranges = np.convolve(ranges, np.ones(mvg_window), 'valid') / (mvg_window)
        ranges[np.isinf(ranges) |
               (ranges > scan_msg.range_max) |
               (ranges < scan_msg.range_min)] = np.nan

        #centered around math.pi? maybe change to - angle minimum to get the correct ranges of angle
        angle_ranges = np.arange(len(ranges_raw)) * scan_msg.angle_increment + scan_msg.angle_min

        proc_ranges = ranges[(angle_ranges >= (-135 / 180) * math.pi) & (angle_ranges <= (135 / 180) * math.pi)]
        angle_ranges = angle_ranges[(angle_ranges >= -135 / 180 * math.pi) & (angle_ranges <= 135 / 180 * math.pi)]

        return angle_ranges, proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges

        The max gap should not include nan 
        """
        start_idx = 0
        max_length = 0
        curr_length = 0
        curr_idx = 0
        threshold = 1.1
        while(max_length<1):
            for k in range(len(free_space_ranges)):
                if free_space_ranges[k] > threshold:
                    curr_length += 1

                    # New sequence, store beginning index
                    if curr_length == 1:
                        curr_idx = k
                else:
                    if curr_length > max_length:
                        max_length = curr_length
                        start_idx = curr_idx
                    curr_length = 0

            if curr_length > max_length:
                max_length = curr_length
                start_idx = curr_idx
            
            if(max_length<2):
                start_idx = 0
                max_length = 0
                curr_length = 0
                curr_idx = 0
                threshold = threshold-0.1

        return start_idx, start_idx + max_length - 1

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of the best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        return int((start_i+end_i)/2)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        angle_ranges, proc_ranges = self.preprocess_lidar(data)

        # Find closest point to LiDAR
        min_idx = np.nanargmin(proc_ranges)

        print(min_idx)

        # Eliminate all points inside 'bubble' (set them to zero)
        rb = 105 # radius of bubble around the nearest point
        free_space_ranges = np.array(proc_ranges, copy=True)
        free_space_ranges[max(min_idx - rb, 0): min(min_idx + rb, len(free_space_ranges) - 1) + 1] = 0

        # Find max length gap
        start_idx, end_idx = self.find_max_gap(free_space_ranges)

        # Find the best point in the gap
        best_pt = self.find_best_point(start_idx, end_idx, free_space_ranges)
        if best_pt>=1080: best_pt = 1079

        # Publish Drive message
        angle = angle_ranges[best_pt]
        print("angle:")
        print(angle)
        angle_velocity = 0.0
        velocity = 1.0

        if 0 <= abs(angle) <= 10 / 180 * math.pi:
            velocity = 1.0
        elif abs(angle) <= 20 / 180 * math.pi:
            velocity = 0.5
        else:
            velocity = 0.2
            angle_velocity = 1.0  # rad/s

        #self.angle_pub.publish(Float32(data=angle))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.steering_angle_velocity = angle_velocity
        drive_msg.drive.speed = float(velocity)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    reactive_node= ReactiveFollowGap()
    rclpy.spin(reactive_node)
    reactive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
