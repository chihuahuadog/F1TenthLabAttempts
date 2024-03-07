import rclpy
from rclpy.node import Node

import math
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import logging
class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'


        # TODO: create subscribers and publishers
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

        # TODO: set PID gains
        # self.kp = 0..01
        # self.kd = 0.10
        # self.ki = 0.001
        self.kp = 1.6
        self.kd = 0.05
        self.ki = 0.001

        # TODO: store history
        # self.integral = 
        # self.prev_error = 
        # self.error = 
        self.integral = 0.0
        self.prev_error = 0.0
        self.my_prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.servo_offset = 0.0
        self.angle_range = 270
        self.desired_distance = 0.8
        self.car_length = 0.5

        self.prev_secs = None
        self.prev_nsecs = None
        self.secs = None
        self.prev_nsecs = None
        self.lookahead_distance = 0.5

        self.logger = self.get_logger()
        self.logger.set_level(logging.DEBUG)  # Set the log level to DEBUG or another desired level
        self.logger.info("Wall follow initialized") 
    
    def my_get_range(self, range_data, angle):

        #TODO: implement
        proc_range = np.array(range_data.ranges);
        new_range = proc_range[(proc_range <= range_data.range_max) & (proc_range >= range_data.range_min)]

        x =((angle + 135)/270.0) * len(new_range)
        ind = int(x)
        self.logger.debug(f"my_ind: {ind}")
        return new_range[ind]
    
    
    def my_get_error(self, range_data, dist):
        a = self.my_get_range(range_data, 30)
        b = self.my_get_range(range_data, 90)

        #forward = self.my_get_range(range_data, 0);
        #print(forward);

        self.logger.debug(f"my_error a/b: a: {a} b: {b}")

        theta = (60 / 180) * math.pi
        num = a * math.cos(theta) - b
        den = a * math.sin(theta)
        alpha = math.atan(num/den)
        distance = b * math.cos(alpha) + self.lookahead_distance * math.sin(alpha)

        self.logger.debug(f"my_distance: {distance}")

        error = dist - distance
       
        self.logger.debug(f"my_error: {error}")
        return error

    def my_pid_control(self, error, dt):
        # TODO: Use kp, ki & kd to implement a PID controller
        integral = error * dt
        angle_deg = -(self.kp * error + self.kd * (error - self.my_prev_error) / dt + self.ki * integral)
        print(f"kp: {self.kp * error} derivative: {(error - self.my_prev_error)} integral: {self.ki * integral}")
        temp = 0;
        if angle_deg < 0:
            temp = -1;
        else:
            temp = 1;
        angle = temp * (angle_deg%((4/3)*math.pi));
        self.logger.debug(f"my_angle: {angle}")
        if 0 <= abs(angle) <= 10 / 180 * math.pi:
            velocity = 1.5
        elif abs(angle) <= 20 / 180 * math.pi:
            velocity = 1.0
        else:
            velocity = 0.5
        #self.logger.debug(f"velocity: {velocity}")
        self.my_prev_error = error
    
        
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        self.logger.debug(f"New cycle")
        my_error = self.my_get_error(msg, self.desired_distance);

        self.secs = msg.header.stamp.sec
        self.nsecs = msg.header.stamp.nanosec

        if self.prev_nsecs is None or self.prev_secs is None:
            self.prev_secs = self.secs;
            self.prev_nsecs = self.nsecs
            return

        self.dt = (self.secs - self.prev_secs) + (self.nsecs - self.prev_nsecs)*(1e-9)
        self.my_pid_control(my_error, self.dt)

        self.prev_secs = self.secs;
        self.prev_nsecs = self.nsecs


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()