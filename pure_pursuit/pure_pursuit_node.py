#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from pathlib import Path
import pandas as pd
import tf2_ros
import tf2_geometry_msgs
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # topic names
        self.rviz_marker_topic = "waypoint_vis"
        self.odom_topic = '/ego_racecar/odom'
        self.drive_topic = '/drive'

        # frame name
        self.target_frame = "ego_racecar/base_link"
        self.global_frame = "map"

        #self.main_waypoint_filepath = Path(self.get_parameter("/f1tenth/waypoint_file").get_parameter_value().string_value)
        self.main_waypoint_filepath = Path("/sim_ws/src/pure_pursuit/logs/waypoint_obstacle.csv")
        self.lookahead_dist = 0.5
        self.steering_gain = 1.0  # gain k for the steering angle

        self.main_waypoint_df = pd.read_csv(self.main_waypoint_filepath)

        # create ROS subscribers and publishers
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.pose_callback, 10)
        self.current_waypoint_pub = self.create_publisher(Marker, self.rviz_marker_topic, 10)

        # "listen" for the transformation
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)


    def visualize_waypoint(self, x, y):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.id = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        return marker

    def get_transformed_point(self, row, trans):
        initial_point = PointStamped()
        initial_point.point.x = row["pos_x"]
        initial_point.point.y = row["pos_y"]
        transformed_point = tf2_geometry_msgs.do_transform_point(initial_point, trans)

        return transformed_point

    def get_waypoint(self, trans):
        """Select a waypoint based on its x and y, in the car frame
        """
        self.main_waypoint_df["trans_pt"] = self.main_waypoint_df.apply(lambda row: self.get_transformed_point(row, trans), axis=1)
        self.main_waypoint_df["trans_x"] = self.main_waypoint_df["trans_pt"].apply(lambda row: row.point.x)
        self.main_waypoint_df["trans_y"] = self.main_waypoint_df["trans_pt"].apply(lambda row: row.point.y)
        self.main_waypoint_df["dist"] = (self.main_waypoint_df["trans_x"]**2 + self.main_waypoint_df["trans_y"]**2)**0.5

        # only choose points in front of the car, to prevent going backwards
        ahead_waypoint_df = self.main_waypoint_df[self.main_waypoint_df["trans_x"] > 0]

        # choose the waypoint with distance closest to lookahead distance
        idx = abs(ahead_waypoint_df["dist"] - self.lookahead_dist).idxmin()
        point = ahead_waypoint_df.loc[idx]

        return point["pos_x"], point["pos_y"], point["trans_x"], point["trans_y"], point["dist"]

    def pose_callback(self, pose_msg):
        try:
            # get transformation from global frame to local frame
            trans = self.tfBuffer.lookup_transform(self.target_frame, self.global_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warning("Cannot find transform")
            return

        # find the current waypoint to track using methods mentioned in lecture
        x, y, trans_x, trans_y, L = self.get_waypoint(trans)

        # calculate curvature/steering angle
        angle = self.steering_gain * 2 * trans_y / (L**2)
        if angle > 0.4189:
            angle = 0.4189
        elif angle < -0.4189:
            angle = -0.4189

        # change velocity depending on the steering angle
        velocity = 1.0
        time_stamp = self.get_clock().now()
        if 0 <= abs(angle) <= 10 / 180 * math.pi:
            velocity = 1.5
        elif abs(angle) <= 20 / 180 * math.pi:
            velocity = 1.0
        else:
            velocity = 0.5

        # publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = time_stamp.to_msg()
        drive_msg.header.frame_id = self.target_frame
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        marker = self.visualize_waypoint(x, y)
        self.current_waypoint_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    pp_node = PurePursuit()
    rclpy.spin(pp_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
