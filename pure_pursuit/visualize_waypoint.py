#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from pathlib import Path
import pandas as pd

class VisualizeWaypointNode(Node):
    def __init__(self):
        super().__init__('visualize_waypoint_node')
        self.rviz_markers_topic = "/waypoint_vis_array"
        self.frame_id = "map"
        self.main_waypoint_filepath = Path("/sim_ws/src/pure_pursuit/logs/waypoint_obstacle.csv")
        self.main_waypoint_df = pd.read_csv(self.main_waypoint_filepath)
        self.marker_array_msg = MarkerArray()

        self.publisher = self.create_publisher(MarkerArray, self.rviz_markers_topic, 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def make_markers(self, row):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.id = row.name
        marker.pose.position.x = row["pos_x"]
        marker.pose.position.y = row["pos_y"]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        return marker

    def publish_markers(self):
        self.marker_array_msg.markers = self.main_waypoint_df.apply(self.make_markers, axis=1).tolist()
        self.get_logger().info('Publishing waypoint markers')
        self.publisher.publish(self.marker_array_msg)

def main(args=None):
    rclpy.init(args=args)
    visualize_node = VisualizeWaypointNode()
    rclpy.spin(visualize_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
