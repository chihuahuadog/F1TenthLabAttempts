"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
from platform import node
import numpy as np
from numpy import linalg as LA
import math

import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

# TODO: import as you need

#global variables
NUM_ROWS = 10
NUM_COLS = 10
MAP_PATH = "sim.png"
START_X = 0
START_Y = 0
GOAL_X = 200
GOAL_Y = 200
NUM_SEGMENTS = 100
MAX_NODES = 100
MAX_CONNECT_LENGTH =  200
REACHED_GOAL = False

#global function
def png_to_occupancy_grid(png_path):
    # Read the PNG image
    image = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)

    # Normalize pixel values to range between 0 and 100
    normalized_image = cv2.normalize(image, None, 0, 100, cv2.NORM_MINMAX)

    # Invert values (if needed) - obstacles should have higher values
    inverted_image = 100 - normalized_image

    # Convert to numpy array
    occupancy_grid_array = np.array(inverted_image)

    return occupancy_grid_array

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT* distance from start
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 1)

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.og_pub = self.create_publisher(OccupancyGrid, '/og', 1)


        # class attributes
        # TODO: maybe create your occupancy grid here
        self.car_lidar = LaserScan()
        self.car_odom = Odometry()
        self.og = OccupancyGrid();
        self.nodes = [Node()]
        self.nodes_count = 1
    
        self.occupancy_map = png_to_occupancy_grid(MAP_PATH)

    def scan_callback(self, scan_msg):
        self.car_lidar = scan_msg
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        og_msg = OccupancyGrid()
        og_msg.header.stamp = self.get_clock().now().to_msg()
        og_msg.frame_id ="map_frame"
        og_msg.info.resolution = 0.05
        og_msg.info.width = 10
        og_msg.info.height = 10
        og_msg.info.orgin_position.x = 0.0
        og_msg.info.orgin_position.y = 0.0
        og_msg.info.orgin_position.z = 0.0
        og_msg.info.orgin_orientation.x = 0.0
        og_msg.info.orgin_orientation.y = 0.0
        og_msg.info.orgin_orientation.z = 0.0
        og_msg.data = [100, 0, 0, -1, 0, 0, 0, 100]

        self.og_pub.publish(og_msg)


    def odom_callback(self, odometry_info):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        self.car_odom = odometry_info

        REACHED_GOAL = False

        #initialize the start position
        self.nodes[0].x = odometry_info.pose.pose.position.x 
        self.nodes[0].y = odometry_info.pose.pose.position.y
        self.nodes[0].cost = 0;
        self.nodes[0].parent = None
        self.nodes[0].is_root = True
        
        while self.nodes_count < MAX_NODES and not REACHED_GOAL:
        ## RRT* Algorithm:
        # 1.Pick a random node q_rand
            rand_node = Node()
            coordinates = self.sample()
            rand_node.x = coordinates[0]
            rand_node.y = coordinates[1]

            #check if goal is reached and break if true
            for i in range(self.nodes_count):
                if self.in_goal_range(self.nodes[i].x, self.nodes[i].y):
                    REACHED_GOAL = True
                    break

        # 2.Find the closest node q_near from the explored nodes to branch out from towards q_rand
            closest_node = self.nearest(self.nodes, rand_node)
        # 3. Steer from q_near toward q_rand: interpolate if the node is too far away,
        # reach q_new. Check that obstacle is not hit
            q_new = self.steer(closest_node, rand_node)
            if self.check_collision():
                #drive node
                q_new.cost = self.cost(self.nodes, closest_node) + self.line_cost(q_new, closest_node)

        # 4. Update the cost of reaching q_new from q_near, treat it as C min. 
                q_min = closest_node
                c_min = q_new.cost
        # For now, q_near acts as the parent node of q_new
                q_new.parent = closest_node
        # 5. From the list of visited nodes, check for the nearest neighbors with a given radius, insert in a list q_nearest
                list_q_nearest = self.near(self.nodes, q_new)
        # 6. In all the members of q_nearest, check if q_new can be reached from a different parent node
        # with lower cost than Cmin, and without colliding with the obstacle
                index_smallest = 0
                lowest_cost = list_q_nearest[0].cost
                for i in range(len(list_q_nearest)):
                    if self.check_collision(q_new, list_q_nearest[i]):
                        current_cost = self.cost(self.nodes, list_q_nearest[i]) + self.line_cost(q_new, list_q_nearest[i])
                        if current_cost < lowest_cost:
                            index_smallest = i
                            lowest_cost = current_cost
        #Update the parent to the least cost-from node
        if lowest_cost < q_new.cost:
            q_new.parent = list_q_nearest[index_smallest]
            q_new.cost = lowest_cost
        # 7. Add q_new to nodelist
            self.nodes.append(q_new)
        # 8. Continue until maximum number of nodes if reached or goal is hit
            self.nodes_count += 1
            
        
    #Look for the closest node to the goal
        goal_node = Node()
        goal_node.x = GOAL_X
        goal_node.y = GOAL_Y
        D = []
        for j in range(self.nodes_count):
            temp_dist = self.line_cost(self.nodes[j], goal_node)
            D.append(temp_dist)
    
    #Search backward from goal to start to find the optimal least cost path
        [val, idx] = min(D)
        q_final = self.nodes[idx]
        goal_node.parent = q_final
        self.nodes.append(goal_node)
            
        return None

    def in_goal_range(self, x, y):
        inRange_X = x<=10+GOAL_X and x>=GOAL_X-10
        inRange_Y = y<=10+GOAL_Y and y>=GOAL_Y-10
        return inRange_X and inRange_Y
    
    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """

        already_exists = True
        is_obstacle = True

        while already_exists or is_obstacle:
            x = int(random.random() * NUM_ROWS)
            y = int(random.random() * NUM_COLS)
            
            already_exists = False
            is_obstacle = False

            for i in range(self.nodes_count):
                if x == self.nodes[0].x or y == self.nodes[0].y:
                    already_exists = True
            
            val_og = self.occupancy_map[x, y]
            if val_og != 0:
                is_obstacle = True

        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        ndist = []

        for n in tree:
            tmp = self.line_cost(n, sampled_point)
            ndist.append(tmp)

        val, idx = min((val, idx) for idx, val in enumerate(ndist))
        #q_near = tree[idx]
        return val, idx

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        distance = self.line_cost(nearest_node, sampled_point)
        new_node = Node()
        if distance > MAX_CONNECT_LENGTH:
            new_node.x = new_node.x + ((sampled_point.x-nearest_node.x)*MAX_CONNECT_LENGTH)/distance
            new_node.y = new_node.y + ((sampled_point.y-nearest_node.y)*MAX_CONNECT_LENGTH)/distance
        else:
            new_node.x = sampled_point.x 
            new_node.y = sampled_point.y

        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method checks if the path between nearest_node and new_node is collision-free.

        Args:
            nearest_node (Node): Nearest node on the tree.
            new_node (Node): New node from steering.
        Returns:
            collision (bool): True if the path between the two nodes is collision-free.
        """
        # Ensure nearest_node and new_node are not None
        if nearest_node is None or new_node is None:
            return False

        # Simple collision check: Check if the line between two points crosses any obstacle in the occupancy grid
        resolution = 0.05  # Assuming a resolution of 0.05 for simplicity
        num_points = int(math.ceil(self.line_cost(nearest_node, new_node) / resolution))

        for i in range(num_points):
            alpha = float(i) / float(num_points - 1)
            x = nearest_node.x + alpha * (new_node.x - nearest_node.x)
            y = nearest_node.y + alpha * (new_node.y - nearest_node.y)

            # Check if the point (x, y) is inside an obstacle
            if self.occupancy_map[int(x)][int(y)] != 0:
                return False  # Collision detected

        return True 


    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        total_cost = 0
        while not node.is_root:
            total_cost += self.cost(self.nodes, node.parent)
        
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return math.sqrt((n1.x-n2.x)**2 + (n1.y - n2.y)**2)

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        r = 60
        neighbor_count = 0
        for j in range(self.nodes_count):
            if self.check_collision(self.nodes[j], node) and self.line_cost(node, self.nodes[j])<r:
                neighbor_node = Node()
                neighbor_node.x = self.nodes[j].x
                neighbor_node.y = self.nodes[j].y
                neighbor_node.cost = self.nodes[j].cost
                neighborhood.append(neighbor_node)
                neighbor_count+=1
        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()