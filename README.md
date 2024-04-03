﻿## Lab2: Automatic Emergency Braking
\
The goal of the automatic emergency braking system was to get the car to stop in time if there was a collision about to happen. This is important so that the car remains safe and does not cause damage to itself or anything else. This is accomplished by developing a ros2 safety node in python which will stop the car when traveling at higher velocities. There are a couple of messages that are used in the system which include Laserscan messages which are published by the laser scanner attached to the car and the Odometry message which is published by the car in real life and by the simulation and the AckermannDriveStamped Message. The Laserscan message contains all the range data that the lidar recorded, the Odometry message includes the car’s position, orientation, and velocity. The safety node listens to the Odometry Messages and the Laserscan messages and makes a time to collision calculation and then publishes a AckermannDriveStamped Message to the car. Essentially the calculation is as follows in which each element from the array of distances from the lidar is divided by the velocity in the particular direction from which the distance was measured. The infinity values and naf values are filtered out and the closest time to collision is examined for the threshold value.
\
  

## Lab3: Wall Following
\
The goal of the wall following node was to get the car to follow a wall autonomously. The wall following node mostly dealt with the use of PID control to control the car as it moves around a hallway. The general equation for the PID controller is described by the following equation:
\
![](https://lh7-us.googleusercontent.com/dcIK0mzY9S4Y08dp1e15e7ld0PymU6ndzK9LVCXprQi_ZICPoD2b1-XPjc9UyOIoeWxfe03eKrCbklev5BjzIKU2a_R7IHJ5GOpd77iOaFLlox_B39NhN9YiEMn7D5E1jmCMHV_9WCadJbtIGjcmziU)

Kp, Ki and Kd are constants that describe how much weight each of the proportional, integral and derivative components contribute to the control output u(t). First the proportional control is added to improve the rise time, followed by the derivative control to reduce the overshoot and then the integral control to reduce the steady state error. Essentially we increase the kp until we get a small oscillation and ki is used to reduce steady state error and then increase kd until oscillations are removed. In the wall following node, we set a desired position from the wall for the car and the error is consequentially the error between the desired and the actual distance to the wall. We generally only considered the vehicle following the right wall. Following the instructions provided by the UPenn open repo labs, we used the formula
\
![](https://lh7-us.googleusercontent.com/BCaLA6EHp6jWiS_qlRkOKhryFF8M5ClksKCIgLQ0efJ8Nlf220xgx4s8jSxwIweJbJOQd72gUE3jdG56tg7MFQhQUcw6tOE1fcFEqTJx1qBZ2T9EbGnHwF8fScuEzQIVj3BMBYkgGARU8yOEySanfGU) and \![](https://lh7-us.googleusercontent.com/0MWaYTNIaCI435FFsNCgxNPSRgHegVq4x0giskxYmjItXg9Lx4OkZR52DtmvuIBdp7QO08ay6kqIIgrwKHT39f7L067n8yn2HYOy0N3Ozk1dhF7Bo9H1Fz5ov5wtzJqfMLKl6CQfTvXRzBRX7cb5xVQ)![](https://lh7-us.googleusercontent.com/gdvUQ8--xwRmcTGyz1M9CCOk4t6wxgQJ28O10zJqLNF68Jb6bRkTByxMrGtgIKPwZtENTkMaq8nbV0JITGmdt5Kjwysus7GX4KFap3U2vfQtCyViHjY-eaFlKqqPE2caCmln996t8Tp8ZSgp3JGKOt8)
\
The two laser scan distances are a and b which is used to calculate the angle α between the car’s x-axis and the right wall. α is then used to find the distance Dt to the car which is then used to estimate the future distance Dt+1 to the car. This distance is run through the PID function above to get the steering angle and then depending on the steering angle, the speed of the car is then set. The error is 1-Dt and we use Dt+1 since the car is traveling very fast and by the time Dt is calculated and the car responds to it, the car may actually be somewhere else so we use Dt+1 instead. Also, in order to slow the car down around corners for safety reasons, we also varied the speed of the car depending on the steering angle. If the steering angle was between 0-10 degrees the car drives at 1.5m per second. If the steering angle is between 10 and 20 degrees, the speed is 1.0 m/s and otherwise the speed is 0.5m/s. Generally the problem with this approach is that it tries to do a one size fits all. Sometimes depending on the angles, the calculation is not made fast enough to turn the car sharply enough before the car gets stuck.
\
We changed the way the car moved by adjusting the kp, kd, and ki values. The proportional term controls how quickly to turn the steering wheel when the heading is not set at the right value. A low p results in sluggish steering, a higher p will give a quicker response, too high a p will result in exaggerated oscillations. The derivative term is used to limit the speed of the steering response. The integral term adds more steering action if the error persists for too long.

\
![Imgur](https://i.imgur.com/E1NEh11.png)

  \

# Lab 4: Follow the Gap

The follow the gap node essentially serves as a way for the car to learn how to find gaps between obstacles or cars and make its way through the gap without colliding into anything else. The algorithm used obtains the laser scans and preprocesses them by smoothing them out to a line. The smoothing is done using a moving window of 5 numbers. np.nan values are appended to the ends of the array so that the resulting moving average array has the same length as the original array.The values that are too large or infinity are replaced with np.nan. Since we only wanted to limit the gaps to be found to be the gaps in front of the car and not in the back, we limit the ranges to those corresponding to the angles between -135 degrees and 135 degrees where 0 degrees is the direction right in front of the car.The closest point is then found in the lidar array and a bubble is drawn around it. This is so that the car learns to avoid the nearest obstacle and the bubble essentially forms a protective area around the nearest obstacle so that the car does not bump into it. The nonzero points are then considered empty gaps. It is not enough to steer in the direction of the furthest distance obtained by the laser scan since the gap may not be wide enough for the car to pass through. The maximum gap is a better solution since it has the highest chance of ensuring that the car can pass through the gap. The maximum gap is found by finding the largest consecutive non-zero elements in the ranges array. The car is then moved towards the center of this gap by publishing a AckermannDriveStamped to the /drive topic.
\
  

# Lab 6: Pure Pursuit

The slam toolbox is ros2 is essentially used to create a map. The slam_toolbox is built into ros2 and can be used as long as there is a laser scan message being published. The map is essentially built based on the laserscan and the object's perception of its surroundings. Essentially the pure pursuit node is the car following a planned trajectory through the map. The way the car moves to follow the path is dependent on the curvature of the arc. To create the points of the trajectory, we also wrote the waypoint logger node which essentially logs the position of the car in the simulation when the car moves forward. So basically in the simulation we used the keyboard teleop in ros2 to drive the car in the simulation while the waypoint logger node logged the path of the node. The pure pursuit algorithm we utilized can be described as follows. The path is defined by a series of waypoints described by the csv file created earlier by the waypoint logger node. With the lookahead distance, we determine the next point to pursue on the waypoints. The target point is the point on the path that the vehicle will try to reach.Once the target point is determined, the steering command is calculated based on the current position and the orientation of the vehicle’s current heading and the line connecting the vehicle to the target point. The steering command is used to adjust the vehicle’s steering angle which causes it to turn towards the target point and the process is repeated as the vehicle moves. The algorithm is simple, but does have considerable limitations with regards to sudden changes in the path.
\
  

https://github.com/chihuahuadog/F1TenthLabAttempts/assets/40186166/cb1ea75f-0582-420a-9a74-9fa9c4519fe3



# Lab 7: RRT and path planning

Essentially what happens in the rrt node in python is that a goal location and a start position is provided and the rrt algorithm is used to find a path for the car and the car follows the path using the pure pursuit algorithm. The rrt algorithm changes the path based on the occupancy grid node. The occupancy grid node shows where there are obstacles in the map and is updated using the Laserscan messages received from the simulation. The rrt algorithm essentially works by building a tree. The tree starts with containing only the initial configuration of the robot which is given through the 2D Pose Estimate in the rviz simulation and when used, publishes to the /initialpose topic. A while loop is used to explore the current known environment. At each iteration, a random point is sampled in the configuration space. Then find the nearest node in the existing tree to the sampled point and take a step towards the random point. If the step is collision free, add the new node to the tree and connect it to the nearest node.  
\
The while loop is terminated when the number of predefined iterations is reached or when the goal configuration is reached. Once the goal configuration is reached, there is a traceback from the goal node to the root of the tree. The final path is the sequence of configurations from the initial configuration to the goal configuration, obtained by following the parent links from the goal node to the root. RRT samples random configurations in the configuration space and the tree is grown incrementally by adding new nodes and edges which allows the algorithm to explore the space more efficiently. RRT generates paths locally which makes it suitable for dynamic environments such as the one in the car simulation. The car’s environment is being continuously updated by the lidar scans. However, RRT is not guaranteed to find the optimal solution. A modification to the RRT algorithm to help find a better path would be the RRT* algorithm.
\
RRT* optimizes the paths generated by the tree to minimize path length and achieves this by adjusting the tree structure during the expansion process. In RRT*, each node in the tree maintains a cost to come value which represents the cost of reaching that node from the start configuration and a cost to goal value which represents the estimated cost of reaching the goal from that node. During expansion, RRT* considers the cost to goal value of potential nodes and selects the one that minimizes the total cost to come and cost to go. Unlike the basic RRT algorithm, the RRT* considered rewiring the tree structure and after adding a new node, RRT* evaluates nearby nodes and considers whether rewiring the connections to the new node would reduce the overall path cost. If rewiring results in a lower total cost, RRT* updates the tree structure. So far we are still working on the RRT algorithm and how to locate the car on the occupancy grid. We have plans to continue with the RRT* algorithm once the RRT algoirthum has been figured out.ft corner of the navigation bar.


