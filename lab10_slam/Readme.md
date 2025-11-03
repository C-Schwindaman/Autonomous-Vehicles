# Lab 10: Mapping
## ECE-CSE 434

This can be done with physical robots *or* with simulated robots in Gazebo on HPCC. However, the expectation is to do it for TurtleBot3 on `ROS:Humble` on HPCC.  

# Exercise 1: Obstacles in LiDAR (10 Points)

LiDAR is a very useful sensor for navigation including finding obstacles.  In this exercise we will investigate clustering as a means to partition the nearby world into obstacles. Your task is to convert a LiDAR scan into a collection of obstacles and visualize the obstacles in RViz. 

Create a ROS package called `bot_slam` and use the provided code `obstacles.py` as a starting point for your ROS node. The code shows how to subscribe to laser returns, turn them into 2D or 3D points, and how to publish detected obstacles as spherical marks that can be viewed in RViz.  Your task is to complete the point processing that clusters LiDAR points into obstacles.  You can run your node on output from a simulated robot and view both the LiDAR points and obstacles using RViz.

You will be creating a block world in Gazebo with some obstacles like this around the Turtlebot:

![Obstacles](.Images/gazebo.jpg)

Use an emtpy world model with Gazebo and the Turtlebot3 robot:
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
Add a collection of obstacles around your robot.  Then run your code with:
```
ros2 run bot_map obstacles
```
Visualize the output, in RViz, and you should see something like the below with the red dots being the LiDAR points and a sphere marker at the centroid of each obstacle:

![RViz](.Images/rviz.jpg)

Note that in RViz you'll need to add the various topics indicated in the figure in order to see the above.  Make sure to select `base_link` in the `Fixed Frame` line of `Global Options`.  For the `LaserScan`, select style `Points` for better visibility.  Your detected obstacle centers should show up using the `MarkerArray` topic and setting its topic as `/obstacles`

Now, you will need to decide what algorithm to use for clustering.  Consider the below diagram showing a set of lidar rays each having a `range` that specifies how far away the object is along that ray.  The black dots are the centroids of the visible points on the obstacle.

![rays](.Images/rays.png)

One way to cluster obstacles is to do connected components, where sequentially adjacent points are connected if the difference in their ranges is small.  In the above example, there are two objects modeled with two connected components and the difference in range between points `5` and `6`, labeled $\Delta$r, is greater than a threshold `T`, resulting in points `5` and `6` being disconnnected.  

Here is how to implement this.  The Lidar rays are scanned every one-degree interval in azimuth.  For each point with index *i*, consider its left neighbor with index *i+1*.  If the range (or length of the ray) *i+1* is close to the range of point *i*, then declare *i* and *i+1* are on the same object.  Of course, exclude all points with range 0 or range infinity.  In comparing ranges, you can use a threshold `T` of 0.25 meters.  Now, each point also has its own $x$ and $y$ coordinates.   Starting at the first valid point (above would be point 2), you can create an obstacle and start summing the $x$ and $y$ coordinates until you find a point not on the obstacle, here point `6`.  Then divide the cumulative sum of $x$ and $y$ by the number of points on the obstacle (`4` for the green obstacle), and you'll get the centroid of the obstacle points. Store the centroid in a list and repeat until you have iterated through all points.  Also, be careful about wrapping abound from the last to first point, as those may be on the same obstacle or not.  
Make a screen capture of the output sphere markers and lidar points on RViz.  Save this capture as an image `sphere.png`.  

For this exercise, submit the following:  
* ROS package code `bot_slam` in `lab10_slam/` with node `obstacles.py`,
* Output screenshot `sphere.png` in `lab10_slam/ex1/`.


# Exercise 2: SLAM + Navigation (5 Points)

This exercise will explore mapping and navigation using the Turtlebot3 on HPCC.  

**(a)** Follow this [Turtlebot3 SLAM tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/).  Make sure to select **Humble** at the top of the page.  
You need to do it for the `AutoRace` world. Launch it using the following in the terminal with `Humble` shell activated:
```
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
```

Ensure the environment around your robot has some static obstacles.  Then teleoperate your robot around and build a map of its environment. Save your map as `map.pgm` and `map.yaml` in the `lab10_slam/bot_slam/maps/` folder.

**(b)** For navigation, follow the [Turtlebot3 navigation tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/).  Make sure to select **Humble** at the top of the page. 

This will use the map you just created in *(a)*.  You should be able to specify a target pose for your robot and have it navigate to that location.  Make a screen capture of the navigation map as the robot navigates to its target pose.  Save this capture as an image `nav_map.png`.  
**Note:** while you specified a target pose manually, it is not too hard to write a ROS node to specify the target, which is what we'll do for the next exercise.  

For this exercise, submit the following:
* `map.pgm`  and `map.yaml` in `lab10_slam/bot_slam/maps/`
* `nav_map.png` in `lab10_slam/ex2/`

# Exercise 3: Waypoint Follower (10 Points)

In this exercise, we will write a ROS Node that reads a list of poses from a file and sends them one by one to the `Nav2` action server. Before you go ahead and do that, read through these tutorials and familiarize yourself with the Actions and Action Servers/Clients in ROS:
* [Understanding Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
* [Creating Actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)
* [Writing Action Servers and Clients](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)  

First, we write a `JSON` file to specify the list of waypoints. A `YAML` file is also very common for such purposes in ROS ecosystem and can be loaded directly as a parameter. Read through the following to familiarize yourself with ROS Parameters:
* [Understanding ROS Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)  
The waypoints are specified as $(x, y, yaw)$ in the `map` reference frame, which is a static, world-fixed frame that is created by the SLAM node (Ex 2). When you ran SLAM, it picked a point to be the origin $(0,0)$, which is usually the exact spot where the robot was when you started the SLAM node. The `.pgm` and `.yaml` files are saved relative to this `map` frame.  

### Why `map` frame and not `odom` frame?  
The `odom` frame tracks the robot's movement from where its wheel odometry started. This frame drifts over time (wheel slips, bumps, etc.).  The localization package (AMCL) running in `nav2` constantly calculates the difference (the transform) between the map frame and the odom frame to correct for this drift.  By sending goals in the `map` frame, you are giving `nav2` a static, reliable target.  

For simplicity, you have been provided with the `waypoints.json` file that contains a list of 4 waypoints **relative to the initial spawn location of the TurtleBot in `autorace` world**. Copy this file to `bot_slam/config/` directory.  


Next, write a Python node called `waypoint_follower.py` that uses the `rclpy.action.ActionClient`.  The skeleton of this code has been provided to get you started.  The `nav2` stack uses an action called `MapsToPose` of type `nav2_msgs/action/NavigateToPose`.  
Key steps for `waypoint_follower.py`:
* Declare a parameter (e.g., waypoint_file_path) using `self.declare_parameter` to get the path to the `JSON` file.
* Create an action client like:   
```
self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
    self.get_logger().info('Action server not available, waiting...')
```  
* Read the file path parameter and use the json library to load the waypoints like:  
```
waypoint_file = self.get_parameter('waypoint_file_path').get_parameter_value().string_value
with open(waypoint_file, 'r') as f:
    data = json.load(f)
    self.waypoints = data['waypoints']
```
* Write the `send_next_goal()` function which achieves the following:
    * Creates a `MapsToPose.Goal` message,
    * Converts the $(x, y, yaw)$ into a `PoseStamped` message. This involves converting the yaw (an Euler angle) to a Quaternion,
    * Sends the goal using `self.nav_to_pose_client.send_goal_async()`, and finally
    * Waits for the result.  
* When the action successfully completes a goal, the response callback for the action should call `send_next_goal()` for the next waypoint or else quit if done all the waypoints.

At this point, you'll need to run your `waypoint_follower` node using
```
ros2 run bot_slam waypoint_follower --ros-args -p waypoint_file_path:="<path/to/waypoint.json>"
```  

As we have seen before (Lab 6), it is pretty annoying to provide the path to your `JSON` file to run the node. Same as before, you will create a launch file called `waypoint_nav.launch.py` in `lab10_slam/bot_slam/launch/` that launches your `waypoint_follower` node and passes the `waypoint.json` file to it.  Don't forget to update your `setup.py` code to copy your launch and `JSON/YAML` files to the install folder during `colcon build`. For this, add the following to your `setup.py` file:
```
import os
from glob import glob
```
and update the `data_files=[]` statement to incude the files in `launch` and `config` folders:
```
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
            
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.[jy][sa][om][nl]'))),
    
    ]
```
Additionally, your `package.xml` file should include:
```
<exec_depend>launch_ros</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>ament_index_python</exec_depend>
```

Before running your launch file using: 
```
ros2 launch bot_slam waypoint_nav.launch.py
```  
make sure that the RViz is running with the `Maps_to_pose` action server that you start using
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```  

For this exercise, submit the following:
* `waypoint_follower.py` in `lab10_slam/bot_slam/bot_slam/`
* `waypoint.json` in `lab10_slam/bot_slam/config/`
* `waypoint_nav.launch.py` in `lab10_slam/bot_slam/launch/`

# Submitting this lab
Your ROS package and files should all be inside your `<student_repo>/lab10_slam/` folder.  Here is what we are expecting using the command `tree lab10_slam`:
```
lab10_slam
├── ex1
│   └── sphere.png
├── ex2
│   └── nav_map.png
└── bot_slam
    ├── bot_slam
    │   ├── obstacles.py
    │   ├── waypoint_follower.py
    │   └── __init__.py
    ├── config
    │   └── waypoints.json
    ├── maps
    │   ├── map.pgm
    │   └── map.yaml
    ├── launch
    │   └── waypoint_nav.launch.py
    ├── LICENSE
    ├── package.xml
    ├── resource
    ├── setup.cfg
    ├── setup.py
    └── test
```
There may be additional files, but these are the required ones.


