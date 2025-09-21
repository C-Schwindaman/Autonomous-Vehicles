#!/usr/bin/env python
''' waypoint_nav.py

    To drive the TurtleBot to the way points being published to /waypoints topic.

    <...> Complete missing portions

    Copyright: Ankur Kamboj, 2025
'''
import rclpy
import argparse
import os
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation as R
import numpy as np

def angdiff(a,b,maxdiff=np.pi):
    ''' Returns difference of two angles: a - b in range -maxdiff to maxdiff
        For radians, maxdiff is pi, in degrees maxdiff is 180
    '''
    return (a-b+maxdiff)%(2*maxdiff)-maxdiff

def check_heading(waypoint: tuple[float, float],
                            current_pos: tuple[float, float],
                            current_yaw: float) -> float:
    ''' Checks the heading of the robot towards the waypoint. '''  
    # Compute the desired yaw angle
    desired_yaw = np.arctan2(waypoint[1] - current_pos[1], waypoint[0] - current_pos[0])

    # Check and return the difference
    return angdiff(desired_yaw, current_yaw)

def yaw_from_quaternion(quat) -> float:
    ''' Returns yaw from a quaternion '''
    rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    return rot.as_rotvec()[2]

class WaypointNav(Node):
    ''' Drives to waypoints '''

    def __init__(self, lin_x: float, ang_z: float, simulator: bool):
        super().__init__('waypoint_nav')

        namespace = os.environ.get("ROBOT_NAMESPACE","")
        self.simulator = simulator
    
        # <...> create a /cmd_vel publisher with an appropriate namespace and output a log with what is being published

        # These are useful variables for the callbacks:
        self.prev_dist = np.inf # Set to infinity initially
        self.rotated  = False
        self.waiting_for_waypoint = False
        self.current_waypoint = None
        self.prev_waypoint = None

        self.ang_z = ang_z
        self.lin_x = lin_x

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # <...> Create a subscriber for /odom (with appropriate namespace).  The callback should be: self.odom_callback

        # <...> Make a log describing the subscriber

        # <...> Create a subscriber for /waypoints with callback: self.odom_callback

        # <...> Make a log describing the subscriber
        
    def waypoint_callback(self, msg):
        ''' Callback to receive waypoints '''
        # <...> This should set the current waypoint from the message
        #       It should also check if the waypoint is different from the previous and turn off 
        #       a condition in which we are stopped at a waypoint
        #       Also, output a log message if we have a new waypoint
        pass

    def odom_callback(self, msg):

        if self.current_waypoint is None:
            if not self.waiting_for_waypoint:
                self.get_logger().info('No waypoint received yet, waiting ...')
                self.waiting_for_waypoint = True # Set flag so it doesn't print again

            # <...> publish a zero motion
            return

        if self.prev_waypoint is None:
            self.prev_waypoint = self.current_waypoint
            self.get_logger().info(f'Moving to waypoint {self.current_waypoint}')

        if self.prev_waypoint != self.current_waypoint:
            self.rotated = False
            self.prev_waypoint = self.current_waypoint
            self.prev_dist = np.inf
            self.get_logger().info(f'Moving to new waypoint {self.current_waypoint}')
            
        # <...> Find the current position as a tuple (x,y):
        # current_pos =
        # <...> Find the distance to the current waypoint 
        # current_dist = 

        # Determine the motion command with two components with default values of 0:
        angular_vel = 0.0
        linear_vel = 0.0

        if not self.rotated:
            # If we are not yet pointing at the waypoint
            # <...> get the current yaw from odometry:
            # current_yaw = 
            # Rotate till facing the waypoint
            # <...> heading error:
            # heading_error = 

            if np.abs(heading_error) > 3e-2:
                # <...> set angular_vel and log this

            else:
                self.rotated = True
                self.get_logger().info(f'Heading aligned, moving forward to waypoint {self.current_waypoint}')
        
        else:
            # Moving forward to waypoint
            if current_dist < 0.1:
                if self.prev_dist is not np.inf:
                    self.get_logger().info(f'Reached waypoint {self.current_waypoint}')
                self.prev_dist = np.inf
            elif current_dist > self.prev_dist:
                self.get_logger().info(f'Oops, getting further away from waypoint, correcting rotation')
                self.rotated = False
                self.prev_dist = current_dist
            else:
                # <...> set linear_vel
                # linear_vel = 
                self.get_logger().info(f'Moving forward to waypoint {self.current_waypoint}, distance {current_dist:.2f} m')
                self.prev_dist = current_dist
        
        # <...> Create a motion command depending on if simulator or not

        # <...> Publish the motion command


def main(args=None):
    
    rclpy.init(args=args)

    # <...> Read in arguments (see circle_drive.py for an example)

    # <...> Initialize a WaypointNav class variable with appropriate parameters

    # <...> Do a spin    
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)

if __name__ == '__main__':
    main()

