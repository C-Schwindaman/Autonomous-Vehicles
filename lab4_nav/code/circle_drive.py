#!/usr/bin/env python
'''
    circle_drive.py 

    This is an exmple ROS node for driving a Turtlebot 3 or Turtlebot 4 
    in a circle and then stopping. It is open-loop and does not use feedback.

    Daniel Morris, Sep 2022, Sep 2025
'''
import math, os, time, argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, Vector3

class CircleDrive(Node):
    def __init__(self, is_sim: bool, lin_x: float, ang_z: float, topic: str ):
        super().__init__('circle_drive')
        self.is_sim = is_sim
        self.total_time = 2 * math.pi / ang_z  # Time to complete a full circle: 2*pi/angular_speed
        if self.is_sim:
            self.motion = Twist( linear=Vector3(x=lin_x), angular=Vector3(z=ang_z))
        else:
            self.motion = TwistStamped( twist=Twist( linear=Vector3(x=lin_x), angular=Vector3(z=ang_z)) )
        self.publisher_ = self.create_publisher(type(self.motion), topic, 1)
        self.get_logger().info(f'Publishing {type(self.motion).__name__} to {topic}')
	
    def get_motion(self, stop_bot=False):
        motion = self.motion if not stop_bot else type(self.motion)()  # Zero motion if stopped
        if not self.is_sim:
            motion.header.stamp = self.get_clock().now().to_msg() # Add timestamp for real robot
        return motion

    def drive_in_a_circle(self ):
        self.get_logger().info(f'Drive for {self.total_time:.2f} sec')
        start = time.time()
        while time.time() - start < self.total_time:
            self.publisher_.publish( self.get_motion() )
            time.sleep( 0.1 )
        self.publisher_.publish( self.get_motion( True ) )
        self.get_logger().info('Done')
        
def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--lin_x', type=float, default=0.2,  help='Linear velocity in m/s')
    parser.add_argument('--ang_z', type=float, default=0.4,  help='Angular velocity in rad/s')
    parser.add_argument('--real_robot', action='store_true', help='Run in real robot mode')

    parsed_args = parser.parse_args(args=rclpy.utilities.remove_ros_args()[1:])

    topic = os.environ.get("ROBOT_NAMESPACE", "") + "/cmd_vel"

    av = CircleDrive(not parsed_args.real_robot, parsed_args.lin_x, parsed_args.ang_z, topic)  # Initialize Node
    av.drive_in_a_circle()
    av.destroy_node()
    rclpy.shutdown()
