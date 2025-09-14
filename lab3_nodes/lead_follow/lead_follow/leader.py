#!/usr/bin/env python
''' leader.py

    This is a ROS node that defines room-A and robot-A. Robot-A is controlled 
    by the keyboard. Additionally the motion commands are published so other 
    nodes can follow.

    Your task is to complete the missing portions that are marked with
    # <...> 
'''
from std_msgs.msg import Int32
from .robot_room import RobotRoom
import rclpy
from rclpy.node import Node

class Leader(Node):
    def __init__(self, topic_name='robot_move'):
        ''' Initialize Node
            Define a publisher for a message of type Int32
            Initialize a RobotRoom
            Call listKeys() to tell user what to press
       '''
        super().__init__('leader')
        # <...>  Create your publisher
        self.publisher_ = self.create_publisher(
            Int32, 
            topic_name, 
            10)
        self.get_logger().info('Publishing: ' + topic_name)
        # <...> Initialize your RobotRoom for the leader
        self.room = RobotRoom('Robot-A (Leader)', color=(0,255,0)) # Green color for leader
        # <...> list the keys the user can press
        self.room.listKeys()
        
    def run(self):
        ''' Create a loop that:
            Draws the robot and returns a key
            Based on the key pressed, moves it accordingly 
            Publishes the key to the topic 
            Quits when the user presses a 'q'
        '''
        key = 0
        msg = Int32()        
        while key != ord('q'):  # While q has not been pressed:
            # <...> Draw the room
            key = self.room.draw(10) # wait 10ms for key
            # <...> Move according to the key
            self.room.move(key)
            # <...> Publish the key as a Int32()
            msg.data = key
            self.publisher_.publish(msg)

        # Publish one last 'q' to make sure followers quit
        msg.data = ord('q')
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    # <...> Initialize the Leader class
    leader = Leader()
    try:
        # <...> run the leader class
        leader.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Call destroy_node() on your leader
        leader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
