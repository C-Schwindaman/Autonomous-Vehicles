#!/usr/bin/env python
''' follower.py

    This is a ROS node that subscribes to keyboard commands and 
    moves the robot according to these.

    Your task is to complete the missing portions that are marked with
    # <...> 
    pass
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from .robot_room import RobotRoom

class Follower(Node):
    def __init__(self, topic_name='robot_move'):
        ''' Initialize a ROS node.  Permit any number of follower nodes
            Define an variable for a room, but don't initialize the room (see below for reason)
            Define a subscriber to the 'robot_move' topic
        '''
        super().__init__('follow')
        self.room = None            # Do not initialize room here as different thread from the callback
        # <...> Create a subscriber with the move_callback function
        self.subscription = self.create_subscription(
            Int32,
            topic_name,
            self.move_callback,
            10)

    def move_callback(self, msg):
        ''' Initialize a RobotRoom if one has not already been initialized
                Putting the initializer here ensures RobotRoom calls all remain in a single thread
                which is important since they use OpenCV
            If key is a 'q', shut down node with: rospy.signal_shutdown("Received 'q' message")
            Move the robot according to the key in the message
            Draw the RobotRoom
        '''
        if self.room is None:
            # <...> If room is None then it has not been initialized, so initialize it
            self.room = RobotRoom('Robot-B (Follower)', color=(0,0,255)) # Red color for follower
            pass
        if msg.data == ord('q'):
            raise SystemExit    # if we read a `q` then time to quit
        # <...> move according to the message
        self.room.move(msg.data)
        # <...> draw the room
        self.room.draw_no_return_key()
        

def main(args=None):
    rclpy.init(args=args)
    # <...> Create a follower
    follower = Follower()
    try:
        # <...> Use spin to keep your follower going
        rclpy.spin(follower)
        pass
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        # <...> Call destroy_node() on your follower
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

