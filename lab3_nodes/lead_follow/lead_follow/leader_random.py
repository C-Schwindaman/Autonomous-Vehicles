#!/usr/bin/env python
''' leader_random.py

    This is a ROS node that simulates a leader robot that moves randomly.
    It publishes its motion commands on the 'robot_move' topic so a
    follower node can copy its movements.
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from .robot_room import RobotRoom
import random

class LeaderRandom(Node):
    def __init__(self, topic_name='robot_move'):
        ''' Initialize Node, publisher, and RobotRoom. '''
        super().__init__('leader_random')
        
        # Define the publisher for an Int32 message
        self.publisher_ = self.create_publisher(Int32, topic_name, 10)
        self.get_logger().info('Publishing random moves to: ' + topic_name)
        
        # Initialize the RobotRoom for the random leader
        self.room = RobotRoom('Robot-C (Random Leader)', color=(255, 0, 255)) # Magenta color
        self.get_logger().info("Press 'q' in the window to quit.")

    def run(self):
        ''' 
        Create a loop that:
            - Waits 250ms and checks if 'q' has been pressed.
            - Randomly chooses a move from ('w', 'a', 's', 'd').
            - Moves the robot according to the random choice.
            - Publishes the random move to the topic.
            - Quits when the user presses 'q'.
        '''
        key = 0
        msg = Int32()
        # List of valid moves (as ASCII integers)
        possible_moves = [ord('w'), ord('a'), ord('s'), ord('d')]

        while key != ord('q'):
            # Draw the room and wait for 250ms. 
            # cv.waitKey() will return the key pressed during this time, or -1 if none.
            key = self.room.draw(250)

            # Choose a random move
            random_key = random.choice(possible_moves)

            # Move the robot with the random command
            self.room.move(random_key)

            # Publish the random command
            msg.data = random_key
            self.publisher_.publish(msg)

        # After the loop breaks, publish 'q' to stop any followers
        self.get_logger().info("Quitting and telling followers to quit.")
        msg.data = ord('q')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    random_leader_node = LeaderRandom()
    
    try:
        random_leader_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        random_leader_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()