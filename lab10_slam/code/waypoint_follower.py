#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import json

class WaypointFollower(Node):
    """
    A ROS 2 node to send a sequence of waypoints to the Nav2 stack
    which will navigate the robot to each waypoint in order.

    Assumes that there is a parameter 'waypoint_file_path' that points to a JSON file
    """

    def __init__(self):
        super().__init__('waypoint_follower')

        # Declare a parameter for the waypoint file path
        self.declare_parameter('waypoint_file_path', 'default_path.json')

        # Get the file path from the parameter
        # file_path = ...

        # Load the waypoints from the file
        self.waypoints = self.load_waypoints(file_path)

        # Create the action client of type: NavigateToPose

        # Start the action client and wait for the server to be available:
        self.get_logger().info('Connecting to "navigate_to_pose" action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available! Shutting down.')
            self.destroy_node()
            return
        
        self.get_logger().info('Action server connected.')

        # Set waypoint index to the first goal and send the goal:        
        self.current_waypoint_index = 0
        self.send_next_goal()

    def load_waypoints(self, file_path):
        """ Loads waypoints from a JSON file """
        # complete this function
        pass

    def send_next_goal(self):
        """ Sends the next waypoint in the list as a goal to Nav2. """

        # Check if we are done waypoints and return if so

        # Get waypoint data

        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        # Fill in the goal message fields with waypoint data
        # Make sure the frame_id is 'map'

        # Send the goal:
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        # Add a callback for when the goal is accepted or rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """ Logs feedback from the Nav2 action server. 
            Leave this function as is.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} m', throttle_duration_sec=5.0)

    def goal_response_callback(self, future):
        """ Handles the response from the action server after sending a goal. 
            Leave this function as is.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server!')
            # Stop the mission if a goal is rejected
            self.destroy_node()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        
        # Get the result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ Handles the final result of the navigation goal. 
            Complete the missing portion of this function
        """
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            # Move to the next waypoint
            # increment the waypoint index and send the next goal
            # ...
        else:
            self.get_logger().warning(f'Goal failed with status: {status}')
            # You could choose to retry, skip, or stop. For this lab we can simply stop
            self.get_logger().error('Stopping mission due to goal failure.')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
