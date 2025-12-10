#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
import numpy as np
from enum import Enum
import time


class DrivingState(Enum):
    """Robot driving states"""
    LANE_FOLLOWING = 1
    STOPPED_FOR_SIGN = 2
    AVOIDING_OBSTACLE = 3
    TURNING_RIGHT = 4
    TURNING_LEFT = 5
    SEARCHING = 6


class DecisionMakerNode(Node):
    def __init__(self):
        super().__init__('decision_maker_node')
        
        # Parameters 
        self.declare_parameter('obstacle_stop_distance', 0.1)  # meters
        self.declare_parameter('obstacle_warning_distance', 0.2)  # meters
        self.declare_parameter('stop_sign_duration', 3.0)  # seconds
        self.declare_parameter('turn_duration', 1.5)  # seconds for executing turn
        self.declare_parameter('linear_velocity', 0.2)  # m/s
        
        self.obstacle_stop_dist = self.get_parameter('obstacle_stop_distance').value
        self.obstacle_warn_dist = self.get_parameter('obstacle_warning_distance').value
        self.stop_duration = self.get_parameter('stop_sign_duration').value
        self.turn_duration = self.get_parameter('turn_duration').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        
        # State Variables 
        self.current_state = DrivingState.LANE_FOLLOWING
        self.last_state = DrivingState.LANE_FOLLOWING
        
        # Lane following
        self.lane_cmd = Twist() 
        self.lane_detected = False
        
        # Obstacle detection
        self.nearest_obstacle_distance = float('inf')
        self.obstacle_detected = False
        self.obstacle_side = 'none'
        self.last_obstacle_time = None
        
        # Sign detection
        self.current_sign = "none"
        self.last_sign = "none"
        self.stop_start_time = None
        self.turn_start_time = None
        self.recently_stopped_signs = [] 
        self.sign_first_seen_time = {}
        
        # Subscribers 
        # Lane following commands
        self.lane_sub = self.create_subscription(
            Twist,
            '/lane_vel',
            self.lane_callback,
            10
        )
        
        # Sign detection
        self.sign_sub = self.create_subscription(
            String,
            '/signs/detected',
            self.sign_callback,
            10
        )
        
        # Obstacle detection
        self.obstacle_sub = self.create_subscription(
            MarkerArray,
            '/obstacles',
            self.obstacle_callback,
            10
        )
        
        # Publisher 
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.control_timer = self.create_timer(0.1, self.control_loop) 
        
        self.get_logger().info('=== Decision Maker Node Started ===')
        self.get_logger().info(f'Obstacle stop distance: {self.obstacle_stop_dist}m')
        self.get_logger().info(f'Stop sign duration: {self.stop_duration}s')
        self.get_logger().info(f'Linear velocity: {self.linear_vel} m/s')
    
    
    def lane_callback(self, msg):
        """Receive lane following commands"""
        self.lane_cmd = msg
        self.lane_detected = (msg.linear.x != 0.0 or msg.angular.z != 0.0)
    
    def sign_callback(self, msg):
        """Receive detected sign"""
        self.current_sign = msg.data
        
        # Track when sign was first seen
        if self.current_sign != "none":
            if self.current_sign not in self.sign_first_seen_time:
                self.sign_first_seen_time[self.current_sign] = time.time()
        else:
            # Clear timing when no sign visible
            self.sign_first_seen_time = {}
        
        # Log new sign detections
        if self.current_sign != "none" and self.current_sign != self.last_sign:
            self.get_logger().info(f'Detected: {self.current_sign}')
        
        self.last_sign = self.current_sign
    
    def obstacle_callback(self, msg):
        """Receive obstacle positions and find nearest"""
        if len(msg.markers) == 0:
            self.nearest_obstacle_distance = float('inf')
            self.obstacle_detected = False
            self.obstacle_side = 'none'  
            return
        
        # Calculate distance to nearest obstacle
        min_dist = float('inf')
        obstacle_y = 0  
        
        for marker in msg.markers:
            pos = marker.pose.position
            dist = np.sqrt(pos.x**2 + pos.y**2)
            
            if pos.x > 0 and abs(np.arctan2(pos.y, pos.x)) < np.pi/8:  
                if dist < min_dist:
                    min_dist = dist
                    obstacle_y = pos.y 
        
        self.nearest_obstacle_distance = min_dist
        self.obstacle_detected = (min_dist < self.obstacle_warn_dist)
        
        # Determine which side the obstacle is on
        if self.obstacle_detected:
            if obstacle_y > 0:
                self.obstacle_side = 'left'
            elif obstacle_y < 0:
                self.obstacle_side = 'right'
            else:
                self.obstacle_side = 'center'
    
    
    def control_loop(self):
        """Main control loop - state machine logic"""
        
        # Determine next state based on priorities
        next_state = self.decide_state()
        
        # Log state transitions
        if next_state != self.current_state:
            self.get_logger().info(f'[STATE] {self.current_state.name} → {next_state.name}')
            self.last_state = self.current_state
            self.current_state = next_state
        
        # Execute state behavior
        cmd = self.execute_state()
        
        # Publish command
        self.cmd_pub.publish(cmd)
    
    def decide_state(self):
        """
        State machine decision logic with priorities
        """
        
        if self.nearest_obstacle_distance < self.obstacle_stop_dist:
            self.last_obstacle_time = time.time()
            return DrivingState.AVOIDING_OBSTACLE
        elif self.last_obstacle_time and (time.time() - self.last_obstacle_time) < 0.5:
            # Keep avoiding for 2 seconds after obstacle disappears
            return DrivingState.AVOIDING_OBSTACLE
        else:
            self.last_obstacle_time = None
        
        if self.current_state == DrivingState.STOPPED_FOR_SIGN:
            if self.stop_start_time is not None:
                elapsed = time.time() - self.stop_start_time
                if elapsed < self.stop_duration:
                    return DrivingState.STOPPED_FOR_SIGN
                else:
                    if self.current_sign not in self.recently_stopped_signs:
                        self.recently_stopped_signs.append(self.current_sign)
                    self.stop_start_time = None
                    self.get_logger().info('Stop complete, resuming')
        
        if "Stop" in self.current_sign and \
        self.current_sign not in self.recently_stopped_signs and \
        self.current_sign in self.sign_first_seen_time and \
        (time.time() - self.sign_first_seen_time[self.current_sign]) > 0.5:
            self.stop_start_time = time.time()
            self.get_logger().info('Stop sign detected - stopping for 3s')
            return DrivingState.STOPPED_FOR_SIGN
        
        # Clear old signs from recent list
        if len(self.recently_stopped_signs) > 0:
            if self.current_sign == "none" and self.lane_detected:
                if not hasattr(self, 'no_sign_time') or self.no_sign_time is None:
                    self.no_sign_time = time.time()
                elif time.time() - self.no_sign_time > 5.0:
                    self.recently_stopped_signs.clear()
                    self.no_sign_time = None
            else:
                self.no_sign_time = None
        
        if self.current_state == DrivingState.TURNING_RIGHT:
            if self.turn_start_time is not None:
                elapsed = time.time() - self.turn_start_time
                if elapsed < self.turn_duration:
                    return DrivingState.TURNING_RIGHT
                else:
                    self.turn_start_time = None
                    self.get_logger().info('Right turn complete')
        
        if self.current_state == DrivingState.TURNING_LEFT:
            if self.turn_start_time is not None:
                elapsed = time.time() - self.turn_start_time
                if elapsed < self.turn_duration:
                    return DrivingState.TURNING_LEFT
                else:
                    self.turn_start_time = None
                    self.get_logger().info('Left turn complete')
        
        if ("Right" in self.current_sign or "right" in self.current_sign.lower()) and \
        self.current_sign not in self.recently_stopped_signs and \
        self.current_sign in self.sign_first_seen_time and \
        (time.time() - self.sign_first_seen_time[self.current_sign]) > 0.5:
            self.turn_start_time = time.time()
            self.recently_stopped_signs.append(self.current_sign)
            self.get_logger().info('Right turn sign detected')
            return DrivingState.TURNING_RIGHT
        
        if ("Left" in self.current_sign or "left" in self.current_sign.lower()) and \
        self.current_sign not in self.recently_stopped_signs and \
        self.current_sign in self.sign_first_seen_time and \
        (time.time() - self.sign_first_seen_time[self.current_sign]) > 0.5:
            self.turn_start_time = time.time()
            self.recently_stopped_signs.append(self.current_sign)
            self.get_logger().info('Left turn sign detected')
            return DrivingState.TURNING_LEFT
        
        #Lane Following
        if self.lane_detected:
            return DrivingState.LANE_FOLLOWING
        
        #Searching
        return DrivingState.SEARCHING
    
    def execute_state(self):
        
        cmd = Twist()
        
        if self.current_state == DrivingState.AVOIDING_OBSTACLE:
            if self.obstacle_side == 'left':
                # Obstacle on left, steer right
                cmd.linear.x = 0.03  
                cmd.angular.z = -0.5 
            elif self.obstacle_side == 'right':
                # Obstacle on right, steer left
                cmd.linear.x = 0.03  
                cmd.angular.z = 0.5  
            else:
                cmd.linear.x = 0.0  
                cmd.angular.z = 0.0
        
        elif self.current_state == DrivingState.STOPPED_FOR_SIGN:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        elif self.current_state == DrivingState.TURNING_RIGHT:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.3  
        
        elif self.current_state == DrivingState.TURNING_LEFT:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 
        
        elif self.current_state == DrivingState.LANE_FOLLOWING:

            cmd = self.lane_cmd
        
        elif self.current_state == DrivingState.SEARCHING:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3
            
        return cmd


def main(args=None):
    rclpy.init(args=args)
    
    node = DecisionMakerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()