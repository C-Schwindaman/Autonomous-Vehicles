import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from .pid import pid_controller

class LaneControllerNode(Node):
    """
    This node combines lane detection and PID control.
    - Subscribes to: /camera/image_raw 
    - Publishes to: /lane_vel 
    - Publishes to: /image_processed 
    """
    def __init__(self):
        super().__init__('lane_controller_node')

        # --- Parameters for Tuning ---
        # Declare parameters for PID gains and velocity
        self.declare_parameter('kp', 0.6)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('linear_velocity', 0.15)
        self.declare_parameter('focal_length', 550.0)

        # Get the parameters
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.linear_velocity = self.get_parameter('linear_velocity').get_parameter_value().double_value
        self.focal_length = self.get_parameter('focal_length').get_parameter_value().double_value

        self.get_logger().info(f"PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info(f"Linear Velocity: {self.linear_velocity} m/s")

        # --- PID Controller ---
        self.pid = pid_controller(kp=self.kp, ki=self.ki, kd=self.kd)
        self.pid.previous_time_sec = self.get_clock().now().nanoseconds / 1e9
        
        # --- ROS Subscriptions & Publishers ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Add a parameter for debug mode
        self.declare_parameter('debug_mode', False)
        is_debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        if is_debug_mode:
            # In debug mode, publish directly to /cmd_vel for testing
            publish_topic = '/cmd_vel'
            self.get_logger().warn('--- DEBUG MODE ACTIVE: Publishing to /cmd_vel ---')
        else:
            # In normal mode, publish to /lane_vel for the decision_maker
            publish_topic = '/lane_vel'
            self.get_logger().info(f'--- NORMAL MODE: Publishing to /lane_vel ---')

        # Publisher uses the topic variable
        self.cmd_pub = self.create_publisher(
            Twist,
            publish_topic,
            10)
        
        self.debug_img_pub = self.create_publisher(
            Image,
            '/image_processed',
            10)
        self.white_mask_pub = self.create_publisher(Image, '/mask_white', 10)
        self.orange_mask_pub = self.create_publisher(Image, '/mask_orange', 10)

        # --- Lane Detection State ---
        self.lane_width = 950
        self.get_logger().info("Lane Controller Node has started.")

    def filter_mask(self, mask, color_name):
        """
        Filters a binary mask to keep only blobs within a specific size range.
        Removes tiny noise and GIANT walls.
        """
        # Find all contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create a clean empty mask
        clean_mask = np.zeros_like(mask)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)                

            # Orange lanes are less likely to be walls
            if color_name == "ORANGE":
                min_blob_area = 1000
                max_blob_area = 100000  
            else:
                min_blob_area = 15000
                max_blob_area = 49000

            # 1. Area Filter
            if min_blob_area < area < max_blob_area:                
                # 2. Aspect Ratio Filter
                # Lane lines are wide and short. Walls are tall/square.
                x,y,w,h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                
                # If width is at least 1.2x the height, it's likely a line, not a wall block
                if aspect_ratio > 1.2: 
                    cv2.drawContours(clean_mask, [cnt], -1, 255, thickness=cv2.FILLED)
        
        return clean_mask

    def image_callback(self, msg):
        """
        Main callback that processes the image, finds the lane center,
        calculates the control command, and publishes it.
        """
        # --- 1. Lane Detection ---
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width, _ = cv_image.shape
        roi_top = int(height * 0.85)
        roi = cv_image[roi_top:, :, :]
        
        # --- Color Masking ---
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 170])
        upper_white = np.array([180, 100, 255])
        lower_orange = np.array([10, 180, 150])
        upper_orange = np.array([30, 255, 255])

        raw_white_mask = cv2.inRange(hsv_roi, lower_white, upper_white)
        raw_orange_mask = cv2.inRange(hsv_roi, lower_orange, upper_orange)

        white_mask = self.filter_mask(raw_white_mask, "WHITE")
        orange_mask = self.filter_mask(raw_orange_mask, "ORANGE")
        
        # Publish debug masks
        try:
            self.white_mask_pub.publish(self.bridge.cv2_to_imgmsg(white_mask, "mono8"))
            self.orange_mask_pub.publish(self.bridge.cv2_to_imgmsg(orange_mask, "mono8"))
        except Exception as e:
            self.get_logger().warn(f'Failed to publish masks: {e}')

        # Find Centroids
        M_white = cv2.moments(white_mask)
        M_orange = cv2.moments(orange_mask)

        white_found = False
        orange_found = False
        lane_center_full_coords = None

        if M_white["m00"] > 0:
            cx_white = int(M_white["m10"] / M_white["m00"])
            cy_white = int(M_white["m01"] / M_white["m00"])
            white_found = True

        if M_orange["m00"] > 0:
            cx_orange = int(M_orange["m10"] / M_orange["m00"])
            cy_orange = int(M_orange["m01"] / M_orange["m00"])
            orange_found = True

        roi_height = height - roi_top
        cy_roi_center = roi_height // 2

        if white_found and orange_found:
            cx_lane = (cx_white + cx_orange) // 2
            cy_lane = (cy_white + cy_orange) // 2
            lane_center_full_coords = (cx_lane, cy_lane + roi_top)
        elif white_found and not orange_found:
            cx_orange_est = cx_white - self.lane_width
            cx_lane = (cx_white + cx_orange_est) // 2
            cy_lane = cy_roi_center
            lane_center_full_coords = (cx_lane, cy_lane + roi_top)
        elif not white_found and orange_found:
            cx_white_est = cx_orange + (self.lane_width * 1.25) 
            cx_lane = (cx_orange + cx_white_est) // 2
            cy_lane = cy_roi_center
            lane_center_full_coords = (cx_lane, cy_lane + roi_top)

        # --- 2. PID Control ---
        twist_msg = Twist()
        image_center_x = width // 2
        
        if lane_center_full_coords:
            # Draw the detected center point for debugging
            draw_center = (int(lane_center_full_coords[0]), int(lane_center_full_coords[1]))
            cv2.circle(cv_image, draw_center, 7, (0, 0, 255), -1)
            
            # --- PID Logic ---
            current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            current_state_pixels = lane_center_full_coords[0]

            # Calculate error in "angle"
            target_angle = 0.0
            current_angle_state = (current_state_pixels - image_center_x) / self.focal_length

            # Get control output
            (angular_vel, _, _, _) = self.pid.update_control(
                target=target_angle,
                state=current_angle_state,
                current_time_sec=current_time_sec
            )
            
            # Create the Twist message
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = angular_vel
            
        else:
            # No lanes detected, tell the robot to stop
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
            # Reset PID integral when lanes are lost
            self.pid.reset_integral()

        # --- 3. Publish Commands and Debug Image ---
        # Publish the command to /lane_vel
        self.cmd_pub.publish(twist_msg)

        # Publish the debug image
        try:
            # Draw the center line for reference
            cv2.line(cv_image, (image_center_x, 0), (image_center_x, height), (0, 255, 0), 1)
            processed_img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            processed_img_msg.header = msg.header
            self.debug_img_pub.publish(processed_img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish processed image: {e}')

def main(args=None):
    rclpy.init(args=args)
    lane_controller_node = LaneControllerNode()
    try:
        rclpy.spin(lane_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a stop command on shutdown
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        lane_controller_node.cmd_pub.publish(stop_msg)
        lane_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()