#!/usr/bin/env python
''' center_target.py

    This node uses a pre-trained color model to find a target sign
    in the TurtleBot's camera feed and rotates the robot to center it.
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, TwistStamped, Vector3
import cv2 as cv
from cv_bridge import CvBridge
import argparse
import os
from pathlib import Path
import numpy as np
import json
from scipy.special import expit # Sigmoid function

class CenterTarget(Node):
    def __init__(self, real_robot: bool, json_path: str, outim_path: str):
        super().__init__('center_target')
        self.bridge = CvBridge()
        self.outim_path = Path(outim_path)
        self.real_robot = real_robot

        # Ensure output directory exists and set filename
        self.outim_path.mkdir(parents=True, exist_ok=True)
        self.filename = self.outim_path / ('ex3_target_img_robot.png' if real_robot else 'ex3_target_img.png')

        # Load logistic regression model parameters from the specified JSON file
        try:
            with open(json_path, 'r') as f:
                self.get_logger().info(f"Loading model parameters from: {json_path}")
                params = json.load(f)
                self.cvec = np.array(params['cvec'])
                self.intercept = np.array(params['intercept'])
        except Exception as e:
            self.get_logger().error(f"Failed to load model from {json_path}: {e}")
            raise SystemExit

        # --- ROS2 Publishers and Subscribers ---
        namespace = os.environ.get("ROBOT_NAMESPACE", "")
        
        # Publisher for motor commands
        cmd_vel_topic = f"{namespace}/cmd_vel"
        self.motion_type = TwistStamped if self.real_robot else Twist
        self.publisher_ = self.create_publisher(self.motion_type, cmd_vel_topic, 10)
        self.get_logger().info(f"Publishing to {cmd_vel_topic}")

        # Subscriber for camera feed
        if self.real_robot:
            image_topic = f"/{namespace}/oakd/rgb/image_raw"
            self.compressed = 'compressed' in image_topic
        else:
            image_topic = "/camera/image_raw"
            self.compressed = False
        
        self.get_logger().info(f"Subscribing to {image_topic}")
        if self.compressed:
            self.subscription = self.create_subscription(CompressedImage, image_topic, self.image_callback, 10)
        else:
            self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)
        
        # --- Member variable to store the latest image ---
        # <<< NEW
        self.latest_image = None

        # --- Control Parameters ---
        self.search_speed = 0.3  # rad/s to rotate when searching
        self.center_tolerance = 25 # Pixels from center is "close enough"
        self.turn_gain = 0.002 # Proportional gain for turning controller

        # --- Timer to control processing rate ---
        # <<< NEW: Create a timer to run the control loop at 10 Hz (every 0.1 seconds)
        timer_period = 0.1  # seconds
        self.control_timer = self.create_timer(timer_period, self.control_loop_callback)

    # --- Logistic Regression Helper Functions (copied from logist_reg.py) ---
    def _apply_model(self, img):
        data = img.reshape((-1, img.shape[2])).astype(float)
        score_flat = data @ self.cvec.T + self.intercept
        return score_flat.reshape(img.shape[0:2])

    def _prob_target(self, score):
        return expit(score)

    def _find_largest_target(self, prob_target, threshold=0.5, minpix=50):
        binary_mask = (prob_target > threshold).astype(np.uint8)
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(binary_mask, connectivity=8)
        if num_labels < 2:
            return None, None # No target found
        
        areas = stats[1:, cv.CC_STAT_AREA]
        valid_indices = np.where(areas >= minpix)[0]
        if len(valid_indices) == 0:
            return None, None
        
        largest_idx = valid_indices[np.argmax(areas[valid_indices])] + 1
        target_mask = (labels == largest_idx).astype(np.uint8)
        return centroids[largest_idx], target_mask

    def image_callback(self, msg):
        # <<< MODIFIED: This callback now only stores the latest image message.
        self.latest_image = msg

    # <<< NEW METHOD
    def control_loop_callback(self):
        '''
        This function is called by a timer and contains the core logic
        for processing images and publishing motor commands.
        '''
        # If we haven't received any images yet, do nothing.
        if self.latest_image is None:
            return

        try:
            # Use the most recent image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(self.latest_image, "bgr8") if self.compressed else self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # --- Detection ---
        score = self._apply_model(cv_image)
        prob = self._prob_target(score)
        centroid, target_mask = self._find_largest_target(prob)

        # --- Control Logic ---
        angular_vel = 0.0
        if centroid is None:
            # State 1: No target found -> Search by rotating
            angular_vel = self.search_speed
        else:
            # State 2: Target found -> Center it
            image_center_x = cv_image.shape[1] / 2.0
            error = image_center_x - centroid[0]
            if abs(error) > self.center_tolerance:
                # Target is not centered, apply proportional control to turn
                angular_vel = self.turn_gain * error
            else:
                # Target is centered, stop turning
                angular_vel = 0.0
        
        # --- Publish Motion Command ---
        if self.real_robot:
            motion_cmd = TwistStamped(twist=Twist(angular=Vector3(z=angular_vel)))
            motion_cmd.header.stamp = self.get_clock().now().to_msg()
        else:
            motion_cmd = Twist(angular=Vector3(z=angular_vel))
        self.publisher_.publish(motion_cmd)

        # --- Visualization and User Interaction ---
        display_image = cv_image.copy()
        if centroid is not None:
            # Highlight target and draw crosshair at centroid
            display_image[target_mask > 0, 1] = 255 
            c = tuple(int(x) for x in centroid)
            cv.line(display_image, (c[0]-10, c[1]), (c[0]+10, c[1]), (0,0,255), 2)
            cv.line(display_image, (c[0], c[1]-10), (c[0], c[1]+10), (0,0,255), 2)
        
        cv.imshow("Target Centering", display_image)
        key = cv.waitKey(1) & 0xFF
        if key == ord('s'):
            cv.imwrite(str(self.filename), display_image)
            self.get_logger().info(f'Image saved to: {self.filename}')
        elif key == ord('q'):
            self.get_logger().info('Quitting...')
            cv.destroyAllWindows()
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Center a colored target.')
    parser.add_argument('--json_path', type=str, required=True, help='Path to the logr_coefs.json file.')
    parser.add_argument('--outim_path', type=str, required=True, help='Path to folder for saving images.')
    parser.add_argument('--real_robot', action='store_true', help='Set if using the physical TurtleBot4.')
    parsed_args = parser.parse_args(args=rclpy.utilities.remove_ros_args()[1:])

    node = CenterTarget(real_robot=parsed_args.real_robot, json_path=parsed_args.json_path, outim_path=parsed_args.outim_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()