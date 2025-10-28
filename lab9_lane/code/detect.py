#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os


class DetectNode(Node):
    """
    A ROS2 node to detect lane lines using OpenCV RGB/BGR color masking
    and publish a target point.
    """
    def __init__(self):
        super().__init__('detect_node')
        
        self.bridge = CvBridge()
        
        # Color Ranges for Line Detection in BGR Space ---
        # BGR for Orange
        self.lower_orange_bgr = np.array([])  # Complete with appropriate values
        self.upper_orange_bgr = np.array([])
        
        # BGR for White (allowing for shadows/gray)
        self.lower_white_bgr = np.array([])
        self.upper_white_bgr = np.array([])

        # Kernel for morphological operations (for noise reduction)
        self.morph_kernel = np.ones((5, 5), np.uint8)
        
        # Lane Width Memory to Handle Sharp Turns ---
        # Initial guess for lane width in pixels. This will be updated
        # automatically whenever both lines are detected.
        self.DEFAULT_LANE_WIDTH_PX = 700 
        self.last_known_lane_width = self.DEFAULT_LANE_WIDTH_PX

        # Create Subscriber and Publishers ---

        self.get_logger().info('Lane detection node started (using BGR + Memory for sharp turns).')


    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Processes the image to find the target point.
        """
        try:
            # Convert ROS Image message to OpenCV image (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Define Region of Interest (ROI)
        height, width, _ = cv_image.shape

        # roi = 
        
        # Detect Lines using OpenCV BGR Masking  and optionally cv2.morphologyEx ---

        # Find Centroids ---
        
        # Define PointStamped message to publish
        
        # Depending on whether orange or white or both or neither lines are detected,
        # compute the target point accordingly 

        self.get_logger().debug('Output the which lines were detected as a debug message.')

        # Draw visualization dot ONLY if we have a valid (non-zero) point

        # Publish the detected point

        # Publish the processed image for detected center visualization


def main(args=None):
    rclpy.init(args=args)
    
    detect_node = DetectNode()
    
    try:
        rclpy.spin(detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        if rclpy.ok():
            detect_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()


