#!/usr/bin/env python3

"""
Collect sign images from Gazebo autorace world
Press 's' to save current frame
Press 'q' to quit
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class SignImageCollector(Node):
    def __init__(self):
        super().__init__('sign_image_collector')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('save_dir', './autorace_signs')
        
        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        
        # Create save directory
        os.makedirs(self.save_dir, exist_ok=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Image subscriber
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.current_image = None
        self.image_count = 0
        
        self.get_logger().info(f'Image collector started')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info(f'Saving to: {self.save_dir}')
        self.get_logger().info('Press "s" to save image, "q" to quit')
        
    def image_callback(self, msg):
        """Store latest image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
    
    def run(self):
        """Main loop with keyboard control"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.current_image is not None:
                display_img = self.current_image.copy()
                
                cv2.putText(
                    display_img,
                    'Press "s" to save, "q" to quit',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
                
                cv2.putText(
                    display_img,
                    f'Images saved: {self.image_count}',
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
                
                cv2.imshow('Sign Collector', display_img)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('s'):
                    self.save_image()
                elif key == ord('q'):
                    break
        
        cv2.destroyAllWindows()
    
    def save_image(self):
        """Save current image"""
        if self.current_image is None:
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'sign_{self.image_count:04d}_{timestamp}.jpg'
        filepath = os.path.join(self.save_dir, filename)
        
        cv2.imwrite(filepath, self.current_image)
        self.image_count += 1
        
        self.get_logger().info(f'Saved: {filename}')


def main(args=None):
    rclpy.init(args=args)
    
    node = SignImageCollector()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()