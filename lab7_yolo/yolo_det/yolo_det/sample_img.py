import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSampler(Node):
    def __init__(self):
        super().__init__('image_sampler_node')
        self.topic_name = '/oakd/rgb/preview/image_raw'        
        self.save_dir = os.path.expanduser('~/yolo_training_images')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f'Created directory: {self.save_dir}')
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10)
        self.subscription
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.window_name = 'Image Preview'
        self.get_logger().info(f'Node started. Subscribing to {self.topic_name}')
        self.get_logger().info(f'Press "s" in the preview window to save an image.')
        self.get_logger().info(f'Images will be saved to: {self.save_dir}')

    def image_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        cv2.imshow(self.window_name, self.latest_cv_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.save_current_image()

    def save_current_image(self):
        if self.latest_cv_image is not None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f'{timestamp}.png'
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, self.latest_cv_image)
            self.get_logger().info(f'Saved image: {filepath}')
        else:
            self.get_logger().warn('No image received yet to save.')

def main(args=None):
    rclpy.init(args=args)
    image_sampler_node = ImageSampler()    
    try:
        rclpy.spin(image_sampler_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_sampler_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()