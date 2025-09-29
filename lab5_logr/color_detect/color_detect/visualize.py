import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2 as cv
from cv_bridge import CvBridge
import argparse
import os
from pathlib import Path

class Visualize(Node):
    def __init__(self, real_robot: bool, outim_path: str):
        super().__init__('visualize')
        self.bridge = CvBridge()
        self.real_robot = real_robot
        self.outim_path = Path(outim_path)
        self.outim_path.mkdir(parents=True, exist_ok=True)
        if self.real_robot:
            self.filename = self.outim_path / 'ex3_train_img_robot.png'
        else:
            self.filename = self.outim_path / 'ex3_train_img.png'
        namespace = os.environ.get("ROBOT_NAMESPACE", "")
        if self.real_robot:
            image_topic = f"/{namespace}/oakd/rgb/image_raw"
            self.compressed = 'compressed' in image_topic
        else:
            image_topic = "/camera/image_raw"
            self.compressed = False
        self.get_logger().info(f'Subscribing to image topic: {image_topic}')
        if self.compressed:
            self.subscription = self.create_subscription(
                CompressedImage, image_topic, self.image_callback, 10)
        else:
            self.subscription = self.create_subscription(
                Image, image_topic, self.image_callback, 10)

    def image_callback(self, msg):
        try:
            if self.compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        cv.imshow("Camera View", cv_image)
        key = cv.waitKey(1) & 0xFF
        if key == ord('s'):
            cv.imwrite(str(self.filename), cv_image)
            self.get_logger().info(f'Image saved to: {self.filename}')
        elif key == ord('q'):
            self.get_logger().info('Quitting viewer...')
            cv.destroyAllWindows()
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Visualize TurtleBot camera feed.')
    parser.add_argument('--outim_path', type=str, required=True, help='Path to the folder where the image will be saved.')
    parser.add_argument('--real_robot', action='store_true', help='Set if using the physical TurtleBot4.')
    parsed_args = parser.parse_args(args=rclpy.utilities.remove_ros_args()[1:])
    node = Visualize(real_robot=parsed_args.real_robot, outim_path=parsed_args.outim_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
