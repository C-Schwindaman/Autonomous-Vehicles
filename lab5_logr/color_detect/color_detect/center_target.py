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
        self.outim_path.mkdir(parents=True, exist_ok=True)
        if self.real_robot:
            self.filename = self.outim_path / 'ex3_target_img_robot.png'
        else:
            self.filename = self.outim_path / 'ex3_target_img.png'
        try:
            with open(json_path, 'r') as f:
                self.get_logger().info(f"Loading model parameters from: {json_path}")
                params = json.load(f)
                self.cvec = np.array(params['cvec'])
                self.intercept = np.array(params['intercept'])
        except Exception as e:
            self.get_logger().error(f"Failed to load model from {json_path}: {e}")
            raise SystemExit
        namespace = os.environ.get("ROBOT_NAMESPACE", "")
        cmd_vel_topic = f"{namespace}/cmd_vel"
        self.motion_type = TwistStamped if self.real_robot else Twist
        self.publisher_ = self.create_publisher(self.motion_type, cmd_vel_topic, 10)
        self.get_logger().info(f"Publishing to {cmd_vel_topic}")
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
        self.search_speed = 0.2
        self.center_tolerance = 40
        self.turn_gain = 0.002
        self.target_lost_counter = 0
        self.target_lost_threshold = 10
        self.current_state = "SEARCHING"
        self.last_log_message = ""

    def _log_state(self, message):
        if message != self.last_log_message:
            self.get_logger().info(message)
            self.last_log_message = message

    def _apply_model(self, img):
        data = img.reshape((-1, img.shape[2])).astype(float)
        score_flat = data @ self.cvec.T + self.intercept
        return score_flat.reshape(img.shape[0:2])

    def _prob_target(self, score):
        return expit(score)

    def _find_largest_target(self, prob_target, threshold=0.75, minpix=250):
        binary_mask = (prob_target > threshold).astype(np.uint8)
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(binary_mask, connectivity=8)
        if num_labels < 2:
            return None, None
        areas = stats[1:, cv.CC_STAT_AREA]
        valid_indices = np.where(areas >= minpix)[0]
        if len(valid_indices) == 0:
            return None, None
        largest_idx = valid_indices[np.argmax(areas[valid_indices])] + 1
        target_mask = (labels == largest_idx).astype(np.uint8)
        return centroids[largest_idx], target_mask

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") if self.compressed else self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        score = self._apply_model(cv_image)
        prob = self._prob_target(score)
        centroid, target_mask = self._find_largest_target(prob)
        angular_vel = 0.0
        if centroid is None:
            self.target_lost_counter += 1
            if self.target_lost_counter >= self.target_lost_threshold:
                angular_vel = self.search_speed
                self._log_state('Target lost, searching...')
            else:
                angular_vel = 0.0
                self._log_state('Target lost, waiting...')
        else:
            self.target_lost_counter = 0
            image_center_x = cv_image.shape[1] / 2.0
            error = image_center_x - centroid[0]
            if abs(error) < self.center_tolerance:
                angular_vel = 0.0
                self._log_state('Target centered.')
            else:
                angular_vel = self.turn_gain * error
                self._log_state(f'Target detected, centering... Error: {error:.1f}')
        if self.real_robot:
            motion_cmd = TwistStamped(twist=Twist(angular=Vector3(z=angular_vel)))
            motion_cmd.header.stamp = self.get_clock().now().to_msg()
        else:
            motion_cmd = Twist(angular=Vector3(z=angular_vel))
        self.publisher_.publish(motion_cmd)
        display_image = cv_image.copy()
        if centroid is not None:
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