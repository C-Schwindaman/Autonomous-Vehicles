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
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.img_pub = self.create_publisher(
            Image,
            '/image_processed',
            10)
        self.lane_pub = self.create_publisher(
            PointStamped,
            '/lane_point',
            10)
        self.white_mask_pub = self.create_publisher(Image, '/mask_white', 10)
        self.orange_mask_pub = self.create_publisher(Image, '/mask_orange', 10)
        self.bridge = CvBridge()
        self.lane_width = 300

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Processes the image to find the target point.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        height, width, _ = cv_image.shape
        roi_top = int(height * 0.80)
        roi = cv_image[roi_top:, :, :]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 170])
        upper_white = np.array([180, 100, 255])
        lower_orange = np.array([10, 200, 100])
        upper_orange = np.array([30, 255, 255])
        white_mask = cv2.inRange(hsv_roi, lower_white, upper_white)
        orange_mask = cv2.inRange(hsv_roi, lower_orange, upper_orange)
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
        try:
            white_mask_msg = self.bridge.cv2_to_imgmsg(white_mask, "mono8")
            orange_mask_msg = self.bridge.cv2_to_imgmsg(orange_mask, "mono8")
            white_mask_msg.header = msg.header
            orange_mask_msg.header = msg.header
            self.white_mask_pub.publish(white_mask_msg)
            self.orange_mask_pub.publish(orange_mask_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish masks: {e}')
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
            self.lane_width = abs(cx_white - cx_orange)
            lane_center_full_coords = (cx_lane, cy_lane + roi_top)
        elif white_found and not orange_found:
            cx_orange_est = cx_white - self.lane_width
            cx_lane = (cx_white + cx_orange_est) // 2
            cy_lane = cy_roi_center
            lane_center_full_coords = (cx_lane, cy_lane + roi_top)
        elif not white_found and orange_found:
            cx_white_est = cx_orange + self.lane_width
            cx_lane = (cx_orange + cx_white_est) // 2
            cy_lane = cy_roi_center
            lane_center_full_coords = (cx_lane, cy_lane + roi_top)
        if lane_center_full_coords:
            cv2.circle(cv_image, lane_center_full_coords, 7, (0, 0, 255), -1)
        try:
            processed_img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            processed_img_msg.header = msg.header
            self.img_pub.publish(processed_img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish processed image: {e}'
        lane_point_msg = PointStamped()
        lane_point_msg.header = msg.header
        image_center_x = float(width // 2)
        if lane_center_full_coords:
            lane_point_msg.point.x = float(lane_center_full_coords[0])
            lane_point_msg.point.y = float(lane_center_full_coords[1])
            lane_point_msg.point.z = image_center_x
        else:
            lane_point_msg.point.x = 0.0
            lane_point_msg.point.y = 0.0
            lane_point_msg.point.z = image_center_x 
        self.lane_pub.publish(lane_point_msg)

def main(args=None):
    rclpy.init(args=args)
    detect_node = DetectNode()
    try:
        rclpy.spin(detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            detect_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()