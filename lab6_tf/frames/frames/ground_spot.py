'''
GroundSpot Node
  Subscribes to camera images and camera_info
  Uses a trained logistic regression model to find a landmark in the image
  Uses camera intrinsics and extrinsics to convert the landmark pixel location
    to a ground point in base_footprint coordinates
  Publishes the ground point as a PointStamped message
'''
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
import json
from scipy.special import expit
from scipy.spatial.transform import Rotation as R
import argparse
import os

NAMESPACE = os.environ.get("ROBOT_NAMESPACE", "")

class GroundSpotNode(Node):
    def __init__(self, json_path: str, simulator=True):
        super().__init__('ground_spot')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.point_publisher = self.create_publisher(PointStamped, 'ground_point', 1)
        self.get_logger().info(f"Loading model from {json_path}")
        with open(json_path, 'r') as f:
            params = json.load(f)
            self.cvec = np.array(params['cvec'])
            self.intercept = np.array(params['intercept'])
        self.K = None
        self.D = None
        self.cam_rot = None
        self.cam_tran = None
        self.bridge = CvBridge()
        callback_group = MutuallyExclusiveCallbackGroup()
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.cam_info_callback, 1,
            callback_group=callback_group)
        cam_topic = '/camera/image_raw' if simulator else f'/{NAMESPACE}/camera/image_raw'
        self.image_sub = self.create_subscription(
            Image, cam_topic, self.image_callback, 1,
            callback_group=callback_group)
        self.get_logger().info("Ground Spot node started.")

    def cam_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d)
            self.get_logger().info("Camera intrinsics and distortion received and stored.")

    def initialize_extrinsics(self, stamp):
        ''' Initialize camera extrinsics from tf2 
            stamp: the current timestamp for finding the transformation
        '''
        if self.cam_rot is None:
            try:
                t = self.tf_buffer.lookup_transform(
                        "base_footprint",
                        "camera_rgb_optical_frame",
                        stamp)
                self.cam_rot = R.from_quat([t.transform.rotation.x, t.transform.rotation.y, 
                                            t.transform.rotation.z, t.transform.rotation.w])
                self.cam_tran = t.transform.translation
                self.get_logger().info('Initialized camera extrinsics')
                return True
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform camera to base: {ex}')
                return False
        return True

    def image_callback(self, msg):
        if self.K is None:
            self.get_logger().info('Waiting for camera_info...', throttle_duration_sec=5)
            return
        if not self.initialize_extrinsics(msg.header.stamp):
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        score = self.apply_model(cv_image)
        prob = self.prob_target(score)
        centroid, area, target_mask = self.find_largest_target(prob, minpix=100)
        self.plot_targets(cv_image, target_mask, centroid)
        cv.waitKey(1)
        if centroid is None:
            return        
        pixel_coords = np.array(centroid).reshape((-1, 2, 1))
        unit_focal_point = cv.undistortPoints(pixel_coords, self.K, self.D, R=None, P=None)
        unit_focal_point = unit_focal_point.reshape((2,))
        cam_point_vector = np.array([unit_focal_point[0], unit_focal_point[1], 1.0])
        nvec = self.cam_rot.apply(cam_point_vector)
        scale_lambda = -self.cam_tran.z / nvec[2]
        ground_point = Point(x=nvec[0] * scale_lambda + self.cam_tran.x,
                             y=nvec[1] * scale_lambda + self.cam_tran.y,
                             z=nvec[2] * scale_lambda + self.cam_tran.z)
        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp
        point_msg.header.frame_id = 'base_footprint'
        point_msg.point = ground_point
        self.point_publisher.publish(point_msg)

    def apply_model(self, img):
        ''' Application of trained logisitic regression to an image
            img:         [MxNx3] input 3-channel color image
            score:       [MxN] logistic regression score for each pixel (between -inf and inf)
        '''
        score = (img.astype(float) * self.cvec).sum(axis=2) + self.intercept
        return score

    def prob_target(self, score):
        ''' Transforms score to probability of target using sigmoid '''
        return expit(score)

    def find_largest_target(self, prob_target, threshold=0.5, minpix=20):
        ''' Finds largest contiguous target region
            prob_target: [MxN] input probability of target
            centroid:    [x,y] coordinates of the largest centroid
            area:        number of pixels in target
            target_mask: [MxN] binary mask with 1's at target
        '''
        binary_target = (prob_target > threshold).astype(np.uint8)
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(binary_target)
        if num_labels < 2:
            return None, None, []
        sorted_indices = np.argsort(stats[:, cv.CC_STAT_AREA])[:-1]
        target_mask = np.zeros_like(binary_target)
        for i in reversed(sorted_indices):
            if stats[i, cv.CC_STAT_AREA] >= minpix:
                target_mask = (labels == i).astype(np.uint8)
                return centroids[i], stats[i, cv.CC_STAT_AREA], target_mask
        return None, None, []

    def plot_targets(self, image, target_mask, centroid):
        ''' Plot detected target_mask and output to file
            target_mask: (NxM) numpy array, or else empty list
            centroids: list of [x,y] centroids
        ''' 
        out_im = image.copy()
        if target_mask.size:
            green_overlay = np.zeros_like(out_im)
            green_overlay[:, :, 1] = 128
            mask_3channel = target_mask[:, :, None].repeat(3, axis=2)
            out_im = out_im * (1 - mask_3channel) + (out_im // 2 + green_overlay) * mask_3channel
        if centroid is not None:
            loc = tuple(np.array(centroid).astype(int))
            cv.circle(out_im, loc, 12, (0, 0, 255), -1)
        cv.imshow("Target Detection", cv.resize(out_im, (800, 400)))

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--real_robot', action='store_true', help='Run in real robot mode (default is simulator)')
    parser.add_argument('--json_path', default='', help='Path to the JSON file with model coefficients')
    parsed_args = parser.parse_args(args=rclpy.utilities.remove_ros_args()[1:])
    if not parsed_args.json_path:
        raise ValueError("Please provide the --json_path to the model parameters file.")
    ground_spot_node = GroundSpotNode(json_path=parsed_args.json_path,
                                      simulator=not parsed_args.real_robot)
    try:
        rclpy.spin(ground_spot_node)
    except (KeyboardInterrupt, SystemExit):
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        cv.destroyAllWindows()
        ground_spot_node.destroy_node()
        rclpy.shutdown()