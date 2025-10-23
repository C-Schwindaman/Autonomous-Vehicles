#!/usr/bin/env python3

"""Please implement the core tracking logic and camera projection functions in the TODO sections."""

import math
import csv
from typing import List

import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import PointCloud2, Image
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge
import tf2_geometry_msgs
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point

BBOX_LENGTH = 4.8
BBOX_WIDTH = 2.0
BBOX_HEIGHT = 1.5


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert roll, pitch, yaw (in radians) into a quaternion tuple (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def euler_from_quaternion(q):
    """Convert quaternion iterable (x, y, z, w) into roll, pitch, yaw (radians)."""
    x, y, z, w = q

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class Obstacle:
    def __init__(self):
        # State: [x, y, v, yaw, yaw_rate]
        self.x = np.zeros((5, 1))
        self.P = np.zeros((5, 5))
        self.update_number = 0
        self.coast = 0
        self.object_status = False
        self.object_class = 0
        self.object_id = 0
        self.last_update_time = 0.0  # timestamp in seconds


class PredMeas:
    def __init__(self):
        self.zp = np.zeros((3, 1))
        self.S = np.zeros((3, 3))


class SparcStyleTrackerWithProjectionSkeleton(Node):
    """Skeleton node exposing TODOs for the core tracking logic and camera projection functions."""

    def __init__(self):
        super().__init__('sparc_style_tracker_with_projection_skeleton')

        # parameters to create the filtering recursion
        self.r = self.declare_parameter('r_pos', 0.5).value
        self.q = self.declare_parameter('q', 0.5).value
        self.r_asso = self.declare_parameter('r_asso', 0.1).value
        self.update_threshold = self.declare_parameter('update_threshold', 10).value
        self.coast_threshold = self.declare_parameter('coast_threshold', 5).value
        self.coast_threshold_confirmed = self.declare_parameter('coast_threshold_confirmed', 15).value
        self.track_max_age = self.declare_parameter('track_max_age', 3.0).value  # seconds
        self.filter_rate = self.declare_parameter('filter_rate', 10.0).value  # Hz

        # trackers
        self.tracks_prior: List[Obstacle] = []
        self.tracks_post: List[Obstacle] = []
        self.next_track_id = 0  # Counter for unique track IDs

        # iteration/time keeping
        self.iteration = 0
        self.current_time = self.get_clock().now()
        self.old_time = self.get_clock().now()
        self.now_time = 0.0  # Latest detection timestamp in seconds
        self.prev_detection_time = 0.0  # Previous detection timestamp for dt calculation

        # Latest detections buffer
        self.latest_markers: List[Marker] = []
        self.has_new_detections = False
        
        # Camera detection parameters (higher noise)
        self.declare_parameter('use_camera', False)
        self.use_camera = self.get_parameter('use_camera').value
        self.r_camera = self.declare_parameter('r_camera', 2.0).value  # Higher noise for camera
        self.r_asso_camera = self.declare_parameter('r_asso_camera', 0.5).value  # Higher association threshold

        # QoS and topics
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        # Point cloud topic parameter
        self.declare_parameter('pointcloud_topic', '/sparc/luminar_front_points/cartesian')
        pointcloud_topic = self.get_parameter('pointcloud_topic').value

        # CSV logging parameters
        self.declare_parameter('enable_csv_logging', False)
        self.declare_parameter('csv_output_path', '/home/sparc/sparc_ws/src/target_tracking/test/tracks_log.csv')
        self.enable_csv_logging = self.get_parameter('enable_csv_logging').value
        self.csv_output_path = self.get_parameter('csv_output_path').value
        
        # Initialize CSV file if logging is enabled
        self.csv_file = None
        self.csv_writer = None
        if self.enable_csv_logging:
            self.csv_file = open(self.csv_output_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'track_id', 'x_local', 'y_local', 'velocity', 'yaw', 'yaw_rate', 'update_number', 'coast', 'status'])
            self.get_logger().info(f'CSV logging enabled: {self.csv_output_path}')

        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lidar detection subscription
        self.sub = self.create_subscription(MarkerArray, '/polimove/pointpillars/det_markers', self.detection_callback, qos)
        self.pointcloud_sub = self.create_subscription(PointCloud2, pointcloud_topic, self.pointcloud_callback, qos)
        
        # Camera image subscriptions
        self.declare_parameter('front_camera_topic', '/front_camera/image')
        self.declare_parameter('rear_camera_topic', '/rear_camera/image')
        front_camera_topic = self.get_parameter('front_camera_topic').value
        rear_camera_topic = self.get_parameter('rear_camera_topic').value
        
        self.front_image_sub = self.create_subscription(
            Image, front_camera_topic, self.front_image_callback, qos)
        self.rear_image_sub = self.create_subscription(
            Image, rear_camera_topic, self.rear_image_callback, qos)

        # Publishers
        self.pub = self.create_publisher(MarkerArray, '/sparc/point_pillars/tracks', 1)
        self.marker_pub = self.create_publisher(Marker, '/sparc/point_pillars/overlay_text', 10)

        # Camera calibration parameters
        self.camera_front_params = {
            'name': 'camera_front_hoop',
            'frame_id': 'camera_rollhoop_front',
            'width': 1032,
            'height': 400,
            'fx': 1731.270,
            'fy': 1731.270,
            'cx': 530.000,
            'cy': 170.000,
            'radial_dist': [0.0, 0.0, 0.0],
            'tangential_dist': [0.0, 0.0]
        }
        
        self.camera_rear_params = {
            'name': 'camera_back_hoop',
            'frame_id': 'camera_rollhoop_rear',
            'width': 1032,
            'height': 400,
            'fx': 1731.270,
            'fy': 1731.270,
            'cx': 530.000,
            'cy': 170.000,
            'radial_dist': [0.0, 0.0, 0.0],
            'tangential_dist': [0.0, 0.0]
        }

        # Camera intrinsic matrices
        self.front_K = np.array([
            [self.camera_front_params['fx'], 0, self.camera_front_params['cx']],
            [0, self.camera_front_params['fy'], self.camera_front_params['cy']],
            [0, 0, 1]
        ])
        
        self.rear_K = np.array([
            [self.camera_rear_params['fx'], 0, self.camera_rear_params['cx']],
            [0, self.camera_rear_params['fy'], self.camera_rear_params['cy']],
            [0, 0, 1]
        ])

        # Distortion coefficients (k1, k2, p1, p2, k3)
        self.front_dist = np.array([
            self.camera_front_params['radial_dist'][0],
            self.camera_front_params['radial_dist'][1],
            self.camera_front_params['tangential_dist'][0],
            self.camera_front_params['tangential_dist'][1],
            self.camera_front_params['radial_dist'][2]
        ])
        
        self.rear_dist = np.array([
            self.camera_rear_params['radial_dist'][0],
            self.camera_rear_params['radial_dist'][1],
            self.camera_rear_params['tangential_dist'][0],
            self.camera_rear_params['tangential_dist'][1],
            self.camera_rear_params['radial_dist'][2]
        ])

        # Camera image buffers
        self.front_image = None
        self.rear_image = None
        self.front_image_timestamp = 0.0
        self.rear_image_timestamp = 0.0
        self.image_buffer_timeout = 2.0  # seconds
        self.bbox_dims = (BBOX_LENGTH, BBOX_WIDTH, BBOX_HEIGHT)
        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Image display parameters
        self.declare_parameter('display_images', True)
        self.declare_parameter('display_window_front', 'Front Camera')
        self.declare_parameter('display_window_rear', 'Rear Camera')
        self.display_images = self.get_parameter('display_images').value
        self.window_front = self.get_parameter('display_window_front').value
        self.window_rear = self.get_parameter('display_window_rear').value
        
        # Create OpenCV windows if display is enabled
        if self.display_images:
            cv2.namedWindow(self.window_front, cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(self.window_rear, cv2.WINDOW_AUTOSIZE)
            self.get_logger().info('Image display enabled')

        self.get_logger().info('Sparc-style tracker with projection skeleton started (point cloud-driven filtering)')
        self.get_logger().info(f'Camera image subscriptions:')
        self.get_logger().info(f'  Front camera: {front_camera_topic}')
        self.get_logger().info(f'  Rear camera: {rear_camera_topic}')


    def _current_stamp_sec(self) -> float:
        """Return the latest known ROS time in seconds."""
        if self.now_time > 0.0:
            return self.now_time
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # TODO sections - Core tracking functions
    # ------------------------------------------------------------------
    def initialize_tracks(self, markers: List[Marker]):
        """Create initial tracks from the first set of detections."""
        # TODO: Create Obstacle instances from each marker, set their initial
        # state vectors and covariances, and append them to self.tracks_post.
        raise NotImplementedError('TODO: implement track initialization logic')

    def predict_track(self, track: Obstacle, dt: float) -> Obstacle:
        """Predict a single track forward by dt using CTRV (Constant Turn Rate and Velocity) model.
        State: [x, y, v, yaw, yaw_rate]
        """
        prior = Obstacle()
        prior = track
        
        # Extract current state
        x, y, v, yaw, yaw_rate = track.x[0, 0], track.x[1, 0], track.x[2, 0], track.x[3, 0], track.x[4, 0]
        
        # CTRV motion model
        if abs(yaw_rate) > 1e-6:  # turning
            x_new = x + (v / yaw_rate) * (np.sin(yaw + yaw_rate * dt) - np.sin(yaw))
            y_new = y + (v / yaw_rate) * (-np.cos(yaw + yaw_rate * dt) + np.cos(yaw))
        else:  # straight line
            x_new = x + v * np.cos(yaw) * dt
            y_new = y + v * np.sin(yaw) * dt
        
        v_new = v  # constant velocity
        yaw_new = yaw + yaw_rate * dt
        yaw_rate_new = yaw_rate  # constant turn rate
        
        prior.x = np.array([[x_new], [y_new], [v_new], [yaw_new], [yaw_rate_new]])
        
        # Simplified Jacobian (linearized F matrix for covariance propagation)
        F = np.eye(5)
        if abs(yaw_rate) > 1e-6:
            F[0, 2] = (1.0 / yaw_rate) * (np.sin(yaw + yaw_rate * dt) - np.sin(yaw))
            F[0, 3] = (v / yaw_rate) * (np.cos(yaw + yaw_rate * dt) - np.cos(yaw))
            F[0, 4] = (v / (yaw_rate**2)) * (-np.sin(yaw + yaw_rate * dt) + np.sin(yaw)) + (v / yaw_rate) * np.cos(yaw + yaw_rate * dt) * dt
            F[1, 2] = (1.0 / yaw_rate) * (-np.cos(yaw + yaw_rate * dt) + np.cos(yaw))
            F[1, 3] = (v / yaw_rate) * (np.sin(yaw + yaw_rate * dt) - np.sin(yaw))
            F[1, 4] = (v / (yaw_rate**2)) * (np.cos(yaw + yaw_rate * dt) - np.cos(yaw)) + (v / yaw_rate) * np.sin(yaw + yaw_rate * dt) * dt
        else:
            F[0, 2] = np.cos(yaw) * dt
            F[0, 3] = -v * np.sin(yaw) * dt
            F[1, 2] = np.sin(yaw) * dt
            F[1, 3] = v * np.cos(yaw) * dt
        
        F[3, 4] = dt
        
        Q = np.eye(5) * self.q
        prior.P = F.dot(track.P).dot(F.T) + Q
        prior.coast = track.coast + 1
        return prior

    def predict_track_constant_velocity(self, track: Obstacle, dt: float, update_time: bool = False) -> Obstacle:
        """Propagate a track forward assuming constant velocity."""
        # TODO: Implement the constant velocity motion model and covariance
        # prediction. Remember to update coast counts and timestamps.
        raise NotImplementedError('TODO: implement the prediction step')

    def predict_all(self, dt: float):
        self.tracks_prior = [self.predict_track_constant_velocity(t, dt) for t in self.tracks_post]

    def build_predicted_measurement(self, track: Obstacle) -> PredMeas:
        """Project a track into measurement space (x, y, yaw)."""
        # TODO: Fill in the measurement matrix H, measurement noise R, and
        # compute the predicted measurement mean (zp) and covariance (S).
        raise NotImplementedError('TODO: implement the measurement prediction')

    def compute_likelihood(self, measurement: np.ndarray, p_meas: PredMeas) -> float:
        """Return the likelihood of a measurement for a given track."""
        # TODO: Implement the Gaussian likelihood computation using the
        # innovation and the predicted measurement covariance, you could also play with Euclidean Distance as simpler alternative.
        raise NotImplementedError('TODO: implement likelihood computation')

    def build_cost_matrix(self, markers: List[Marker], tracks=None):
        """Build the association cost matrix between measurements and tracks."""
        # TODO: Use build_predicted_measurement and compute_likelihood to fill
        # in the association weights for each measurement-track pair.
        raise NotImplementedError('TODO: implement cost matrix construction')

    def association_solver(self, w_ij: np.ndarray):
        """Greedy max-weight matching (simple and deterministic).
        Returns list of (meas_index, state_index).
        """
        n_meas, n_cols = w_ij.shape
        entries = []
        for i in range(n_meas):
            for j in range(n_cols):
                entries.append((i, j, w_ij[i, j]))
        entries.sort(key=lambda x: x[2], reverse=True)
        assigned_meas = set()
        assigned_state = set()
        assignments = []
        for i, j, w in entries:
            if i in assigned_meas or j in assigned_state:
                continue
            if w <= 0:
                break
            assigned_meas.add(i)
            assigned_state.add(j)
            assignments.append((i, j))
        return assignments

    def kalman_update(self, prior: Obstacle, measurement: np.ndarray, r_noise: float = None) -> Obstacle:
        """Perform the measurement update step."""
        # TODO: Implement the Kalman gain, state update, covariance update, and
        # status bookkeeping for a confirmed track.
        raise NotImplementedError('TODO: implement the measurement update')

    # ------------------------------------------------------------------
    # Helper utilities (kept to show full node plumbing)
    # ------------------------------------------------------------------
    def add_state_from_marker(self, marker):
        yaw = self._yaw_from_quat(marker.pose.orientation)
        track = Obstacle()
        # State: [x, y, v, yaw, yaw_rate]
        track.x = np.array([[marker.pose.position.x], [marker.pose.position.y], [0.0], [yaw], [0.0]])
        track.P = np.diag([0.1, 0.1, 3.0, 1.0, 1.0])  # [x, y, v, yaw, yaw_rate]
        track.object_status = False
        track.update_number = 0
        track.object_class = 0
        track.object_id = self.next_track_id
        self.next_track_id += 1
        track.coast = 0
        track.last_update_time = self._current_stamp_sec()
        self.tracks_post.append(track)

    def keep_obstacles(self):
        chosen = []
        # current_stamp = self._current_stamp_sec()
        
        for t in self.tracks_post:
            # Check coasting thresholds
            if t.object_status:
                if t.coast <= self.coast_threshold_confirmed:
                    chosen.append(t)
                else:
                    self.get_logger().debug(
                        f"Dropping confirmed track (ID {t.object_id}) after coasting {t.coast} frames"
                    )
            else:
                if t.coast <= self.coast_threshold:
                    chosen.append(t)
                else:
                    self.get_logger().debug(
                        f"Dropping provisional track after coasting {t.coast} frames"
                    )
        self.tracks_post = chosen
        self.publish_overlay_text(chosen)

    def logger(self, tracks):
        """Print state values for all active tracks."""
        if len(tracks) == 0:
            return
        
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Active Tracks: {len(tracks)}")
        self.get_logger().info("-" * 80)
        
        for idx, track in enumerate(tracks):
            if track.object_status:
                x = float(track.x[0, 0])
                y = float(track.x[1, 0])
                v = float(track.x[2, 0])
                yaw = float(track.x[3, 0])
                yaw_rate = float(track.x[4, 0])
                
                # Convert yaw to degrees for readability
                yaw_deg = np.degrees(yaw)
                yaw_rate_deg = np.degrees(yaw_rate)
                
                self.get_logger().info(
                    f"Track {idx} [ID:{track.object_id}] | "
                    f"x:{x:7.2f}m y:{y:7.2f}m | "
                    f"v:{v:5.2f}m/s | "
                    f"yaw:{yaw_deg:6.1f}° | "
                    f"yaw_rate:{yaw_rate_deg:6.2f}°/s | "
                    f"updates:{track.update_number} coast:{track.coast}"
                )
        self.get_logger().info("=" * 80)

    # ---------- Publishing / helpers ----------
    def publish_overlay_text(self, tracks_post):
        marker = Marker()
        marker.header.frame_id = 'center_of_gravity'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'overlay_text'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        q = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        lines = []
        lines.append(f'Number of obstacles: {len(tracks_post)}')
        for ob in tracks_post:
            lines.append(f'ID: {ob.object_id} | Class: {ob.object_class} | Update #: {ob.update_number} | Coast: {ob.coast} | Status: {"Active" if ob.object_status else "Inactive"}')
        marker.text = '\n'.join(lines)
        self.marker_pub.publish(marker)

    def add_markers(self, track_post: Obstacle, id_counter: int, tracked_objects: MarkerArray):
        box = Marker()
        box.header.frame_id = 'center_of_gravity'
        box.header.stamp = self.get_clock().now().to_msg()
        box.ns = 'bounding_boxes'
        box.id = id_counter
        box.type = Marker.CUBE
        box.pose.position.x = float(track_post.x[0])
        box.pose.position.y = float(track_post.x[1])
        box.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, float(track_post.x[3]))
        box.pose.orientation.x = q[0]
        box.pose.orientation.y = q[1]
        box.pose.orientation.z = q[2]
        box.pose.orientation.w = q[3]
        box.scale.x = 5.0
        box.scale.y = 2.0
        box.scale.z = 1.5
        box.color.r = 0.0
        box.color.g = 0.0
        box.color.b = 1.0
        box.color.a = 1.0
        box.lifetime = Duration(sec=0, nanosec=100000000)
        tracked_objects.markers.append(box)
        arrow = Marker()
        arrow.header.frame_id = 'center_of_gravity'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'bounding_box_direction'
        arrow.id = id_counter  # Arrow uses same ID as box, but different namespace
        arrow.type = Marker.ARROW
        arrow.pose.position.x = float(track_post.x[0])
        arrow.pose.position.y = float(track_post.x[1])
        arrow.pose.position.z = 0.75
       
        q = quaternion_from_euler(0, 0, float(track_post.x[3]))
        arrow.pose.orientation.x = q[0]
        arrow.pose.orientation.y = q[1]
        arrow.pose.orientation.z = q[2]
        arrow.pose.orientation.w = q[3]
        arrow.scale.x = 10.25
        arrow.scale.y = 0.6
        arrow.scale.z = 1.0
        arrow.color.r = 0.0
        arrow.color.g = 0.0
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.lifetime = Duration(sec=0, nanosec=100000000)
        tracked_objects.markers.append(arrow)

    def _yaw_from_quat(self, q_msg):
        q = [q_msg.x, q_msg.y, q_msg.z, q_msg.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        return yaw

    def angle_in_range(self, Theta_diff: float) -> float:
        if Theta_diff > math.pi:
            Theta_diff -= 2.0 * math.pi
        if Theta_diff < -math.pi:
            Theta_diff += 2.0 * math.pi
        return Theta_diff

    def correct_angle(self, angle_diff: float) -> float:
        angle_diff = self.angle_in_range(angle_diff)
        if angle_diff > math.pi / 2.0:
            angle_diff -= math.pi
        if angle_diff < -math.pi / 2.0:
            angle_diff += math.pi
        angle_diff = self.angle_in_range(angle_diff)
        return angle_diff

    def save_tracks_to_csv(self, tracks, timestamp_sec):
        """Save tracks to CSV in local (center_of_gravity) frame."""
        if not self.enable_csv_logging or self.csv_writer is None:
            return
        
        try:
            # Save each track in the local center_of_gravity frame
            for track in tracks:
                if not track.object_status:  # Only save confirmed tracks
                    continue
                
                # Extract track state in center_of_gravity frame
                x_local = float(track.x[0, 0])
                y_local = float(track.x[1, 0])
                v = float(track.x[2, 0])
                yaw_local = float(track.x[3, 0])
                yaw_rate = float(track.x[4, 0])
                
                # Write to CSV (local frame coordinates)
                self.csv_writer.writerow([
                    f'{timestamp_sec:.6f}',
                    track.object_id,
                    f'{x_local:.3f}',
                    f'{y_local:.3f}',
                    f'{v:.3f}',
                    f'{yaw_local:.6f}',
                    f'{yaw_rate:.6f}',
                    track.update_number,
                    track.coast,
                    'active' if track.object_status else 'inactive'
                ])
            
            # Flush to ensure data is written
            self.csv_file.flush()
            
        except Exception as e:
            self.get_logger().warn(f'Failed to save tracks: {e}')

    # ---------- Main callback wiring the functions ----------
    def detection_callback(self, msg: MarkerArray):
        """Callback for receiving detections - buffers them and updates with time synchronization."""
        markers = [
            m for m in msg.markers
            if not (abs(m.pose.position.x) < 1e-6 and abs(m.pose.position.y) < 1e-6)
        ]

        if markers:
            try:
                _stamp = markers[0].header.stamp
                _det_ts = _stamp.sec + _stamp.nanosec * 1e-9
                print(f"\033[91mDetection timestamp: {_det_ts:.3f}s\033[0m")
                
                # Project detections to camera images and get projections
                front_projections, rear_projections = self.project_detections_to_cameras(markers, _det_ts)
                
                # Display images with projections
                self.display_camera_images(front_projections, rear_projections)
                
            except AttributeError:
                pass
            except Exception as exc:
                self.get_logger().warn(f'Failed to process detection: {exc}')
        if len(markers) > 0:
            # Extract detection timestamp
            m0 = markers[0]
            try:
                stamp = m0.header.stamp
                detection_time = stamp.sec + stamp.nanosec * 1e-9
                self.get_logger().debug(f'Detection timestamp: {detection_time:.3f}s')
            except Exception as e:
                self.get_logger().warn(f'Failed to extract timestamp: {e}')
                return
            
            # If we have existing tracks, predict each to detection time
            if len(self.tracks_post) > 0:
                predicted_tracks = []
                for t in self.tracks_post:
                    # dt = detection_time - track's last update time
                    track_time = t.last_update_time
                    time_delta = detection_time - track_time
                    
                    if abs(time_delta) > 0.001:  # More than 1ms difference
                        if time_delta > 0 and time_delta <= 0.25:
                            # Predict track forward to detection time
                            self.get_logger().debug(f'Predicting track from {track_time:.3f}s by {time_delta:.3f}s to detection time {detection_time:.3f}s')
                            predicted_tracks.append(self.predict_track_constant_velocity(t, time_delta))
                        elif time_delta > 0.25:
                            self.get_logger().warn(f'Large time delta: {time_delta:.3f}s, capping to 0.1s')
                            predicted_tracks.append(self.predict_track_constant_velocity(t, 0.1))
                        else:
                            self.get_logger().warn(f'Negative time delta: {time_delta:.3f}s, using track as-is')
                            predicted_tracks.append(t)
                    else:
                        # Time delta too small, use track as-is
                        predicted_tracks.append(t)
                
                self.tracks_post = predicted_tracks
            
            # Now perform the update step
            if len(self.tracks_post) > 0:
                w_ij = self.build_cost_matrix(markers)
                # Use tracks_post as prior for this update since we just predicted
                self.tracks_prior = list(self.tracks_post)
                assignments = self.association_solver(w_ij)

                updated_tracks = list(self.tracks_prior)
                n_state = len(self.tracks_prior)
                for (meas_idx, state_idx) in assignments:
                    if state_idx < n_state:
                        oa = markers[meas_idx]
                        yaw_a = self._yaw_from_quat(oa.pose.orientation)
                        measurement = np.array([[oa.pose.position.x], [oa.pose.position.y], [yaw_a]])
                        updated_tracks[state_idx] = self.kalman_update(self.tracks_prior[state_idx], measurement)
                    else:
                        # New track from unmatched detection
                        yaw = self._yaw_from_quat(markers[meas_idx].pose.orientation)
                        track = Obstacle()
                        track.x = np.array([[markers[meas_idx].pose.position.x], [markers[meas_idx].pose.position.y], [0.0], [yaw], [0.0]])
                        track.P = np.diag([0.1, 0.1, 3.0, 1.0, 1.0])
                        track.object_status = False
                        track.update_number = 0
                        track.object_class = 0
                        track.object_id = self.next_track_id
                        self.next_track_id += 1
                        track.coast = 0
                        track.last_update_time = detection_time
                        updated_tracks.append(track)
                
                self.tracks_post = updated_tracks
                self.get_logger().debug(f'Updated {len(markers)} detections with tracks')
            else:
                # No existing tracks, initialize from detections
                self.initialize_tracks(markers)
                # Update all track timestamps to detection time
                for t in self.tracks_post:
                    t.last_update_time = detection_time
                self.get_logger().info(f'Initialized {len(self.tracks_post)} tracks from detections')
            
            # Update now_time with detection timestamp
            self.now_time = detection_time
            self.current_time = self.get_clock().now()
        else:
            self.get_logger().debug('Received empty detection array')

    def camera1_detection_callback(self, msg: MarkerArray):
        """Callback for camera 1 detections."""
        self.process_camera_detections(msg, camera_id=1)

    def camera2_detection_callback(self, msg: MarkerArray):
        """Callback for camera 2 detections."""

        try:
            stamp = msg.header.stamp
            timestamp_sec = stamp.sec + stamp.nanosec * 1e-9
            print(f"\033[92mCamera 2 timestamp: {timestamp_sec:.3f}s\033[0m")
        except AttributeError:
            self.get_logger().warn('Camera 2 message has no header stamp')
        self.process_camera_detections(msg, camera_id=2)

    def process_camera_detections(self, msg: MarkerArray, camera_id: int):
        """Shared processing function for camera detections with higher measurement noise."""
        markers = [
            m for m in msg.markers
            if not (abs(m.pose.position.x) < 1e-6 and abs(m.pose.position.y) < 1e-6)
        ]
        
        if len(markers) > 0:
            # Extract detection timestamp
            m0 = markers[0]
            try:
                stamp = m0.header.stamp
                detection_time = stamp.sec + stamp.nanosec * 1e-9
                self.get_logger().info(f'Camera {camera_id} detection timestamp: {detection_time:.3f}s')
            except Exception as e:
                self.get_logger().warn(f'Failed to extract camera {camera_id} timestamp: {e}')
                return
            
            # If we have existing tracks, predict each to detection time
            if len(self.tracks_post) > 0:
                predicted_tracks = []
                for t in self.tracks_post:
                    # dt = detection_time - track's last update time
                    track_time = t.last_update_time
                    time_delta = detection_time - track_time
                    
                    if abs(time_delta) > 0.001:  # More than 1ms difference
                        if time_delta > 0 and time_delta <= 0.25:
                            # Predict track forward to detection time
                            self.get_logger().debug(f'Predicting track from {track_time:.3f}s by {time_delta:.3f}s to camera {camera_id} detection time {detection_time:.3f}s')
                            predicted_tracks.append(self.predict_track_constant_velocity(t, time_delta))
                        elif time_delta > 0.25:
                            self.get_logger().warn(f'Large time delta: {time_delta:.3f}s, capping to 0.1s')
                            predicted_tracks.append(self.predict_track_constant_velocity(t, 0.1))
                        else:
                            self.get_logger().warn(f'Negative time delta: {time_delta:.3f}s, using track as-is')
                            predicted_tracks.append(t)
                    else:
                        # Time delta too small, use track as-is
                        predicted_tracks.append(t)
                
                self.tracks_post = predicted_tracks
            
            # Now perform the update step with higher noise
            if len(self.tracks_post) > 0:
                w_ij = self.build_cost_matrix(markers)
                # Use tracks_post as prior for this update since we just predicted
                self.tracks_prior = list(self.tracks_post)
                assignments = self.association_solver(w_ij)

                updated_tracks = list(self.tracks_prior)
                n_state = len(self.tracks_prior)
                for (meas_idx, state_idx) in assignments:
                    if state_idx < n_state:
                        oa = markers[meas_idx]
                        yaw_a = self._yaw_from_quat(oa.pose.orientation)
                        measurement = np.array([[oa.pose.position.x], [oa.pose.position.y], [yaw_a]])
                        # Use higher noise for camera detections
                        updated_tracks[state_idx] = self.kalman_update(self.tracks_prior[state_idx], measurement, r_noise=self.r_camera)
                    else:
                        # New track from unmatched camera detection
                        yaw = self._yaw_from_quat(markers[meas_idx].pose.orientation)
                        track = Obstacle()
                        track.x = np.array([[markers[meas_idx].pose.position.x], [markers[meas_idx].pose.position.y], [0.0], [yaw], [0.0]])
                        track.P = np.diag([0.1, 0.1, 3.0, 1.0, 1.0])
                        track.object_status = False
                        track.update_number = 0
                        track.object_class = camera_id  # Mark with camera ID (1 or 2)
                        track.object_id = self.next_track_id
                        self.next_track_id += 1
                        track.coast = 0
                        track.last_update_time = detection_time
                        updated_tracks.append(track)
                
                self.tracks_post = updated_tracks
                self.get_logger().debug(f'Updated {len(markers)} camera {camera_id} detections with tracks')
            else:
                # No existing tracks, initialize from camera detections
                self.initialize_tracks(markers)
                # Update all track timestamps to detection time and mark as camera
                for t in self.tracks_post:
                    t.last_update_time = detection_time
                    t.object_class = camera_id  # Mark with camera ID (1 or 2)
                self.get_logger().info(f'Initialized {len(self.tracks_post)} tracks from camera {camera_id} detections')
            
            # Update now_time with detection timestamp
            self.now_time = detection_time
            self.current_time = self.get_clock().now()
        else:
            self.get_logger().debug('Received empty camera detection array')

    def pointcloud_callback(self, msg: PointCloud2):
        """Point cloud-driven prediction - runs at lidar frequency."""
        # Extract timestamp from point cloud message
        pc_stamp = msg.header.stamp
        pc_time_sec = pc_stamp.sec + pc_stamp.nanosec * 1e-9
        
        print(f"\033[93mPoint cloud timestamp: {pc_time_sec:.3f}s\033[0m")
        self.current_time = self.get_clock().now()

        # Display camera images (even without detections) on every pointcloud callback
        self.display_camera_images()

        # Clean up stale tracks at every cycle
        if len(self.tracks_post) > 0:
            stale_removed = 0
            tracks_to_keep = []
            for t in self.tracks_post:
                age = pc_time_sec - t.last_update_time
                if age <= self.track_max_age:
                    tracks_to_keep.append(t)
                else:
                    stale_removed += 1
                    self.get_logger().debug(
                        f"Removing stale track (ID {t.object_id}) - age {age:.2f}s"
                    )
            self.tracks_post = tracks_to_keep
            if stale_removed > 0:
                self.get_logger().info(f"Removed {stale_removed} stale track(s)")

        # If no tracks exist, nothing to predict
        if len(self.tracks_post) < 1:
            self.prev_detection_time = pc_time_sec
            return

        # Compute dt for prediction using point cloud timestamp
        self.iteration += 1
        
        # Predict each track from its last update time to current point cloud time
        predicted_tracks = []
        for t in self.tracks_post:
            # dt = point_cloud_time - track's last update time
            track_time = t.last_update_time
            dt = pc_time_sec - track_time
            
            if dt <= 0 or dt > 0.25:  # Sanity check: reject negative or >0.25s dt
                self.get_logger().warn(f"Invalid dt: {dt:.3f}s for track at time {track_time:.3f}s, using fallback 0.1s")
                dt = 0.1  # Use reasonable fallback
            
            # Predict this track using its individual dt and update its time to pc_time_sec
            self.get_logger().debug(f"Predicting track from {track_time:.3f}s by dt={dt:.3f}s to PC time {pc_time_sec:.3f}")
            predicted_track = self.predict_track_constant_velocity(t, dt, update_time=True)
            predicted_tracks.append(predicted_track)
        
        self.tracks_post = predicted_tracks

        # Apply track management logic (coasting checks)
        self.keep_obstacles()
        self.logger(self.tracks_post)

        # Publish tracked objects
        tracked_objects = MarkerArray()
        idc = 0
        for t in self.tracks_post:
            self.add_markers(t, idc, tracked_objects)
            box, arrow = tracked_objects.markers[-2], tracked_objects.markers[-1]
            if t.object_status:
                color = (0.0, 0.0, 1.0, 1.0)  # active: blue
            else:
                color = (0.6, 0.6, 0.6, 0.6)  # inactive: gray
            for marker in (box, arrow):
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            idc += 2
        self.pub.publish(tracked_objects)
        
        # Save tracks to CSV in map frame
        self.save_tracks_to_csv(self.tracks_post, pc_time_sec)
        
        # Update timestamp for next iteration
        self.prev_detection_time = pc_time_sec
        self.now_time = pc_time_sec

    def front_image_callback(self, msg: Image):
        """Callback for front camera images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.front_image = cv_image
            stamp = msg.header.stamp
            self.front_image_timestamp = stamp.sec + stamp.nanosec * 1e-9
            self.get_logger().debug(f'Received front camera image: {self.front_image_timestamp:.3f}s')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert front camera image: {e}')

    def rear_image_callback(self, msg: Image):
        """Callback for rear camera images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rear_image = cv_image
            stamp = msg.header.stamp
            self.rear_image_timestamp = stamp.sec + stamp.nanosec * 1e-9
            self.get_logger().debug(f'Received rear camera image: {self.rear_image_timestamp:.3f}s')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert rear camera image: {e}')

    def is_image_valid(self, image, timestamp, current_time):
        """Check if the image is valid and not too old."""
        if image is None:
            return False
        age = current_time - timestamp
        return age <= self.image_buffer_timeout

    def determine_camera_for_detection(self, detection_x, detection_y):
        
        if detection_x > 0:
            return 'front'
        elif detection_x < 0:
            return 'rear'
        else:
            return None  # Object at x=0, unclear which camera

    def transform_point_to_camera_frame(self, point_cog, target_frame, timestamp_sec):
        try:
            # Create PointStamped message
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'center_of_gravity'
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = float(point_cog[0])
            point_stamped.point.y = float(point_cog[1])
            point_stamped.point.z = float(point_cog[2])
            
            # Transform to camera frame
            transformed = self.tf_buffer.transform(point_stamped, target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
            
            return [transformed.point.x, transformed.point.y, transformed.point.z]
            
        except Exception as e:
            self.get_logger().warn(f'Transform to {target_frame} failed: {e}')
            return None

    # ------------------------------------------------------------------
    # TODO sections - Camera projection functions
    # ------------------------------------------------------------------
    def project_3d_to_2d(self, point_3d, camera_matrix, dist_coeffs):
        """Project a 3D point in camera frame to 2D image coordinates."""
        # TODO: Implement 3D to 2D projection using OpenCV's projectPoints.
        # Handle cases where the point is behind the camera (z <= 0).
        # Return pixel coordinates [u, v] or None if projection fails.
        raise NotImplementedError('TODO: implement 3D to 2D projection')

    def is_point_in_image(self, pixel_coords, width, height):
        """Check if pixel coordinates are within image boundaries."""
        if pixel_coords is None:
            return False
        u, v = pixel_coords
        return 0 <= u < width and 0 <= v < height

    def _compute_bbox_corners_cog(self, center_xyz, yaw_rad, dims):
        """Compute 3D bounding box corners in center_of_gravity frame."""
        # TODO: Create the 8 corners of a 3D bounding box centered at center_xyz
        # with the given dimensions (length, width, height) and rotated by yaw_rad.
        # Return the 8 corner points as a numpy array of shape (8, 3).
        raise NotImplementedError('TODO: implement 3D bounding box corner computation')

    def _transform_points_to_camera_frame(self, points_cog, target_frame, timestamp_sec):
        try:
            tf_time = Time(seconds=float(timestamp_sec))
        except (TypeError, ValueError):
            tf_time = self.get_clock().now()
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                'center_of_gravity',
                tf_time,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as exc:
            self.get_logger().warn(f'TF lookup to {target_frame} failed: {exc}')
            return None
        transformed_pts = []
        stamp_msg = tf_time.to_msg()
        for pt in points_cog:
            ps = PointStamped()
            ps.header.frame_id = 'center_of_gravity'
            ps.header.stamp = stamp_msg
            ps.point.x, ps.point.y, ps.point.z = map(float, pt)
            try:
                out = do_transform_point(ps, transform)
                transformed_pts.append([out.point.x, out.point.y, out.point.z])
            except Exception as exc:
                self.get_logger().debug(f'Point transform failed: {exc}')
        return transformed_pts if transformed_pts else None

    def _project_points_to_image(self, points_cam, camera_matrix, dist_coeffs):
        """Project multiple 3D points in camera frame to 2D image coordinates."""
        # TODO: Project a list of 3D points to 2D image coordinates.
        # Use OpenCV's projectPoints with the appropriate rotation matrix
        # to handle coordinate frame conversion from ROS to OpenCV convention.
        # Return a list of [u, v] pixel coordinates.
        raise NotImplementedError('TODO: implement multiple point projection')

    def _build_projection_packet(self, bbox_pixels, points_cam):
        if not bbox_pixels or not points_cam:
            return None
        center_cam = np.mean(np.asarray(points_cam), axis=0)
        return {
            'bbox_pixels': bbox_pixels,
            'point_3d': center_cam.tolist(),
        }

    def add_image_info_overlay(self, image, camera_name, timestamp, num_detections=0):
        if image is None:
            return image
        img_copy = image.copy()
        lines = [
            f'{camera_name} Camera',
            f'Time: {timestamp:.3f}s',
            f'Detections: {num_detections}',
        ]
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thickness = 2
        y = 30
        for text in lines:
            size = cv2.getTextSize(text, font, scale, thickness)[0]
            cv2.rectangle(img_copy,
                          (10, y - size[1] - 6),
                          (10 + size[0] + 10, y + 6),
                          (0, 0, 0),
                          -1)
            cv2.putText(img_copy, text, (15, y), font, scale, (255, 255, 255), thickness)
            y += 28
        return img_copy

    def _draw_point_overlay(self, image, pixel_coords, detection_info=None):
        if image is None or pixel_coords is None:
            return image
        img_copy = image.copy()
        u, v = int(pixel_coords[0]), int(pixel_coords[1])
        cv2.circle(img_copy, (u, v), 10, (0, 255, 0), 2)
        cv2.line(img_copy, (u - 15, v), (u + 15, v), (0, 255, 0), 2)
        cv2.line(img_copy, (u, v - 15), (u, v + 15), (0, 255, 0), 2)
        text = f"({u}, {v})"
        if detection_info and 'point_3d' in detection_info:
            text += f" 3D:{tuple(round(val, 1) for val in detection_info['point_3d'])}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        cv2.rectangle(
            img_copy,
            (u + 15, v - text_size[1] - 5),
            (u + 15 + text_size[0], v + 5),
            (0, 0, 0),
            -1,
        )
        cv2.putText(img_copy, text, (u + 15, v), font, font_scale, (0, 255, 0), thickness)
        return img_copy

    def draw_detection_on_image(self, image, pixel_payload, detection_info=None):
        if image is None or pixel_payload is None:
            return image
        if isinstance(pixel_payload, list) and len(pixel_payload) == 8:
            img_copy = image.copy()
            corners = [(int(p[0]), int(p[1])) for p in pixel_payload]
            bottom = corners[0:4]
            top = corners[4:8]
            for i in range(4):
                cv2.line(img_copy, bottom[i], bottom[(i + 1) % 4], (0, 0, 255), 2)
                cv2.line(img_copy, top[i], top[(i + 1) % 4], (0, 0, 255), 2)
                cv2.line(img_copy, corners[i], corners[i + 4], (0, 0, 255), 2)
            cv2.line(img_copy, corners[1], corners[2], (0, 0, 255), 2)
            cv2.line(img_copy, corners[5], corners[6], (0, 0, 255), 2)
            centroid = tuple(np.mean(corners, axis=0).astype(int))
            cv2.circle(img_copy, centroid, 4, (0, 0, 255), -1)
            if detection_info and 'point_3d' in detection_info:
                text = f"{centroid} | 3D:{tuple(round(v, 1) for v in detection_info['point_3d'])}"
                cv2.putText(
                    img_copy,
                    text,
                    (centroid[0] + 10, centroid[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 0, 255),
                    1,
                )
            return img_copy
        return self._draw_point_overlay(image, pixel_payload, detection_info)

    def display_camera_images(self, front_projections=None, rear_projections=None):
        """Display camera images with projected detections.
        
        Args:
            front_projections: List of front camera projections
            rear_projections: List of rear camera projections
        """
        if not self.display_images:
            return
        
        # Display front camera image
        if self.front_image is not None:
            front_img = self.front_image.copy()
            num_front_detections = 0
            
            if front_projections:
                num_front_detections = len(front_projections)
                for proj in front_projections:
                    payload = proj.get('bbox_pixels') or proj.get('pixel_coords')
                    front_img = self.draw_detection_on_image(front_img, payload, proj)
            
            # Add info overlay
            front_img = self.add_image_info_overlay(
                front_img, "Front", self.front_image_timestamp, num_front_detections
            )
            
            cv2.imshow(self.window_front, front_img)
        else:
            # Show black image with "No Image" text if no front image available
            no_img = np.zeros((400, 640, 3), dtype=np.uint8)
            cv2.putText(no_img, "No Front Camera Image", (200, 200), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow(self.window_front, no_img)
        
        # Display rear camera image
        if self.rear_image is not None:
            rear_img = self.rear_image.copy()
            num_rear_detections = 0
            
            if rear_projections:
                num_rear_detections = len(rear_projections)
                for proj in rear_projections:
                    payload = proj.get('bbox_pixels') or proj.get('pixel_coords')
                    rear_img = self.draw_detection_on_image(rear_img, payload, proj)
            
            # Add info overlay
            rear_img = self.add_image_info_overlay(
                rear_img, "Rear", self.rear_image_timestamp, num_rear_detections
            )
            
            cv2.imshow(self.window_rear, rear_img)
        else:
            # Show black image with "No Image" text if no rear image available
            no_img = np.zeros((400, 640, 3), dtype=np.uint8)
            cv2.putText(no_img, "No Rear Camera Image", (200, 200), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow(self.window_rear, no_img)
        
        # Process OpenCV events (required for display)
        cv2.waitKey(1)

    def project_detections_to_cameras(self, detections, current_time_sec):
        front_projections = []
        rear_projections = []
        if not detections:
            return front_projections, rear_projections
        front_valid = self.is_image_valid(self.front_image, self.front_image_timestamp, current_time_sec)
        rear_valid = self.is_image_valid(self.rear_image, self.rear_image_timestamp, current_time_sec)
        dims = self.bbox_dims
        for detection in detections:
            x_cog = detection.pose.position.x
            y_cog = detection.pose.position.y
            z_cog = detection.pose.position.z 
            yaw_cog = self._yaw_from_quat(detection.pose.orientation)
            bbox_corners_cog = self._compute_bbox_corners_cog([x_cog, y_cog, z_cog], yaw_cog, dims)
            camera_choice = self.determine_camera_for_detection(x_cog, y_cog)
            if camera_choice == 'front' and front_valid:
                pts_cam = self._transform_points_to_camera_frame(
                    bbox_corners_cog, self.camera_front_params['frame_id'], current_time_sec
                )

                print(pts_cam)
                pixels = self._project_points_to_image(pts_cam, self.front_K, self.front_dist) if pts_cam else None

                if pixels:
                    print(f"Front camera pixels: {pixels}")
                else:
                    print("Front camera: No valid pixel projections")
                packet = self._build_projection_packet(pixels, pts_cam)
                if packet:
                    front_projections.append(packet)
                elif pts_cam:
                    center_cam = np.mean(pts_cam, axis=0)
                    center_pixels = self.project_3d_to_2d(center_cam, self.front_K, self.front_dist)
                    if center_pixels:
                        front_projections.append({'pixel_coords': center_pixels, 'point_3d': center_cam.tolist()})
            elif camera_choice == 'rear' and rear_valid:
                pts_cam = self._transform_points_to_camera_frame(
                    bbox_corners_cog, self.camera_rear_params['frame_id'], current_time_sec
                )
                pixels = self._project_points_to_image(pts_cam, self.rear_K, self.rear_dist) if pts_cam else None

                if pixels:
                    print(f"Front camera pixels: {pixels}")
                else:
                    print("Front camera: No valid pixel projections")

                print(pts_cam)
                packet = self._build_projection_packet(pixels, pts_cam)
                if packet:
                    rear_projections.append(packet)
                elif pts_cam:
                    center_cam = np.mean(pts_cam, axis=0)
                    center_pixels = self.project_3d_to_2d(center_cam, self.rear_K, self.rear_dist)
                    if center_pixels:
                        rear_projections.append({'pixel_coords': center_pixels, 'point_3d': center_cam.tolist()})
        return front_projections, rear_projections

def main(args=None):
    rclpy.init(args=args)
    node = SparcStyleTrackerWithProjectionSkeleton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.display_images:
            cv2.destroyAllWindows()
        if node.csv_file is not None:
            node.csv_file.close()
            node.get_logger().info(f'CSV file closed: {node.csv_output_path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()