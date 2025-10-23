#!/usr/bin/env python3

"""Please implement the core tracking logic in the TODO sections."""

import math
import csv
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformListener, Buffer


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

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

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


class SparcStyleTrackerSkeleton(Node):
    """Skeleton node exposing TODOs for the core tracking logic."""

    def __init__(self):
        super().__init__('sparc_style_tracker_skeleton')

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
        self.declare_parameter('csv_output_path', '/tmp/sparc_tracks_log.csv')
        self.enable_csv_logging = self.get_parameter('enable_csv_logging').value
        self.csv_output_path = self.get_parameter('csv_output_path').value

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

        # Subscriptions
        self.sub = self.create_subscription(MarkerArray, '/polimove/pointpillars/det_markers', self.detection_callback, qos)
        self.pointcloud_sub = self.create_subscription(PointCloud2, pointcloud_topic, self.pointcloud_callback, qos)

        # Camera subscriptions (only if enabled)
        self.camera1_sub = None
        self.camera2_sub = None
        if self.use_camera:
            self.declare_parameter('camera1_topic', '/polimove/perception/camera/car_boxes_back_hoop_yolo')
            self.declare_parameter('camera2_topic', '/polimove/perception/camera/car_boxes_front_hoop_yolo')
            camera1_topic = self.get_parameter('camera1_topic').value
            camera2_topic = self.get_parameter('camera2_topic').value

            self.camera1_sub = self.create_subscription(MarkerArray, camera1_topic, self.camera1_detection_callback, qos)
            self.camera2_sub = self.create_subscription(MarkerArray, camera2_topic, self.camera2_detection_callback, qos)
            self.get_logger().info('Camera fusion enabled')
            self.get_logger().info(f'  Camera 1: {camera1_topic}')
            self.get_logger().info(f'  Camera 2: {camera2_topic}')
        else:
            self.get_logger().info('Camera fusion disabled (lidar-only mode)')

        # Publishers
        self.pub = self.create_publisher(MarkerArray, '/sparc/point_pillars/tracks', 1)
        self.marker_pub = self.create_publisher(Marker, '/sparc/point_pillars/overlay_text', 10)

        self.get_logger().info('Sparc-style tracker skeleton started (point cloud-driven filtering)')

    def _current_stamp_sec(self) -> float:
        """Return the latest known ROS time in seconds."""
        if self.now_time > 0.0:
            return self.now_time
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # TODO sections
    # ------------------------------------------------------------------
    def initialize_tracks(self, markers: List[Marker]):
        """Create initial tracks from the first set of detections."""
        # TODO: Create Obstacle instances from each marker, set their initial
        # state vectors and covariances, and append them to self.tracks_post.
        for marker in markers:
            self.add_state_from_marker(marker)

    def predict_track_constant_velocity(self, track: Obstacle, dt: float, update_time: bool = False) -> Obstacle:
        """Propagate a track forward assuming constant velocity."""
        # TODO: Implement the constant velocity motion model and covariance
        # prediction. Remember to update coast counts and timestamps.
        x_in = track.x
        P_in = track.P
        x = x_in[0, 0]
        y = x_in[1, 0]
        v = x_in[2, 0]
        psi = x_in[3, 0]
        psi_dot = x_in[4, 0]
        x_pred = np.zeros((5, 1))
        if abs(psi_dot) < 0.001:
            x_pred[0] = x + v * dt * math.cos(psi)
            x_pred[1] = y + v * dt * math.sin(psi)
            x_pred[2] = v
            x_pred[3] = psi
            x_pred[4] = psi_dot
        else:
            psi_k = psi + psi_dot * dt
            x_pred[0] = x + (v / psi_dot) * (math.sin(psi_k) - math.sin(psi))
            x_pred[1] = y + (v / psi_dot) * (math.cos(psi) - math.cos(psi_k))
            x_pred[2] = v
            x_pred[3] = psi_k
            x_pred[4] = psi_dot
            x_pred[3] = self.angle_in_range(x_pred[3])
        F = np.eye(5)
        if abs(psi_dot) < 0.001:
            F[0, 2] = dt * math.cos(psi)
            F[0, 3] = -v * dt * math.sin(psi)
            F[1, 2] = dt * math.sin(psi)
            F[1, 3] = v * dt * math.cos(psi)
            F[3, 4] = dt
        else:
            psi_k = x_pred[3, 0]
            dfx_dv = (math.sin(psi_k) - math.sin(psi)) / psi_dot
            dfx_dpsi = (v / psi_dot) * (math.cos(psi_k) - math.cos(psi))
            dfx_dpsi_dot = (v / (psi_dot**2)) * (psi_dot * dt * math.cos(psi_k) - math.sin(psi_k) + math.sin(psi))
            dfy_dv = (math.cos(psi) - math.cos(psi_k)) / psi_dot
            dfy_dpsi = (v / psi_dot) * (math.sin(psi_k) - math.sin(psi))
            dfy_dpsi_dot = (v / (psi_dot**2)) * (psi_dot * dt * math.sin(psi_k) - math.cos(psi) + math.cos(psi_k))
            F[0, 2] = dfx_dv
            F[0, 3] = dfx_dpsi
            F[0, 4] = dfx_dpsi_dot
            F[1, 2] = dfy_dv
            F[1, 3] = dfy_dpsi
            F[1, 4] = dfy_dpsi_dot
            F[3, 4] = dt
        q_scaled = self.q * dt
        Q = np.diag([0.0, 0.0, q_scaled, 0.0, q_scaled])
        P_pred = F @ P_in @ F.T + Q
        track.x = x_pred
        track.P = P_pred
        track.coast += 1
        track.last_update_time = track.last_update_time + dt
        return track

    def build_predicted_measurement(self, track: Obstacle) -> PredMeas:
        """Project a track into measurement space (x, y, yaw)."""
        # TODO: Fill in the measurement matrix H, measurement noise R, and
        # compute the predicted measurement mean (zp) and covariance (S).
        pred_meas = PredMeas()
        H = np.zeros((3, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 3] = 1.0
        r_val = self.r_asso
        R = np.diag([r_val, r_val, 0.1])
        pred_meas.zp = H @ track.x
        pred_meas.S = H @ track.P @ H.T + R
        return pred_meas

    def compute_likelihood(self, measurement: np.ndarray, p_meas: PredMeas) -> float:
        """Return the likelihood of a measurement for a given track."""
        # TODO: Implement the Gaussian likelihood computation using the
        # innovation and the predicted measurement covariance, you could also play with Euclidean Distance as simpler alternative.
        innovation = measurement - p_meas.zp
        innovation[2, 0] = self.angle_in_range(innovation[2, 0])
        try:
            S_inv = np.linalg.inv(p_meas.S)
            mah_dist_sq = innovation.T @ S_inv @ innovation
            if mah_dist_sq < 0:
                mah_dist_sq = 0.0
        except np.linalg.LinAlgError:
            self.get_logger().warn("Singular matrix S in compute_likelihood, track is likely lost.")
            return 0.0
        likelihood = math.exp(-0.5 * mah_dist_sq)
        return float(likelihood)

    def build_cost_matrix(self, markers: List[Marker], tracks=None):
        """Build the association cost matrix between measurements and tracks."""
        # TODO: Use build_predicted_measurement and compute_likelihood to fill
        # in the association weights for each measurement-track pair.
        if tracks is None:
            tracks = self.tracks_post
        num_measurements = len(markers)
        num_tracks = len(tracks)
        w_ij = np.zeros((num_measurements, num_tracks))
        for i, marker in enumerate(markers):
            yaw = self._yaw_from_quat(marker.pose.orientation)
            measurement_vec = np.array([
                [marker.pose.position.x], 
                [marker.pose.position.y], 
                [yaw]
            ])
            for j, track in enumerate(tracks):
                predicted_meas = self.build_predicted_measurement(track)
                likelihood = self.compute_likelihood(measurement_vec, predicted_meas)
                w_ij[i, j] = likelihood
        return w_ij

    def kalman_update(self, prior: Obstacle, measurement: np.ndarray, r_noise: float = None) -> Obstacle:
        """Perform the measurement update step."""
        # TODO: Implement the Kalman gain, state update, covariance update, and
        # status bookkeeping for a confirmed track.
        post = Obstacle()
        post.object_id = prior.object_id
        post.object_class = prior.object_class
        post.last_update_time = prior.last_update_time
        H = np.zeros((3, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 3] = 1.0
        if r_noise is not None:
            r_val = r_noise
        else:
            r_val = self.r
        R = np.diag([r_val, r_val, 0.1]) 
        predicted_measurement = H @ prior.x
        innovation = measurement - predicted_measurement
        innovation[2, 0] = self.angle_in_range(innovation[2, 0])
        try:
            P_prior = prior.P
            S = H @ P_prior @ H.T + R
            S_inv = np.linalg.inv(S)
            K = P_prior @ H.T @ S_inv
            x_post = prior.x + K @ innovation
            x_post[3, 0] = self.angle_in_range(x_post[3, 0])
            I = np.eye(5)
            P_post = (I - K @ H) @ P_prior
            post.x = x_post
            post.P = P_post
            post.coast = 0
            post.update_number = prior.update_number + 1
            if not prior.object_status and post.update_number > self.update_threshold:
                post.object_status = True
                self.get_logger().info(f"Track {post.object_id} CONFIRMED.")
            else:
                post.object_status = prior.object_status
            return post
        except np.linalg.LinAlgError:
            self.get_logger().warn(f"Kalman update failed for track {prior.object_id} (singular matrix S).")
            return prior
    
    # ------------------------------------------------------------------
    # Helper utilities (kept to show full node plumbing)
    # ------------------------------------------------------------------
    def add_state_from_marker(self, marker):
        yaw = self._yaw_from_quat(marker.pose.orientation)
        track = Obstacle()
        track.x = np.array([[marker.pose.position.x], [marker.pose.position.y], [0.0], [yaw], [0.0]])
        track.P = np.diag([0.1, 0.1, 3.0, 1.0, 1.0])
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
        for t in self.tracks_post:
            if t.object_status:
                if t.coast <= self.coast_threshold_confirmed:
                    chosen.append(t)
                else:
                    self.get_logger().debug(f"Dropping confirmed track (ID {t.object_id}) after coasting {t.coast} frames")
            else:
                if t.coast <= self.coast_threshold:
                    chosen.append(t)
                else:
                    self.get_logger().debug(f"Dropping provisional track after coasting {t.coast} frames")
        self.tracks_post = chosen
        self.publish_overlay_text(chosen)

    def logger(self, tracks):
        if len(tracks) == 0:
            return
        self.get_logger().info('=' * 80)
        self.get_logger().info(f"Active Tracks: {len(tracks)}")
        self.get_logger().info('-' * 80)
        for idx, track in enumerate(tracks):
            if track.object_status:
                x = float(track.x[0, 0])
                y = float(track.x[1, 0])
                v = float(track.x[2, 0])
                yaw = float(track.x[3, 0])
                yaw_rate = float(track.x[4, 0])
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
        self.get_logger().info('=' * 80)

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
        lines.append(f"Number of obstacles: {len(tracks_post)}")
        for ob in tracks_post:
            lines.append(
                f"ID: {ob.object_id} | Class: {ob.object_class} | Update #: {ob.update_number} | "
                f"Coast: {ob.coast} | Status: {'Active' if ob.object_status else 'Inactive'}"
            )
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
        arrow.id = id_counter
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
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def angle_in_range(self, theta_diff: float) -> float:
        if theta_diff > math.pi:
            theta_diff -= 2.0 * math.pi
        if theta_diff < -math.pi:
            theta_diff += 2.0 * math.pi
        return theta_diff

    def correct_angle(self, angle_diff: float) -> float:
        angle_diff = self.angle_in_range(angle_diff)
        if angle_diff > math.pi / 2.0:
            angle_diff -= math.pi
        if angle_diff < -math.pi / 2.0:
            angle_diff += math.pi
        angle_diff = self.angle_in_range(angle_diff)
        return angle_diff

    def save_tracks_to_csv(self, tracks, timestamp_sec):
        if not self.enable_csv_logging or self.csv_writer is None:
            return
        try:
            for track in tracks:
                if not track.object_status:
                    continue
                x_local = float(track.x[0, 0])
                y_local = float(track.x[1, 0])
                v = float(track.x[2, 0])
                yaw_local = float(track.x[3, 0])
                yaw_rate = float(track.x[4, 0])
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
            self.csv_file.flush()
        except Exception as exc:
            self.get_logger().warn(f'Failed to save tracks: {exc}')

    # ------------------------------------------------------------------
    # Main callbacks (unchanged to show realistic data flow)
    # ------------------------------------------------------------------
    def detection_callback(self, msg: MarkerArray):
        markers = [
            m for m in msg.markers
            if not (abs(m.pose.position.x) < 1e-6 and abs(m.pose.position.y) < 1e-6)
        ]

        if len(markers) > 0:
            m0 = markers[0]
            try:
                stamp = m0.header.stamp
                detection_time = stamp.sec + stamp.nanosec * 1e-9
                self.get_logger().debug(f'Detection timestamp: {detection_time:.3f}s')
            except Exception as exc:
                self.get_logger().warn(f'Failed to extract timestamp: {exc}')
                return

            if len(self.tracks_post) > 0:
                predicted_tracks = []
                for t in self.tracks_post:
                    track_time = t.last_update_time
                    time_delta = detection_time - track_time
                    if abs(time_delta) > 0.001:
                        if 0 < time_delta <= 0.25:
                            predicted_tracks.append(self.predict_track_constant_velocity(t, time_delta))
                        elif time_delta > 0.25:
                            self.get_logger().warn(f'Large time delta: {time_delta:.3f}s, capping to 0.1s')
                            predicted_tracks.append(self.predict_track_constant_velocity(t, 0.1))
                        else:
                            self.get_logger().warn(f'Negative time delta: {time_delta:.3f}s, using track as-is')
                            predicted_tracks.append(t)
                    else:
                        predicted_tracks.append(t)
                self.tracks_post = predicted_tracks

            if len(self.tracks_post) > 0:
                w_ij = self.build_cost_matrix(markers)
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
                self.initialize_tracks(markers)
                for t in self.tracks_post:
                    t.last_update_time = detection_time
                self.get_logger().info(f'Initialized {len(self.tracks_post)} tracks from detections')

            self.now_time = detection_time
            self.current_time = self.get_clock().now()
        else:
            self.get_logger().debug('Received empty detection array')

    def camera1_detection_callback(self, msg: MarkerArray):
        self.process_camera_detections(msg, camera_id=1)

    def camera2_detection_callback(self, msg: MarkerArray):
        self.process_camera_detections(msg, camera_id=2)

    def process_camera_detections(self, msg: MarkerArray, camera_id: int):
        markers = [
            m for m in msg.markers
            if not (abs(m.pose.position.x) < 1e-6 and abs(m.pose.position.y) < 1e-6)
        ]

        if len(markers) > 0:
            m0 = markers[0]
            try:
                stamp = m0.header.stamp
                detection_time = stamp.sec + stamp.nanosec * 1e-9
                self.get_logger().info(f'Camera {camera_id} detection timestamp: {detection_time:.3f}s')
            except Exception as exc:
                self.get_logger().warn(f'Failed to extract camera {camera_id} timestamp: {exc}')
                return

            if len(self.tracks_post) > 0:
                predicted_tracks = []
                for t in self.tracks_post:
                    track_time = t.last_update_time
                    time_delta = detection_time - track_time
                    if abs(time_delta) > 0.001:
                        if 0 < time_delta <= 0.25:
                            predicted_tracks.append(self.predict_track_constant_velocity(t, time_delta))
                        elif time_delta > 0.25:
                            self.get_logger().warn(f'Large time delta: {time_delta:.3f}s, capping to 0.1s')
                            predicted_tracks.append(self.predict_track_constant_velocity(t, 0.1))
                        else:
                            self.get_logger().warn(f'Negative time delta: {time_delta:.3f}s, using track as-is')
                            predicted_tracks.append(t)
                    else:
                        predicted_tracks.append(t)
                self.tracks_post = predicted_tracks

            if len(self.tracks_post) > 0:
                w_ij = self.build_cost_matrix(markers)
                self.tracks_prior = list(self.tracks_post)
                assignments = self.association_solver(w_ij)

                updated_tracks = list(self.tracks_prior)
                n_state = len(self.tracks_prior)
                for (meas_idx, state_idx) in assignments:
                    if state_idx < n_state:
                        oa = markers[meas_idx]
                        yaw_a = self._yaw_from_quat(oa.pose.orientation)
                        measurement = np.array([[oa.pose.position.x], [oa.pose.position.y], [yaw_a]])
                        updated_tracks[state_idx] = self.kalman_update(self.tracks_prior[state_idx], measurement, r_noise=self.r_camera)
                    else:
                        yaw = self._yaw_from_quat(markers[meas_idx].pose.orientation)
                        track = Obstacle()
                        track.x = np.array([[markers[meas_idx].pose.position.x], [markers[meas_idx].pose.position.y], [0.0], [yaw], [0.0]])
                        track.P = np.diag([0.1, 0.1, 3.0, 1.0, 1.0])
                        track.object_status = False
                        track.update_number = 0
                        track.object_class = camera_id
                        track.object_id = self.next_track_id
                        self.next_track_id += 1
                        track.coast = 0
                        track.last_update_time = detection_time
                        updated_tracks.append(track)
                self.tracks_post = updated_tracks
                self.get_logger().debug(f'Updated {len(markers)} camera {camera_id} detections with tracks')
            else:
                self.initialize_tracks(markers)
                for t in self.tracks_post:
                    t.last_update_time = detection_time
                    t.object_class = camera_id
                self.get_logger().info(f'Initialized {len(self.tracks_post)} tracks from camera {camera_id} detections')

            self.now_time = detection_time
            self.current_time = self.get_clock().now()
        else:
            self.get_logger().debug('Received empty camera detection array')

    def pointcloud_callback(self, msg: PointCloud2):
        pc_stamp = msg.header.stamp
        pc_time_sec = pc_stamp.sec + pc_stamp.nanosec * 1e-9
        self.current_time = self.get_clock().now()

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

        if len(self.tracks_post) < 1:
            self.prev_detection_time = pc_time_sec
            return

        self.iteration += 1
        predicted_tracks = []
        for t in self.tracks_post:
            track_time = t.last_update_time
            dt = pc_time_sec - track_time
            if dt <= 0 or dt > 0.25:
                self.get_logger().warn(f"Invalid dt: {dt:.3f}s for track at time {track_time:.3f}s, using fallback 0.1s")
                dt = 0.1
            predicted_track = self.predict_track_constant_velocity(t, dt, update_time=True)
            predicted_tracks.append(predicted_track)
        self.tracks_post = predicted_tracks

        self.keep_obstacles()
        self.logger(self.tracks_post)

        tracked_objects = MarkerArray()
        idc = 0
        for t in self.tracks_post:
            self.add_markers(t, idc, tracked_objects)
            box, arrow = tracked_objects.markers[-2], tracked_objects.markers[-1]
            if t.object_status:
                color = (0.0, 0.0, 1.0, 1.0)
            else:
                color = (0.6, 0.6, 0.6, 0.6)
            for marker in (box, arrow):
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            idc += 2
        self.pub.publish(tracked_objects)

        self.save_tracks_to_csv(self.tracks_post, pc_time_sec)
        self.prev_detection_time = pc_time_sec
        self.now_time = pc_time_sec

    def association_solver(self, w_ij: np.ndarray):
        """Simple greedy max-weight matching (kept to demonstrate association wiring)."""
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


def main(args=None):
    rclpy.init(args=args)
    node = SparcStyleTrackerSkeleton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv_file is not None:
            node.csv_file.close()
            node.get_logger().info(f'CSV file closed: {node.csv_output_path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
