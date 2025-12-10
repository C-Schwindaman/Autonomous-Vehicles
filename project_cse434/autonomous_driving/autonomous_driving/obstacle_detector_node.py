import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np


class ObstacleDetectorNode(Node):
    """
    This node detects obstacles using camera images.
    - Subscribes to: /camera/image_raw 
    - Publishes to: /obstacle_vel 
    """
    def __init__(self, delta_r=0.25):
        super().__init__('obstacle_detector_node')

        self.delta_r = delta_r
        self.publisher = self.create_publisher(MarkerArray, 'obstacles', 1)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.lidar_centroids, 1)
        self.subscription


    def lidar_centroids(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # Replace invalid ranges with 0
        ranges[~np.isfinite(ranges)] = 0.0

        # Keep only valid indices
        valid = ranges > 0.0
        if not np.any(valid):
            return

        # Precompute x, y
        x_all = ranges * np.cos(angles)
        y_all = ranges * np.sin(angles)

        # Threshold for range difference (meters)
        T = 0.25

        obstacles = []
        current_cluster = []
        n = len(ranges)

        for i in range(n):
            if not valid[i]:
                # End cluster if currently in one
                if len(current_cluster) > 0:
                    obstacles.append(np.array(current_cluster))
                    current_cluster = []
                continue

            # Start new cluster if empty
            if len(current_cluster) == 0:
                current_cluster = [(x_all[i], y_all[i])]
                continue

            # Compare with previous valid point
            j = i - 1
            while j >= 0 and not valid[j]:
                j -= 1

            if j < 0:
                # No previous valid point
                current_cluster.append((x_all[i], y_all[i]))
            else:
                if abs(ranges[i] - ranges[j]) < T:
                    # Same object
                    current_cluster.append((x_all[i], y_all[i]))
                else:
                    # New object
                    obstacles.append(np.array(current_cluster))
                    current_cluster = [(x_all[i], y_all[i])]

        # Handle last cluster
        if len(current_cluster) > 0:
            obstacles.append(np.array(current_cluster))

        # Optional: handle wrap-around (first and last cluster)
        if len(obstacles) > 1:
            first_mean_r = np.mean(np.linalg.norm(obstacles[0], axis=1))
            last_mean_r = np.mean(np.linalg.norm(obstacles[-1], axis=1))
            if abs(first_mean_r - last_mean_r) < T:
                merged = np.vstack((obstacles[-1], obstacles[0]))
                obstacles = obstacles[1:-1]
                obstacles.insert(0, merged)

        # Compute centroids
        centroids = np.array([[np.mean(o[:, 0]), np.mean(o[:, 1]), 0.0] for o in obstacles])
        ids = np.arange(len(centroids)).astype(int)

        # Publish centroids
        self.pub_centroids(centroids.tolist(), ids, msg.header)

      
    def pub_centroids(self, points, ids, header):

        ma = MarkerArray()

        for id, p in zip(ids, points):
            mark = Marker()            
            mark.header = header
            mark.id = id.item()
            mark.type = Marker.SPHERE
            mark.pose = Pose(position=Point(x=p[0],y=p[1],z=p[2]), orientation=Quaternion(x=0.,y=0.,z=0.,w=1.))
            mark.scale.x = 0.25
            mark.scale.y = 0.25
            mark.scale.z = 0.25
            mark.color.a = 0.75
            mark.color.r = 0.25
            mark.color.g = 1.
            mark.color.b = 0.25
            mark.lifetime = Duration(seconds=0.4).to_msg()
            ma.markers.append(mark)

        self.publisher.publish( ma )



   


def main(args=None):   
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()