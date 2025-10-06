''' crumbs.py

    Outline of solution to Lab 6, Exercise 2

'''
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class CrumbsNode(Node):
    def __init__(self):
        super().__init__('crumbs')
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.parent_frame = 'odom'
        self.crumb_counter = 0
        self.distance_threshold = 0.5
        self.crumbs_list = []
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info('Crumbs node started. Waiting for robot to move...')

    def on_timer(self):
        target_frame = self.parent_frame
        source_frame = 'base_footprint'
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform from {target_frame} to {source_frame}: {e}', throttle_duration_sec=5)
            return
        translation = t.transform.translation
        distance = np.sqrt(translation.x**2 + translation.y**2)
        if distance > self.distance_threshold:
            self.get_logger().info(f'Distance {distance:.2f}m > {self.distance_threshold}m. Dropping crumb_{self.crumb_counter}.')
            new_crumb = TransformStamped()
            new_crumb.header.stamp = self.get_clock().now().to_msg() # Use current time for the new static frame
            new_crumb.header.frame_id = self.parent_frame
            new_crumb.child_frame_id = f'crumb_{self.crumb_counter}'
            new_crumb.transform = t.transform
            self.crumbs_list.append(new_crumb)
            self.static_broadcaster.sendTransform(self.crumbs_list)
            self.parent_frame = new_crumb.child_frame_id
            self.crumb_counter += 1

def main(args=None):
    rclpy.init(args=args)
    crumbs_node = CrumbsNode()
    try:
        rclpy.spin(crumbs_node)
    except KeyboardInterrupt:
        pass
    finally:
        crumbs_node.destroy_node()
        rclpy.shutdown()