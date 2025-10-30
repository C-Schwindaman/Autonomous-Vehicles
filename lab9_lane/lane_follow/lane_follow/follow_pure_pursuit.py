import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import numpy as np
import math

class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__('follow_pure_pursuit_node')
        self.linear_velocity = 0.25
        self.ground_point_sub = self.create_subscription(
            PointStamped,
            '/ground_point',
            self.ground_point_callback,
            10)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.start_time = None
        self.is_running = False
        self.is_finished = False
        self.get_logger().info("Pure Pursuit Follower node started. Waiting for ground point...")

    def ground_point_callback(self, msg):
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        target_x = msg.point.x
        target_y = msg.point.y
        if target_x == 0.0 and target_y == 0.0:
            if self.is_running and not self.is_finished:
                self.is_running = False
                self.is_finished = True
                self.stop_robot()
                total_time = current_time_sec - self.start_time
                output_str = f"Linear Velocity:{self.linear_velocity}:Time to complete:{total_time:.2f}"
                self.get_logger().info("--- CURVY ROAD COMPLETE (Pure Pursuit) ---")
                self.get_logger().info(output_str)
                try:
                    with open('pp_params.txt', 'w') as f:
                        f.write(output_str + '\n')
                    self.get_logger().info("Saved results to pp_params.txt")
                except Exception as e:
                    self.get_logger().error(f"Failed to write pp_params.txt: {e}")
            return
        if self.start_time is None:
            self.start_time = current_time_sec
            self.is_running = True
        if not self.is_running or self.is_finished:
            return
        lookahead_distance = math.sqrt(target_x**2 + target_y**2)
        alpha = math.atan2(target_y, target_x)
        self.get_logger().info(f"Target: (x={target_x:.2f}, y={target_y:.2f}), "
                           f"Dist: {lookahead_distance:.2f}, Alpha: {math.degrees(alpha):.1f} deg")
        if lookahead_distance < 0.01:
            angular_vel = 0.0
        else:
            angular_vel = (2.0 * self.linear_velocity * math.sin(alpha)) / lookahead_distance
        self.get_logger().info(f"Calculated Angular Vel: {angular_vel:.3f} rad/s")
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = angular_vel
        self.cmd_pub.publish(twist_msg)

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info("Robot stop command sent.")

def main(args=None):
    rclpy.init(args=args)
    follow_pure_pursuit_node = PurePursuitFollower()
    try:
        rclpy.spin(follow_pure_pursuit_node)
    except KeyboardInterrupt:
        pass
    finally:
        follow_pure_pursuit_node.stop_robot()
        follow_pure_pursuit_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()