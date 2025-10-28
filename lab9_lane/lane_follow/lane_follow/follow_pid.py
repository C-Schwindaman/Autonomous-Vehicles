import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from lane_follow.pid import pid_controller

class Follower(Node):
    def __init__(self):
        super().__init__('follow_pid_node')
        kp = 2.0
        ki = 0.0
        kd = 0.3
        self.linear_velocity = 0.2
        self.focal_length = 550.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pid = pid_controller(kp=self.kp, ki=self.ki, kd=self.kd)
        self.lane_sub = self.create_subscription(
            PointStamped,
            '/lane_point',
            self.lane_callback,
            10)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.start_time = None
        self.is_running = False
        self.is_finished = False
        self.get_logger().info("Follower node started. Waiting for lane data...")

    def lane_callback(self, msg):
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_state_pixels = msg.point.x
        if current_state_pixels == 0.0 and msg.point.y == 0.0:
            if self.is_running and not self.is_finished:
                self.is_running = False
                self.is_finished = True
                self.stop_robot()
                total_time = current_time_sec - self.start_time
                output_str = f"Kp:{self.kp}:Ki:{self.ki}:Kd:{self.kd}:Linear Velocity:{self.linear_velocity}:Time to complete:{total_time:.2f}"
                self.get_logger().info("--- CURVY ROAD COMPLETE ---")
                self.get_logger().info(output_str)
                try:
                    with open('pid_params.txt', 'w') as f:
                        f.write(output_str + '\n')
                    self.get_logger().info("Saved results to pid_params.txt")
                except Exception as e:
                    self.get_logger().error(f"Failed to write pid_params.txt: {e}")
            return
        if self.start_time is None:
            self.start_time = current_time_sec
            self.is_running = True
        if not self.is_running or self.is_finished:
            return
        image_center_x = msg.point.z
        if image_center_x == 0.0:
            return
        target_angle = 0.0
        current_angle_state = (current_state_pixels - image_center_x) / self.focal_length
        (angular_vel, _, _, _) = self.pid.update_control(
            target=target_angle,
            state=current_angle_state,
            current_time_sec=current_time_sec
        )
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
    follow_pid_node = Follower()
    try:
        rclpy.spin(follow_pid_node)
    except KeyboardInterrupt:
        pass
    finally:
        follow_pid_node.stop_robot()
        follow_pid_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()