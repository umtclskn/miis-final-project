import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

def quaternion_to_euler(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        prefix = f"/{self.robot_name}"
        self.cmd_pub = self.create_publisher(Twist, f"{prefix}/cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, f"{prefix}/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, f"{prefix}/odom", self.odom_callback, 10)

        self.scan = None
        self.current_pose = None
        self.goal_threshold = 0.2
        self.current_goal_index = 0

        path_file = f'{self.robot_name}_static_path.yml'
        pkg_dir = get_package_share_directory('robot_path_tracker')
        path_path = os.path.join(pkg_dir, 'config', path_file)
        self.goals = self.load_path(path_path)
        self.get_logger().info(f"Loaded {len(self.goals)} goals.")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

    def load_path(self, path_file):
        with open(path_file, 'r') as f:
            data = yaml.safe_load(f)
        return data['path']

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.current_pose = (x, y, yaw)

    def scan_callback(self, msg: LaserScan):
        self.scan = msg.ranges

    def control_loop(self):
        if self.current_pose is None or self.scan is None:
            return

        if self.current_goal_index >= len(self.goals):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("All goals reached!")
            return

        goal = self.goals[self.current_goal_index]
        gx, gy = goal
        x, y, yaw = self.current_pose

        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(angle_to_goal - yaw), math.cos(angle_to_goal - yaw))

        twist = Twist()

        if distance > self.goal_threshold:
            twist.angular.z = max(-0.5, min(0.5, 2.0 * angle_diff))
            if abs(angle_diff) < math.pi / 4:
                twist.linear.x = min(0.3, 0.5 * distance)
        else:
            self.get_logger().info(f"Goal {self.current_goal_index + 1} reached, moving to next.")
            self.current_goal_index += 1

        front = self.scan[175:185] if len(self.scan) >= 360 else []
        if front and min(front) < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().warn("Obstacle detected, rotating...")

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
