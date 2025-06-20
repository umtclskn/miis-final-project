import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, LookupException
from geometry_msgs.msg import TransformStamped

LOOKAHEAD_DIST = 0.3
MAX_LIN_VEL = 0.1
MAX_ANG_VEL = 0.5
ANGLE_GAIN = 2.0
DIST_GAIN = 1.0
GOAL_THRESH = 0.15
ROBOT_DETECTION_RADIUS = 0.5  # Diğer robotları tespit etme mesafesi
ROBOT_SAFETY_RADIUS = 0.3     # Diğer robotlardan güvenli mesafe

def quaternion_to_euler(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('obstacle_avoidance', True)
        self.declare_parameter('robot_collision_avoidance', True)  # Yeni parametre

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.avoid_obstacle = self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value
        self.avoid_robots = self.get_parameter('robot_collision_avoidance').get_parameter_value().bool_value

        ns = f"/{self.robot_name}"
        self.cmd_pub = self.create_publisher(Twist, f"{ns}/cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, f"{ns}/scan", self.scan_callback, 10)

        self.scan = None
        self.current_goal_index = 0
        self.other_robot_positions = {}  # Diğer robotların pozisyonlarını saklamak için

        # Mevcut robotun adına göre diğer robotların listesini oluştur
        self.all_robot_names = ['robot1', 'robot2', 'robot3']
        self.other_robots = [robot for robot in self.all_robot_names if robot != self.robot_name]
        self.get_logger().info(f"Tracking other robots: {self.other_robots}")

        # Load static path
        path_file = f"{self.robot_name}_static_path.yml"
        pkg_dir = get_package_share_directory('robot_path_tracker')
        path_path = os.path.join(pkg_dir, 'config', path_file)
        self.goals = self.load_path(path_path)
        self.get_logger().info(f"Loaded {len(self.goals)} goals for {self.robot_name}")

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = f"{self.robot_name}/base_link"
        self.global_frame = "world"

        self.robot_check_timer = self.create_timer(0.2, self.check_other_robots)
        self.timer = self.create_timer(0.1, self.control_loop)

    def load_path(self, path_file):
        with open(path_file, 'r') as f:
            data = yaml.safe_load(f)
        return data['path']

    def get_robot_pose(self, robot_name=None):
        """Belirtilen robotun konumunu al, eğer robot_name None ise kendi konumunu al"""
        if robot_name is None:
            base_frame = self.base_frame
        else:
            base_frame = f"{robot_name}/base_link"
            
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.global_frame, base_frame, now, timeout=rclpy.duration.Duration(seconds=0.5))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            return (x, y, yaw)
        except (LookupException, Exception) as e:
            # Diğer robotları takip ederken hata oluşursa sessizce geç
            if robot_name is not None:
                return None
            # Kendi konumumuz için hata oluşursa uyarı ver
            self.get_logger().warn(f"TF Lookup failed: {e}")
            return None

    def check_other_robots(self):
        """Diğer robotların konumlarını kontrol et ve kaydet"""
        self.other_robot_positions.clear()
        
        for robot in self.other_robots:
            pose = self.get_robot_pose(robot)
            if pose is not None:
                self.other_robot_positions[robot] = pose

    def scan_callback(self, msg: LaserScan):
        self.scan = msg.ranges

    def is_robot_nearby(self, x, y):
        """Başka bir robotun yakında olup olmadığını kontrol et"""
        for robot, pose in self.other_robot_positions.items():
            other_x, other_y, _ = pose
            dist = math.hypot(other_x - x, other_y - y)
            if dist < ROBOT_DETECTION_RADIUS:
                return True, robot, pose
        return False, None, None

    def calculate_avoidance_vector(self, x, y, other_x, other_y):
        """Diğer robottan kaçınma vektörünü hesapla"""
        dx = x - other_x
        dy = y - other_y
        dist = math.hypot(dx, dy)
        
        if dist < 0.01:  # Çok yakınsa rastgele bir yön seç
            return math.cos(math.pi/4), math.sin(math.pi/4)
            
        # Normalize ve güvenli mesafeye göre ölçekle
        strength = max(0, 1.0 - dist/ROBOT_DETECTION_RADIUS)
        return dx/dist * strength, dy/dist * strength

    def control_loop(self):
        pose = self.get_robot_pose()
        if pose is None or self.scan is None:
            return

        x, y, yaw = pose

        # Lookahead kontrolü
        while self.current_goal_index < len(self.goals) - 1:
            gx, gy = self.goals[self.current_goal_index]
            if math.hypot(gx - x, gy - y) < LOOKAHEAD_DIST:
                self.current_goal_index += 1
            else:
                break

        if self.current_goal_index >= len(self.goals):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("All goals reached!")
            return

        gx, gy = self.goals[self.current_goal_index]

        # Robot çarpışma kontrolü
        avoidance_active = False
        avoidance_dx, avoidance_dy = 0.0, 0.0
        
        if self.avoid_robots and self.other_robot_positions:
            robot_nearby, nearby_robot, nearby_pose = self.is_robot_nearby(x, y)
            if robot_nearby:
                other_x, other_y, _ = nearby_pose
                avoidance_dx, avoidance_dy = self.calculate_avoidance_vector(x, y, other_x, other_y)
                
                # Eğer çarpışma kaçınma aktifse, geçici olarak hedefi değiştir
                if math.hypot(avoidance_dx, avoidance_dy) > 0.1:
                    avoidance_active = True
                    self.get_logger().info(f"Avoiding robot {nearby_robot} at distance {math.hypot(other_x - x, other_y - y):.2f}m")
                    # Hedef konumu düzelt
                    gx = x + avoidance_dx * ROBOT_SAFETY_RADIUS * 2
                    gy = y + avoidance_dy * ROBOT_SAFETY_RADIUS * 2

        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(angle_to_goal - yaw), math.cos(angle_to_goal - yaw))

        twist = Twist()
        if avoidance_active or distance > GOAL_THRESH:
            # Çarpışma önleme aktifse veya hedefe uzaksa
            twist.angular.z = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, ANGLE_GAIN * angle_error))
            
            # Eğer kaçınma aktifse veya doğru açıya yakınsa ileri git
            if avoidance_active or abs(angle_error) < math.pi / 4:
                speed = min(MAX_LIN_VEL, DIST_GAIN * distance)
                # Eğer çarpışma önleme aktifse, hızı azalt
                if avoidance_active:
                    speed *= 0.7
                twist.linear.x = speed
        elif not avoidance_active and distance <= GOAL_THRESH:
            self.get_logger().info(f"Goal {self.current_goal_index+1} reached")
            self.current_goal_index += 1

        # Engel kontrolü (opsiyonel)
        if self.avoid_obstacle and self.scan and len(self.scan) >= 360:
            front = self.scan[170:190]
            front_filtered = [r for r in front if 0.1 < r < 3.5]
            if front_filtered and min(front_filtered) < 0.15:
                twist.linear.x = 0.0
                twist.angular.z = 0.4
                self.get_logger().warn("Obstacle detected! Rotating...")

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()