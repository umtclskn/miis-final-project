#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "internal_simulation/internal_simulator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <limits>
#include <array>

// --- Sabitler ---
constexpr double WORLD_X_MIN = -2.0;
constexpr double WORLD_X_MAX = 2.0;
constexpr double WORLD_Y_MIN = -1.0;
constexpr double WORLD_Y_MAX = 1.0;
constexpr int GRID_COLS = 16;
constexpr int GRID_ROWS = 10;
constexpr double CELL_WIDTH = (WORLD_X_MAX - WORLD_X_MIN) / GRID_COLS;
constexpr double CELL_HEIGHT = (WORLD_Y_MAX - WORLD_Y_MIN) / GRID_ROWS;
constexpr double GOAL_X = 1.8;
constexpr double GOAL_Y = 0.0;

class EgoRobotDecisionNode : public rclcpp::Node {
public:
    EgoRobotDecisionNode()
        : Node("ego_robot_decision_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          internal_simulator_(std::make_shared<InternalSimulator>())
    {
        // FOV marker subscription
        fov_marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "grid_cells_in_fov_marker", 1,
            std::bind(&EgoRobotDecisionNode::fov_callback, this, std::placeholders::_1));

        // Robot isimleri
        robot_names_ = {"main_robot", "robot1", "robot2", "robot3"};

        // Robot state update timer
        state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&EgoRobotDecisionNode::update_robot_states, this));

        // Karar/prediction timer
        decision_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&EgoRobotDecisionNode::decision_callback, this));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("predicted_paths", 10);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/main_robot/cmd_vel", 10);

    }

private:
    // ROS üyeleri
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr fov_marker_sub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr decision_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Robot listesi, state’leri ve grid cell’leri
    std::vector<std::string> robot_names_;
    std::map<std::string, RobotState> robot_states_;
    std::vector<GridCell> latest_fov_cells_;
    std::shared_ptr<InternalSimulator> internal_simulator_;

    // --- Callback: FOV Marker’dan grid cell toplama ---
    void fov_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
        latest_fov_cells_.clear();
        for (const auto& pt : msg->points) {
            GridCell cell;
            cell.cx = pt.x;
            cell.cy = pt.y;
            latest_fov_cells_.push_back(cell);
        }
        RCLCPP_DEBUG(this->get_logger(), "FOV'daki grid cell sayısı: %zu", latest_fov_cells_.size());
    }

    // --- Yardımcı: Robot state güncelleme (TF ile) ---
    void update_robot_states() {
        for (const auto& robot_name : robot_names_) {
            try {
                auto tf = tf_buffer_.lookupTransform("world", robot_name + "/base_link", tf2::TimePointZero);
                double x = tf.transform.translation.x;
                double y = tf.transform.translation.y;
                auto q = tf.transform.rotation;
                double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                double yaw = std::atan2(siny_cosp, cosy_cosp);
                robot_states_[robot_name] = {x, y, yaw};
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "TF Exception for %s: %s", robot_name.c_str(), ex.what());
            }
        }
    }

    // --- Yardımcı: Basit differential drive trajectory ---
    std::vector<geometry_msgs::msg::Point> predict_trajectory(
        double x0, double y0, double theta0,
        double v, double w, int steps, double dt)
    {
        std::vector<geometry_msgs::msg::Point> traj;
        double x = x0, y = y0, theta = theta0;
        for (int i = 0; i < steps; ++i) {
            x += v * dt * std::cos(theta);
            y += v * dt * std::sin(theta);
            theta += w * dt;
            geometry_msgs::msg::Point pt;
            pt.x = x; pt.y = y; pt.z = 0.01;
            traj.push_back(pt);
        }
        return traj;
    }

    // --- Ana karar fonksiyonu ---
    void decision_callback() {
        // Diğer robotlar için future trajectory/prediction
        std::vector<PredictedObstacle> predicted_obstacle_list;
        int marker_id = 0;
        for (const auto& [robot_name, state] : robot_states_) {
            if (robot_name == "main_robot") continue;

            double v = 0.22;
            double w = 0.0;
            auto traj = predict_trajectory(state.x, state.y, state.theta, v, w, 100, 0.05);

            PredictedObstacle obs;
            obs.name = robot_name;
            obs.trajectory.reserve(traj.size());
            for (const auto& pt : traj) {
                obs.trajectory.push_back({pt.x, pt.y});
            }
            predicted_obstacle_list.push_back(obs);

            // Her robot için prediction path marker
            publish_predicted_path_marker(traj, marker_id++);
        }

        // Main robot için: grid cell skorlama
        GridCell best_cell;
        double best_score = std::numeric_limits<double>::lowest();

        // Ana robotun güncel durumu:
        auto it = robot_states_.find("main_robot");
        if (it == robot_states_.end()) {
            RCLCPP_WARN(this->get_logger(), "Main robot state bulunamadı!");
            return;
        }
        const RobotState& main_robot_state = it->second;

        // Goal: sabit hedef
        std::array<double,2> goal = {GOAL_X, GOAL_Y};

        for (const auto& cell : latest_fov_cells_) {
            double score = internal_simulator_->run(
                main_robot_state,  // ego robot state
                cell,              // intermediate target
                predicted_obstacle_list, // predicted obstacles
                goal               // ana hedef!
            );
            if (score > best_score) {
                best_score = score;
                best_cell = cell;
            }
        }
        publish_best_cell_marker(best_cell);
        RCLCPP_INFO(this->get_logger(), "Best target grid: (%.2f, %.2f) with score %.2f", best_cell.cx, best_cell.cy, best_score);
    }

    // --- Marker yayını (diğer robot path) ---
    void publish_predicted_path_marker(const std::vector<geometry_msgs::msg::Point>& traj, int marker_id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "predicted_path";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;
        // Renk seçimi
        if (marker_id == 0) { marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; }
        else if (marker_id == 1) { marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; }
        else { marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; }
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker.points = traj;
        marker_pub_->publish(marker);
    }

    // --- Marker yayını (best cell) ---
    void publish_best_cell_marker(const GridCell& best_cell) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "best_target_cell";
        marker.id = 999;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = best_cell.cx;
        marker.pose.position.y = best_cell.cy;
        marker.pose.position.z = 0.04;
        marker.scale.x = CELL_WIDTH * 0.9;
        marker.scale.y = CELL_HEIGHT * 0.9;
        marker.scale.z = 0.07;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.95;
        marker.lifetime = rclcpp::Duration::from_seconds(0.6);
        marker.pose.orientation.w = 1.0;
        marker_pub_->publish(marker);


        const RobotState& main_robot_state = robot_states_["main_robot"];

        // Grid hücresinin merkezi hedef
        double dx = best_cell.cx - main_robot_state.x;
        double dy = best_cell.cy - main_robot_state.y;
        double yaw_to_target = std::atan2(dy, dx);
        double yaw_robot = main_robot_state.theta;
        double yaw_error = yaw_to_target - yaw_robot;
        // Normalize angle
        while (yaw_error > M_PI) yaw_error -= 2*M_PI;
        while (yaw_error < -M_PI) yaw_error += 2*M_PI;

        double distance = std::hypot(dx, dy);

        // Basit P kontrol
        double k_lin = 0.2; // lineer hız kazancı (ayarla!)
        double k_ang = 1.2; // açısal hız kazancı (ayarla!)
        double v = k_lin * distance;
        double w = k_ang * yaw_error;

        if (v > 0.22) v = 0.22; // Turtlebot için max
        if (distance < 0.03) v = 0.0; // Çok yakınsa dur

        // Mesaj oluştur ve yay
        geometry_msgs::msg::Twist twist;
        twist.linear.x = v;
        twist.angular.z = w;
        cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EgoRobotDecisionNode>());
    rclcpp::shutdown();
    return 0;
}
