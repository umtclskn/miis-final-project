#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <vector>
#include <memory>

constexpr double WORLD_X_MIN = -2.0;
constexpr double WORLD_X_MAX = 2.0;
constexpr double WORLD_Y_MIN = -1.0;
constexpr double WORLD_Y_MAX = 1.0;
constexpr int GRID_COLS = 16;
constexpr int GRID_ROWS = 10;
constexpr double CELL_WIDTH = (WORLD_X_MAX - WORLD_X_MIN) / GRID_COLS;
constexpr double CELL_HEIGHT = (WORLD_Y_MAX - WORLD_Y_MIN) / GRID_ROWS;
constexpr double FOV_RADIUS = 0.75;

struct GridCell {
    int i, j;
    double cx, cy;
};

class GridCellsInFOV : public rclcpp::Node {
public:
    GridCellsInFOV()
        : Node("grid_cells_in_fov"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("grid_cells_in_fov_marker", 1);
        border_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("room_border_marker", 1);
        grid_cells_ = generate_grid_cells();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&GridCellsInFOV::timer_callback, this));
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr border_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<GridCell> grid_cells_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<GridCell> generate_grid_cells() {
        std::vector<GridCell> cells;
        for (int i = 0; i < GRID_COLS; ++i) {
            for (int j = 0; j < GRID_ROWS; ++j) {
                double cx = WORLD_X_MIN + (i + 0.5) * CELL_WIDTH;
                double cy = WORLD_Y_MIN + (j + 0.5) * CELL_HEIGHT;
                cells.push_back({i, j, cx, cy});
            }
        }
        return cells;
    }

    void timer_callback() {
        double robot_x = 0.0;
        double robot_y = 0.0;
        try {
            auto tf = tf_buffer_.lookupTransform("world", "main_robot/base_link", tf2::TimePointZero);
            robot_x = tf.transform.translation.x;
            robot_y = tf.transform.translation.y;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
            return;
        }

        std::vector<GridCell> fov_cells;
        for (const auto& cell : grid_cells_) {
            double dx = cell.cx - robot_x;
            double dy = cell.cy - robot_y;
            double dist = std::hypot(dx, dy);
            if (dist > FOV_RADIUS) continue;
            if (cell.cx < WORLD_X_MIN || cell.cx > WORLD_X_MAX) continue;
            if (cell.cy < WORLD_Y_MIN || cell.cy > WORLD_Y_MAX) continue;
            fov_cells.push_back(cell);
        }

        // Marker for grid cells in FOV
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "grid_cells_in_fov";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = CELL_WIDTH * 0.9;
        marker.scale.y = CELL_HEIGHT * 0.9;
        marker.scale.z = 0.03;
        marker.color.r = 0.2;
        marker.color.g = 0.6;
        marker.color.b = 1.0;
        marker.color.a = 0.7;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        for (const auto& cell : fov_cells) {
            geometry_msgs::msg::Point pt;
            pt.x = cell.cx;
            pt.y = cell.cy;
            pt.z = 0.015;
            marker.points.push_back(pt);
        }
        marker_pub_->publish(marker);

        // Marker for room border
        visualization_msgs::msg::Marker border;
        border.header.frame_id = "world";
        border.header.stamp = this->now();
        border.ns = "room_border";
        border.id = 1;
        border.type = visualization_msgs::msg::Marker::LINE_STRIP;
        border.action = visualization_msgs::msg::Marker::ADD;
        border.scale.x = 0.04;
        border.color.r = 1.0;
        border.color.g = 0.6;
        border.color.b = 0.0;
        border.color.a = 1.0;
        border.pose.orientation.w = 1.0;

        std::vector<std::pair<double, double>> corners = {
            {WORLD_X_MIN, WORLD_Y_MIN},
            {WORLD_X_MAX, WORLD_Y_MIN},
            {WORLD_X_MAX, WORLD_Y_MAX},
            {WORLD_X_MIN, WORLD_Y_MAX},
            {WORLD_X_MIN, WORLD_Y_MIN}
        };
        for (const auto& [x, y] : corners) {
            geometry_msgs::msg::Point pt;
            pt.x = x;
            pt.y = y;
            pt.z = 0.02;
            border.points.push_back(pt);
        }
        border_pub_->publish(border);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridCellsInFOV>());
    rclcpp::shutdown();
    return 0;
}
