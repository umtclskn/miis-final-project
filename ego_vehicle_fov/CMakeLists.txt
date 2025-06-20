cmake_minimum_required(VERSION 3.8)
project(ego_vehicle_fov)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(grid_cells_in_fov src/grid_cells_in_fov.cpp)

ament_target_dependencies(grid_cells_in_fov
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

install(TARGETS grid_cells_in_fov
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
