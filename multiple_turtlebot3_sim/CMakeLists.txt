cmake_minimum_required(VERSION 3.8)
project(multiple_turtlebot3_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_image REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)  # PoseTFPublisher icin lazim

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# dependencies list
set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "std_msgs"
)

# build PoseTFPublisher library
add_library(multi_robot_library src/PoseTFPublisher.cpp)
ament_target_dependencies(multi_robot_library ${dependencies})

# build main executable
add_executable(multi_robot_simulation_main src/MultiRobotSimulationMain.cpp)
target_link_libraries(multi_robot_simulation_main multi_robot_library)
ament_target_dependencies(multi_robot_simulation_main ${dependencies})

add_executable(tf_republisher src/tf_republisher.cpp)
ament_target_dependencies(tf_republisher rclcpp tf2_msgs)
install(TARGETS tf_republisher DESTINATION lib/${PROJECT_NAME})

# install everything
install(TARGETS
  multi_robot_simulation_main
  multi_robot_library
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
