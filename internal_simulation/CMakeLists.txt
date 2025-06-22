cmake_minimum_required(VERSION 3.8)
project(internal_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directory (header files)
include_directories(include)

add_library(internal_simulator STATIC
  src/internal_simulator.cpp
  src/potential_field.cpp     # <-- bunu ekle
)
target_include_directories(internal_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(internal_simulator PUBLIC cxx_std_17)
ament_target_dependencies(
  internal_simulator
  "rclcpp"
)


# Main node executable
add_executable(ego_robot_decision_node
  src/ego_robot_decision_node.cpp
)
target_include_directories(ego_robot_decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ego_robot_decision_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  ego_robot_decision_node
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs"
  "tf2_ros"
)

# Link node to internal simulator library
target_link_libraries(ego_robot_decision_node internal_simulator)

# Installation rules
install(TARGETS ego_robot_decision_node internal_simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
