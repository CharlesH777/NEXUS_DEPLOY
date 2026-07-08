cmake_minimum_required(VERSION 3.8)
project(sensor_conversion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(slam_output
  src/slam_output.cpp
)

ament_target_dependencies(slam_output
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

add_executable(slam_sim_output_node src/slam_output_node.cpp)
target_link_libraries(slam_sim_output_node slam_output)
ament_target_dependencies(slam_sim_output_node rclcpp)

add_executable(sensor_odom_to_world_node src/sensor_odom_to_world.cpp)
ament_target_dependencies(sensor_odom_to_world_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  slam_output
  slam_sim_output_node
  sensor_odom_to_world_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/map_odom_bridge.py
  scripts/base_footprint_link_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
