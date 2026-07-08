cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(localPlanner src/localPlanner.cpp)
ament_target_dependencies(localPlanner
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)
target_link_libraries(localPlanner ${PCL_LIBRARIES})

add_executable(pathFollower src/pathFollower.cpp)
ament_target_dependencies(pathFollower
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)
target_link_libraries(pathFollower ${PCL_LIBRARIES})

install(TARGETS localPlanner pathFollower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY paths/
  DESTINATION share/${PROJECT_NAME}/paths
)

ament_package()
