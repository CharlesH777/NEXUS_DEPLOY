cmake_minimum_required(VERSION 3.8)
project(lrae_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(visual src/visiual.cpp)
ament_target_dependencies(visual
  geometry_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

add_library(utils src/utils.cpp)
ament_target_dependencies(utils
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_library(path_planning src/path_planning.cpp src/graph_search.cpp src/heap.cpp)
ament_target_dependencies(path_planning
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)
target_link_libraries(path_planning ${PCL_LIBRARIES})

add_library(exploration_planning src/exploration_planning.cpp)
ament_target_dependencies(exploration_planning
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(exploration_planning utils visual path_planning ${PCL_LIBRARIES})

add_executable(lrae_planner_node src/lrae_planner_node.cpp)
target_link_libraries(lrae_planner_node exploration_planning)
ament_target_dependencies(lrae_planner_node rclcpp)

add_executable(exploration_map_merge src/exploration_map_merge.cpp)
ament_target_dependencies(exploration_map_merge
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  visual
  utils
  path_planning
  exploration_planning
  lrae_planner_node
  exploration_map_merge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/realTimePlot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
