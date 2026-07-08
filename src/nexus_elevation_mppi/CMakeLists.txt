cmake_minimum_required(VERSION 3.8)
project(nexus_elevation_mppi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mppi_navigator src/mppi_navigator.cpp)
target_compile_features(mppi_navigator PUBLIC cxx_std_17)
ament_target_dependencies(mppi_navigator
  geometry_msgs
  grid_map_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS
  mppi_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/elevation_map_exporter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME elevation_map_exporter
)
install(PROGRAMS
  scripts/traversability_to_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME traversability_to_map
)
install(PROGRAMS
  scripts/novelty_explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME novelty_explorer
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
