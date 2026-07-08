cmake_minimum_required(VERSION 3.8)
project(gen_local_goal)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(gen_local_goal_node
  src/gen_local_goal_node.cpp
  src/gen_local_goal.cpp
)

ament_target_dependencies(gen_local_goal_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(TARGETS gen_local_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
