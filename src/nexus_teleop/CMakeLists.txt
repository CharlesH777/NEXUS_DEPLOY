cmake_minimum_required(VERSION 3.5)
project(nexus_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/teleop_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_gui
)
install(PROGRAMS
  scripts/pose_to_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_to_tf_bridge
)

ament_package()
