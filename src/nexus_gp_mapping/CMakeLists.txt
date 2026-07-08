cmake_minimum_required(VERSION 3.5)
project(nexus_gp_mapping)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/gp_mapping_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gp_mapping_node
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
