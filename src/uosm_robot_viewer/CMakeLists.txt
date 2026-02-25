cmake_minimum_required(VERSION 3.22.1)
project(uosm_robot_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY
  launch
  urdf
  rviz
  DESTINATION
  share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
  scripts/odom_to_tf_publisher.py
  DESTINATION
  lib/${PROJECT_NAME}
)

ament_package()