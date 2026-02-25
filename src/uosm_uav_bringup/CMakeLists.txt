cmake_minimum_required(VERSION 3.22.1)
project(uosm_uav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install python scripts
install(PROGRAMS
  scripts/reboot_px4_node.py
  scripts/odom_to_tf_node.py
  scripts/waypoint_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()