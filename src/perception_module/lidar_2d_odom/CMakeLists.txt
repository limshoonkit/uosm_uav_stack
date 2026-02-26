cmake_minimum_required(VERSION 3.22.1)
project(lidar_2d_odom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -O3)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(scan_to_pointcloud_component SHARED
  src/scan_to_pointcloud_component.cpp
)
target_include_directories(scan_to_pointcloud_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scan_to_pointcloud_component
  rclcpp rclcpp_components sensor_msgs
)
rclcpp_components_register_nodes(scan_to_pointcloud_component "uosm::perception::ScanToPointCloudComponent")

add_library(tilt_gated_odom_component SHARED
  src/tilt_gated_odom_component.cpp
)
target_include_directories(tilt_gated_odom_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tilt_gated_odom_component
  rclcpp rclcpp_components nav_msgs geometry_msgs
)
rclcpp_components_register_nodes(tilt_gated_odom_component "uosm::perception::TiltGatedOdomComponent")

install(TARGETS scan_to_pointcloud_component tilt_gated_odom_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(scan_to_pointcloud_component tilt_gated_odom_component)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs)
ament_package()
