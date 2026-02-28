cmake_minimum_required(VERSION 3.22.1)
project(jetson_gscam2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(uosm_uav_interface REQUIRED)
find_package(OpenCV REQUIRED)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

# Include directories
include_directories(
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
)

# Create component library
add_library(csi_camera_component SHARED
  src/csi_camera_component.cpp
)

# Set component properties
target_compile_definitions(csi_camera_component PRIVATE "CSI_CAMERA_BUILDING_LIBRARY")

# Dependencies
ament_target_dependencies(csi_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  foxglove_msgs
  image_transport
  cv_bridge
  std_msgs
  camera_info_manager
  uosm_uav_interface
  OpenCV
)

# Link GStreamer libraries
target_link_libraries(csi_camera_component
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GST_VIDEO_LIBRARIES}
)

# Register component
rclcpp_components_register_nodes(csi_camera_component "uosm::perception::CSICameraComponent")

# Install
install(TARGETS
  csi_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()