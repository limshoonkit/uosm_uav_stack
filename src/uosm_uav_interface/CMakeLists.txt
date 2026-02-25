cmake_minimum_required(VERSION 3.22.1)
project(uosm_uav_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/EgoPlanner.action"
)

set(srv_files
)

set(msg_files
  "msg/MincoTraj.msg"
  "msg/PolynomialTraj.msg"
  "msg/TrunkObservation.msg"
  "msg/TrunkObservationArray.msg"
  "msg/WifiInfoStamped.msg"
)

# Define messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs rclcpp builtin_interfaces
)

# Create interface library for common headers
add_library(common_headers INTERFACE)
target_include_directories(common_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the common headers
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.cuh"
)

# Install the interface library
install(TARGETS common_headers
  EXPORT common_headers_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the interface library for other packages to use
install(EXPORT common_headers_targets
  FILE common_headers_targets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export include directories for downstream packages
ament_export_include_directories(include)
ament_export_targets(common_headers_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()