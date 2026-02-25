cmake_minimum_required(VERSION 3.22.1)
project(map_alignment)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -O3 -g -Wno-deprecated-declarations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find additional required packages
find_package(uosm_uav_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)

add_library(map_alignment_component SHARED
  src/map_alignment_component.cpp
)

# Target include directories
target_include_directories(map_alignment_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(map_alignment_component
    rclcpp
    rclcpp_components
    composition_interfaces
    std_msgs
    std_srvs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    pcl_ros
    uosm_uav_interface
)

# Link libraries
target_link_libraries(map_alignment_component
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Register the component
rclcpp_components_register_nodes(map_alignment_component "uosm::perception::MapAlignmentComponent")

# Install targets
install(TARGETS map_alignment_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install share files
install(DIRECTORY
  config
  maps
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts for testing
install(PROGRAMS
  scripts/mock_trunk_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(map_alignment_component)

ament_package()