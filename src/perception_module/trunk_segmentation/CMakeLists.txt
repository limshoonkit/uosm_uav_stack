cmake_minimum_required(VERSION 3.22.1)
project(trunk_segmentation)

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
find_package(pcl_conversions REQUIRED)

add_library(trunk_segmentation_component SHARED
  src/trunk_seg_viz.cpp
  src/circle_fitting.cpp
  src/clustering_algorithms.cpp
  src/trunk_seg_component.cpp
  src/trunk_tracking.cpp
)

# Target include directories
target_include_directories(trunk_segmentation_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(trunk_segmentation_component
    rclcpp
    rclcpp_components
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    sensor_msgs
    diagnostic_updater
    tf2_geometry_msgs
    tf2
    tf2_ros
    tf2_eigen
    uosm_uav_interface
)

# Link libraries
target_link_libraries(trunk_segmentation_component
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Register the component
rclcpp_components_register_nodes(trunk_segmentation_component "uosm::perception::TrunkSegmentationComponent")

# Install targets
install(TARGETS trunk_segmentation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(trunk_segmentation_component)

ament_package()