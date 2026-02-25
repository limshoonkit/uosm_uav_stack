cmake_minimum_required(VERSION 3.22.1)
project(odom_republisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -O3 -g -Wno-deprecated-declarations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

add_library(odom_republisher_component SHARED
  src/odom_republisher_component.cpp
)

target_include_directories(odom_republisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(odom_republisher_component
    rclcpp
    rclcpp_components
    std_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    uosm_uav_interface
)

target_link_libraries(odom_republisher_component
  ${EIGEN3_LIBRARIES}
)

target_compile_definitions(odom_republisher_component PRIVATE
  EIGEN_NO_DEBUG
  EIGEN_MPL2_ONLY
)

rclcpp_components_register_nodes(odom_republisher_component "uosm::perception::OdomRepublisher")

install(TARGETS odom_republisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(odom_republisher_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
)

ament_package()
