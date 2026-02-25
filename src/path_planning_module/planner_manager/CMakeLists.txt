cmake_minimum_required(VERSION 3.22.1)
project(planner_manager)

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

find_package(uosm_uav_interface REQUIRED)

# Add your component library
add_library(planner_manager_component SHARED
  src/flight_controller_component.cpp
  src/ego_planner_component.cpp
)

# Target include directories
target_include_directories(planner_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${planner_utils_INCLUDE_DIRS}
  ${grid_map_INCLUDE_DIRS}
)

ament_target_dependencies(planner_manager_component
    rclcpp
    rclcpp_components
    std_msgs
    nav_msgs
    geometry_msgs
    mavros_msgs
    nmpc_controller
    diagnostic_updater
    tf2
    tf2_ros
    tf2_geometry_msgs
    planner_utils
    grid_map
    uosm_uav_interface
)

# Register the component
rclcpp_components_register_nodes(planner_manager_component "uosm::path_planning::FlightController")
rclcpp_components_register_nodes(planner_manager_component "uosm::path_planning::EgoPlanner")

# Install targets
install(TARGETS planner_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(planner_manager_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  planner_utils
  grid_map
)

ament_package()