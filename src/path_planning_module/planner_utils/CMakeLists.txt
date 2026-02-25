cmake_minimum_required(VERSION 3.22.1)
project(planner_utils)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -O3 -g -Wno-deprecated-declarations -Wno-maybe-uninitialized -Wno-pedantic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if(CMAKE_SYSTEM_NAME2 MATCHES "aarch64") # Jetson
  set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

# Find dependencies first
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(grid_map REQUIRED) 
find_package(uosm_uav_interface REQUIRED)
  
ament_auto_add_library(planner_utils_lib SHARED
  src/plan_visualizer.cpp
  src/poly_traj_optimizer.cpp
  src/dyn_a_star.cpp
)

# Additional libraries not handled by ament_auto
target_link_libraries(planner_utils_lib
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_include_directories(planner_utils_lib
  PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


# Install
ament_auto_package()