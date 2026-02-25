cmake_minimum_required(VERSION 3.22.1)
project(grid_map)

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

option(USE_CUDA "Enable CUDA acceleration" OFF)

if(USE_CUDA)
  message(STATUS "CUDA acceleration enabled")
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  
  ament_auto_add_library(grid_map_lib SHARED
    src/grid_map.cpp
    src/grid_map_cuda.cu
  )
  
  target_compile_options(grid_map_lib
    PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
  )

  # CUDA properties
  set_target_properties(grid_map_lib PROPERTIES
    CUDA_ARCHITECTURES "86;90"
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_HOST_COMPILATION_CPP ON
  )
  
  # Additional libraries not handled by ament_auto
  target_link_libraries(grid_map_lib
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    CUDA::cudart
  )
  
  target_include_directories(grid_map_lib
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  
  target_compile_definitions(grid_map_lib PRIVATE USE_CUDA)
  
else()
  message(STATUS "CUDA acceleration disabled")
  
  ament_auto_add_library(grid_map_lib SHARED
    src/grid_map.cpp
    src/raycast.cpp
  )
  
  # Additional libraries not handled by ament_auto
  target_link_libraries(grid_map_lib
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  
  target_include_directories(grid_map_lib
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
endif()

# Install
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)