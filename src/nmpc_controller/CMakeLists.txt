cmake_minimum_required(VERSION 3.22.1)
project(nmpc_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -O3)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(acados_vendor_ros2 REQUIRED)
find_package(Eigen3 REQUIRED)

# Collect all generated C sources (exclude standalone main files)
file(GLOB ACADOS_GENERATED_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/quadrotor_nmpc_model/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/c_generated_code/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/c_generated_code/quadrotor_nmpc_model/*.c"
)
list(FILTER ACADOS_GENERATED_SOURCES EXCLUDE REGEX "main_.*\\.c$")

# Build the nmpc_controller shared library
add_library(nmpc_controller SHARED
  src/nmpc_controller.cpp
  ${ACADOS_GENERATED_SOURCES}
)

target_include_directories(nmpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados_generated>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/c_generated_code>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nmpc_controller PUBLIC
  acados_vendor_ros2
)

target_link_libraries(nmpc_controller PUBLIC
  Eigen3::Eigen
)

# Install library
install(TARGETS nmpc_controller
  EXPORT nmpc_controllerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install generated headers (for downstream packages)
install(
  DIRECTORY acados_generated/
  DESTINATION include/acados_generated
  FILES_MATCHING PATTERN "*.h"
)

# Export
ament_export_include_directories(include)
ament_export_libraries(nmpc_controller)
ament_export_targets(nmpc_controllerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(acados_vendor_ros2 Eigen3)

ament_package()
