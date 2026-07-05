cmake_minimum_required(VERSION 3.20)

project(floam_core
  VERSION 1.0.0
  DESCRIPTION "FLOAM Core C++ Library"
  LANGUAGES CXX)

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)

# Define the static library target
add_library(${PROJECT_NAME} STATIC
  src/lidar_processing.cpp
  src/lidar_optimization.cpp
  src/odom_estimation.cpp
  src/lidar_mapping.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Ceres::ceres
    Eigen3::Eigen
    pcl_common
  PRIVATE
    pcl_filters
    pcl_kdtree)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)  # Require C++17 for consumers

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -O3)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib)

# Install header files
install(DIRECTORY include/
  DESTINATION include)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL Eigen3 Ceres)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
