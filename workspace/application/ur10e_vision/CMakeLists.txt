cmake_minimum_required(VERSION 3.8)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(ur10e_vision)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

# ---------------------------------------------------------------------------
# Node A: cloud_processor (C++ / PCL)
# ---------------------------------------------------------------------------
add_executable(cloud_processor src/cloud_processor.cpp)
target_include_directories(cloud_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_processor ${PCL_LIBRARIES})
ament_target_dependencies(cloud_processor
  rclcpp sensor_msgs geometry_msgs std_srvs
  tf2 tf2_ros tf2_eigen tf2_sensor_msgs
  pcl_ros pcl_conversions)

install(TARGETS cloud_processor
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Node B: path_generator (Python / Open3D)
# ---------------------------------------------------------------------------
install(PROGRAMS
  scripts/path_generator.py
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Shared resources
# ---------------------------------------------------------------------------
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
