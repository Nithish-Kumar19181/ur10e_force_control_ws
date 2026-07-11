cmake_minimum_required(VERSION 3.8)
project(ur10e_vertical_mixer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# vertical_mixer_ptp: seeded-IK PTP move to a normal standoff in front of the
# first vision waypoint (nearest IK branch, mixer-stationary collision gate)
# ---------------------------------------------------------------------------
add_executable(vertical_mixer_ptp src/vertical_mixer_ptp.cpp)
target_include_directories(vertical_mixer_ptp PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(vertical_mixer_ptp
  rclcpp rclcpp_action geometry_msgs std_msgs sensor_msgs
  controller_manager_msgs control_msgs trajectory_msgs
  moveit_msgs shape_msgs)

install(TARGETS vertical_mixer_ptp
  DESTINATION lib/${PROJECT_NAME})

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
