cmake_minimum_required(VERSION 3.8)
project(automower_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

get_filename_component(AUTOMOWER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
# Reuse the root-level prototype logic inside the ROS 2 package.
add_library(
  drive_controller
  ${AUTOMOWER_ROOT}/src/DriveController.cpp
  ${AUTOMOWER_ROOT}/src/SimulationDriver.cpp
)

target_include_directories(
  drive_controller
  PUBLIC
  ${AUTOMOWER_ROOT}/include
)

add_executable(
  automower_drive_node
  src/automower_drive_node.cpp
)

add_executable(
  automower_wasd_teleop
  src/automower_wasd_teleop.cpp
)

add_executable(
  automower_joystick_teleop
  src/automower_joystick_teleop.cpp
)

add_executable(
  automower_tool_controller
  src/automower_tool_controller.cpp
)

target_include_directories(
  automower_drive_node
  PRIVATE
  ${AUTOMOWER_ROOT}/include
)

target_include_directories(
  automower_wasd_teleop
  PRIVATE
  ${AUTOMOWER_ROOT}/include
)

target_link_libraries(
  automower_drive_node
  drive_controller
)

ament_target_dependencies(
  automower_drive_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(
  automower_wasd_teleop
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(
  automower_joystick_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
)

ament_target_dependencies(
  automower_tool_controller
  rclcpp
  geometry_msgs
  std_msgs
)

install(
  TARGETS
    automower_drive_node
    automower_wasd_teleop
    automower_joystick_teleop
    automower_tool_controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
