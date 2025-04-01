################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_teleop)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlebot3_manipulation_bringup REQUIRED)
find_package(turtlebot3_manipulation_moveit_config REQUIRED)
find_package(turtlebot3_manipulation_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(turtlebot3_manipulation_teleop src/turtlebot3_manipulation_teleop.cpp)
target_include_directories(turtlebot3_manipulation_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  turtlebot3_manipulation_teleop
  "rclcpp"
  "turtlebot3_manipulation_bringup"
  "turtlebot3_manipulation_moveit_config"
  "turtlebot3_manipulation_description"
  "geometry_msgs"
  "nav_msgs"
  "control_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_action"
)

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_manipulation_teleop
  DESTINATION lib/${PROJECT_NAME})

################################################################################
# Macro for ament package
################################################################################
ament_package()
