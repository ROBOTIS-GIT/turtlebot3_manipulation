cmake_minimum_required(VERSION 3.5)
project(turtlebot3_lime_teleop)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlebot3_lime_bringup REQUIRED)
find_package(turtlebot3_lime_moveit_config REQUIRED)
find_package(turtlebot3_lime_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(turtlebot3_lime_teleop src/turtlebot3_lime_teleop.cpp)
target_include_directories(turtlebot3_lime_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  turtlebot3_lime_teleop
  "rclcpp"
  "turtlebot3_lime_bringup"
  "turtlebot3_lime_moveit_config"
  "turtlebot3_lime_description"
  "geometry_msgs"
  "nav_msgs"
  "control_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_action"
)

install(TARGETS turtlebot3_lime_teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
