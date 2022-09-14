cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_moveit_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)
find_package(turtlebot3_manipulation_description REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
