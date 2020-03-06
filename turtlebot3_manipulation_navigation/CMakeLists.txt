################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_manipulation_navigation)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  moveit_msgs
  trajectory_msgs
  open_manipulator_msgs
  ar_track_alvar_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

################################################################################
# Setup for python modules and scripts
################################################################################
# catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs nav_msgs moveit_msgs trajectory_msgs open_manipulator_msgs ar_track_alvar_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface
)

################################################################################
# Build
################################################################################
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
# catkin_install_python(PROGRAMS
#  nodes/pick_and_place_state_machine
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY maps launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
