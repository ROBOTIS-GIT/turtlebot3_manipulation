################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_bringup)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(imu_sensor_broadcaster REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(position_controllers REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(joint_state_broadcaster)
find_package(controller_manager REQUIRED)
find_package(xacro REQUIRED)
find_package(gripper_controllers REQUIRED)
################################################################################
# Install
################################################################################
install(
  DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
################################################################################
# Macro for ament package
################################################################################
ament_package()
