/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#ifndef TURTLEBOT3_MANIPULATION_TELEOP_JOYSTICK_HPP_
#define TURTLEBOT3_MANIPULATION_TELEOP_JOYSTICK_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#define PI 3.14159265359
#define NUM_OF_JOINT 4


namespace turtlebot3_manipulation_teleop_joystick
{
class TurtleBot3ManipulationTeleopJoystick : public rclcpp::Node
{
 public:
  TurtleBot3ManipulationTeleopJoystick();
  virtual ~TurtleBot3ManipulationTeleopJoystick();

 private:
  /*****************************************************************************
  ** Position in joint space and task space
  *****************************************************************************/
  geometry_msgs::msg::Twist present_base_velocity_;
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;

  /*****************************************************************************
  ** ROS subscribers, callback functions and relevant functions
  *****************************************************************************/
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr kinematics_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_command_sub_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  bool set_base_velocity(geometry_msgs::msg::Twist base_velocity);
  bool set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time);
  bool set_tool_control(std::vector<double> joint_angle);

  /*****************************************************************************
  ** ROS clients
  *****************************************************************************/
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_client_;

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  void set_goal(const char *str);
  geometry_msgs::msg::Twist get_present_base_velocity();

};
}  // namespace turtlebot3_manipulation_teleop_joystick
#endif  // TURTLEBOT3_MANIPULATION_TELEOP_JOYSTICK_HPP_
