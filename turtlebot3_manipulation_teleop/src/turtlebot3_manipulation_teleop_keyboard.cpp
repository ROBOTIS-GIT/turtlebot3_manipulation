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

#include "turtlebot3_manipulation_teleop/turtlebot3_manipulation_teleop_keyboard.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;


namespace turtlebot3_manipulation_teleop_keyboard
{
TurtleBot3ManipulationTeleopKeyboard::TurtleBot3ManipulationTeleopKeyboard()
: Node("turtlebot3_manipulation_teleop_keyboard")
{
  /********************************************************************************
  ** Initialise joint angle and kinematic position size 
  ********************************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /********************************************************************************
  ** Initialise ROS publishers, subscribers and clients
  ********************************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&TurtleBot3ManipulationTeleopKeyboard::odom_callback, this, _1));
  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "open_manipulator_x/joint_states", qos, std::bind(&TurtleBot3ManipulationTeleopKeyboard::joint_states_callback, this, _1));
  kinematics_pose_sub_ = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "open_manipulator_x/kinematics_pose", qos, std::bind(&TurtleBot3ManipulationTeleopKeyboard::kinematics_pose_callback, this, _1));

  // Initialise clients
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_x/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_x/goal_tool_control");
  goal_task_space_path_from_present_position_only_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("open_manipulator_x/goal_task_space_path_from_present_position_only");
  goal_joint_space_path_from_present_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_x/goal_joint_space_path_from_present");

  /********************************************************************************
  ** Display in terminal
  ********************************************************************************/
  this->disable_waiting_for_enter();
  timer_ = this->create_wall_timer(10ms, std::bind(&TurtleBot3ManipulationTeleopKeyboard::display_callback, this));

  RCLCPP_INFO(this->get_logger(), "TurtleBot3 manipulation teleop keyboard node has been initialised.");
}

TurtleBot3ManipulationTeleopKeyboard::~TurtleBot3ManipulationTeleopKeyboard() 
{
  this->restore_terminal_settings();
  RCLCPP_INFO(this->get_logger(), "TurtleBot3 manipulation teleop keyboard node has been initialised.");
}

/********************************************************************************
** Callback functions and relevant functions
********************************************************************************/
void TurtleBot3ManipulationTeleopKeyboard::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  present_base_velocity_ = msg->twist.twist;
}

void TurtleBot3ManipulationTeleopKeyboard::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (uint8_t i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1")) temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2")) temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3")) temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4")) temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void TurtleBot3ManipulationTeleopKeyboard::kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void TurtleBot3ManipulationTeleopKeyboard::set_goal(char ch)
{
  geometry_msgs::msg::Twist goal_base_velocity;
  std::vector<double> goal_pose; goal_pose.resize(3);
  std::vector<double> goal_joint; goal_joint.resize(4);
  const double delta = 0.01;
  const double joint_delta = 0.05;
  double path_time = 0.5;

  if (ch == '8')
  {
    // base linear velocity +
    goal_base_velocity = get_present_base_velocity();
    goal_base_velocity.linear.x += 0.05;
    set_base_velocity(goal_base_velocity);
  }
  else if (ch == '2')
  {
    // base linear velocity -
    goal_base_velocity = get_present_base_velocity();
    goal_base_velocity.linear.x -= 0.05;
    set_base_velocity(goal_base_velocity);
  }
  if (ch == '4')
  {
    // base angular velocity +
    goal_base_velocity = get_present_base_velocity();
    goal_base_velocity.angular.z += 0.05;
    set_base_velocity(goal_base_velocity);
  }
  else if (ch == '6')
  {
    // base angular velocity -
    goal_base_velocity = get_present_base_velocity();
    goal_base_velocity.angular.z -= 0.05;
    set_base_velocity(goal_base_velocity);
  }
  else if (ch == '5')
  {
    // Stop base
    set_base_velocity(goal_base_velocity);
  }
  else if (ch == 'w' || ch == 'W')
  {
    // printf("input : w \tincrease(++) x axis in task space\n");
    goal_pose.at(0) = delta;
    set_task_space_path_from_present_position_only(goal_pose, path_time);
  }
  else if (ch == 's' || ch == 'S')
  {
    // printf("input : s \tdecrease(--) x axis in task space\n");
    goal_pose.at(0) = -delta;
    set_task_space_path_from_present_position_only(goal_pose, path_time);
  }
  else if (ch == 'a' || ch == 'A')
  {
    // printf("input : a \tincrease(++) y axis in task space\n");
    goal_pose.at(1) = delta;
    set_task_space_path_from_present_position_only(goal_pose, path_time);
  }
  else if (ch == 'd' || ch == 'D')
  {
    // printf("input : d \tdecrease(--) y axis in task space\n");
    goal_pose.at(1) = -delta;
    set_task_space_path_from_present_position_only(goal_pose, path_time);
  }
  else if (ch == 'z' || ch == 'Z')
  {
    // printf("input : z \tincrease(++) z axis in task space\n");
    goal_pose.at(2) = delta;
    set_task_space_path_from_present_position_only(goal_pose, path_time);
  }
  else if (ch == 'x' || ch == 'X')
  {
    // printf("input : x \tdecrease(--) z axis in task space\n");
    goal_pose.at(2) = -delta;
    set_task_space_path_from_present_position_only(goal_pose, path_time);
  }
  else if (ch == 'y' || ch == 'Y')
  {
    // printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goal_joint.at(0) = joint_delta;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'h' || ch == 'H')
  {
    // printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goal_joint.at(0) = -joint_delta;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'u' || ch == 'U')
  {
    // printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goal_joint.at(1) = joint_delta;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'j' || ch == 'J')
  {
    // printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goal_joint.at(1) = -joint_delta;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'i' || ch == 'I')
  {
    // printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goal_joint.at(2) = joint_delta;
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'k' || ch == 'K')
  {
    // printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goal_joint.at(2) = -joint_delta;
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'o' || ch == 'O')
  {
    // printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goal_joint.at(3) = joint_delta;
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'l' || ch == 'L')
  {
    // printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goal_joint.at(3) = -joint_delta;
    set_joint_space_path_from_present(joint_name, goal_joint, path_time);
  }
  else if (ch == 'g' || ch == 'G')
  {
    // printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    set_tool_control(joint_angle);
  }
  else if (ch == 'f' || ch == 'F')
  {
    // printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    set_tool_control(joint_angle);
  }
  else if (ch == '1')
  {
    // printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-PI/3);
    joint_name.push_back("joint3"); joint_angle.push_back(PI/9);
    joint_name.push_back("joint4"); joint_angle.push_back(PI*2/9);
    set_joint_space_path(joint_name, joint_angle, path_time);
  }
  else if (ch == '0')
  {
    // printf("input : 1 \tinit pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    set_joint_space_path(joint_name, joint_angle, path_time);
  }
  else if (ch == 'q')
  {
    printf("Pressed Quit \n");
    rclcpp::shutdown();
  }
}

bool TurtleBot3ManipulationTeleopKeyboard::set_base_velocity(geometry_msgs::msg::Twist base_velocity)
{
  cmd_vel_pub_->publish(base_velocity);

  return false;
}

bool TurtleBot3ManipulationTeleopKeyboard::set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  return false;
}

bool TurtleBot3ManipulationTeleopKeyboard::set_tool_control(std::vector<double> joint_angle)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper");
  request->joint_position.position = joint_angle;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);

  return false;
}

bool TurtleBot3ManipulationTeleopKeyboard::set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
  request->planning_group = "gripper";
  request->kinematics_pose.pose.position.x = kinematics_pose.at(0);
  request->kinematics_pose.pose.position.y = kinematics_pose.at(1);
  request->kinematics_pose.pose.position.z = kinematics_pose.at(2);
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_task_space_path_from_present_position_only_client_->async_send_request(request, response_received_callback);

  return false;
}

bool TurtleBot3ManipulationTeleopKeyboard::set_joint_space_path_from_present(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_from_present_client_->async_send_request(request, response_received_callback);

  return false;
}

/********************************************************************************
** Other functions
********************************************************************************/
void TurtleBot3ManipulationTeleopKeyboard::print_text()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control TurtleBot3 + OpenManipulatorX\n");
  printf("---------------------------\n");
  printf("8 : increase linear velocity\n");
  printf("2 : decrease linear velocity\n");
  printf("4 : increase angular velocity\n");
  printf("6 : decrease angular velocity\n");
  printf("5 : base stop\n");
  printf("\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("y : increase joint 1 angle\n");
  printf("h : decrease joint 1 angle\n");
  printf("u : increase joint 2 angle\n");
  printf("j : decrease joint 2 angle\n");
  printf("i : increase joint 3 angle\n");
  printf("k : decrease joint 3 angle\n");
  printf("o : increase joint 4 angle\n");
  printf("l : decrease joint 4 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("0 : init pose\n");
  printf("1 : home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Linear Velocity: %.3lf, Angular Velocity: %.3lf\n",
    get_present_base_velocity().linear.x,
    get_present_base_velocity().angular.z);
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
    get_present_joint_angle().at(0),
    get_present_joint_angle().at(1),
    get_present_joint_angle().at(2),
    get_present_joint_angle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
    get_present_kinematics_pose().at(0),
    get_present_kinematics_pose().at(1),
    get_present_kinematics_pose().at(2));
  printf("---------------------------\n");  
}

geometry_msgs::msg::Twist TurtleBot3ManipulationTeleopKeyboard::get_present_base_velocity()
{
  return present_base_velocity_;
}

std::vector<double> TurtleBot3ManipulationTeleopKeyboard::get_present_joint_angle()
{
  return present_joint_angle_;
}

std::vector<double> TurtleBot3ManipulationTeleopKeyboard::get_present_kinematics_pose()
{
  return present_kinematic_position_;
}

void TurtleBot3ManipulationTeleopKeyboard::restore_terminal_settings()
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void TurtleBot3ManipulationTeleopKeyboard::disable_waiting_for_enter()
{
  struct termios newt;

  tcgetattr(0, &oldt_);             /* Save terminal settings */
  newt = oldt_;                     /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO); /* Change settings */
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &newt);     /* Apply settings */
}

void TurtleBot3ManipulationTeleopKeyboard::display_callback()  
{
  this->print_text();  
  
  char ch = std::getchar();
  this->set_goal(ch);
}
}  // namespace turtlebot3_manipulation_teleop_keyboard

/********************************************************************************
** Main
********************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot3_manipulation_teleop_keyboard::TurtleBot3ManipulationTeleopKeyboard>());
  rclcpp::shutdown();

  return 0;
}
