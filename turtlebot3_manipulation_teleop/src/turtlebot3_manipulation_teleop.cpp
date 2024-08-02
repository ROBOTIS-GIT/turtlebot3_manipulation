// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Hye-jong KIM

#include <algorithm>
#include <memory>

#include "turtlebot3_manipulation_teleop/turtlebot3_manipulation_teleop.hpp"

// KeyboardReader
KeyboardReader::KeyboardReader()
: kfd(0)
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

void KeyboardReader::readOne(char * c)
{
  int rc = read(kfd, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}

void KeyboardReader::shutdown()
{
  tcsetattr(kfd, TCSANOW, &cooked);
}

// KeyboardServo

KeyboardServo::KeyboardServo()
: publish_task_(false), publish_joint_(false)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  servo_start_client_ =
    nh_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_stop_client_ =
    nh_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");

  base_twist_pub_ =
    nh_->create_publisher<geometry_msgs::msg::Twist>(BASE_TWIST_TOPIC, ROS_QUEUE_SIZE);
  arm_twist_pub_ =
    nh_->create_publisher<geometry_msgs::msg::TwistStamped>(ARM_TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(ARM_JOINT_TOPIC, ROS_QUEUE_SIZE);

  cmd_vel_ = geometry_msgs::msg::Twist();
}

KeyboardServo::~KeyboardServo()
{
  stop_moveit_servo();
}

int KeyboardServo::keyLoop()
{
  char c;

  // Ros Spin
  std::thread{std::bind(&KeyboardServo::spin, this)}.detach();
  connect_moveit_servo();
  start_moveit_servo();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use o|k|l|; keys to move turtlebot base and use 'space' key to stop the base");
  puts("Use s|x|z|c|a|d|f|v keys to Cartesian jog");
  puts("Use 1|2|3|4|q|w|e|r keys to joint jog.");
  puts("'ESC' to quit.");

  std::thread{std::bind(&KeyboardServo::pub, this)}.detach();

  bool servoing = true;
  while (servoing) {
    // get the next event from the keyboard
    try {
      input.readOne(&c);
    } catch (const std::runtime_error &) {
      perror("read():");
      return -1;
    }

    RCLCPP_INFO(nh_->get_logger(), "value: 0x%02X", c);
    joint_msg_.joint_names.clear();
    joint_msg_.velocities.clear();

    // Use read key-press
    switch (c) {
      case KEYCODE_O:  // KEYCODE_UP:
        cmd_vel_.linear.x =
          std::min(cmd_vel_.linear.x + BASE_LINEAR_VEL_STEP, BASE_LINEAR_VEL_MAX);
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.linear.z = 0.0;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "LINEAR VEL : " << cmd_vel_.linear.x);
        break;
      case KEYCODE_L:  // KEYCODE_DOWN:
        cmd_vel_.linear.x =
          std::max(cmd_vel_.linear.x - BASE_LINEAR_VEL_STEP, -BASE_LINEAR_VEL_MAX);
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.linear.z = 0.0;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "LINEAR VEL : " << cmd_vel_.linear.x);
        break;
      case KEYCODE_K:  // KEYCODE_LEFT:
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z =
          std::min(cmd_vel_.angular.z + BASE_ANGULAR_VEL_STEP, BASE_ANGULAR_VEL_MAX);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "ANGULAR VEL : " << cmd_vel_.angular.z);
        break;
      case KEYCODE_SEMICOLON:  // KEYCODE_RIGHT:
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z =
          std::max(cmd_vel_.angular.z - BASE_ANGULAR_VEL_STEP, -BASE_ANGULAR_VEL_MAX);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "ANGULAR VEL : " << cmd_vel_.angular.z);
        break;
      case KEYCODE_SPACE:
        cmd_vel_ = geometry_msgs::msg::Twist();
        RCLCPP_INFO_STREAM(nh_->get_logger(), "STOP base");
        break;
      case KEYCODE_A:
        task_msg_.twist.linear.z = ARM_TWIST_VEL;
        publish_task_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm z UP");
        break;
      case KEYCODE_D:
        task_msg_.twist.linear.z = -ARM_TWIST_VEL;
        publish_task_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm z DOWN");
        break;
      case KEYCODE_S:
        task_msg_.twist.linear.x = ARM_TWIST_VEL;
        publish_task_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm x Front");
        break;
      case KEYCODE_X:
        task_msg_.twist.linear.x = -ARM_TWIST_VEL;
        publish_task_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm x Back");
        break;
      case KEYCODE_Z:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm Turn left.");
        break;
      case KEYCODE_C:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm Turn right.");
        break;
      case KEYCODE_F:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Down.");
        break;
      case KEYCODE_V:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Up.");
        break;
      case KEYCODE_1:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint1 +");
        break;
      case KEYCODE_2:
        joint_msg_.joint_names.push_back("joint2");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint2 +");
        break;
      case KEYCODE_3:
        joint_msg_.joint_names.push_back("joint3");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint3 +");
        break;
      case KEYCODE_4:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint4 +");
        break;
      case KEYCODE_Q:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint1 -");
        break;
      case KEYCODE_W:
        joint_msg_.joint_names.push_back("joint2");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint2 -");
        break;
      case KEYCODE_E:
        joint_msg_.joint_names.push_back("joint3");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint3 -");
        break;
      case KEYCODE_R:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint4 -");
        break;
      case KEYCODE_ESC:
        RCLCPP_INFO_STREAM(nh_->get_logger(), "quit");
        servoing = false;
        break;
      default:
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Unassigned input : " << c);
        break;
    }
  }

  return 0;
}

void KeyboardServo::connect_moveit_servo()
{
  for (int i = 0; i < 10; i++) {
    if (servo_start_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS TO CONNNET SERVO START SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "WAIT TO CONNNET SERVO START SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'turtlebot3_manipulation_moveit_configs' pkg.");
    }
  }
  for (int i = 0; i < 10; i++) {
    if (servo_stop_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS TO CONNNET SERVO STOP SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "WAIT TO CONNNET SERVO STOP SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'turtlebot3_manipulation_moveit_configs' pkg.");
    }
  }
}

void KeyboardServo::start_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call 'moveit_servo' start srv.");
  auto future = servo_start_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  auto result = future.wait_for(std::chrono::seconds(1));
  if (result == std::future_status::ready) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS to start 'moveit_servo'");
    future.get();
  } else {
    RCLCPP_ERROR_STREAM(
      nh_->get_logger(), "FAIL to start 'moveit_servo', excute without 'moveit_servo'");
  }
}

void KeyboardServo::stop_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call 'moveit_servo' END srv.");
  auto future = servo_stop_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  auto result = future.wait_for(std::chrono::seconds(1));
  if (result == std::future_status::ready) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS to stop 'moveit_servo'");
    future.get();
  }
}

void KeyboardServo::pub()
{
  while (rclcpp::ok()) {
    // If a key requiring a publish was pressed, publish the message now
    if (publish_task_) {
      task_msg_.header.stamp = nh_->now();
      task_msg_.header.frame_id = BASE_FRAME_ID;
      arm_twist_pub_->publish(task_msg_);
      publish_task_ = false;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "TASK PUB");
    } else if (publish_joint_) {
      joint_msg_.header.stamp = nh_->now();
      joint_msg_.header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(joint_msg_);
      publish_joint_ = false;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint PUB");
    }
    // Base pub
    base_twist_pub_->publish(cmd_vel_);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void KeyboardServo::spin()
{
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh_);
  }
}