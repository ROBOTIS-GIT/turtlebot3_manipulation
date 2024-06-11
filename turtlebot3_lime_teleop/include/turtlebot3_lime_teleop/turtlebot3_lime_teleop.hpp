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

#ifndef TURTLEBOT3_LIME_TELEOP__TURTLEBOT3_LIME_TELEOP_HPP_
#define TURTLEBOT3_LIME_TELEOP__TURTLEBOT3_LIME_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <string>

// Define used keys
#define KEYCODE_UP 0x41     // Base Front
#define KEYCODE_DOWN 0x42   // Base Back
#define KEYCODE_RIGHT 0x43  // Base Right
#define KEYCODE_LEFT 0x44   // Base Left
#define KEYCODE_SPACE 0x20  // Base Stop

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_X 0x78
#define KEYCODE_Z 0x7A
#define KEYCODE_C 0x63
#define KEYCODE_F 0x66
#define KEYCODE_V 0x76

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_Y 0x79

#define KEYCODE_ESC 0x1B

#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_O 0x6F
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_P 0x70
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_PERIOD 0x2E

// Some constants used in the Servo Teleop demo
const char BASE_TWIST_TOPIC[] = "cmd_vel";
const char ARM_TWIST_TOPIC[] = "/servo_node/delta_twist_cmds";
const char ARM_JOINT_TOPIC[] = "/servo_node/delta_joint_cmds";

const size_t ROS_QUEUE_SIZE = 10;

const double BASE_LINEAR_VEL_MAX = 0.26;  // m/s
const double BASE_LINEAR_VEL_STEP = 0.01;  // m/s

const double BASE_ANGULAR_VEL_MAX = 1.8;  // rad/s
const double BASE_ANGULAR_VEL_STEP = 0.1;  // rad/s

const char BASE_FRAME_ID[] = "link2";

const double ARM_TWIST_VEL = 0.2;  // m/s
const double ARM_JOINT_VEL = 1.0;  // rad/s

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader();
  void readOne(char * c);
  void shutdown();

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  ~KeyboardServo();
  int keyLoop();

  void connect_moveit_servo();
  void start_moveit_servo();
  void stop_moveit_servo();

private:
  void spin();
  void pub();

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::TwistStamped task_msg_;
  control_msgs::msg::JointJog joint_msg_;

  bool publish_task_;
  bool publish_joint_;
};

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();

  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

#endif  // TURTLEBOT3_LIME_TELEOP__TURTLEBOT3_LIME_TELEOP_HPP_
