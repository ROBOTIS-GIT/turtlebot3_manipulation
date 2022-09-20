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
// Author: Darby Lim

#include "turtlebot3_manipulation_hardware/turtlebot3_manipulation_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
hardware_interface::return_type TurtleBot3ManipulationSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  (void) info;

  dynamixel_sdk_wrapper_ = std::make_unique<DynamixelSDKWrapper>(200);
  if (dynamixel_sdk_wrapper_->open_port("/dev/ttyACM0")) {
    RCLCPP_INFO(
      rclcpp::get_logger("turtlebot3_manipulation"),
      "Succeeded to open port");
  } else {
    RCLCPP_FATAL(
      rclcpp::get_logger("turtlebot3_manipulation"),
      "Failed to open port");
    return hardware_interface::return_type::ERROR;
  }

  if (dynamixel_sdk_wrapper_->set_baud_rate(1000000)) {
    RCLCPP_INFO(
      rclcpp::get_logger("turtlebot3_manipulation"),
      "Succeeded to set baudrate");
  } else {
    RCLCPP_FATAL(
      rclcpp::get_logger("turtlebot3_manipulation"),
      "Failed to set baudrate");
    return hardware_interface::return_type::ERROR;
  }

  std::string log;
  int32_t model_number = dynamixel_sdk_wrapper_->is_connected(log);
  RCLCPP_INFO(
    rclcpp::get_logger("turtlebot3_manipulation"),
    "model number %d [%s]", model_number, log.c_str());

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
TurtleBot3ManipulationSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // for (uint8_t i = 0; i < info_.joints.size(); i++)
  // {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  // }

  // for (uint8_t i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  // {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  // }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TurtleBot3ManipulationSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // for (uint8_t i = 0; i < info_.joints.size(); i++)
  // {
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_commands_[i]));
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  // }

  return command_interfaces;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::start()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::stop()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::read()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::write()
{
  return hardware_interface::return_type::OK;
}
}  // turtlebot3_manipulation_hardware
}  // robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotis::turtlebot3_manipulation_hardware::TurtleBot3ManipulationSystemHardware,
  hardware_interface::SystemInterface)
