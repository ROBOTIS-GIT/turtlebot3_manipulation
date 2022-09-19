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
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
TurtleBot3ManipulationSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TurtleBot3ManipulationSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
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
