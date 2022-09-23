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

#ifndef TURTLEBOT3_MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_
#define TURTLEBOT3_MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"

#include "turtlebot3_manipulation_hardware/opencr.hpp"
#include "turtlebot3_manipulation_hardware/visibility_control.h"

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
class TurtleBot3ManipulationSystemHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TurtleBot3ManipulationSystemHardware);

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  uint8_t id_;
  std::string usb_port_;
  uint32_t baud_rate_;

  std::array<int32_t, 4> joints_acceleration_;
  std::array<int32_t, 4> joints_velocity_;

  int32_t gripper_acceleration_;
  int32_t gripper_velocity_;

  std::unique_ptr<OpenCR> opencr_;

  std::vector<double> dxl_wheel_commands_;
  std::vector<double> dxl_joint_commands_;
  std::vector<double> dxl_gripper_commands_;

  std::vector<double> dxl_positions_;
  std::vector<double> dxl_velocities_;

  std::vector<double> opencr_sensor_states_;
};
}  // turtlebot3_manipulation_hardware
}  // robotis
#endif  // TURTLEBOT3_MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_
