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

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "turtlebot3_manipulation_hardware/turtlebot3_manipulation_system.hpp"

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
auto logger = rclcpp::get_logger("turtlebot3_manipulation");
hardware_interface::return_type TurtleBot3ManipulationSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  opencr_ = std::make_unique<OpenCR>(200);
  if (opencr_->open_port("/dev/ttyACM0")) {
    RCLCPP_INFO(logger, "Succeeded to open port");
  } else {
    RCLCPP_FATAL(logger, "Failed to open port");
    return hardware_interface::return_type::ERROR;
  }

  if (opencr_->set_baud_rate(1000000)) {
    RCLCPP_INFO(logger, "Succeeded to set baudrate");
  } else {
    RCLCPP_FATAL(logger, "Failed to set baudrate");
    return hardware_interface::return_type::ERROR;
  }

  std::string log;
  int32_t model_number = opencr_->ping(log);
  RCLCPP_INFO(logger, "OpenCR Model Number %d [%s]", model_number, log.c_str());

  if (opencr_->is_connect_manipulator()) {
    RCLCPP_INFO(logger, "Connected manipulator");
  } else {
    RCLCPP_FATAL(logger, "Not connected manipulator");
    return hardware_interface::return_type::ERROR;
  }

  if (opencr_->is_connect_wheels()) {
    RCLCPP_INFO(logger, "Connected wheels");
  } else {
    RCLCPP_FATAL(logger, "Not connected wheels");
    return hardware_interface::return_type::ERROR;
  }

  dxl_wheel_commands_.resize(2, 0.0);
  dxl_joint_commands_.resize(4, 0.0);
  dxl_gripper_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

  opencr_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), 0.0);

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
TurtleBot3ManipulationSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_velocities_[i]));
  }

  for (uint8_t i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name,
      info_.sensors[0].state_interfaces[i].name,
      &opencr_sensor_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TurtleBot3ManipulationSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name.find("wheel") != std::string::npos) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[i]));
    } else if (info_.joints[i].name.find("gripper") != std::string::npos) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[i]));
    } else {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::start()
{
  RCLCPP_INFO(logger, "Ready for start");
  RCLCPP_INFO(logger, "Wait for IMU re-calibration");
  opencr_->imu_recalibration();
  rclcpp::sleep_for(std::chrono::seconds(3));

  RCLCPP_INFO(logger, "Joints and wheels torque ON");
  opencr_->joints_torque(opencr::ON);
  opencr_->wheels_torque(opencr::ON);

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(logger, "System starting");
  opencr_->play_sound(opencr::SOUND::ASCENDING);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::stop()
{
  RCLCPP_INFO(logger, "Ready for stop");
  opencr_->play_sound(opencr::SOUND::DESCENDING);

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(logger, "System stopped");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::read()
{
  std::string log;
  if (opencr_->read_all(log) == false) {
    RCLCPP_WARN(logger, "Failed to read all control table [%s]", log.c_str());
  }

    //  wheel_left_joint!
    //  wheel_right_joint!
    //  joint1!
    //  joint2!
    //  joint3!
    //  joint4!
    //  gripper_left_joint!
    //  gripper_right_joint!

  dxl_positions_[0] = opencr_->wheel_positions()[opencr::wheels::LEFT];
  dxl_velocities_[0] = opencr_->wheel_velocities()[opencr::wheels::LEFT];

  dxl_positions_[1] = opencr_->wheel_positions()[opencr::wheels::RIGHT];
  dxl_velocities_[1] = opencr_->wheel_velocities()[opencr::wheels::RIGHT];

  dxl_positions_[2] = opencr_->joint_positions()[opencr::joints::JOINT1];
  dxl_velocities_[2] = opencr_->joint_velocities()[opencr::joints::JOINT1];

  dxl_positions_[3] = opencr_->joint_positions()[opencr::joints::JOINT2];
  dxl_velocities_[3] = opencr_->joint_velocities()[opencr::joints::JOINT2];

  dxl_positions_[4] = opencr_->joint_positions()[opencr::joints::JOINT3];
  dxl_velocities_[4] = opencr_->joint_velocities()[opencr::joints::JOINT3];

  dxl_positions_[5] = opencr_->joint_positions()[opencr::joints::JOINT4];
  dxl_velocities_[5] = opencr_->joint_velocities()[opencr::joints::JOINT4];

  dxl_positions_[6] = opencr_->gripper_position();
  dxl_velocities_[6] = opencr_->gripper_velocity();

  dxl_positions_[7] = opencr_->gripper_position();
  dxl_velocities_[7] = opencr_->gripper_velocity();

  for (uint8_t i = 0; i < dxl_positions_.size(); i++) {
    RCLCPP_INFO(logger, "Got state %.5f  %.5f for joint %s!",
      dxl_positions_[i], dxl_velocities_[i], info_.joints[i].name.c_str());
  }

  opencr_sensor_states_[0] = opencr_->imu().orientation.x;
  opencr_sensor_states_[1] = opencr_->imu().orientation.y;
  opencr_sensor_states_[2] = opencr_->imu().orientation.z;
  opencr_sensor_states_[3] = opencr_->imu().orientation.w;

  opencr_sensor_states_[4] = opencr_->imu().angular_velocity.x;
  opencr_sensor_states_[5] = opencr_->imu().angular_velocity.y;
  opencr_sensor_states_[6] = opencr_->imu().angular_velocity.z;

  opencr_sensor_states_[7] = opencr_->imu().linear_acceleration.x;
  opencr_sensor_states_[8] = opencr_->imu().linear_acceleration.y;
  opencr_sensor_states_[9] = opencr_->imu().linear_acceleration.z;

  for (uint8_t i = 0; i < opencr_sensor_states_.size(); i++) {
    RCLCPP_DEBUG(logger, "Got state %e for interface %s!",
      opencr_sensor_states_[i], info_.sensors[0].state_interfaces[i].name.c_str());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::write()
{
  RCLCPP_INFO(logger, "Write opencr");

  return hardware_interface::return_type::OK;
}
}  // turtlebot3_manipulation_hardware
}  // robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotis::turtlebot3_manipulation_hardware::TurtleBot3ManipulationSystemHardware,
  hardware_interface::SystemInterface)
