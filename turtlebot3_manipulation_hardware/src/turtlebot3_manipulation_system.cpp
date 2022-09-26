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
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

  joints_acceleration_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);

  joints_velocity_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);

  gripper_acceleration_ = stoi(info_.hardware_parameters["dxl_gripper_profile_acceleration"]);
  gripper_velocity_ = stoi(info_.hardware_parameters["dxl_gripper_profile_velocity"]);

  opencr_ = std::make_unique<OpenCR>(id_);
  if (opencr_->open_port(usb_port_)) {
    RCLCPP_INFO(logger, "Succeeded to open port");
  } else {
    RCLCPP_FATAL(logger, "Failed to open port");
    return hardware_interface::return_type::ERROR;
  }

  if (opencr_->set_baud_rate(baud_rate_)) {
    RCLCPP_INFO(logger, "Succeeded to set baudrate");
  } else {
    RCLCPP_FATAL(logger, "Failed to set baudrate");
    return hardware_interface::return_type::ERROR;
  }

  int32_t model_number = opencr_->ping();
  RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);

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
  dxl_joint_commands_[0] = 0.0;
  dxl_joint_commands_[1] = -1.57;
  dxl_joint_commands_[2] = 1.37;
  dxl_joint_commands_[3] = 0.26;

  dxl_gripper_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

  opencr_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size() +
    info_.sensors[1].state_interfaces.size(),
    0.0);

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
TurtleBot3ManipulationSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_velocities_[i]));
  }

  for (uint8_t i = 0, k = 0; i < info_.sensors.size(); i++) {
    for (uint8_t j = 0; j < info_.sensors[i].state_interfaces.size(); j++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[i].name,
          info_.sensors[i].state_interfaces[j].name,
          &opencr_sensor_states_[k++])
      );
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TurtleBot3ManipulationSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[1]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[2].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[3].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[4].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[5].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[3]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[6].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[7].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[1]));

  return command_interfaces;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::start()
{
  RCLCPP_INFO(logger, "Ready for start");
  opencr_->send_heartbeat(heartbeat_++);

  RCLCPP_INFO(logger, "Wait for IMU re-calibration");
  opencr_->imu_recalibration();
  rclcpp::sleep_for(std::chrono::seconds(3));

  RCLCPP_INFO(logger, "Joints and wheels torque ON");
  opencr_->joints_torque(opencr::ON);
  opencr_->wheels_torque(opencr::ON);

  opencr_->send_heartbeat(heartbeat_++);
  RCLCPP_INFO(logger, "Set profile acceleration and velocity to joints");
  opencr_->set_joint_profile_acceleration(joints_acceleration_);
  opencr_->set_joint_profile_velocity(joints_velocity_);

  RCLCPP_INFO(logger, "Set profile acceleration and velocity to gripper");
  opencr_->set_gripper_profile_acceleration(gripper_acceleration_);
  opencr_->set_gripper_profile_velocity(gripper_velocity_);

  RCLCPP_INFO(logger, "Set goal current value to gripper");
  opencr_->set_gripper_current();

  RCLCPP_INFO(logger, "System starting");
  opencr_->play_sound(opencr::SOUND::ASCENDING);

  status_ = hardware_interface::status::STARTED;

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
  RCLCPP_INFO_ONCE(logger, "Start to read wheels and manipulator states");

  if (opencr_->read_all() == false) {
    RCLCPP_WARN(logger, "Failed to read all control table");
  }

  dxl_positions_[0] = opencr_->get_wheel_positions()[opencr::wheels::LEFT];
  dxl_velocities_[0] = opencr_->get_wheel_velocities()[opencr::wheels::LEFT];

  dxl_positions_[1] = opencr_->get_wheel_positions()[opencr::wheels::RIGHT];
  dxl_velocities_[1] = opencr_->get_wheel_velocities()[opencr::wheels::RIGHT];

  dxl_positions_[2] = opencr_->get_joint_positions()[opencr::joints::JOINT1];
  dxl_velocities_[2] = opencr_->get_joint_velocities()[opencr::joints::JOINT1];

  dxl_positions_[3] = opencr_->get_joint_positions()[opencr::joints::JOINT2];
  dxl_velocities_[3] = opencr_->get_joint_velocities()[opencr::joints::JOINT2];

  dxl_positions_[4] = opencr_->get_joint_positions()[opencr::joints::JOINT3];
  dxl_velocities_[4] = opencr_->get_joint_velocities()[opencr::joints::JOINT3];

  dxl_positions_[5] = opencr_->get_joint_positions()[opencr::joints::JOINT4];
  dxl_velocities_[5] = opencr_->get_joint_velocities()[opencr::joints::JOINT4];

  dxl_positions_[6] = opencr_->get_gripper_position();
  dxl_velocities_[6] = opencr_->get_gripper_velocity();

  dxl_positions_[7] = opencr_->get_gripper_position();
  dxl_velocities_[7] = opencr_->get_gripper_velocity();

  opencr_sensor_states_[0] = opencr_->get_imu().orientation.x;
  opencr_sensor_states_[1] = opencr_->get_imu().orientation.y;
  opencr_sensor_states_[2] = opencr_->get_imu().orientation.z;
  opencr_sensor_states_[3] = opencr_->get_imu().orientation.w;

  opencr_sensor_states_[4] = opencr_->get_imu().angular_velocity.x;
  opencr_sensor_states_[5] = opencr_->get_imu().angular_velocity.y;
  opencr_sensor_states_[6] = opencr_->get_imu().angular_velocity.z;

  opencr_sensor_states_[7] = opencr_->get_imu().linear_acceleration.x;
  opencr_sensor_states_[8] = opencr_->get_imu().linear_acceleration.y;
  opencr_sensor_states_[9] = opencr_->get_imu().linear_acceleration.z;

  opencr_sensor_states_[10] = opencr_->get_battery().voltage;
  opencr_sensor_states_[11] = opencr_->get_battery().percentage;
  opencr_sensor_states_[12] = opencr_->get_battery().design_capacity;
  opencr_sensor_states_[13] = opencr_->get_battery().present;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::write()
{
  RCLCPP_INFO_ONCE(logger, "Start to write wheels and manipulator commands");
  opencr_->send_heartbeat(heartbeat_++);

  if (opencr_->set_wheel_velocities(dxl_wheel_commands_) == false) {
    RCLCPP_ERROR(logger, "Can't control wheels");
  }

  if (opencr_->set_joint_positions(dxl_joint_commands_) == false) {
    RCLCPP_ERROR(logger, "Can't control joints");
  }

  if (opencr_->set_gripper_position(dxl_gripper_commands_[0]) == false) {
    RCLCPP_ERROR(logger, "Can't control gripper");
  }

  return hardware_interface::return_type::OK;
}
}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotis::turtlebot3_manipulation_hardware::TurtleBot3ManipulationSystemHardware,
  hardware_interface::SystemInterface)
