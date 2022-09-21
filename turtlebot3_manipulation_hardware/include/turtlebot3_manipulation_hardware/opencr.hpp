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

#ifndef TURTLEBOT3_MANIPULATION_HARDWARE__OPENCR_HPP_
#define TURTLEBOT3_MANIPULATION_HARDWARE__OPENCR_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <stdlib.h>
#include <string>

#include "turtlebot3_manipulation_hardware/dynamixel_sdk_wrapper.hpp"
#include "turtlebot3_manipulation_hardware/opencr_control_table.hpp"
#include "turtlebot3_manipulation_hardware/opencr_definitions.hpp"

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
class OpenCR
{
public:
  explicit OpenCR(const uint8_t & id);
  virtual ~OpenCR();

  template<typename DataByteT>
  DataByteT get_data(const uint16_t & address, const uint16_t & length)
  {
    DataByteT data = 0;
    uint8_t * p_data = reinterpret_cast<uint8_t *>(&data);

    std::lock_guard<std::mutex> lock(buffer_m_);
    switch (length) {
      case 1:
        p_data[0] = data_[address + 0];
        break;

      case 2:
        p_data[0] = data_[address + 0];
        p_data[1] = data_[address + 1];
        break;

      case 4:
        p_data[0] = data_[address + 0];
        p_data[1] = data_[address + 1];
        p_data[2] = data_[address + 2];
        p_data[3] = data_[address + 3];
        break;

      default:
        p_data[0] = data_[address + 0];
        break;
    }

    return data;
  }

  bool open_port(const std::string & usb_port);
  bool set_baud_rate(const uint32_t & baud_rate);

  uint16_t ping(std::string & log);

  bool is_connect_manipulator();
  bool is_connect_wheels();

  void play_sound(uint8_t sound) const;

  void imu_recalibration();
  opencr::IMU imu();

  void joints_torque(uint8_t onoff) const;
  void wheels_torque(uint8_t onoff) const;

  bool read_all(std::string & log);

  std::array<double, 2> wheel_positions();
  std::array<double, 2> wheel_velocities();

  std::array<double, 4> joint_positions();
  std::array<double, 4> joint_velocities();

  double gripper_position();

private:
  std::unique_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  uint8_t data_[CONTROL_TABLE_SIZE];
  uint8_t data_buffer_[CONTROL_TABLE_SIZE];

  std::mutex buffer_m_;

  opencr::IMU imu_;
};
}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis
#endif  // TURTLEBOT3_MANIPULATION_HARDWARE__OPENCR_HPP_

