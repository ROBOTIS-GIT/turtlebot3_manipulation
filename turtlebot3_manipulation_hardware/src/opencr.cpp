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

#include "turtlebot3_manipulation_hardware/opencr.hpp"

#include <string>

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
OpenCR::OpenCR(const uint8_t & id)
{
  dxl_sdk_wrapper_ = std::make_unique<DynamixelSDKWrapper>(id);
}

OpenCR::~OpenCR()
{
}

bool OpenCR::open_port(const std::string & usb_port)
{
  return dxl_sdk_wrapper_->open_port(usb_port);
}

bool OpenCR::set_baud_rate(const uint32_t & baud_rate)
{
  return dxl_sdk_wrapper_->set_baud_rate(baud_rate);
}

uint16_t OpenCR::ping(std::string & log)
{
  return dxl_sdk_wrapper_->ping(log);
}

bool OpenCR::is_connect_manipulator()
{
  return dxl_sdk_wrapper_->read_byte(opencr_control_table.connect_manipulator.address);
}

bool OpenCR::is_connect_wheels()
{
  return dxl_sdk_wrapper_->read_byte(opencr_control_table.connect_wheels.address);
}

void OpenCR::play_sound(uint8_t sound) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.sound.address, sound);
}

void OpenCR::imu_recalibration()
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.imu_re_calibration.address, 1);
}

const OpenCR::IMU OpenCR::imu()
{
  imu_.angular_velocity.x= get_data<double>(
    opencr_control_table.imu_angular_velocity_x.address,
    opencr_control_table.imu_angular_velocity_x.length);

  imu_.angular_velocity.y= get_data<double>(
    opencr_control_table.imu_angular_velocity_y.address,
    opencr_control_table.imu_angular_velocity_y.length);

  imu_.angular_velocity.z= get_data<double>(
    opencr_control_table.imu_angular_velocity_z.address,
    opencr_control_table.imu_angular_velocity_z.length);

  imu_.linear_acceleration.x= get_data<double>(
    opencr_control_table.imu_linear_acceleration_x.address,
    opencr_control_table.imu_linear_acceleration_x.length);

  imu_.linear_acceleration.y= get_data<double>(
    opencr_control_table.imu_linear_acceleration_y.address,
    opencr_control_table.imu_linear_acceleration_y.length);

  imu_.linear_acceleration.z= get_data<double>(
    opencr_control_table.imu_linear_acceleration_z.address,
    opencr_control_table.imu_linear_acceleration_z.length);

  imu_.orientation.x= get_data<double>(
    opencr_control_table.imu_orientation_x.address,
    opencr_control_table.imu_orientation_x.length);

  imu_.orientation.y= get_data<double>(
    opencr_control_table.imu_orientation_y.address,
    opencr_control_table.imu_orientation_y.length);

  imu_.orientation.z= get_data<double>(
    opencr_control_table.imu_orientation_z.address,
    opencr_control_table.imu_orientation_z.length);

  imu_.orientation.w= get_data<double>(
    opencr_control_table.imu_orientation_w.address,
    opencr_control_table.imu_orientation_w.length);

  return imu_;
}

void OpenCR::joints_torque(uint8_t onoff) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.torque_joints.address, onoff);
}

void OpenCR::wheels_torque(uint8_t onoff) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.torque_wheels.address, onoff);
}

bool OpenCR::read_all(std::string & log)
{
  bool comm_result = dxl_sdk_wrapper_->read(0, CONTROL_TABLE_SIZE, &data_buffer_[0], log);

  if (comm_result) {
    std::lock_guard<std::mutex> lock(buffer_m_);
    std::copy(data_buffer_, data_buffer_ + CONTROL_TABLE_SIZE, data_);
  }

  return comm_result;
}

}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis
