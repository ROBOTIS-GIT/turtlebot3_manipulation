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

opencr::IMU OpenCR::get_imu()
{
  imu_.angular_velocity.x = get_data<float>(
    opencr_control_table.imu_angular_velocity_x.address,
    opencr_control_table.imu_angular_velocity_x.length);

  imu_.angular_velocity.y = get_data<float>(
    opencr_control_table.imu_angular_velocity_y.address,
    opencr_control_table.imu_angular_velocity_y.length);

  imu_.angular_velocity.z = get_data<float>(
    opencr_control_table.imu_angular_velocity_z.address,
    opencr_control_table.imu_angular_velocity_z.length);

  imu_.linear_acceleration.x = get_data<float>(
    opencr_control_table.imu_linear_acceleration_x.address,
    opencr_control_table.imu_linear_acceleration_x.length);

  imu_.linear_acceleration.y = get_data<float>(
    opencr_control_table.imu_linear_acceleration_y.address,
    opencr_control_table.imu_linear_acceleration_y.length);

  imu_.linear_acceleration.z = get_data<float>(
    opencr_control_table.imu_linear_acceleration_z.address,
    opencr_control_table.imu_linear_acceleration_z.length);

  imu_.orientation.x = get_data<float>(
    opencr_control_table.imu_orientation_x.address,
    opencr_control_table.imu_orientation_x.length);

  imu_.orientation.y = get_data<float>(
    opencr_control_table.imu_orientation_y.address,
    opencr_control_table.imu_orientation_y.length);

  imu_.orientation.z = get_data<float>(
    opencr_control_table.imu_orientation_z.address,
    opencr_control_table.imu_orientation_z.length);

  imu_.orientation.w = get_data<float>(
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

std::array<double, 2> OpenCR::get_wheel_positions()
{
  static std::array<int32_t, 2> last_diff_ticks = {0, 0};
  static std::array<int32_t, 2> last_ticks = {0, 0};
  std::array<double, 2> positions = {0.0, 0.0};

  std::array<int32_t, 2> ticks = {
    get_data<int32_t>(
      opencr_control_table.present_position_left.address,
      opencr_control_table.present_position_left.length),
    get_data<int32_t>(
      opencr_control_table.present_position_right.address,
      opencr_control_table.present_position_right.length)
  };

  positions[opencr::wheels::LEFT] =
    opencr::wheels::TICK_TO_RAD * last_diff_ticks[opencr::wheels::LEFT];
  positions[opencr::wheels::RIGHT] =
    opencr::wheels::TICK_TO_RAD * last_diff_ticks[opencr::wheels::RIGHT];

  last_diff_ticks[opencr::wheels::LEFT] +=
    (ticks[opencr::wheels::LEFT] - last_ticks[opencr::wheels::LEFT]);
  last_diff_ticks[opencr::wheels::RIGHT] +=
    (ticks[opencr::wheels::RIGHT] - last_ticks[opencr::wheels::RIGHT]);

  last_ticks = ticks;

  return positions;
}

std::array<double, 2> OpenCR::get_wheel_velocities()
{
  std::array<double, 2> velocities = {0.0, 0.0};

  std::array<int32_t, 2> rpms = {
    get_data<int32_t>(
      opencr_control_table.present_velocity_left.address,
      opencr_control_table.present_velocity_left.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_right.address,
      opencr_control_table.present_velocity_right.length)
  };

  velocities[opencr::wheels::LEFT] = opencr::wheels::RPM_TO_MS * rpms[opencr::wheels::LEFT];
  velocities[opencr::wheels::RIGHT] = opencr::wheels::RPM_TO_MS * rpms[opencr::wheels::RIGHT];

  return velocities;
}

inline int32_t convert_radian_to_tick(
  const double & radian,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  int32_t tick = 0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (radian > 0) {
    tick = (radian * (max_tick - zero_tick) / max_radian) + zero_tick;
  } else if (radian < 0) {
    tick = (radian * (min_tick - zero_tick) / min_radian) + zero_tick;
  } else {
    tick = zero_tick;
  }

  return tick;
}

inline double convert_tick_to_radian(
  const int32_t & tick,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  double radian = 0.0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (tick > zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * max_radian /
      static_cast<double>(max_tick - zero_tick);
  } else if (tick < zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * min_radian /
      static_cast<double>(min_tick - zero_tick);
  } else {
    radian = 0.0;
  }

  return radian;
}

std::array<double, 4> OpenCR::get_joint_positions()
{
  std::array<double, 4> positions = {0.0, 0.0, 0.0, 0.0};

  std::array<int32_t, 4> ticks = {
    get_data<int32_t>(
      opencr_control_table.present_position_joint_1.address,
      opencr_control_table.present_position_joint_1.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_2.address,
      opencr_control_table.present_position_joint_2.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_3.address,
      opencr_control_table.present_position_joint_3.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_4.address,
      opencr_control_table.present_position_joint_4.length),
  };

  for (uint8_t i = 0; i < ticks.size(); i++) {
    positions[i] = convert_tick_to_radian(
      ticks[i],
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);
  };

  return positions;
}

std::array<double, 4> OpenCR::get_joint_velocities()
{
  std::array<double, 4> velocities = {0.0, 0.0, 0.0, 0.0};

  std::array<int32_t, 4> rpms = {
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_1.address,
      opencr_control_table.present_velocity_joint_1.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_2.address,
      opencr_control_table.present_velocity_joint_2.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_3.address,
      opencr_control_table.present_velocity_joint_3.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_4.address,
      opencr_control_table.present_velocity_joint_4.length),
  };

  for (uint8_t i = 0; i < rpms.size(); i++) {
    velocities[i] = rpms[i] * opencr::joints::RPM_TO_RAD_PER_SEC;
  };

  return velocities;
}

double OpenCR::get_gripper_position()
{
  double position = 0.0;

  int32_t tick = get_data<int32_t>(
    opencr_control_table.present_position_gripper.address,
    opencr_control_table.present_position_gripper.length);

  position = convert_tick_to_radian(
    tick,
    opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);

  return position * opencr::joints::GRIPPER_RAD_TO_METER;
}

double OpenCR::get_gripper_velocity()
{
  double velocity = 0.0;

  int32_t rpm = get_data<int32_t>(
    opencr_control_table.present_velocity_gripper.address,
    opencr_control_table.present_velocity_gripper.length);

  return velocity = rpm * opencr::joints::RPM_TO_RAD_PER_SEC;
}

bool OpenCR::set_joint_positions(std::array<double, 4> commands, std::string & log)
{
  std::array<int32_t, 4> tick = {0, 0, 0, 0};

  union Data {
    int32_t dword[4];
    uint8_t byte[4 * 4];
  } data;

  for (uint8_t i = 0; i < commands.size(); i++) {
    tick[i] = convert_radian_to_tick(
      commands[i],
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);

    data.dword[i] = tick[i];
  };

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(
    opencr_control_table.goal_position_joint_1.address,
    opencr_control_table.goal_position_joint_1.length * 4,
    p_data,
    log
  );

  return comm_result;
}

bool OpenCR::set_init_pose(std::string & log)
{
  std::array<double, 4> init_pose = {0.0, 0.0, 0.0, 0.0};
  return set_joint_positions(init_pose, log);
}
}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis
