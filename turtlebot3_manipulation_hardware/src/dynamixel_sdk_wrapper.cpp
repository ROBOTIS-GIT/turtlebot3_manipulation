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

#include "turtlebot3_manipulation_hardware/dynamixel_sdk_wrapper.hpp"

#include <string>

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
DynamixelSDKWrapper::DynamixelSDKWrapper(const uint8_t & id)
: id_(id)
{
}

DynamixelSDKWrapper::~DynamixelSDKWrapper()
{
  port_handler_->closePort();
}

bool DynamixelSDKWrapper::open_port(const std::string & usb_port)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port.c_str());
  return port_handler_->openPort();
}

bool DynamixelSDKWrapper::set_baud_rate(const uint32_t & baud_rate)
{
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
  return port_handler_->setBaudRate(baud_rate);
}

int32_t DynamixelSDKWrapper::is_connected(const char ** log)
{
  std::lock_guard<std::mutex> lock(sdk_handler_m_);

  uint8_t dxl_error = 0;
  int32_t model_number = packet_handler_->ping(port_handler_, id_, &dxl_error);

  if (dxl_error != 0) {
    if (log != NULL) {
      *log = packet_handler_->getRxPacketError(dxl_error);
    }
  }

  return model_number;
}
}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis
