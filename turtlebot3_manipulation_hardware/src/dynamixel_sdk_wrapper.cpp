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

int32_t DynamixelSDKWrapper::ping(std::string & log)
{
  std::lock_guard<std::mutex> lock(sdk_handler_m_);

  int32_t dxl_comm_result = COMM_RX_FAIL;
  uint8_t dxl_error = 0;

  uint16_t model_number = 0;
  dxl_comm_result = packet_handler_->ping(port_handler_, id_, &model_number, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    log = std::string(packet_handler_->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    log = std::string(packet_handler_->getRxPacketError(dxl_error));
  }

  return model_number;
}

bool DynamixelSDKWrapper::read(
  const uint16_t & address,
  const uint16_t & length,
  uint8_t * data,
  std::string & log)
{
  std::lock_guard<std::mutex> lock(sdk_handler_m_);

  int32_t dxl_comm_result = COMM_RX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packet_handler_->readTxRx(
    port_handler_, id_, address, length, data, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    log = std::string(packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  } else if (dxl_error != 0) {
    log = std::string(packet_handler_->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

uint8_t DynamixelSDKWrapper::read(const uint16_t & address)
{
  std::string log;
  uint8_t data[0];
  return this->read(address, 1, &data[0], log);
}
}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis
