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

#include "turtlebot3_lime_hardware/dynamixel_sdk_wrapper.hpp"

#include <iostream>
#include <string>

namespace robotis {
namespace turtlebot3_lime_hardware {
DynamixelSDKWrapper::DynamixelSDKWrapper(const uint8_t& id)
    : id_(id) {
}

DynamixelSDKWrapper::~DynamixelSDKWrapper() {
    port_handler_->closePort();
}

inline void error_print(const char* log) {
    std::cerr << "\033[1;31m[ERROR] [DynamixelSDKWrapper] " << log << "\033[0m" << std::endl;
}

bool DynamixelSDKWrapper::open_port(const std::string& usb_port) {
    port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port.c_str());
    return port_handler_->openPort();
}

bool DynamixelSDKWrapper::set_baud_rate(const uint32_t& baud_rate) {
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    return port_handler_->setBaudRate(baud_rate);
}

uint16_t DynamixelSDKWrapper::ping() {
    std::lock_guard<std::mutex> lock(sdk_handler_m_);

    int32_t dxl_comm_result = COMM_RX_FAIL;
    uint8_t dxl_error = 0;

    uint16_t model_number = 0;
    dxl_comm_result = packet_handler_->ping(port_handler_, id_, &model_number, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        error_print(packet_handler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        error_print(packet_handler_->getRxPacketError(dxl_error));
    }

    return model_number;
}

bool DynamixelSDKWrapper::read(
    const uint16_t& address,
    const uint16_t& length,
    uint8_t* data) {
    std::lock_guard<std::mutex> lock(sdk_handler_m_);

    int32_t dxl_comm_result = COMM_RX_FAIL;
    uint8_t dxl_error = 0;

    dxl_comm_result = packet_handler_->readTxRx(
        port_handler_, id_, address, length, data, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        error_print(packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
    } else if (dxl_error != 0) {
        error_print(packet_handler_->getRxPacketError(dxl_error));
        return false;
    }

    return true;
}

uint8_t DynamixelSDKWrapper::read_byte(const uint16_t& address) {
    uint8_t data[1];
    this->read(address, 1, &data[0]);

    return data[0];
}

bool DynamixelSDKWrapper::write(
    const uint16_t& address,
    const uint16_t& length,
    uint8_t* data) {
    std::lock_guard<std::mutex> lock(sdk_handler_m_);

    int32_t dxl_comm_result = COMM_RX_FAIL;
    uint8_t dxl_error = 0;

    dxl_comm_result = packet_handler_->writeTxRx(
        port_handler_, id_, address, length, data, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        error_print(packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
    } else if (dxl_error != 0) {
        error_print(packet_handler_->getRxPacketError(dxl_error));
        return false;
    }

    return true;
}

void DynamixelSDKWrapper::write_byte(const uint16_t& address, uint8_t data) {
    uint8_t* data_ptr = &data;
    this->write(address, 1, &data_ptr[0]);
}
}  // namespace turtlebot3_lime_hardware
}  // namespace robotis
