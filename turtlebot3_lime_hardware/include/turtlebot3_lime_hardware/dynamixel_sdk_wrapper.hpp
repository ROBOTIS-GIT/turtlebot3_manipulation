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

#ifndef TURTLEBOT3_LIME_HARDWARE__DYNAMIXEL_SDK_WRAPPER_HPP_
#define TURTLEBOT3_LIME_HARDWARE__DYNAMIXEL_SDK_WRAPPER_HPP_

#include <stdlib.h>

#include <mutex>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace robotis {
namespace turtlebot3_lime_hardware {
class DynamixelSDKWrapper {
   public:
    explicit DynamixelSDKWrapper(const uint8_t& id);
    virtual ~DynamixelSDKWrapper();

    bool open_port(const std::string& usb_port);
    bool set_baud_rate(const uint32_t& baud_rate);

    uint16_t ping();

    bool read(
        const uint16_t& address,
        const uint16_t& length,
        uint8_t* data);

    uint8_t read_byte(const uint16_t& address);

    bool write(
        const uint16_t& address,
        const uint16_t& length,
        uint8_t* data);

    void write_byte(const uint16_t& address, uint8_t data);

   private:
    dynamixel::PortHandler* port_handler_;
    dynamixel::PacketHandler* packet_handler_;

    std::mutex sdk_handler_m_;

    uint8_t id_;
};
}  // namespace turtlebot3_lime_hardware
}  // namespace robotis
#endif  //  TURTLEBOT3_LIME_HARDWARE__DYNAMIXEL_SDK_WRAPPER_HPP_
