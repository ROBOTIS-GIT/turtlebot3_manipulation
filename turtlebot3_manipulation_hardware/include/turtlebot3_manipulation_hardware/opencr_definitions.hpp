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

#ifndef TURTLEBOT3_MANIPULATION_HARDWARE__OPENCR_DEFINITIONS_HPP_
#define TURTLEBOT3_MANIPULATION_HARDWARE__OPENCR_DEFINITIONS_HPP_

#include <stdint.h>

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
namespace opencr
{
struct IMU {
  struct Vector {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct Quaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };

  Vector angular_velocity;
  Vector linear_acceleration;
  Quaternion orientation;
};

namespace wheels
{
  // ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-velocity104
  // v(m/s) = RPM * (2 * PI * wheel_radius(= 0.033) / 60)
  constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
  constexpr double TICK_TO_RAD = 0.001533981;

  constexpr uint8_t LEFT = 0;
  constexpr uint8_t RIGHT = 1;
}  // namespace wheels

enum SOUND {
  DESCENDING = 0,
  ASCENDING,
  WARNING,
  ERROR
};

constexpr uint8_t ON = 1;
constexpr uint8_t OFF = 0;
}  // namespace opencr
}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis

#endif  // TURTLEBOT3_MANIPULATION_HARDWARE__OPENCR_DEFINITIONS_HPP_
