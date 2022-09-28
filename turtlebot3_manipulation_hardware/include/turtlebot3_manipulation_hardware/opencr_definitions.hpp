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
#include <cmath>

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
namespace opencr
{
struct IMU
{
  struct Vector
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct Quaternion
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };

  Vector angular_velocity;
  Vector linear_acceleration;
  Quaternion orientation;
};

struct Battery
{
  double voltage = 0.0;
  double percentage = 0.0;
  double design_capacity = 0.0;
  bool present = false;
};

namespace wheels
{
constexpr double SEPERATION = 0.287;
constexpr double RADIUS = 0.033;

// ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-velocity104
constexpr double RPM_TO_MS = 0.229 * (2.0 * M_PI * RADIUS) / 60.0;

// ref) https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-position116
constexpr double DEG_PER_PULSE = 0.087890625;
constexpr double TICK_TO_RAD = DEG_PER_PULSE * M_PI / 180.0;

constexpr uint8_t LEFT = 0;
constexpr uint8_t RIGHT = 1;
}  // namespace wheels

namespace joints
{
constexpr int32_t MIN_TICK = 0;
constexpr int32_t MAX_TICK = 4096;

constexpr double MIN_RADIAN = -M_PI;
constexpr double MAX_RADIAN = M_PI;

constexpr uint8_t JOINT1 = 0;
constexpr uint8_t JOINT2 = 1;
constexpr uint8_t JOINT3 = 2;
constexpr uint8_t JOINT4 = 3;

constexpr double RPM_TO_RAD_PER_SEC = 0.104719755;
}  // namespace joints

namespace grippers
{
constexpr double RAD_TO_METER = -0.015;
constexpr int16_t GOAL_CURRENT = 80;
}  // namespace grippers

enum SOUND
{
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
