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

#ifndef TURTLEBOT3_LIME_HARDWARE__OPENCR_CONTROL_TABLE_HPP_
#define TURTLEBOT3_LIME_HARDWARE__OPENCR_CONTROL_TABLE_HPP_

#include <stdint.h>

#define CONTROL_TABLE_SIZE 392

namespace robotis {
namespace turtlebot3_lime_hardware {
typedef struct
{
    int64_t address;
    int64_t length;
} ControlItem;

typedef struct
{
    ControlItem model_information = {0, 2};

    ControlItem millis = {10, 4};

    ControlItem connect_ROS2 = {15, 1};
    ControlItem connect_manipulator = {16, 1};

    ControlItem device_status = {18, 1};
    ControlItem heartbeat = {19, 1};

    ControlItem led_1 = {20, 1};
    ControlItem led_2 = {21, 1};
    ControlItem led_3 = {22, 1};
    ControlItem led_4 = {23, 1};

    ControlItem button_1 = {26, 1};
    ControlItem button_2 = {27, 1};

    ControlItem bumper_1 = {28, 1};
    ControlItem bumper_2 = {29, 1};

    ControlItem illumination = {30, 4};
    ControlItem ir = {34, 4};
    ControlItem sonar = {38, 4};

    ControlItem battery_voltage = {42, 4};
    ControlItem battery_percentage = {46, 4};

    ControlItem sound = {50, 1};

    ControlItem imu_re_calibration = {59, 1};

    ControlItem imu_angular_velocity_x = {60, 4};
    ControlItem imu_angular_velocity_y = {64, 4};
    ControlItem imu_angular_velocity_z = {68, 4};
    ControlItem imu_linear_acceleration_x = {72, 4};
    ControlItem imu_linear_acceleration_y = {76, 4};
    ControlItem imu_linear_acceleration_z = {80, 4};
    ControlItem imu_magnetic_x = {84, 4};
    ControlItem imu_magnetic_y = {88, 4};
    ControlItem imu_magnetic_z = {92, 4};
    ControlItem imu_orientation_w = {96, 4};
    ControlItem imu_orientation_x = {100, 4};
    ControlItem imu_orientation_y = {104, 4};
    ControlItem imu_orientation_z = {108, 4};

    ControlItem present_current_left = {120, 4};
    ControlItem present_current_right = {124, 4};

    ControlItem present_velocity_left = {128, 4};
    ControlItem present_velocity_right = {132, 4};

    ControlItem present_position_left = {136, 4};
    ControlItem present_position_right = {140, 4};

    ControlItem connect_wheels = {148, 1};
    ControlItem torque_wheels = {149, 1};

    ControlItem cmd_velocity_linear_x = {150, 4};
    ControlItem cmd_velocity_linear_y = {154, 4};
    ControlItem cmd_velocity_linear_z = {158, 4};

    ControlItem cmd_velocity_angular_x = {162, 4};
    ControlItem cmd_velocity_angular_y = {166, 4};
    ControlItem cmd_velocity_angular_z = {170, 4};

    ControlItem profile_acceleration_left = {174, 4};
    ControlItem profile_acceleration_right = {178, 4};

    ControlItem torque_joints = {199, 1};

    ControlItem goal_position_joint_1 = {200, 4};
    ControlItem goal_position_joint_2 = {204, 4};
    ControlItem goal_position_joint_3 = {208, 4};
    ControlItem goal_position_joint_4 = {212, 4};
    ControlItem goal_position_joint_5 = {216, 4};
    ControlItem goal_position_joint_6 = {220, 4};
    ControlItem goal_position_gripper = {224, 4};

    ControlItem goal_position_write_joints = {228, 1};
    ControlItem goal_position_write_gripper = {229, 1};

    ControlItem present_position_joint_1 = {232, 4};
    ControlItem present_position_joint_2 = {236, 4};
    ControlItem present_position_joint_3 = {240, 4};
    ControlItem present_position_joint_4 = {244, 4};
    ControlItem present_position_joint_5 = {248, 4};
    ControlItem present_position_joint_6 = {252, 4};
    ControlItem present_position_gripper = {256, 4};

    ControlItem present_velocity_joint_1 = {260, 4};
    ControlItem present_velocity_joint_2 = {264, 4};
    ControlItem present_velocity_joint_3 = {268, 4};
    ControlItem present_velocity_joint_4 = {272, 4};
    ControlItem present_velocity_joint_5 = {276, 4};
    ControlItem present_velocity_joint_6 = {280, 4};
    ControlItem present_velocity_gripper = {284, 4};

    ControlItem present_current_joint_1 = {288, 2};
    ControlItem present_current_joint_2 = {290, 2};
    ControlItem present_current_joint_3 = {292, 2};
    ControlItem present_current_joint_4 = {294, 2};
    ControlItem present_current_joint_5 = {296, 2};
    ControlItem present_current_joint_6 = {298, 2};
    ControlItem present_current_gripper = {300, 2};

    ControlItem profile_acceleration_joint_1 = {312, 4};
    ControlItem profile_acceleration_joint_2 = {316, 4};
    ControlItem profile_acceleration_joint_3 = {320, 4};
    ControlItem profile_acceleration_joint_4 = {324, 4};
    ControlItem profile_acceleration_joint_5 = {328, 4};
    ControlItem profile_acceleration_joint_6 = {332, 4};
    ControlItem profile_acceleration_gripper = {336, 4};

    ControlItem profile_acceleration_write_joints = {340, 1};
    ControlItem profile_acceleration_write_gripper = {341, 1};

    ControlItem profile_velocity_joint_1 = {344, 4};
    ControlItem profile_velocity_joint_2 = {348, 4};
    ControlItem profile_velocity_joint_3 = {352, 4};
    ControlItem profile_velocity_joint_4 = {356, 4};
    ControlItem profile_velocity_joint_5 = {360, 4};
    ControlItem profile_velocity_joint_6 = {364, 4};
    ControlItem profile_velocity_gripper = {368, 4};

    ControlItem profile_velocity_write_joints = {372, 1};
    ControlItem profile_velocity_write_gripper = {373, 1};

    ControlItem goal_current_joint_1 = {376, 2};
    ControlItem goal_current_joint_2 = {378, 2};
    ControlItem goal_current_joint_3 = {380, 2};
    ControlItem goal_current_joint_4 = {382, 2};
    ControlItem goal_current_joint_5 = {384, 2};
    ControlItem goal_current_joint_6 = {386, 2};
    ControlItem goal_current_gripper = {388, 2};

    ControlItem goal_current_write_joints = {390, 1};
    ControlItem goal_current_write_gripper = {391, 1};
} ControlTable;

const ControlTable opencr_control_table;
}  // namespace turtlebot3_lime_hardware
}  // namespace robotis

#endif  // TURTLEBOT3_LIME_HARDWARE__OPENCR_CONTROL_TABLE_HPP_
