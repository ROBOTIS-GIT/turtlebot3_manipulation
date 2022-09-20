#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Hye-jong KIM

######## setup assistant (humble) ########
#from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_spawn_controllers_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("turtlebot3_manipulation", package_name="turtlebot3_manipulation_moveit_config").to_moveit_configs()
#    return generate_spawn_controllers_launch(moveit_config)

##########################################
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    ld = LaunchDescription()

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    ld.add_action(load_controllers)

    return ld
