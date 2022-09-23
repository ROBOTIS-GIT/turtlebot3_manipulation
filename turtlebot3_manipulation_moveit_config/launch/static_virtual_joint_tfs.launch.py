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

# setup assistant (humble)
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("turtlebot3_manipulation",
#     package_name="turtlebot3_manipulation_moveit_config").to_moveit_configs()
#     return generate_static_virtual_joint_tfs_launch(moveit_config)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "base_footprint"],
    )
    ld.add_action(static_tf)

    return ld
