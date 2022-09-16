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
#from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("turtlebot3_manipulation", package_name="turtlebot3_manipulation_moveit_config").to_moveit_configs()
#    return generate_static_virtual_joint_tfs_launch(moveit_config)

##########################################
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#from srdfdom.srdf import SRDF   # srdf not available


def generate_launch_description():

    ld = LaunchDescription()

#    # Robot description Semantic config
#    robot_description_semantic_path = os.path.join(
#        get_package_share_directory("turtlebot3_manipulation_moveit_config"),
#            "config",
#            "turtlebot3_manipulation.srdf",
#    )
#    with open(robot_description_semantic_path, "r") as file:
#        robot_description_semantic_config = file.read()
#    srdf = SRDF.from_xml_string(robot_description_semantic_config)
#    name_counter = 0
#    for vj in srdf.virtual_joints:
#        ld.add_action(
#            Node(
#                package="tf2_ros",
#                executable="static_transform_publisher",
#                name=f"static_transform_publisher{name_counter}",
#                output="log",
#                arguments=[
#                    "--frame-id",
#                    vj.parent_frame,
#                    "--child-frame-id",
#                    vj.child_link,
#                ],
#            )
#        )
#        name_counter += 1

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
