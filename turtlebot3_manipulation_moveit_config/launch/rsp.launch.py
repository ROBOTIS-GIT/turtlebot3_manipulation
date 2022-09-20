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
#from moveit_configs_utils.launches import generate_rsp_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("turtlebot3_manipulation", package_name="turtlebot3_manipulation_moveit_config").to_moveit_configs()
#    return generate_rsp_launch(moveit_config)

##########################################

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  """Launch file for robot state publisher (rsp)"""

  ld = LaunchDescription()
  ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

  # Robot description
  robot_description_config = xacro.process_file(
      os.path.join(
          get_package_share_directory("turtlebot3_manipulation_description"),
          "urdf",
          "turtlebot3_manipulation.urdf.xacro",
      )
  )
  robot_description = {"robot_description": robot_description_config.toxml()}

  # Given the published joint states, publish tf for the robot links and the robot description
  rsp_node = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      respawn=True,
      output="screen",
      parameters=[{"publish_frequency": LaunchConfiguration("publish_frequency")}, robot_description]
  )

  ld.add_action(rsp_node)

  return ld
