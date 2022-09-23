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
# from moveit_configs_utils.launches import generate_demo_launch
# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("turtlebot3_manipulation",
#     package_name="turtlebot3_manipulation_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    launch_dir = os.path.join(
        get_package_share_directory(
            'turtlebot3_manipulation_moveit_config'), 'launch')

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)

    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py'])
    )
    ld.add_action(move_group_launch)

    # gazebo_control with robot_state_publisher
    rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether execute rviz2')
    ld.add_action(rviz_arg)

    # Warehouse mongodb server
    warehouse_db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/warehouse_db.launch.py'])
    )
    ld.add_action(warehouse_db_launch)

    return ld
