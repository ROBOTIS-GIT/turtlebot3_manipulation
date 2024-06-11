#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS JAPAN CO., LTD.
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
# Authors: Keisuke Nagashima

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    
    # Moveit 2
    moveit_launch_dir = os.path.join(
        get_package_share_directory(
            'turtlebot3_lime_moveit_config'), 'launch')

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_launch_dir, '/moveit_rviz.launch.py'])
    )

    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([moveit_launch_dir, '/move_group.launch.py']),
            launch_arguments={
                'use_sim': 'false',
            }.items(),
        )
    
    # Navigation 2
    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=PathJoinSubstitution(
            [
                FindPackageShare('turtlebot3_lime_navigation2'),
                'map',
                'turtlebot3_world.yaml'
            ]
        )
    )
    
    nav2_map_file_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=map_yaml_file
    )
    
    nav2_launch_dir = os.path.join(
        get_package_share_directory(
            'turtlebot3_lime_navigation2'), 'launch')
    
    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation2.launch.py']),
            launch_arguments={
                'map_yaml_file': map_yaml_file,
            }.items(),
        )

    return LaunchDescription([
        moveit_rviz_launch,
        move_group_launch,
        nav2_map_file_arg,
        nav2_launch
    ])
