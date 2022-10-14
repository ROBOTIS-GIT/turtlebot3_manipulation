#!/usr/bin/env python3
#
# Copyright 2022 ROBOTIS CO., LTD.
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
# Author: Darby Lim

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation'),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell of occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Publishing period of occupancy grid'),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec],
        ),
    ])
