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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=PathJoinSubstitution(
            [
                FindPackageShare('turtlebot3_manipulation_navigation2'),
                'map',
                'turtlebot3_world.yaml'
            ]
        )
    )

    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution(
            [
                FindPackageShare('turtlebot3_manipulation_navigation2'),
                'param',
                'turtlebot3.yaml'
            ]
        )
    )

    nav2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bringup'),
            'launch',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('turtlebot3_manipulation_navigation2'),
            'rviz',
            'navigation2.rviz'
        ]
    )

    default_bt_xml_filename = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether execute rviz2'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation'),

        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=map_yaml_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=default_bt_xml_filename,
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim,
                'params_file': params_file,
                'default_bt_xml_filename': default_bt_xml_filename,
                'autostart': autostart,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)),
    ])
