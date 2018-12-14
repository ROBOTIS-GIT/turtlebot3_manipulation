/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;
trajectory_msgs::JointTrajectory joint_trajectory_;
bool is_moving_;

void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  joint_trajectory_ = *msg;
  is_moving_ = true;
}

void gripperPositionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  std_msgs::Float64 positions;
  positions.data = msg->data[0];
  gazebo_goal_joint_position_pub_[4].publish(positions);
}

void publishCallback(const ros::TimerEvent&)
{
  static uint32_t point_cnt = 0;

  if (is_moving_ == true)
  {
    uint32_t position_cnt = joint_trajectory_.points[point_cnt].positions.size();

    for (uint8_t index = 0; index < position_cnt; index)
    {
      std_msgs::Float64 positions;
      positions.data = joint_trajectory_.points[point_cnt].positions[index];
      gazebo_goal_joint_position_pub_.at(index).publish(positions);
    }

    point_cnt++;

    if (point_cnt > joint_trajectory_.points.size())
    {
      is_moving_ = false;
      point_cnt = 0;
    }
  }
}

void initPublisher(ros::NodeHandle nh)
{
  ros::Publisher pb;
  pb = nh.advertise<std_msgs::Float64>("joint1_position/command", 10);
  gazebo_goal_joint_position_pub_.push_back(pb);

  pb = nh.advertise<std_msgs::Float64>("joint2_position/command", 10);
  gazebo_goal_joint_position_pub_.push_back(pb);

  pb = nh.advertise<std_msgs::Float64>("joint3_position/command", 10);
  gazebo_goal_joint_position_pub_.push_back(pb);

  pb = nh.advertise<std_msgs::Float64>("joint4_position/command", 10);
  gazebo_goal_joint_position_pub_.push_back(pb);

  pb = nh.advertise<std_msgs::Float64>("gripper_position/command", 10);
  gazebo_goal_joint_position_pub_.push_back(pb);
}

void initSubscriber(ros::NodeHandle nh)
{
  ros::Subscriber joint_trajectory_sub_ = nh.subscribe("joint_trajectory", 100, jointTrajectoryCallback);
  ros::Subscriber gripper_position_sub_ = nh.subscribe("gripper_position", 10, gripperPositionCallback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_bridge");
  ros::NodeHandle node_handle("");

  initPublisher(node_handle);
  initSubscriber(node_handle);

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.010f), publishCallback);

  ros::spin();
  return 0;
}
