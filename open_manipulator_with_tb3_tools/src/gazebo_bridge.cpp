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

std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;
ros::Subscriber joint_trajectory_point_sub_, gripper_position_sub_;
std_msgs::Float64MultiArray joint_trajectory_point_;
bool is_moving_;

const float CONTROL_PERIOD = 0.001f;

void jointTrajectoryPointCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (is_moving_ == false)
  {
    joint_trajectory_point_ = *msg;
    is_moving_ = true;
  }
}

void gripperPositionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  std_msgs::Float64 positions;
  positions.data = msg->data[0];
  gazebo_goal_joint_position_pub_[4].publish(positions);
}

void publishCallback(const ros::TimerEvent&)
{
  const uint8_t POINT_SIZE = 4 + 1; //Joint num + Time
  static uint32_t points = 0;

  static uint8_t wait_for_pub = 0;
  static uint8_t loop_cnt = 0;

  if (is_moving_ == true)
  {
    uint32_t all_points_cnt = joint_trajectory_point_.data.size();
    uint8_t pub_cnt = 0;

    if (loop_cnt < wait_for_pub)
    {
      loop_cnt++;
      return;
    }
    else
    {
      for (uint32_t positions = points + 1; positions < (points + POINT_SIZE); positions++)
      {
        std_msgs::Float64 joint_position;
        joint_position.data = joint_trajectory_point_.data[positions];

        gazebo_goal_joint_position_pub_.at(pub_cnt).publish(joint_position);
        pub_cnt++;
      }

      points = points + POINT_SIZE;
      wait_for_pub = (joint_trajectory_point_.data[points] - joint_trajectory_point_.data[points - POINT_SIZE]) / CONTROL_PERIOD;

      loop_cnt = 0;

      if (points >= all_points_cnt)
      {
        joint_trajectory_point_.data.clear();
        points = 0;
        wait_for_pub = 0;
        is_moving_ = false;
      }
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
  joint_trajectory_point_sub_ = nh.subscribe("joint_trajectory_point", 1000, jointTrajectoryPointCallback);
  gripper_position_sub_ = nh.subscribe("gripper_position", 10, gripperPositionCallback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_bridge");
  ros::NodeHandle node_handle("");

  initPublisher(node_handle);
  initSubscriber(node_handle);

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(CONTROL_PERIOD), publishCallback);

  ros::spin();
  return 0;
}
