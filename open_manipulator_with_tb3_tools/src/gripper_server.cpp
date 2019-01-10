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
#include <open_manipulator_msgs/SetJointPosition.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

ros::Publisher platform_gripper_pub_, gazebo_gripper_pub_;
ros::ServiceServer gripper_server_;

bool gripperCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                     open_manipulator_msgs::SetJointPosition::Response &res)
{
  std_msgs::Float64 position;
  position.data = req.joint_position.position.at(0);

  std_msgs::Float64MultiArray position_array;
  position_array.data.push_back(position.data);

  gazebo_gripper_pub_.publish(position);
  platform_gripper_pub_.publish(position_array);

  res.is_planned = true;

  return true;
}

void initPublisher(ros::NodeHandle nh)
{
  platform_gripper_pub_ = nh.advertise<std_msgs::Float64MultiArray>("gripper_position", 10);
  gazebo_gripper_pub_ = nh.advertise<std_msgs::Float64>("gripper_position/command", 10);
}

void initServer(ros::NodeHandle nh)
{  
  gripper_server_ = nh.advertiseService("gripper", gripperCallback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_server");
  ros::NodeHandle node_handle("");

  initPublisher(node_handle);
  initServer(node_handle);

  ros::spin();
  return 0;
}
