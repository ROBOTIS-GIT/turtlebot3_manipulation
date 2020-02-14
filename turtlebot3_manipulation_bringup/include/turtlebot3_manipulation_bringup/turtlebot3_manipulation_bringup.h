/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
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

/* Authors: Ryan Shim */

#ifndef TURTLEBOT3_MANIPULATION_BRINGUP_H
#define TURTLEBOT3_MANIPULATION_BRINGUP_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


class Turtlebot3ManipulationBringup
{
 public:
  Turtlebot3ManipulationBringup();
  ~Turtlebot3ManipulationBringup();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // ROS Publisher
  ros::Publisher joint_trajectory_point_pub_;
  ros::Publisher gripper_pub_;

  // ROS Subscriber
  ros::Subscriber display_planned_path_sub_;

  // ROS Server
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> arm_action_server_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> gripper_action_server_;

  // Callback Funcdtions
  void armActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &msg);
  void gripperActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &msg);
};

#endif //TURTLEBOT3_MANIPULATION_BRINGUP_H
