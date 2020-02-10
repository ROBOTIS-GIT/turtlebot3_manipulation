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

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TURTLEBOT3_MANIPULATION_GUI_QNODE_HPP_
#define TURTLEBOT3_MANIPULATION_GUI_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>


#define NUM_OF_JOINT_AND_TOOL 5

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtlebot3_manipulation_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
	};

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPosition();

  bool setJointSpacePath(std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

Q_SIGNALS:
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  QStringListModel logging_model;

  // ROS Subscribers
  ros::Subscriber open_manipulator_joint_states_sub_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematics_position_;
  Eigen::Quaterniond present_kinematics_orientation_;
  Eigen::Vector3d present_kinematics_orientation_rpy_;

  // MoveIt! interface
  moveit::planning_interface::MoveGroupInterface* move_group_;
  moveit::planning_interface::MoveGroupInterface* move_group2_;
};

}  // namespace turtlebot3_manipulation_gui

#endif /* TURTLEBOT3_MANIPULATION_GUI_QNODE_HPP_ */
