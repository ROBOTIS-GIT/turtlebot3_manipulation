^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2025-05-28)
------------------
* Removed the TurtleBot3 Manipulation Gazebo simulation
* Contributors: ChanHyeong Lee

2.2.0 (2025-04-01)
------------------
* Added gripper control in teleoperation.
* Resolved the issue where teleoperation was not functioning on the actual robot.
* Fixed the intermittent issue of Gazebo not launching.
* Fixed the error log related to the mimic joint.
* Contributors: Sungho Woo

2.1.1 (2022-10-14)
------------------
* Support ROS 2 Humble
* MoveIt environment configured
* use ros2_control framework instead of ROBOTIS custom library
* removed dependency to `turtlebot3_*`` and `open_manipulator` packages
* Contributors: Hye-Jong KIM, Darby Lim, Will Son

1.1.0 (2019-02-08)
------------------
* updated the CHANGELOG and version to release binary packages
* added gazebo and moveit config
* deleted world
* updated to sync for new open_manipulator_package `#7 <https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3/issues/7>`_
* updated inertia params
* added params for ar_marker
* added launch file to run demo
* added gripper controller
* added smach and smach_ros
* added waffle model, ar marker argument, ar tracker, prefix
* added joint control for platform
* added filter
* changed logerr to logwarn
* changed joint trajectory variable
* updated params releated to odom
* added waffle model
* added new moveit configuration
* modified position_only_ik
* Contributors: Darby Lim, Pyo

1.0.1 (2018-06-04)
------------------
* added dependency package option
* Contributors: Pyo

1.0.0 (2018-06-01)
------------------
* updated the CHANGELOG and version to release binary packages
* Contributors: Darby Lim, Pyo
