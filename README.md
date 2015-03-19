calvin_robot
=============

This stack contains all necessary components to operate the Calvin robot, both in reality and simulation.

Specifically, it provides:

* URDF descriptions (package `calvin_description`),
* configs and launch files for starting the robot (package `calvin_bringup`),
* configs and launch files for simulating the robot in Gazebo (package `calvin_gazebo`),
* a tool to publish states for the not actively actuated joints (package `calvin_moveit_config`),
* a pick and place behavior (package `calvin_pick_n_place`)

For more information, visit the [calvin_robot ROS wiki page](http://www.ros.org/wiki/calvin_robot).
