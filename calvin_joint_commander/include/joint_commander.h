/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2012  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * joint_commander.h
 *
 *  Created on: 24.08.2012
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef JOINT_COMMANDER_H_
#define JOINT_COMMANDER_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <calvin_joint_commander/JointCommanderConfig.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

namespace calvin_joint_commander
{

class JointCommander
{
public:
  JointCommander();
  virtual ~JointCommander();
  void update_config(calvin_joint_commander::JointCommanderConfig &new_config, uint32_t level = 0);
  void loop_once();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher yaw_controller_pub_;
  ros::Publisher pitch_controller_pub_;
  ros::Publisher joint_states_pub_;

  // Dynamic Reconfigure
  JointCommanderConfig config_;
  dynamic_reconfigure::Server<calvin_joint_commander::JointCommanderConfig> dynamic_reconfigure_server_;

};

} /* namespace kurtana_pole_joint_commander */
#endif /* JOINT_COMMANDER_H_ */
