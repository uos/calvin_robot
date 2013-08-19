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
 * joint_commander.cpp
 *
 *  Created on: 24.08.2012
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <joint_commander.h>

namespace calvin_joint_commander
{

JointCommander::JointCommander()
{
  dynamic_reconfigure::Server<calvin_joint_commander::JointCommanderConfig>::CallbackType f;
  f = boost::bind(&calvin_joint_commander::JointCommander::update_config, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(f);

  //yaw_controller_pub_ = nh_.advertise<std_msgs::Float64>("kinect_yaw_joint_controller/command", 1);
  kinect_pitch_controller_pub_ = nh_.advertise<std_msgs::Float64>("kinect_pitch_joint_controller/command", 1);
  webcam_pitch_controller_pub_ = nh_.advertise<std_msgs::Float64>("webcam_pitch_joint_controller/command", 1);
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

JointCommander::~JointCommander()
{
}

void JointCommander::update_config(calvin_joint_commander::JointCommanderConfig &new_config, uint32_t level)
{
  config_ = new_config;
}

void JointCommander::loop_once()
{
  if (config_.publish_controller_commands)
  {
    std_msgs::Float64 control_msg;

    //control_msg.data = config_.kinect_yaw_joint;
    //yaw_controller_pub_.publish(control_msg);
    control_msg.data = config_.kinect_pitch_joint;
    kinect_pitch_controller_pub_.publish(control_msg);
    control_msg.data = config_.webcam_pitch_joint;
    webcam_pitch_controller_pub_.publish(control_msg);
  }

  if (config_.publish_joint_states)
  {
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.push_back("kinect_pitch_joint");
    joint_state_msg.position.push_back(config_.kinect_pitch_joint);
    joint_state_msg.name.push_back("webcam_pitch_joint");
    joint_state_msg.position.push_back(config_.webcam_pitch_joint);
    joint_states_pub_.publish(joint_state_msg);
  }
}

} /* namespace calvin_joint_commander */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_commander");
  calvin_joint_commander::JointCommander joint_commander_node;

  ros::Rate r(25.0);
  while (ros::ok())
  {
    ros::spinOnce();
    joint_commander_node.loop_once();
    r.sleep();
  }

  return 0;
}
