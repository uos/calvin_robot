/* https://raw.github.com/ros-planning/moveit_advanced/hydro-devel/pr2_pick_place/src/pick_place_demo.cpp */
/* Author: Sachin Chitta, Ioan Sucan */

#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>

#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <tf/transform_listener.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

class PickPlaceGroup
{
  public:
    PickPlaceGroup(moveit::planning_interface::MoveGroup &group): group_(group)
  {
  }

    bool pick(const std::string &object)
    {
      return group_.pick(object);
    }

    tf::TransformListener tf_;
    ros::NodeHandle node_handle_;
    moveit::planning_interface::MoveGroup& group_;
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "calvin_picktest");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  ros::WallDuration(2.0).sleep();

  moveit::planning_interface::PlanningSceneInterface scene_interface;
  moveit::planning_interface::MoveGroup group("arm");
  group.setPlanningTime(45.0);

  PickPlaceGroup pg(group);

  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.0;
  co.primitive_poses[0].position.y = 0.0;
  co.primitive_poses[0].position.z = 0.0;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);

  // wait a bit for ros things to initialize
  ros::WallDuration(2.0).sleep();

  std::vector<double> pose_zeros(5,0.0);
  std::vector<double> pose_store(5,0.0);
  pose_store[0] = 0.9320;
  pose_store[1] = 0.1596;
  pose_store[2] = -1.0093;
  pose_store[3] = 0.6966;
  pose_store[4] = -2.7886;
  //joint_values[0] = -2.01;
  //joint_values[1] = 0.737;
  //joint_values[2] = -2.00;
  //joint_values[3] = -2.00;
  //joint_values[4] = -2.5774;
  //joint_values[5] = -1.184;
  //joint_values[6] = -1.16;

  group.setJointValueTarget(pose_zeros);
  group.move();

  const static unsigned int START = 0;
  const static unsigned int PICK = 1;
  const static unsigned int STORE = 2;
  const static unsigned int CLEAR = 3;
  unsigned int state = START;

  std::vector<std::string> objects;
  while(ros::ok())
  {
    if(state == START)
    {
      objects = scene_interface.getKnownObjectNames(false);
      if(objects.empty())
      {
        ROS_INFO("Could not find any object in workspace");
        group.setJointValueTarget(pose_zeros);
        group.move();
        continue;
      }
      else
      {
        state = PICK;
        continue;
      }
    }
    else if (state == PICK)
    {
      ROS_INFO("Trying to pickup object: %s", objects[0].c_str());
      if(pg.pick(objects[0]))
      {
        ROS_INFO("Done pick");
        ros::WallDuration(1.0).sleep();
        state = STORE;
      }
      else
      {
        state = START;
        continue;
      }
    }
    else if (state == STORE)
    {
      ROS_INFO("Storing object");
      group.setJointValueTarget(pose_store);
      group.move();
      state = CLEAR;
      continue;
    }
    else if (state == CLEAR)
    {
      ROS_INFO("Back to Start");
      group.setJointValueTarget(pose_zeros);
      group.move();
      state = START;
    }
  }

  ROS_INFO("Waiting for Ctrl-C");
  ros::waitForShutdown();
  return 0;
}