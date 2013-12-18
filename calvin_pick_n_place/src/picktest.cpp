/* https://raw.github.com/ros-planning/moveit_advanced/hydro-devel/pr2_pick_place/src/pick_place_demo.cpp */
/* Author: Sachin Chitta, Ioan Sucan */

#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>

#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
//#include <moveit_msgs/CollisionObject.h>
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

    moveit_msgs::Grasp graspmult(moveit_msgs::Grasp a, moveit_msgs::Grasp b)
    {
      moveit_msgs::Grasp x;
      geometry_msgs::PoseStamped c;
      c.header.frame_id = "base_footprint";
      c.pose.position.x = a.grasp_pose.pose.position.x + b.grasp_pose.pose.position.x;
      c.pose.position.y = a.grasp_pose.pose.position.y + b.grasp_pose.pose.position.y;
      c.pose.position.z = a.grasp_pose.pose.position.z + b.grasp_pose.pose.position.z;
      c.pose.orientation.x = a.grasp_pose.pose.orientation.x + b.grasp_pose.pose.orientation.x;
      c.pose.orientation.y = a.grasp_pose.pose.orientation.y + b.grasp_pose.pose.orientation.y;
      c.pose.orientation.z = a.grasp_pose.pose.orientation.z + b.grasp_pose.pose.orientation.z;
      c.pose.orientation.w = 1;
      x.grasp_pose = c;


      x.pre_grasp_approach.direction.vector.z = 1.0;
      x.pre_grasp_approach.direction.header.frame_id = "katana_gripper_tool_frame";
      x.pre_grasp_approach.min_distance = 0.1;
      x.pre_grasp_approach.desired_distance = 0.2;

      x.post_grasp_retreat.direction.header.frame_id = "base_footprint";
      x.post_grasp_retreat.direction.vector.z = 1.0;
      x.post_grasp_retreat.min_distance = 0.2;
      x.post_grasp_retreat.desired_distance = 0.3;

      x.pre_grasp_posture.joint_names.resize(1, "katana_r_finger_joint");
      x.pre_grasp_posture.points.resize(1);
      x.pre_grasp_posture.points[0].positions.resize(1);
      x.pre_grasp_posture.points[0].positions[0] = 1;

      x.grasp_posture.joint_names.resize(1, "katana_r_finger_joint");
      x.grasp_posture.points.resize(1);
      x.grasp_posture.points[0].positions.resize(1);
      x.grasp_posture.points[0].positions[0] = 0;

      return x;
    }

    /**
     * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
     */
    std::vector<moveit_msgs::Grasp> generate_grasps(double x, double y, double z)
    {
      static const double ANGLE_INC = M_PI / 16;
      static const double STRAIGHT_ANGLE_MIN = 0.0 + ANGLE_INC;  // + ANGLE_INC, because 0 is already covered by side grasps
      static const double ANGLE_MAX = M_PI / 2;

      // how far from the grasp center should the wrist be?
      static const double STANDOFF = -0.12;

      std::vector<moveit_msgs::Grasp> grasps;

      moveit_msgs::Grasp transform;

      moveit_msgs::Grasp standoff_trans;
      geometry_msgs::PoseStamped p0;
      p0.header.frame_id = "base_footprint";
      p0.pose.position.x = STANDOFF;
      p0.pose.position.y = 0.0;
      p0.pose.position.z = 0.0;
      p0.pose.orientation.x = 0;
      p0.pose.orientation.y = 0;
      p0.pose.orientation.z = 0;
      p0.pose.orientation.w = 1;
      standoff_trans.grasp_pose = p0;


      // ----- side grasps
      //
      //  1. side grasp (xy-planes of `katana_motor5_wrist_roll_link` and of `katana_base_link` are parallel):
      //     - standard: `rpy = (0, 0, *)` (orientation of `katana_motor5_wrist_roll_link` in `katana_base_link` frame)
      //     - overhead: `rpy = (pi, 0, *)`
      geometry_msgs::PoseStamped p1;
      p1.header.frame_id = "base_footprint";
      p1.pose.position.x = x;
      p1.pose.position.y = y;
      p1.pose.position.z = z;
      transform.grasp_pose = p1;

      for (double roll = 0.0; roll <= M_PI; roll += M_PI)
      {
        double pitch = 0.0;

        // add yaw = 0 first, then +ANGLE_INC, -ANGLE_INC, 2*ANGLE_INC, ...
        // reason: grasps with yaw near 0 mean that the approach is from the
        // direction of the arm; it is usually easier to place the object back like
        // this
        for (double yaw = ANGLE_INC; yaw <= ANGLE_MAX; yaw += ANGLE_INC)
        {
          // + atan2 to center the grasps around the vector from arm to object
          p1.pose.orientation.x = roll;
          p1.pose.orientation.y = pitch;
          p1.pose.orientation.z = yaw + atan2(y, x);
          p1.pose.orientation.w = 1;
          grasps.push_back(graspmult(transform, standoff_trans));

          if (yaw != 0.0)
          {
            p1.pose.orientation.x = roll;
            p1.pose.orientation.y = pitch;
            p1.pose.orientation.z = -yaw + atan2(y, x);
            p1.pose.orientation.w = 1;
            grasps.push_back(graspmult(transform, standoff_trans));
          }
        }
      }

      // ----- straight grasps
      //
      //  2. straight grasp (xz-plane of `katana_motor5_wrist_roll_link` contains z axis of `katana_base_link`)
      //     - standard: `rpy = (0, *, atan2(y_w, x_w))`   (x_w, y_w = position of `katana_motor5_wrist_roll_link` in `katana_base_link` frame)
      //     - overhead: `rpy = (pi, *, atan2(y_w, x_w))`
      for (double roll = 0.0; roll <= M_PI; roll += M_PI)
      {
        for (double pitch = STRAIGHT_ANGLE_MIN; pitch <= ANGLE_MAX; pitch += ANGLE_INC)
        {
          double yaw = atan2(y, x);
          p1.pose.position.x = x;
          p1.pose.position.y = y;
          p1.pose.position.z = z;
          p1.pose.orientation.x = roll;
          p1.pose.orientation.y = pitch;
          p1.pose.orientation.z = yaw;
          p1.pose.orientation.w = 1;

          grasps.push_back(graspmult(transform, standoff_trans));
        }
      }

      return grasps;
    }

    bool pick(const std::string &object)
    {

      std::vector<moveit_msgs::Grasp> grasps;

      geometry_msgs::PoseStamped p;
      moveit_msgs::Grasp g;

      p.header.frame_id = "base_footprint";
      p.pose.position.x = 0.35;
      p.pose.position.y = 0.0;
      p.pose.position.z = 0.85;
      p.pose.orientation.x = 0;
      p.pose.orientation.y = 0;
      p.pose.orientation.z = 0;
      p.pose.orientation.w = 1;
      g.grasp_pose = p;

      g.pre_grasp_approach.direction.vector.z = 1.0;
      g.pre_grasp_approach.direction.header.frame_id = "katana_gripper_tool_frame";
      g.pre_grasp_approach.min_distance = 0.1;
      g.pre_grasp_approach.desired_distance = 0.2;

      g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
      g.post_grasp_retreat.direction.vector.z = 1.0;
      g.post_grasp_retreat.min_distance = 0.2;
      g.post_grasp_retreat.desired_distance = 0.3;

      g.pre_grasp_posture.joint_names.resize(1, "katana_r_finger_joint");
      g.pre_grasp_posture.points.resize(1);
      g.pre_grasp_posture.points[0].positions.resize(1);
      g.pre_grasp_posture.points[0].positions[0] = 1;

      g.grasp_posture.joint_names.resize(1, "katana_r_finger_joint");
      g.grasp_posture.points.resize(1);
      g.grasp_posture.points[0].positions.resize(1);
      g.grasp_posture.points[0].positions[0] = 0;

      grasps.push_back(g);

      //group.setSupportSurfaceName("table");

      return group_.pick(object, generate_grasps(0.45, 0.0, 0.85));
      //return group_.pick(object, grasps);
    }

    tf::TransformListener tf_;
    ros::NodeHandle node_handle_;
    moveit::planning_interface::MoveGroup& group_;
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "calvin_picktest");
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

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
  co.id = "testbox";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.08;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.04;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.055;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.45;
  co.primitive_poses[0].position.y = 0.0;
  co.primitive_poses[0].position.z = 0.85;
  co.primitive_poses[0].orientation.w = 1.0;

  // wait a bit for ros things to initialize
  ros::WallDuration(2.0).sleep();

  std::vector<double> pose_zeros(5,0.0);
  std::vector<double> pose_store(5,0.0);
  pose_store[0] = 0.9320;
  pose_store[1] = 0.1596;
  pose_store[2] = -1.0093;
  pose_store[3] = 0.6966;
  pose_store[4] = -2.7886;
  /*
     group.setJointValueTarget(pose_zeros);
     group.move();
     */

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
      /**
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
       **/
      pub_co.publish(co);
      objects.push_back("testbox");
      state = PICK;
      continue;
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
