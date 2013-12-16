#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv) {
  ros::init (argc, argv, "calvin_pickdemo");
  ros::NodeHandle nh;
  moveit::planning_interface::MoveGroup group("arm");
  group.setPlanningTime(45.0);

  std::vector<double> pose_approach(5,0.0);
  pose_approach[0] = 0.0;
  pose_approach[1] = 0.0;
  pose_approach[2] = 0.0;
  pose_approach[3] = 0.0;
  pose_approach[4] = 0.0;

  std::vector<double> pose_pick(5,0.0);
  pose_pick[0] = 0.0;
  pose_pick[1] = 0.0;
  pose_pick[2] = 0.0;
  pose_pick[3] = 0.0;
  pose_pick[4] = 0.0;

  std::vector<double> pose_retreat(5,0.0);
  pose_retreat[0] = 0.0;
  pose_retreat[1] = 0.0;
  pose_retreat[2] = 0.0;
  pose_retreat[3] = 0.0;
  pose_retreat[4] = 0.0;

  std::vector<double> pose_store(5,0.0);
  pose_store[0] = 0.9320;
  pose_store[1] = 0.1596;
  pose_store[2] = -1.0093;
  pose_store[3] = 0.6966;
  pose_store[4] = -2.7886;

  group.setJointValueTarget(pose_approach);
  group.move();
  group.setJointValueTarget(pose_pick);
  group.move();
  group.setJointValueTarget(pose_retreat);
  group.move();
  group.setJointValueTarget(pose_store);
  group.move();
}
