#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

ros::Publisher pub_co;
ros::Publisher pub_aco;
ros::Publisher planning_scene_publisher;

moveit_msgs::CollisionObject co;
moveit_msgs::AttachedCollisionObject aco;

void add_collision_object() {
  ROS_INFO("Adding collision object.");
  co.operation = moveit_msgs::CollisionObject::ADD;
  pub_co.publish(co);
}
void remove_collision_object() {
  ROS_INFO("Removing collision object.");
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);
}
void add_attached_collision_object() {
  ROS_INFO("Adding attached collision object.");
  aco.object.operation = moveit_msgs::CollisionObject::ADD;
  pub_aco.publish(aco);
}
void remove_attached_collision_object() {
  ROS_INFO("Removing attached collision object.");
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_aco.publish(aco);
}

moveit_msgs::Grasp tf_transform_to_grasp(tf::Transform t)
{
  moveit_msgs::Grasp grasp;
  geometry_msgs::PoseStamped pose;

  geometry_msgs::Vector3 origin;
  tf::Quaternion rotation;
  tf::Vector3 rotation_axis;

  origin = t.getOrigin();

  rotation = t.getRotation();
  tf::quaternionTFToMsg(rotation, pose.pose.orientation);

  pose.header.frame_id = "base_footprint";
  pose.pose.position.x = origin.x;
  pose.pose.position.y = origin.y;
  pose.pose.position.z = origin.z;
  //pose.pose.orientation.x = 0;
  //pose.pose.orientation.y = 0;
  //pose.pose.orientation.z = 0;
  //pose.pose.orientation.w = 1;
  grasp.grasp_pose = pose;

  return grasp;
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
  static const double STANDOFF = -0.01;

  std::vector<moveit_msgs::Grasp> grasps;

  tf::Transform transform;

  tf::Transform standoff_trans;
  standoff_trans.setOrigin(tf::Vector3(STANDOFF, 0.0, 0.0));
  standoff_trans.setRotation(tf::Quaternion(0.0, sqrt(2)/2, 0.0, sqrt(2)/2));

  // ----- side grasps
  //
  //  1. side grasp (xy-planes of `katana_motor5_wrist_roll_link` and of `katana_base_link` are parallel):
  //     - standard: `rpy = (0, 0, *)` (orientation of `katana_motor5_wrist_roll_link` in `katana_base_link` frame)
  //     - overhead: `rpy = (pi, 0, *)`
  transform.setOrigin(tf::Vector3(x, y, z));

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
      transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw + atan2(y, x)));
      grasps.push_back(tf_transform_to_grasp(transform * standoff_trans));

      if (yaw != 0.0)
      {
        transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, -yaw + atan2(y, x)));
        grasps.push_back(tf_transform_to_grasp(transform * standoff_trans));
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
      transform.setOrigin(tf::Vector3(x, y, z));
      transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

      grasps.push_back(tf_transform_to_grasp(transform * standoff_trans));
    }
  }

  return grasps;
}

int main(int argc, char **argv) {
  ros::init (argc, argv, "calvin_pickdemo");
  ros::NodeHandle nh;
  ros::WallDuration(2.0).sleep();

  pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);

  moveit::planning_interface::MoveGroup group("arm");
  group.setPlanningTime(45.0);
  ros::WallDuration(2.0).sleep();

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper("gripper_grasp_posture_controller", true);
  control_msgs::GripperCommandGoal grippermsg_open;
  control_msgs::GripperCommandGoal grippermsg_close;
  grippermsg_open.command.position = 0.3;
  grippermsg_close.command.position = -0.44;
  gripper.waitForServer();
  ros::WallDuration(2.0).sleep();

  double x = 0.5;
  double y = 0.0;
  double z = 0.77;

  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";
  co.id = "testbox";
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.04;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.055;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.08;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = x;
  co.primitive_poses[0].position.y = y;
  co.primitive_poses[0].position.z = z;
  co.primitive_poses[0].orientation.w = 1.0;

  aco.object = co;
  aco.link_name = "katana_gripper_tool_frame";

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  //acm.setEntry(co.id, true);
  acm.setEntry(co.id, true);

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene.getPlanningSceneMsg(planning_scene_msg);
  planning_scene_publisher.publish(planning_scene_msg);

  /*
     <joint name="katana_motor1_pan_joint" />
     <joint name="katana_motor2_lift_joint" />
     <joint name="katana_motor3_lift_joint" />
     <joint name="katana_motor4_lift_joint" />
     <joint name="katana_motor5_wrist_roll_joint" />
     */
  std::vector<double> pose_approach(5,0.0);
  pose_approach[0] = -2.727076956299257e-05;
  pose_approach[1] = 0.4182292264006562;
  pose_approach[2] = -1.579831110699088;
  pose_approach[3] = 0.431614469864285;
  pose_approach[4] = -0.02369829874973517;

  std::vector<double> pose_pick(5,0.0);
  pose_pick[0] = -0.00628591238413545;
  pose_pick[1] = 0.1435515668297156;
  pose_pick[2] = -0.7481240933277338;
  pose_pick[3] = 0.9881426997091962;
  pose_pick[4] = -0.017439657135161823;

  std::vector<double> pose_retreat(5,0.0);
  pose_retreat[0] = -0.005917756995043266;
  pose_retreat[1] = 0.3214437215951924;
  pose_retreat[2] = -0.8062086273121478;
  pose_retreat[3] = 1.1077932011642755;
  pose_retreat[4] = -0.018053249450316056;

  std::vector<double> pose_storeapproach(5,0.0);
  pose_storeapproach[0] = 0.8376489578790283;
  pose_storeapproach[1] = 0.25826521367364896;
  pose_storeapproach[2] = -0.8052824502326694;
  pose_storeapproach[3] = 1.0005372644753114;
  pose_storeapproach[4] = -2.9736047130853387;

  std::vector<double> pose_store(5,0.0);
  pose_store[0] = 0.8394897348244914;
  pose_store[1] = 0.0920825834130028;
  pose_store[2] = -0.6937442676612324;
  pose_store[3] = 0.9464184222787067;
  pose_store[4] = -2.975322771567771;

  std::vector<double> pose_armaway(5,0.0);
  pose_armaway[0] = 0;
  pose_armaway[1] = 2.1354;
  pose_armaway[2] = -2.1556;
  pose_armaway[3] = -1.9719;
  pose_armaway[4] = 0;

  ros::WallDuration(5.0).sleep();

  add_collision_object();

  ros::WallDuration(5.0).sleep();

  //gripper.sendGoal(grippermsg_open);

  //remove_collision_object();
  //add_attached_collision_object();

  //gripper.sendGoal(grippermsg_close);

  ROS_INFO("Trying to pick");

  bool result = group.pick(co.id, generate_grasps(x, y, z));

  if (result) ROS_INFO("Pick was successful.");
  else ROS_WARN("Pick failed.");

  ros::WallDuration(5.0).sleep();

  ROS_INFO("Trying to store");

  group.setJointValueTarget(pose_storeapproach);
  group.asyncMove();

  ros::WallDuration(5.0).sleep();

  group.setJointValueTarget(pose_store);
  group.asyncMove();

  ros::WallDuration(5.0).sleep();

  gripper.sendGoal(grippermsg_open);

  ros::WallDuration(2.0).sleep();

  remove_attached_collision_object();

  ros::WallDuration(5.0).sleep();

  group.setJointValueTarget(pose_storeapproach);
  group.asyncMove();

  ros::WallDuration(5.0).sleep();

  group.setJointValueTarget(pose_armaway);
  group.asyncMove();

  ros::WallDuration(5.0).sleep();

  ROS_INFO("Done.");
}
