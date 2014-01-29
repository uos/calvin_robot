#include <ros/ros.h>

#include <tf/tf.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlanningScene.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <shape_tools/solid_primitive_dims.h>

ros::Publisher pub_co;
ros::Publisher pub_aco;
ros::Publisher planning_scene_publisher;
ros::Publisher grasps_marker;

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
  static int i = 0;

  moveit_msgs::Grasp grasp;
  geometry_msgs::PoseStamped pose;

  tf::Vector3& origin = t.getOrigin();
  tf::Quaternion rotation = t.getRotation();

  tf::quaternionTFToMsg(rotation, pose.pose.orientation);

  pose.header.frame_id = "base_footprint";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = origin.m_floats[0];
  pose.pose.position.y = origin.m_floats[1];
  pose.pose.position.z = origin.m_floats[2];
  //pose.pose.orientation.x = 0;
  //pose.pose.orientation.y = 0;
  //pose.pose.orientation.z = 0;
  //pose.pose.orientation.w = 1;
  grasp.grasp_pose = pose;

  grasp.id = boost::lexical_cast<std::string>(i);

  grasp.pre_grasp_approach.direction.vector.z = 1.0;
  grasp.pre_grasp_approach.direction.header.frame_id = "katana_gripper_tool_frame";
  grasp.pre_grasp_approach.min_distance = 0.05;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();

  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();

  // TODO: fill in grasp.post_place_retreat (copy of post_grasp_retreat?)

  grasp.pre_grasp_posture.joint_names.push_back("katana_l_finger_joint");
  grasp.pre_grasp_posture.joint_names.push_back("katana_r_finger_joint");
  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.push_back(0.3);
  grasp.pre_grasp_posture.points[0].positions.push_back(0.3);

  grasp.grasp_posture.joint_names.push_back("katana_l_finger_joint");
  grasp.grasp_posture.joint_names.push_back("katana_r_finger_joint");
  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.push_back(-0.44);
  grasp.grasp_posture.points[0].positions.push_back(-0.44);

  grasp.allowed_touch_objects.resize(1);
  grasp.allowed_touch_objects[0] = "testbox";

  i++;
  return grasp;
}

void publish_grasps_as_markerarray(std::vector<moveit_msgs::Grasp> grasps)
{
  visualization_msgs::MarkerArray markers;
  int i = 0;

  for(std::vector<moveit_msgs::Grasp>::iterator it = grasps.begin(); it != grasps.end(); ++it) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "base_footprint";
    marker.id = i;
    marker.type = 1;
    marker.ns = "graspmarker";
    marker.pose = it->grasp_pose.pose;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.2;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    markers.markers.push_back(marker);
    i++;
  }

  grasps_marker.publish(markers);
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

  publish_grasps_as_markerarray(grasps);

  return grasps;
}

int main(int argc, char **argv) {
  ros::init (argc, argv, "calvin_pickdemo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
  grasps_marker = nh.advertise<visualization_msgs::MarkerArray>("grasps_marker", 10);

  moveit::planning_interface::MoveGroup group("arm");
  group.setPlanningTime(45.0);

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

  // --- BEGIN allow collisions with testbox
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  //acm.setEntry(co.id, true);
  acm.setEntry(co.id, true);

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;   // TODO: this line should probably go after the next
  planning_scene.getPlanningSceneMsg(planning_scene_msg);
  planning_scene_publisher.publish(planning_scene_msg);
  ros::WallDuration(1.0).sleep();
  // FIXME: this currently just clears the planning scene
  // --- END allow collisions with testbox

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

  add_collision_object();

  ros::WallDuration(1.0).sleep();

  //gripper.sendGoal(grippermsg_open);

  //remove_collision_object();
  //add_attached_collision_object();

  //gripper.sendGoal(grippermsg_close);

  ROS_INFO("Trying to pick");

  // TODO: group.setSupportSurfaceName("table");     // needed to specify that attached object is allowed to touch table
  bool success = group.pick(co.id, generate_grasps(x, y, z));

  if (success)
    ROS_INFO("Pick was successful.");
  else
  {
    ROS_FATAL("Pick failed.");
    return EXIT_FAILURE;
  }

  ROS_INFO("Moving to store approach pose");
  group.setJointValueTarget(pose_storeapproach);
  success = group.move();

  ROS_INFO("Moving to store pose");
  group.setJointValueTarget(pose_store);
  success &= group.move();

  ROS_INFO("Opening gripper");
  moveit::planning_interface::MoveGroup gripper_group("gripper");
  gripper_group.setNamedTarget("open");
  success &= gripper_group.move();
  //gripper.sendGoal(grippermsg_open);

  // remove_attached_collision_object();
  group.detachObject();

  ros::WallDuration(1.0).sleep();

  ROS_INFO("Moving back to store approach pose");
  // TODO: allow collisions with "testbox"
  group.setJointValueTarget(pose_storeapproach);
  success &= group.move();

  ROS_INFO("Moving arm away");
  group.setJointValueTarget(pose_armaway);
  success &= group.move();

  if (success)
  {
    ROS_INFO("Done.");
    return EXIT_SUCCESS;
  }
  else
  {
    ROS_ERROR("One of the moves failed!");
    return EXIT_FAILURE;

  }
}
