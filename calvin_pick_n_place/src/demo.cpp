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
  co.operation = moveit_msgs::CollisionObject::ADD;
  pub_co.publish(co);
}
void remove_collision_object() {
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);
}
void add_attached_collision_object() {
  aco.object.operation = moveit_msgs::CollisionObject::ADD;
  pub_aco.publish(aco);
}
void remove_attached_collision_object() {
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_aco.publish(aco);
}
                                                                                   
int main(int argc, char **argv) {                                                  
  ros::init (argc, argv, "calvin_pickdemo");                                       
  ros::NodeHandle nh;                                                              
  ros::WallDuration(2.0).sleep();

  pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);

  moveit::planning_interface::MoveGroup group("arm");                              
  group.setPlanningTime(10.0);                                                     
  ros::WallDuration(2.0).sleep();

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper("gripper_grasp_posture_controller", true);
  control_msgs::GripperCommandGoal grippermsg_open;
  control_msgs::GripperCommandGoal grippermsg_close;
  grippermsg_open.command.position = 0.3;
  grippermsg_close.command.position = -0.44;
  gripper.waitForServer();
  ros::WallDuration(2.0).sleep();

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
  co.primitive_poses[0].position.x = 0.5;
  co.primitive_poses[0].position.y = 0.0;
  co.primitive_poses[0].position.z = 0.77;
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

  group.setJointValueTarget(pose_approach);                                        
  group.asyncMove();                                                                    

  ros::WallDuration(5.0).sleep();

  gripper.sendGoal(grippermsg_open);

  ros::WallDuration(2.0).sleep();

  group.setJointValueTarget(pose_pick);                                            
  group.asyncMove();                                                                    

  ros::WallDuration(5.0).sleep();

  remove_collision_object();
  add_attached_collision_object();

  ros::WallDuration(5.0).sleep();

  gripper.sendGoal(grippermsg_close);

  ros::WallDuration(2.0).sleep();

  group.setJointValueTarget(pose_retreat);                                         
  group.asyncMove();                                                                    

  ros::WallDuration(5.0).sleep();

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
}  
