#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <muffin_msgs/PickServerAction.h>
#include <tf/tf.h>
#include <moveit_msgs/Grasp.h>

class PickServer {
  protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<muffin_msgs::PickServerAction> *actionserver;
    moveit::planning_interface::MoveGroup *group;

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
      grasp.grasp_pose = pose;

      grasp.id = boost::lexical_cast<std::string>(i);

      grasp.pre_grasp_approach.direction.vector.z = 1.0;
      grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();
      grasp.pre_grasp_approach.direction.header.frame_id = "katana_gripper_tool_frame";
      grasp.pre_grasp_approach.min_distance = 0.05;
      grasp.pre_grasp_approach.desired_distance = 0.1;

      grasp.post_grasp_retreat.direction.vector.z = 1.0;
      grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();
      grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
      grasp.post_grasp_retreat.min_distance = 0.05;
      grasp.post_grasp_retreat.desired_distance = 0.1;

      // TODO: fill in grasp.post_place_retreat (copy of post_grasp_retreat?)

      grasp.pre_grasp_posture.joint_names.resize(1, "katana_l_finger_joint");
      grasp.pre_grasp_posture.points.resize(1);
      grasp.pre_grasp_posture.points[0].positions.resize(1);
      grasp.pre_grasp_posture.points[0].positions[0] = 0.1;
      // TODO: add katana_r_finger_joint

      grasp.grasp_posture.joint_names.resize(1, "katana_l_finger_joint");
      grasp.grasp_posture.points.resize(1);
      grasp.grasp_posture.points[0].positions.resize(1);
      grasp.grasp_posture.points[0].positions[0] = -0.44;
      // TODO: add katana_r_finger_joint
      // TODO: why isn't the gripper fully closed (to -0.44)?

      grasp.allowed_touch_objects.resize(1);
      grasp.allowed_touch_objects[0] = "testbox";

      i++;
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
  public:
    PickServer(std::string name) {
      group = new moveit::planning_interface::MoveGroup("arm");
      //group.setPlanningTime(45.0);
      actionserver = new actionlib::SimpleActionServer<muffin_msgs::PickServerAction>(nh, name, boost::bind(&PickServer::pick, this, _1), false);
      actionserver->start();
    }
    ~PickServer() {
      delete actionserver;
      delete group;
    }
    void pick(const muffin_msgs::PickServerGoalConstPtr &goal) {
      double x = goal->co.primitive_poses[0].position.x;
      double y = goal->co.primitive_poses[0].position.y;
      double z = goal->co.primitive_poses[0].position.z;
      std::string id = goal->co.id;
      ROS_INFO("Trying to pick object %s at %f, %f, %f.", id.c_str(), x, y, z);
      bool result = group->pick(id, generate_grasps(x, y, z));
    }
};

int main(int argc, char **argv) {
  ros::init (argc, argv, "calvin_pick_server");
  PickServer pickserver(ros::this_node::getName());
  ros::spin();
  return 0;
}
