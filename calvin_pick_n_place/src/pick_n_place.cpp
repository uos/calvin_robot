#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <tf/transform_listener.h>

static const std::string ROBOT_DESCRIPTION="robot_description";


class PickNPlace
{
public:
  PickNPlace(moveit::planning_interface::MoveGroup &group): group_(group), place_resolution_(0.1), listen_tables_(false)
  {
    table_dirty_ = false;
    table_subscriber_ = node_handle_.subscribe("table_array", 1, &PickNPlace::tableCallback, this);
    visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualize_place", 20, true);
    collision_object_publisher_ = node_handle_.advertise<moveit_msgs::CollisionObject>("/collision_object", 20);
  }

  bool pick(const std::string &object)
  {
    return group_.pick(object);
  }

  void visualize(const std::vector<geometry_msgs::PoseStamped> &poses) const
  {
    ROS_DEBUG("Visualizing: %d place poses", (int) poses.size());
    visualization_msgs::MarkerArray marker;
    for(std::size_t i=0; i < poses.size(); ++i)
    {
      visualization_msgs::Marker m;
      m.action = m.ADD;
      m.type = m.SPHERE;
      m.ns = "place_locations";
      m.id = i;
      m.pose = poses[i].pose;
      m.header = poses[i].header;

      m.scale.x = 0.02;
      m.scale.y = 0.02;
      m.scale.z = 0.02;

      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;

      marker.markers.push_back(m);
    }
    visualization_publisher_.publish(marker);
  }
  
  bool setupTables()
  {
    boost::mutex::scoped_lock tlock(table_lock_);
    if(table_array_.tables.empty())
    {
      ROS_ERROR_STREAM("No tables found to place object on");
      return false;
    }
    moveit_msgs::CollisionObject co;
    
    // Remove the existing table
    co.id = "table";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    collision_object_publisher_.publish(co);

    // Add the new table
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    co.meshes.push_back(table_array_.tables[0].convex_hull);
    co.mesh_poses.push_back(table_array_.tables[0].pose.pose);
    co.header = table_array_.tables[0].pose.header;
    collision_object_publisher_.publish(co);
    return true;
  }

  void clear()
  {
    moveit_msgs::CollisionObject co;
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    collision_object_publisher_.publish(co);
  }

  bool place(const std::string &object, double height_above_table)
  {
    std::vector<geometry_msgs::PoseStamped> poses;
    ROS_DEBUG("Finding possible place positions");
    findPossiblePlacePoses(poses, height_above_table);
    visualize(poses);
    if(!setupTables())
      return false;
    group_.setSupportSurfaceName("table");
    return group_.place(object, poses);
  }

  std::vector<geometry_msgs::PoseStamped> generatePlacePoses(const object_recognition_msgs::TableArray &table_array,
                                  double resolution,
                                  double height_above_table,
                                  double min_distance_from_edge = 0.10) const
  {
    std::vector<geometry_msgs::PoseStamped> place_poses;
    // Assumption that the table's normal is along the Z axis
    for(std::size_t i=0; i < table_array.tables.size(); ++i)
    {
      if(table_array.tables[i].convex_hull.vertices.empty())
        continue;
      const int scale_factor = 100;
      std::vector<cv::Point2f> table_contour;
      for(std::size_t j=0; j< table_array.tables[i].convex_hull.vertices.size(); ++j)
        table_contour.push_back(cv::Point((table_array.tables[i].convex_hull.vertices[j].x - table_array.tables[i].x_min) * scale_factor,
                  (table_array.tables[i].convex_hull.vertices[j].y - table_array.tables[i].y_min) * scale_factor));

      double x_range = fabs(table_array.tables[i].x_max - table_array.tables[i].x_min);
      double y_range = fabs(table_array.tables[i].y_max - table_array.tables[i].y_min);
      int max_range = (int) x_range + 1;
      if(max_range < (int) y_range + 1)
        max_range = (int) y_range + 1;

      int image_scale = std::max<int>(max_range, 4);
      cv::Mat src = cv::Mat::zeros(image_scale * scale_factor, image_scale * scale_factor, CV_8UC1);

      for(std::size_t j = 0; j < table_array.tables[i].convex_hull.vertices.size(); ++j)
      {
        cv::line(src, table_contour[j], table_contour[(j+1) % table_array.tables[i].convex_hull.vertices.size()],
              cv::Scalar(255), 3, 8);
      }

      unsigned int num_x = fabs(table_array.tables[i].x_max - table_array.tables[i].x_min) / resolution + 1;
      unsigned int num_y = fabs(table_array.tables[i].y_max - table_array.tables[i].y_min) / resolution + 1;

      ROS_DEBUG("Num points for possible place operations: %d %d", num_x, num_y);

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

      for(std::size_t j=0; j < num_x; ++j)
      {
        int point_x = j * resolution * scale_factor;
        for(std::size_t k=0; k < num_y; ++k)
        {
          int point_y = k * resolution * scale_factor;
          cv::Point2f point(point_x, point_y);
          double result = cv::pointPolygonTest(contours[0], point, true);
          if((int) result >= (int) (min_distance_from_edge* scale_factor))
          {
            tf::Point point((double) (point_x) / scale_factor + table_array.tables[i].x_min,
                (double) (point_y) / scale_factor + table_array.tables[i].y_min,
                0.0);
            tf::Pose pose;
            tf::poseMsgToTF(table_array.tables[i].pose.pose, pose);
            point = pose * point;

            geometry_msgs::PoseStamped place_pose;
            place_pose.pose.orientation.w = 1.0;
            place_pose.pose.position.x = point.x();
            place_pose.pose.position.y = point.y();
            place_pose.pose.position.z = point.z() + height_above_table;
            place_pose.header = table_array.tables[i].pose.header;
            place_poses.push_back(place_pose);

          }
        }
      }
    }
    visualize(place_poses);
    return place_poses;
  }

  void findPossiblePlacePoses(std::vector<geometry_msgs::PoseStamped> &poses, double height_above_table)
  {
    if(table_dirty_)
    {
      boost::mutex::scoped_lock tlock(table_lock_);
      ROS_DEBUG("Generating place poses");
      place_poses_ = generatePlacePoses(table_array_, place_resolution_, height_above_table);
    }
    table_dirty_ = false;
    poses = place_poses_;
  }

  void tableCallback(const object_recognition_msgs::TableArrayPtr &msg)
  {
    if(!listen_tables_)
      return;
    boost::mutex::scoped_lock tlock(table_lock_);
    ROS_DEBUG("Table callback");
    table_array_ = *msg;
    transformTableArray(table_array_);
    table_dirty_ = true;
  }

  void transformTableArray(object_recognition_msgs::TableArray &table_array)
  {
    for(std::size_t i=0; i < table_array.tables.size(); ++i)
    {
      std::string original_frame = table_array.tables[i].pose.header.frame_id;
      std::string root_frame = group_.getPoseReferenceFrame();
      if(table_array.tables[i].convex_hull.vertices.empty())
        continue;
      std::string error_text;
      if(!tf_.waitForTransform(root_frame, original_frame, table_array.tables[i].pose.header.stamp,
                              ros::Duration(0.5), ros::Duration(0.01), &error_text))
        {
          ROS_ERROR_STREAM("TF error: " << error_text);
          continue;
        }

      tf_.transformPose(root_frame, table_array.tables[i].pose, table_array.tables[i].pose);
      ROS_DEBUG_STREAM("Successfully transformed table array from " << original_frame << " to " << root_frame);
    }
  }

  bool listen_tables_;
  tf::TransformListener tf_;
  ros::NodeHandle node_handle_;
  bool table_dirty_;
  moveit::planning_interface::MoveGroup& group_;
  ros::Subscriber table_subscriber_;
  double place_resolution_;
  object_recognition_msgs::TableArray table_array_;
  std::vector<geometry_msgs::PoseStamped> place_poses_;
  boost::mutex table_lock_;
  ros::Publisher visualization_publisher_, collision_object_publisher_;        

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "katana_pick_n_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::WallDuration(2.0).sleep();

  moveit::planning_interface::PlanningSceneInterface scene_interface;
  moveit::planning_interface::MoveGroup group("arm");
  group.setPlanningTime(45);

  PickNPlace pg(group);
  ros::WallDuration(2.0).sleep();

  std::vector<double> joint_values(5, 0.0);
  joint_values[0] = -2.01;
  joint_values[1] = 0.737;
  joint_values[2] = -2.00;
  joint_values[3] = -2.00;
  joint_values[4] = -1.184;

  group.setJointValueTarget(joint_values);
  group.asyncMove();
  pg.listen_tables_ = true;
  ros::WallDuration(3.0).sleep();
  pg.listen_tables_ = false;

  const static unsigned int START = 0;
  const static unsigned int PICK = 1;
  const static unsigned int PLACE = 2;
  const static unsigned int CLEAR = 3;
  unsigned int try_place = 0;
  unsigned int state = START;

  std::vector<std::string> objects;
  while(ros::ok())
  {
    if(state == START)
    {
      objects = scene_interface.getKnownObjectNamesInROI(0.3, -0.5, 0.6, 0.7, 0.5, 0.9, true);
      if(objects.empty())
      {
        ROS_INFO("Could not find recognized object in workspace");
        group.setJointValueTarget(joint_values);
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
        state = PLACE;
      }
      else
      { 
        state = START;
        continue;
      }
    }
    else if (state == PLACE)
    {
      if(pg.place(objects[0], 0.04 + 0.01 * try_place))
      {
        try_place = 0;
        ROS_INFO("Done place");
        state = CLEAR;
      }
      else
      {
        try_place++;
        ROS_INFO("Trying place: %d of %d times", try_place, 6);
      }
      if(try_place > 6)
      {
        ROS_ERROR("Could not place object: HELP");
        break;
      }
    }
    else if (state == CLEAR)
    {
      group.setJointValueTarget(joint_values);
      group.move();
      pg.clear();
      pg.listen_tables_ = true;
      ros::WallDuration(3.0).sleep();
      pg.listen_tables_ = false;
      state = START;
    }
  }


}
