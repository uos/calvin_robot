#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <calvin_pick_server/PickServerAction.h>

class PickServer {
  protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<calvin_pick_server::PickServerAction> *actionserver;
  public:
    PickServer(std::string name) {
      actionserver = new actionlib::SimpleActionServer<calvin_pick_server::PickServerAction>(nh, name, boost::bind(&PickServer::pick, this, _1), false);
      actionserver->start();
    }
    ~PickServer() {
      delete actionserver;
    }
    void pick(const calvin_pick_server::PickServerGoalConstPtr &goal) {
    }
};

int main(int argc, char **argv) {
  ros::init (argc, argv, "calvin_pick_server");
  PickServer pickserver(ros::this_node::getName());
  ros::spin();
  return 0;
}
