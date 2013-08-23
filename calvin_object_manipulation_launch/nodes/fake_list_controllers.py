#!/usr/bin/env python
#
# fakes the `list_controllers` service (needed by `object_manipulator`).

import roslib; roslib.load_manifest('calvin_object_manipulation_launch')

from pr2_mechanism_msgs.srv import ListControllers, ListControllersResponse
import rospy

def handle_list_controllers(req):
    return ListControllersResponse(controllers = ["l_arm_controller"], state = ["running"])

def list_controllers_server():
    rospy.init_node('fake_list_controllers_server')
    s = rospy.Service('list_controllers', ListControllers, handle_list_controllers)
    print "fake_list_controllers_server initialized."
    rospy.spin()

if __name__ == "__main__":
    list_controllers_server()

