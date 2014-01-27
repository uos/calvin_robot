#!/usr/bin/env python                                                           

import roslib; roslib.load_manifest('calvin_pick_server')                        
import rospy
import actionlib
import actionlib.msg
from calvin_msgs.msg import PickServerAction
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header

def main():
  goal = PickServerAction()
  goal.co = CollisionObject(
    header=Header(1, rospy.Time.now(), '/base_footprint'),
    id='testbox',
    primitives=[SolidPrimitive(type=1, dimensions=[0.04, 0.055, 0.08])],
    primitive_poses=[Pose(Point(0.5, 0.0, 0.77), Quaternion(0.0, 0.0, 0.0, 0.0))])

  try:
    move_group_client = actionlib.SimpleActionClient('/calvin_pick_server', PickServerAction)
    move_group_client.wait_for_server(rospy.Duration.from_sec(10.0))
    move_group_client.send_goal(goal)                                                                                                                                                                     
    move_group_client.wait_for_result(rospy.Duration.from_sec(30.0))        
                                                                               
    if move_group_client.get_state() != 3:                                  
      rospy.logwarn("move group client state is %s", move_group_client.get_state())

  except actionlib.ActionException, e:
    rospy.logerr("move_group action failed: %s", e)
