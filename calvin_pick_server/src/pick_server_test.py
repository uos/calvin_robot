#!/usr/bin/env python

import rospy
import actionlib
import actionlib.msg
from calvin_msgs.msg import PickAndStoreAction, PickAndStoreGoal
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

if __name__ == '__main__':
  rospy.init_node('pick_server_test')

  co_pub= rospy.Publisher('/collision_object', CollisionObject, queue_size= 5)
  rospy.sleep(rospy.Duration.from_sec(1.5))

  co = CollisionObject(
    header=Header(1, rospy.Time.now(), '/base_footprint'),
    id='testbox',
    primitives=[SolidPrimitive(type=1, dimensions=[0.04, 0.055, 0.08])],
    primitive_poses=[Pose(Point(0.5, 0.0, 0.77), Quaternion(0.0, 0.0, 0.0, 1.0))])

  co_pub.publish( co )
  rospy.sleep(rospy.Duration.from_sec(5.0))

  try:
    move_group_client = actionlib.SimpleActionClient('/calvin_pick_server', PickAndStoreAction)
    move_group_client.wait_for_server(rospy.Duration.from_sec(10.0))
    move_group_client.send_goal( PickAndStoreGoal(co= co, close_gripper_partially= True) )
    move_group_client.wait_for_result(rospy.Duration.from_sec(30.0))

    if move_group_client.get_state() != 3:
      rospy.logwarn("move group client state is %s", move_group_client.get_state())

  except actionlib.ActionException, e:
    rospy.logerr("move_group action failed: %s", e)
