cmake_minimum_required(VERSION 2.8.3)
project(calvin_gazebo)

find_package(catkin REQUIRED  calvin_description
                              calvin_joint_commander
                              gazebo_ros
                              katana
                              katana_arm_gazebo
                              controller_manager
                              robot_pose_ekf
                              robot_state_publisher
                              rostopic
                              uos_gazebo_worlds
                              uos_common_urdf
                              volksbot_driver
                              katana_description
                              sick_tim)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS  calvin_description
                  calvin_joint_commander
                  gazebo_ros
                  katana
                  katana_arm_gazebo
                  controller_manager
                  robot_pose_ekf
                  robot_state_publisher
                  rostopic
                  uos_gazebo_worlds
                  uos_common_urdf
                  volksbot_driver
                  katana_description
                  sick_tim
  INCLUDE_DIRS
  LIBRARIES
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
