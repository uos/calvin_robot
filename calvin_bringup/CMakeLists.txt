cmake_minimum_required(VERSION 2.8.3)
project(calvin_bringup)

find_package(catkin REQUIRED COMPONENTS calvin_description
                             calvin_joint_commander
                             imu_filter_madgwick
                             katana
                             laser_filters
                             libuvc_camera
                             openni_launch
                             phidgets_imu
                             robot_pose_ekf
                             robot_state_publisher
                             sick_tim
                             volksbot_driver)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS calvin_description
                 calvin_joint_commander
                 imu_filter_madgwick
                 katana
                 laser_filters
                 libuvc_camera
                 openni_launch
                 phidgets_imu
                 robot_pose_ekf
                 robot_state_publisher
                 sick_tim
                 volksbot_driver
  INCLUDE_DIRS
  LIBRARIES
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
