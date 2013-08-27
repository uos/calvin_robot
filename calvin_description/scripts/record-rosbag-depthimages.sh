#!/bin/bash -i
rosbag record /tf /scan_filtered /joint_states /kinect/depth_registered/image /kinect/rgb/camera_info /kinect/depth_registered/camera_info /kinect/rgb/image_color /imu/data /odom /robot_pose_ekf/odom_combined /camera_info /image_raw /amcl_pose

# Notes:
# * raw rgb + depth images         -- ca.  2 MB/frame --> up to 15 Hz
# * kinect/depth_registered/points -- ca. 10 MB/frame --> up to  3 Hz
# * include openni driver like this:
#      <include file="$(find openni_launch)/launch/openni.launch">
#        <arg name="camera" value="kinect" />
#        <arg name="publish_tf" value="false" />
#        <arg name="depth_registration" value="true" />
#      </include>
#      <param name="/kinect/driver/data_skip" value="2" />   # number of frames to skip, i.e. 0 = 30 Hz, 1 = 15 Hz, ..., 10 = 3 Hz
