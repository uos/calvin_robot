cmake_minimum_required(VERSION 2.8.3)

project(calvin_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg message_generation actionlib_msgs moveit_msgs)

add_action_files(DIRECTORY action FILES PickAndStore.action)

generate_messages(DEPENDENCIES actionlib_msgs moveit_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs moveit_msgs message_runtime)
