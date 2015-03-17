cmake_minimum_required(VERSION 2.8.3)
project(calvin_pick_server)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	moveit_ros_planning_interface
	actionlib
	shape_tools

	muffin_msgs
	shape_msgs
	moveit_msgs
	visualization_msgs
)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	tf
	moveit_ros_planning_interface
	actionlib
	shape_tools

	muffin_msgs
	shape_msgs
	moveit_msgs
	visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/pick_server.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
