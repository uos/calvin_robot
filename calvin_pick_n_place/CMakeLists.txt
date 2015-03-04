cmake_minimum_required(VERSION 2.8.3)
project(calvin_pick_n_place)

find_package(catkin REQUIRED COMPONENTS
                             roscpp
                             tf
                             actionlib
                             control_msgs
                             geometry_msgs
                             moveit_msgs
                             moveit_ros_planning_interface
                             object_recognition_msgs
                             shape_msgs
                             visualization_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp
                 tf
                 actionlib
                 control_msgs
                 geometry_msgs
                 moveit_msgs
                 moveit_ros_planning_interface
                 object_recognition_msgs
                 shape_msgs
                 visualization_msgs
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/calvin_pick_n_place.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${Eigen_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
