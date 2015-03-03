cmake_minimum_required(VERSION 2.8.3)
project(calvin_joint_commander)

find_package(catkin REQUIRED  roscpp
                              dynamic_reconfigure
                              std_msgs
                              sensor_msgs)

find_package(PkgConfig REQUIRED)

generate_dynamic_reconfigure_options(cfg/JointCommander.cfg)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS  roscpp
                  dynamic_reconfigure
                  std_msgs
                  sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/joint_commander.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    
install(DIRECTORY docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
