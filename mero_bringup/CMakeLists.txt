cmake_minimum_required(VERSION 2.8.3)
project(mero_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  urdf
  robot_state_publisher
  joint_state_publisher
  map_server
  amcl
  move_base
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  sensor_msgs 
  urdf 
  robot_state_publisher
  joint_state_publisher
  map_server
  amcl
  move_base
)


## Build 
include_directories(include ${catkin_INCLUDE_DIRS})

## Build executables
#add_executable(odom_publisher src/odom_publisher.cpp)
#target_link_libraries(odom_publisher ${catkin_LIBRARIES})

## Install 
#install(TARGETS my_robot_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
catkin_install_python(PROGRAMS
  nodes/mero_teleop_key
  nodes/gui_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install urdf files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
