cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_examples)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  crustcrawler_core_msgs
  crustcrawler_interface
  cv_bridge
  dynamic_reconfigure
  roscpp
  rospy
  sensor_msgs
  trajectory_msgs
  xacro
)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  xacro
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  crustcrawler_core_msgs
  crustcrawler_interface
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
