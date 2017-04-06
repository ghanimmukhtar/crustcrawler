cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_hw)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  dynamixel_driver
  dynamixel_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crustcrawler_hw
#  CATKIN_DEPENDS dynamixel_controllers roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
