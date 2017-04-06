cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_interface)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  roscpp
  rospy
  trajectory_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crustcrawler_interface
#  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs roscpp rospy trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_trajectory_action_server src/joint_trajectory_action_server.cpp)
target_link_libraries(joint_trajectory_action_server ${catkin_LIBRARIES})
