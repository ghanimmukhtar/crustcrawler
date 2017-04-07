cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_control)
find_package(catkin REQUIRED
  roscpp
  rospy
  sensor_msgs
  std_msgs
  crustcrawler_core_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crustcrawler_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(crustcrawler_command_translator src/crustcrawler_command_translator.cpp)
target_link_libraries(crustcrawler_command_translator ${catkin_LIBRARIES})

add_executable(crustcrawler_gripper src/crustcrawler_gripper.cpp)
target_link_libraries(crustcrawler_gripper ${catkin_LIBRARIES})
