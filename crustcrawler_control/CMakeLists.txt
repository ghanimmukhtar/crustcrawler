cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_control)
find_package(catkin REQUIRED
  moveit_ros_planning_interface
  roscpp
  rospy
  sensor_msgs
  std_msgs
  crustcrawler_core_msgs
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Boost REQUIRED COMPONENTS system )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

include_directories(include /usr/local/include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crustcrawler_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(crustcrawler_command_translator src/crustcrawler_command_translator.cpp)
target_link_libraries(crustcrawler_command_translator ${catkin_LIBRARIES})

add_executable(crustcrawler_command_translator_for_throwing src/crustcrawler_command_translator_for_throwing.cpp)
target_link_libraries(crustcrawler_command_translator_for_throwing ${catkin_LIBRARIES})

add_executable(crustcrawler_gripper src/crustcrawler_gripper.cpp)
target_link_libraries(crustcrawler_gripper ${catkin_LIBRARIES})
