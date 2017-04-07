cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_core_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  EndpointState.msg
  JointCommand.msg
  RobustControllerStatus.msg
  EndEffectorCommand.msg
  EndEffectorState.msg
)

add_service_files(DIRECTORY srv FILES
  SolvePositionIK.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
)
