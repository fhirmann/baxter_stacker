cmake_minimum_required(VERSION 2.8.3)
project(manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp	  
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  DIRECTORY srv
  FILES kmr19_pick_up.srv kmr19_put_down.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

install(PROGRAMS nodes/kmr19_manipulation_node.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

