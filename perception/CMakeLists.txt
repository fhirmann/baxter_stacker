cmake_minimum_required(VERSION 2.8.3)
project(perception)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
)

catkin_python_setup()

## Declare ROS messages and services
add_message_files(
  DIRECTORY msg
  FILES Block.msg)

add_service_files(
  DIRECTORY srv
  FILES GetScene.srv)
 
## Generate added messages and services
generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)
 
## Declare a catkin package
## catkin_package()
 
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(filter src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

#add_executable(cloud_publisher src/cloud_publisher.cpp)
#target_link_libraries(cloud_publisher ${catkin_LIBRARIES})

install(PROGRAMS
  nodes/perception_test_service_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
