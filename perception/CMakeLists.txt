cmake_minimum_required(VERSION 2.8.3)
project(perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)
 
## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)
 
## Declare a catkin package
catkin_package()
 
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(filter src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(cloud_publisher src/cloud_publisher.cpp)
target_link_libraries(cloud_publisher ${catkin_LIBRARIES})
