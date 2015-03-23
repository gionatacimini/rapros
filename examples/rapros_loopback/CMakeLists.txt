cmake_minimum_required(VERSION 2.8.3)
project(rapros_loopback)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rospy_tutorials
  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)