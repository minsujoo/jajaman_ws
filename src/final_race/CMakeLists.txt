cmake_minimum_required(VERSION 3.0.2)
project(final_race)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ackermann_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES final_race
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


