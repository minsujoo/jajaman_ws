cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pure_pursuit
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare executables and link libraries
add_executable(lookahead_point_generator src/lookahead_point_generator.cpp src/load_centerline_from_csv.cpp)
target_link_libraries(lookahead_point_generator ${catkin_LIBRARIES})

add_executable(projected_point_publisher src/projected_point_publisher.cpp)
target_link_libraries(projected_point_publisher ${catkin_LIBRARIES})

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})