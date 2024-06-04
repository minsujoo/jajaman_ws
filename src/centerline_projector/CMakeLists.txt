cmake_minimum_required(VERSION 3.0.2)
project(centerline_projector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(centerline_projector_node src/centerline_projector_node.cpp src/csv_reader.cpp)
target_link_libraries(centerline_projector_node ${catkin_LIBRARIES})
