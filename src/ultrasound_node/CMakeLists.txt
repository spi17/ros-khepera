cmake_minimum_required(VERSION 2.8.3)
project(ultrasound_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  genmsg
  tf
)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ultrasound_node src/ultrasound_node.cpp)
target_link_libraries(ultrasound_node ${catkin_LIBRARIES})