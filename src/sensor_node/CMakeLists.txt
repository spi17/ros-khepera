cmake_minimum_required(VERSION 2.8.3)
project(sensor_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_node src/sensor_node.cpp)
target_link_libraries(sensor_node ${catkin_LIBRARIES})