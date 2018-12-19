cmake_minimum_required(VERSION 2.8.3)
project(odometry_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node ${catkin_LIBRARIES})