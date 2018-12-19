cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning)

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

add_executable(dead_reckoning src/dead_reckoning.cpp)
target_link_libraries(dead_reckoning ${catkin_LIBRARIES})