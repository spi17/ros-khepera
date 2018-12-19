cmake_minimum_required(VERSION 2.8.3)
project(speed_sensor)

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

add_executable(speed_sensor src/speed_sensor.cpp)
target_link_libraries(speed_sensor ${catkin_LIBRARIES})
