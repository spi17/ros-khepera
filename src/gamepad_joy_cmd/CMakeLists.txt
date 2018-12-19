cmake_minimum_required(VERSION 2.8.3)
project(gamepad_joy_cmd)

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

add_executable(gamepad_robot_node src/gamepad_joy_cmd.cpp)
target_link_libraries(gamepad_robot_node ${catkin_LIBRARIES})


