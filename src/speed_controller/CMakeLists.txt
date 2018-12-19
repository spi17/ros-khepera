cmake_minimum_required(VERSION 2.8.3)
project(speed_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(speed_controller src/speed_controller.cpp)
target_link_libraries(speed_controller ${catkin_LIBRARIES})


