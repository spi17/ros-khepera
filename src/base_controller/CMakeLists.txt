cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(base_controller src/base_controller.cpp)
target_link_libraries(base_controller ${catkin_LIBRARIES})


