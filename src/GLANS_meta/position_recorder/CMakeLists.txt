cmake_minimum_required(VERSION 2.8.3)
project(position_recorder)

set(CMAKE_CXX_COMPILER "g++")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
	tf
)

find_package(Boost REQUIRED)

#add_action_files(
#  DIRECTORY action
# FILES MoveBase.action
#)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  #DEPENDS PCL Boost
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  /usr/local/include
  /opt/ros/indigo/include	
)


add_executable(position_recorder src/position_recorder.cpp)


target_link_libraries(
   position_recorder
   ${catkin_LIBRARIES}
   /opt/ros/indigo/lib/libtf.so
)

add_dependencies(position_recorder ${catkin_EXPORTED_TARGETS})
