cmake_minimum_required(VERSION 2.8.3)
project(rospg)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS_DEBUG "-lpqxx -lpq")

find_package(catkin REQUIRED COMPONENTS
  kobuki_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED)

#add_action_files(
#  DIRECTORY action
# FILES MoveBase.action
#)

add_message_files(
  FILES
  TopologicalNavigationMap.msg
  TopoNavEdgeMsg.msg
  TopoNavNodeMsg.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs kobuki_msgs message_runtime
  #DEPENDS PCL Boost
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  /usr/local/include
  /opt/ros/indigo/include	
)


add_executable(pgsql_node src/pgsql_node.cpp
		          src/pgsql.cpp)  

add_executable(pose_publisher_node src/pose_publisher_node.cpp)


target_link_libraries(

   pgsql_node
   ${catkin_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libpqxx.so
  /usr/lib/libpq.so
  /opt/ros/indigo/lib/libtf.so
  
)

target_link_libraries(

   pose_publisher_node
   ${catkin_LIBRARIES}
   /opt/ros/indigo/lib/libtf.so
)
