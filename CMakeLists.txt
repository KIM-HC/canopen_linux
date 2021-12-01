cmake_minimum_required(VERSION 3.0.0)
project(dyros_pcd_canopen)
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  actionlib_msgs
  actionlib
  control_msgs
)
# find_package(Eigen3 REQUIRED)

catkin_package(
  #DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
  # INCLUDE_DIRS include
  #LIBRARIES #Todo
  CATKIN_DEPENDS #Todo
)

#SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)