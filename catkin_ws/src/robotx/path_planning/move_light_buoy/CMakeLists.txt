cmake_minimum_required(VERSION 2.8.3)
project(move_light_buoy)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  robotx_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

