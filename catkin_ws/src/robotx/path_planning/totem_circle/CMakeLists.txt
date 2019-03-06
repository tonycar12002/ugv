cmake_minimum_required(VERSION 2.8.3)
project(totem_circle)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  robotx_msgs
  roscpp
  rospy
  robotx_msgs
  message_generation
)

add_service_files(
  FILES
  SetTotemCircle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

