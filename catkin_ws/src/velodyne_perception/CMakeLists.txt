cmake_minimum_required(VERSION 2.8.3)
project(velodyne_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geographic_msgs
  geodesy
  tf
  tf_conversions
  message_generation
)

catkin_package(

)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pcl_preprocessing src/pcl_preprocessing.cpp)
target_link_libraries(pcl_preprocessing ${catkin_LIBRARIES})