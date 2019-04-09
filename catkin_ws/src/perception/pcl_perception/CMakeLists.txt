cmake_minimum_required(VERSION 2.8.3)
project(pcl_perception)

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

add_executable(scan_to_localmap src/scan_to_localmap.cpp)
target_link_libraries(scan_to_localmap ${catkin_LIBRARIES})

add_executable(pcl_ground_segmentation src/pcl_ground_segmentation.cpp)
target_link_libraries(pcl_ground_segmentation ${catkin_LIBRARIES})

add_executable(save_pcd src/save_pcd.cpp)
target_link_libraries(save_pcd ${catkin_LIBRARIES})