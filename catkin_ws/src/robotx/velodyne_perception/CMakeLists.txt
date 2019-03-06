cmake_minimum_required(VERSION 2.8.3)
project(velodyne_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  robotx_msgs
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


add_executable(cluster src/cluster.cpp)
target_link_libraries(cluster ${catkin_LIBRARIES})

add_executable(cluster_with_odom src/cluster_with_odom.cpp)
target_link_libraries(cluster_with_odom ${catkin_LIBRARIES})

add_executable(pcl_preprocessing src/pcl_preprocessing.cpp)
target_link_libraries(pcl_preprocessing ${catkin_LIBRARIES})

add_executable(cluster_no_preprocess src/cluster_no_preprocess.cpp)
target_link_libraries(cluster_no_preprocess ${catkin_LIBRARIES})

add_executable(pcl_cluster src/pcl_cluster.cpp)
target_link_libraries(pcl_cluster ${catkin_LIBRARIES})

add_executable(pcl_to_img src/pcl_to_img.cpp)
target_link_libraries(pcl_to_img ${catkin_LIBRARIES})

add_executable(dock_normal src/dock_normal.cpp)
target_link_libraries(dock_normal ${catkin_LIBRARIES})

add_executable(pcl_preprocessing_rviz src/pcl_preprocessing_rviz.cpp)
target_link_libraries(pcl_preprocessing_rviz ${catkin_LIBRARIES})