#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

int counter = 0;

void cb(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud_in);
  std::string file_name = "map_" + std::to_string(counter) + ".pcd"; 
  pcl::io::savePCDFileASCII (file_name, *cloud_in);
  //++counter; 
  ROS_INFO("Message received: %d", counter);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_pcd_node");
  ros::Subscriber sub;
  ros::NodeHandle nh;
  sub = nh.subscribe("/loam/laser_cloud_surround", 1, cb);
  while(ros::ok()) ros::spinOnce();
  return 0;
}