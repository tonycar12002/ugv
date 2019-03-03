/**********************************
Author: Tony Hsiao
Date: 2018/07/06
Last update: 2018/08/31
Point Cloud Preprocess
Subscribe: 
  /velodyne_points      			(sensor_msgs/PointCloud2)
Publish:
  /velodyne_points_preprocess		(sensor_msgs/PointCloud2)
  /velodyne_points_odom				(sensor_msgs/PointCloud2)
  /boundary_marker					(visualization_msgs/Marker)
***********************************/ 
#include <iostream>
#include <vector>
#include <array>
#include <time.h>
#include <string>
#include <math.h>
//Ros Lib
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
//PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class PreprocessNode{
private:
	string node_name;
	bool visual;
	bool preprocessing_with_gps;
	double range_min;
	double range_max;
	double angle_min;
	double angle_max;
	double z_max;
	double z_min;
	double y_min;
	double x_min;

	int counts;

	ros::NodeHandle nh;
	ros::Subscriber sub_cloud;
	ros::Publisher	pub_cloud;
	ros::Publisher	pub_marker;

public:
	PreprocessNode(ros::NodeHandle&);
	void cbCloud(const sensor_msgs::PointCloud2ConstPtr&);
};
PreprocessNode::PreprocessNode(ros::NodeHandle &n){
	nh = n;
	counts = 0;
	node_name = ros::this_node::getName();

	visual = nh.param("visual", false);
	preprocessing_with_gps = nh.param("preprocessing_with_gps", true);

	range_min = 0.0;
	range_max = 30.0;
	angle_min = 0.0;
	angle_max = 180.0;
	z_min = -5.0;
	z_max = 5.0;
	y_min = 3.5;
	x_min = 1.5;

	//Read yaml file
	nh.getParam("range_min", range_min);
	nh.getParam("range_max", range_max);
	nh.getParam("angle_min", angle_min);
	nh.getParam("angle_max", angle_max);
	nh.getParam("z_min", z_min);
	nh.getParam("z_max", z_max);
	nh.getParam("y_min", y_min);
	nh.getParam("x_min", x_min);

	ROS_INFO("[%s] Initializing ", node_name.c_str());
	ROS_INFO("[%s] Param [visual] = %d", node_name.c_str(), visual);
	ROS_INFO("[%s] Param [preprocessing_with_gps] = %d", node_name.c_str(), preprocessing_with_gps);
	
	ROS_INFO("[%s] Param [range_max] = %f, [range_min] = %f", node_name.c_str(), range_max, range_min);
	ROS_INFO("[%s] Param [angle_max] = %f, [angle_min] = %f", node_name.c_str(), angle_max, angle_min);
	ROS_INFO("[%s] Param [x_min] = %f, [y_min] = %f", node_name.c_str(), x_min, y_min);
	ROS_INFO("[%s] Param [z_max] = %f, [z_min] = %f", node_name.c_str(), z_max, z_min);

	// Publisher
	pub_cloud = nh.advertise< sensor_msgs::PointCloud2 >("velodyne_points_preprocess", 1);
	pub_marker = nh.advertise< visualization_msgs::Marker >("boundary_marker", 1);

	// Subscriber
	sub_cloud = nh.subscribe("velodyne_points", 1, &PreprocessNode::cbCloud, this);

}
void PreprocessNode::cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	
	counts++ ;
	//return if no cloud data
	if ((cloud_msg->width * cloud_msg->height) == 0 || counts % 3 == 0)
		return ;
	const clock_t t_start = clock();
	
	// transfer ros msg to point cloud
	PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
	PointCloudXYZ::Ptr cloud_tmp(new PointCloudXYZ);
	PointCloudXYZ::Ptr cloud_odom(new PointCloudXYZ);
	
	pcl::fromROSMsg(*cloud_msg, *cloud_tmp);

	/*
	***************************************************************
		Remove out of range points and vehicle points
		Remove our of boundary points
	***************************************************************
	*/
	float dis, angle, num = 0;

	for (int i=0 ; i <  cloud_tmp->points.size() ; i++)
	{
		dis = cloud_tmp->points[i].x * cloud_tmp->points[i].x +
				cloud_tmp->points[i].y * cloud_tmp->points[i].y;
		dis = sqrt(dis);
		angle = atan2f(cloud_tmp->points[i].y, cloud_tmp->points[i].x);
		angle = angle * 180 / 3.1415;
		if (dis >= range_min && dis <= range_max && cloud_tmp->points[i].z >= z_min && z_max >= cloud_tmp->points[i].z) 
		{
			if(angle>=angle_min && angle<=angle_max && (abs(cloud_tmp->points[i].y) >= y_min or abs(cloud_tmp->points[i].x) >= x_min) ){
				cloud->points.push_back(cloud_tmp->points[i]);
				num ++;
			}
		
		}
	}	
	//cout << "==================================" << endl;
	cloud->width = num ;
	cloud->height = 1;
	cloud->points.resize(num);

	/*
	***************************************************************
		Voxel Grid Downsample
	***************************************************************
	*/	
	/*
	PointCloudXYZ::Ptr cloud_voxel(new PointCloudXYZ);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize (0.05f, 0.05f, 0.05f);
	vg.filter(*cloud_voxel);
	*/

	clock_t t_end = clock();
	//cout << "PointCloud preprocess time taken = " << (t_end-t_start)/(double)(CLOCKS_PER_SEC) << endl;

	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*cloud, cloud_out);
	cloud_out.header = cloud_msg->header;
	cloud_out.header.stamp = ros::Time::now();
	pub_cloud.publish(cloud_out);

}

int main(int argc, char **argv){
	ros::init (argc, argv, "pcl_preprocessing_node");
	ros::NodeHandle nh("~");
	PreprocessNode pn(nh);
	
	ros::spin ();
	return 0;
}