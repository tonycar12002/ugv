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
//TF lib
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>

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

	vector<double>gps_left_up;	// (latitude, longitude, X, Y) latitude -> Y
	vector<double>gps_left_down;
	vector<double>gps_right_up;
	vector<double>gps_right_down;
	vector< array<double, 2> >boundary_list;

	int counts;

	ros::NodeHandle nh;
	ros::Subscriber sub_cloud;
	ros::Publisher	pub_cloud;
	//ros::Publisher 	pub_cloud_odom;
	//ros::Publisher 	pub_cloud_odom_preprocess;
	ros::Publisher	pub_marker;

	tf::TransformListener listener;

public:
	PreprocessNode(ros::NodeHandle&);
	void getBoundaryXY(vector<double>&, int);
	void cbCloud(const sensor_msgs::PointCloud2ConstPtr&);
	bool pointInBoundary(double, double);
	double product(double, double, double, double);
	void drawBoundary();
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

	nh.getParam("gps_left_up", gps_left_up);
	nh.getParam("gps_left_down", gps_left_down);
	nh.getParam("gps_right_up", gps_right_up);
	nh.getParam("gps_right_down", gps_right_down);

	if(preprocessing_with_gps){
		getBoundaryXY(gps_left_up, 0);
		getBoundaryXY(gps_left_down, 1);
		getBoundaryXY(gps_right_down, 2);
		getBoundaryXY(gps_right_up, 3);
	}

	ROS_INFO("[%s] Initializing ", node_name.c_str());
	ROS_INFO("[%s] Param [visual] = %d", node_name.c_str(), visual);
	ROS_INFO("[%s] Param [preprocessing_with_gps] = %d", node_name.c_str(), preprocessing_with_gps);
	
	ROS_INFO("[%s] Param [range_max] = %f, [range_min] = %f", node_name.c_str(), range_max, range_min);
	ROS_INFO("[%s] Param [angle_max] = %f, [angle_min] = %f", node_name.c_str(), angle_max, angle_min);
	ROS_INFO("[%s] Param [x_min] = %f, [y_min] = %f", node_name.c_str(), x_min, y_min);
	ROS_INFO("[%s] Param [z_max] = %f, [z_min] = %f", node_name.c_str(), z_max, z_min);

	if(preprocessing_with_gps){
		ROS_INFO("[%s] Param [gps_left_up] = (%f, %f), [X, Y] = (%f, %f)", node_name.c_str(), gps_left_up[0], gps_left_up[1], gps_left_up[2], gps_left_up[3]);
		ROS_INFO("[%s] Param [gps_left_down] = (%f, %f), [X, Y] = (%f, %f)", node_name.c_str(), gps_left_down[0], gps_left_down[1], gps_left_down[2], gps_left_down[3]);
		ROS_INFO("[%s] Param [gps_right_down] = (%f, %f), [X, Y] = (%f, %f)", node_name.c_str(), gps_right_down[0], gps_right_down[1], gps_right_down[2], gps_right_down[3]);
		ROS_INFO("[%s] Param [gps_right_up] = (%f, %f), [X, Y] = (%f, %f)", node_name.c_str(), gps_right_up[0], gps_right_up[1], gps_right_up[2], gps_right_up[3]);
	}

	// Publisher
	pub_cloud = nh.advertise< sensor_msgs::PointCloud2 >("velodyne_points_preprocess", 1);
	//pub_cloud_odom = nh.advertise< sensor_msgs::PointCloud2 >("velodyne_points_odom", 1);
	//pub_cloud_odom_preprocess = nh.advertise< sensor_msgs::PointCloud2 >("velodyne_points_odom_preprocess", 1);
	pub_marker = nh.advertise< visualization_msgs::Marker >("boundary_marker", 1);

	// Subscriber
	sub_cloud = nh.subscribe("velodyne_points", 1, &PreprocessNode::cbCloud, this);

}
void PreprocessNode::getBoundaryXY(vector<double> &gps_point, int num){
	/*
	***************************************************************
		Transform the boundary gps point to x, y coordinate of odom frame
	***************************************************************
	*/
	string source_frame="/utm";
	string target_frame="/odom";
	tf::StampedTransform transformStamped;
	bool find = false;
	while(!find){
		try{
			listener.waitForTransform(source_frame, target_frame, ros::Time(), ros::Duration(2.0) );
			listener.lookupTransform(source_frame, target_frame, ros::Time(), transformStamped);
			find = true;
		} 	
		catch (tf::TransformException ex) {
			ROS_INFO("[%s] Can't find transfrom betwen [%s] and [%s] ", node_name.c_str(), source_frame.c_str(), target_frame.c_str());		
		}
	}

	geographic_msgs::GeoPoint boundary;
	boundary.latitude = gps_point[0];
	boundary.longitude = gps_point[1];
	geodesy::UTMPoint utm_point;
	geodesy::fromMsg(boundary, utm_point);
	array<double, 2> point= {utm_point.easting - transformStamped.getOrigin().getX(), utm_point.northing - transformStamped.getOrigin().getY()};
	boundary_list.push_back(point);
	gps_point.push_back(point[0]);
	gps_point.push_back(point[1]);
}
void PreprocessNode::drawBoundary(){
	/*
	***************************************************************
		Draw boundary area
	***************************************************************
	*/
	visualization_msgs::Marker line_list;
	line_list.id = 10;
	line_list.header.stamp = ros::Time::now();
	line_list.header.frame_id = "/odom";
	line_list.pose.orientation.w = 1.0;
	line_list.ns = "points_and_lines";
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.scale.x = 0.1;
	line_list.color.a = 1.0;
	line_list.color.r = 1.0;
	for (int i=0; i<= boundary_list.size()-1 ;i++){
		geometry_msgs::Point p1, p2;
		p1.x = boundary_list[i].at(0);
		p1.y = boundary_list[i].at(1);
		p1.z = 0;
		if(i == boundary_list.size()-1){
			p2.x = boundary_list[0].at(0);
			p2.y = boundary_list[0].at(1);
			p2.z = 0;
		}
		else{
			p2.x = boundary_list[i+1].at(0);
			p2.y = boundary_list[i+1].at(1);
			p2.z = 0;
		}
		line_list.points.push_back(p1);
		line_list.points.push_back(p2);
	}
	pub_marker.publish(line_list);
}
bool PreprocessNode::pointInBoundary(double x, double y){
	/*
	***************************************************************
		Using product to confirm the points are inside the boundary area
	***************************************************************
	*/
	bool inside = true;
	for (int i=0; i< boundary_list.size() ;i++){
		array<double ,2>pre;
		array<double, 2>post = {boundary_list[i].at(0)-x, boundary_list[i].at(1)-y};
		if(i == 0){
			pre = {boundary_list[ boundary_list.size()-1 ].at(0)-x, boundary_list[ boundary_list.size()-1 ].at(1)-y};
		}
		else{
			pre = {boundary_list[ i-1 ].at(0)-x, boundary_list[ i-1 ].at(1)-y};
		}
		
		if(product(pre[0], pre[1], post[0], post[1]) < 0.0){
			inside = false;
			break;
		}
	}
	return inside;
}
double PreprocessNode::product(double v1_x, double v1_y, double v2_x, double v2_y){
	return (v1_x*v2_y - v2_x*v1_y);
}
void PreprocessNode::cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	
	counts++ ;
	//return if no cloud data
	if ((cloud_msg->width * cloud_msg->height) == 0 || counts % 3 == 0)
		return ;
	const clock_t t_start = clock();

	// Draw the boudary area
	if (visual && preprocessing_with_gps)
		drawBoundary();
	
	// transfer ros msg to point cloud
	PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
	PointCloudXYZ::Ptr cloud_tmp(new PointCloudXYZ);
	PointCloudXYZ::Ptr cloud_odom(new PointCloudXYZ);
	
	pcl::fromROSMsg(*cloud_msg, *cloud_tmp);
	pcl::fromROSMsg(*cloud_msg, *cloud_odom);

	string source_frame="/odom";
	string target_frame="/velodyne";
	tf::StampedTransform transformStamped;
	Eigen::Affine3d transform_eigen;
	if(preprocessing_with_gps){
		try{
			listener.waitForTransform(source_frame, target_frame, ros::Time(), ros::Duration(2.0) );
			listener.lookupTransform(source_frame, target_frame, ros::Time(), transformStamped);
			
			tf::transformTFToEigen(transformStamped, transform_eigen);
			pcl::transformPointCloud(*cloud_tmp, *cloud_odom, transform_eigen);
		} 	
		catch (tf::TransformException ex) {
			ROS_INFO("[%s] Can't find transfrom betwen [%s] and [%s] ", node_name.c_str(), source_frame.c_str(), target_frame.c_str());		
			return;
		}
	}

	/*
	***************************************************************
		Remove out of range points and WAM-V points
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
				//cout << cloud_tmp->points[i].x << " " << cloud_tmp->points[i].y << endl;
				if(preprocessing_with_gps && pointInBoundary(cloud_odom->points[i].x, cloud_odom->points[i].y)){
					cloud->points.push_back(cloud_tmp->points[i]);
					num ++;					
				}
				else if (!preprocessing_with_gps){
					cloud->points.push_back(cloud_tmp->points[i]);
					num ++;
				}
				else;
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
	pub_cloud.publish(cloud_out);

	/*
	if(visual && preprocessing_with_gps){

		sensor_msgs::PointCloud2 cloud_out2;
		pcl::toROSMsg(*cloud_odom, cloud_out2);
		cloud_out2.header = cloud_msg->header;
		cloud_out2.header.frame_id="/odom";
		pub_cloud_odom.publish(cloud_out2);

		pcl::transformPointCloud(*cloud, *cloud_tmp, transform_eigen);
		pcl::toROSMsg(*cloud_tmp, cloud_out2);
		cloud_out2.header = cloud_msg->header;
		cloud_out2.header.frame_id="/odom";
		pub_cloud_odom_preprocess.publish(cloud_out2);
	}
	*/
}

int main(int argc, char **argv){
	ros::init (argc, argv, "pcl_preprocessing_node");
	ros::NodeHandle nh("~");
	PreprocessNode pn(nh);
	
	ros::spin ();
	return 0;
}