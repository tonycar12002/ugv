#include <iostream>
#include <vector>
#include <time.h>
#include <string>
#include <math.h>
// Ros lib
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
// PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

class GroundSegmentation{
private:
    // varaible

    // ros
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud;

    ros::Publisher pub_ground;
    ros::Publisher pub_not_ground;

public:
    GroundSegmentation(ros::NodeHandle&);
    ~GroundSegmentation(){}

    void cbCloud(const sensor_msgs::PointCloud2ConstPtr&);

};
GroundSegmentation::GroundSegmentation(ros::NodeHandle& n){
    nh = n;

    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("cloud_ground", 1);
    pub_not_ground = nh.advertise<sensor_msgs::PointCloud2>("cloud_remove_ground", 1);
 
    sub_cloud = nh.subscribe("velodyne_points", 1, &GroundSegmentation::cbCloud, this);
}   

void GroundSegmentation::cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

}

int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_ground_segmeantation");

	ros::NodeHandle nh("~");
	GroundSegmentation gs(nh);
	
	ros::spin ();

    return 0;
}