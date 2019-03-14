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
//opencv library
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
using namespace std;


class GroundSegmentation{
private:
    // varaible
    // VLP16 information
    const int N_SCAN = 16.0;
    const int Horizon_SCAN = 1800; // 360/0.2=1800
    double angle_res_x = 0.2;
    double angle_res_y = 30.0/(N_SCAN-1);
    double angle_bottom = 15;
    double cloud_angle_th = 10.0;
    int ground_scan = 8;

    // ros
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud;

    ros::Publisher pub_ground;
    ros::Publisher pub_remove_ground;

public:
    GroundSegmentation(ros::NodeHandle&);
    ~GroundSegmentation(){}

    void cbCloud(const sensor_msgs::PointCloud2ConstPtr&);

};
GroundSegmentation::GroundSegmentation(ros::NodeHandle& n){
    nh = n;

    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("cloud_ground", 1);
    pub_remove_ground = nh.advertise<sensor_msgs::PointCloud2>("cloud_remove_ground", 1);
 
    sub_cloud = nh.subscribe("velodyne_points", 1, &GroundSegmentation::cbCloud, this);
}   

void GroundSegmentation::cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    
    if(cloud_msg->width * cloud_msg->height == 0){
        return;
    }
    const clock_t t_start = clock();

    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
    PointCloudXYZ::Ptr cloud_full(new PointCloudXYZ);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointXYZ nan_point;
    nan_point.x = numeric_limits<float>::quiet_NaN();
    nan_point.y = numeric_limits<float>::quiet_NaN();
    nan_point.z = numeric_limits<float>::quiet_NaN();
    int* point_exist = new int[N_SCAN*Horizon_SCAN];
    memset(point_exist, 0, sizeof(point_exist));

    cloud_full->points.resize(N_SCAN*Horizon_SCAN);
    fill(cloud_full->points.begin(), cloud_full->points.end(), nan_point);

    /*
	***************************************************************
		Reorganize pointcloud data to N_SCAN * resolution
	***************************************************************
	*/
    int cloud_size = cloud->points.size();

    // calculate every point angle
    pcl::PointXYZ point;
    double vertical_angle, horizontal_angle, range;
    int row_index, column_index;

    for(int i = 0 ; i<cloud_size ; i++){
        point = cloud->points[i];
        vertical_angle = atan2(point.z, sqrt(point.x*point.x+point.y*point.y)) * 180 / M_PI;
        row_index = round((vertical_angle + angle_bottom) / angle_res_y);

        // remove impossible scan id
        if(row_index <0 || row_index >= N_SCAN){
            cout << row_index <<  " " << vertical_angle << " " << point.z << " " << point.x << " " << point.y << endl;
            continue;
        }

        horizontal_angle = atan2(point.x, point.y) * 180 / M_PI;
        column_index = -round((horizontal_angle-90.0)/angle_res_x) + Horizon_SCAN;

        if (column_index >= Horizon_SCAN)
            column_index -= Horizon_SCAN;

        if (column_index < 0 || column_index >= Horizon_SCAN){
            continue;
        }
        
        range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        int index = column_index  + row_index * Horizon_SCAN;
        cloud_full->points[index] = point;
        point_exist[index] = 1;

    }

    /*
	***************************************************************
		Ground remove
	***************************************************************
	*/
    PointCloudXYZ::Ptr cloud_ground(new PointCloudXYZ);
    PointCloudXYZ::Ptr cloud_remove_ground(new PointCloudXYZ);
    int cloud_groud_size = 0, cloud_remove_ground_size = 0;
    int lower, upper;
    double dif_X, dif_Y, dif_Z, angle;
    int* ground_point = new int[N_SCAN*Horizon_SCAN];
    memset(ground_point, 0, sizeof(ground_point));

    for (int j = 0; j < Horizon_SCAN; j++){
        for (int i = 0; i < ground_scan; i++){
            lower = j + i * Horizon_SCAN;
            upper = j + (i+1) * Horizon_SCAN;
            //cout << i << " " << j << " " << lower << endl;
            point = cloud_full->points[lower];
            if(point_exist[lower] == 0 || point_exist[upper] == 0)
                continue;

            dif_X = cloud_full->points[upper].x - cloud_full->points[lower].x;
            dif_Y = cloud_full->points[upper].y - cloud_full->points[lower].y;
            dif_Z = cloud_full->points[upper].z - cloud_full->points[lower].z;
            angle = atan2(dif_Z, sqrt(dif_X*dif_X + dif_Y*dif_Y) ) * 180 / M_PI;

            if(angle <= cloud_angle_th){
                // is ground
                ground_point[lower] = 1;
                ground_point[upper] = 1;
            }
        }
    }
    //cout << "------------" << endl;
    for(int i=0;i<N_SCAN*Horizon_SCAN;i++){
        point = cloud_full->points[i];
        if(ground_point[i] == 1){
            cloud_groud_size += 1;
            cloud_ground->points.push_back(point);
        }
        else if(ground_point[i] == 0 && point_exist[i] == 1){
            cloud_remove_ground_size += 1;
            cloud_remove_ground->points.push_back(point);
        }
        else;
    }
    //cout << "11------------" << endl;
    cloud_ground->width = cloud_groud_size ;
    cloud_ground->height = 1;
    cloud_ground->points.resize(cloud_groud_size);

    cloud_remove_ground->width = cloud_remove_ground_size ;
    cloud_remove_ground->height = 1;
    cloud_remove_ground->points.resize(cloud_remove_ground_size);

    sensor_msgs::PointCloud2 cloud_ground_msg;
	pcl::toROSMsg(*cloud_ground, cloud_ground_msg);
	cloud_ground_msg.header = cloud_msg->header;
	cloud_ground_msg.header.stamp = ros::Time::now();
	pub_ground.publish(cloud_ground_msg);

    sensor_msgs::PointCloud2 cloud_remove_ground_msg;
	pcl::toROSMsg(*cloud_remove_ground, cloud_remove_ground_msg);
	cloud_remove_ground_msg.header = cloud_msg->header;
	cloud_remove_ground_msg.header.stamp = ros::Time::now();
	pub_remove_ground.publish(cloud_remove_ground_msg);

    delete point_exist;
    delete ground_point;

    
    clock_t t_end = clock();
	//cout << "PointCloud ground segmentation time taken = " << (t_end-t_start)/(double)(CLOCKS_PER_SEC) << endl;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_ground_segmeantation");

	ros::NodeHandle nh("~");
	GroundSegmentation gs(nh);
	
	ros::spin ();

    return 0;
}