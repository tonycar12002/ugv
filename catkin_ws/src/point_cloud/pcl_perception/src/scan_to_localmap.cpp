#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include <queue>
#include <time.h>
#include <algorithm>
#include <array>
//ros library
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Bool.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> syncPolicy;

class LocalMap2D{
private:
    double cell_size;
    double map_length;
    double wait_time;
    double vehicle_size;
    double drift_x;
    double drift_y;
    double cell_length;
    string frame_id;
    double frame_period;
    bool first_data_arrive;
    double vehicle_yaw;

    nav_msgs::Odometry odom;
    sensor_msgs::LaserScan scan;

    ros::NodeHandle nh;
    ros::Publisher pub_grid_map;
    ros::Timer timer;

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
    message_filters::Synchronizer<syncPolicy> *sync;

public:
    LocalMap2D(ros::NodeHandle&);

    void cbOdomAndScan(const nav_msgs::OdometryConstPtr&, const sensor_msgs::LaserScanConstPtr&);
    void CreateMap(const ros::TimerEvent&);
    vector<int> GetCellNumber(double scan_range, double rad, nav_msgs::MapMetaData map_data_info);
};

LocalMap2D::LocalMap2D(ros::NodeHandle& n){
    nh = n;

    cell_size = 0.5;
    map_length = 20;
    wait_time = 0.2;
    vehicle_size = 0.2;
    drift_x = 0.0;
    drift_y = 0.0;
    frame_period = 0.5;
    first_data_arrive = false;

    nh.getParam("cell_size", cell_size);
    nh.getParam("map_length", map_length);
    nh.getParam("wait_time", wait_time);
    nh.getParam("vehicle_size", vehicle_size);
    nh.getParam("drift_x", drift_x);
    nh.getParam("drift_y", drift_y);
    nh.getParam("cell_length", cell_length);
    nh.getParam("frame_period", frame_period);
    cell_length = int(map_length/cell_size*2);

    pub_grid_map = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

    timer = nh.createTimer(ros::Duration(frame_period), &LocalMap2D::CreateMap, this);

    //Subscriber
    sub_scan.subscribe(nh, "scan", 1);
    sub_odom.subscribe(nh, "odom", 1);

    sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(2),  sub_odom, sub_scan);
    sync->registerCallback(boost::bind(&LocalMap2D::cbOdomAndScan, this, _1, _2));


}
void LocalMap2D::CreateMap(const ros::TimerEvent& event){
    if (!first_data_arrive){
        return;
    }

    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.header.frame_id = odom.header.frame_id;

    // Map data information
    nav_msgs::MapMetaData map_data_info;
    map_data_info.resolution = cell_size;               // [m/cell]
    map_data_info.width = cell_length;                  // [cells]
    map_data_info.height = cell_length;

    map_data_info.origin.position.x    = odom.pose.pose.position.x - map_length;
    map_data_info.origin.position.y    = odom.pose.pose.position.y - map_length;
    map_data_info.origin.position.z    = 0;
    map_data_info.origin.orientation.x = 0;
    map_data_info.origin.orientation.y = 0;
    map_data_info.origin.orientation.z = 0;
    map_data_info.origin.orientation.w = 1.0;
    occupancy_grid.info = map_data_info ;

    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    vehicle_yaw = tf::getYaw(pose.getRotation());

    
    vector<signed char> data(map_data_info.height*map_data_info.width);
    for(int i = 0 ; i< map_data_info.height*map_data_info.width; i++){
        data[i] = 0;
    }

    for(int laser=0;laser<scan.ranges.size();laser++){
        double scan_range = scan.ranges[laser];
        double rad = scan.angle_min + laser * scan.angle_increment + vehicle_yaw ;

        // remove points out of range 
        if (!scan_range || scan_range < scan.range_min || scan_range > scan.range_max || scan_range > map_length - 1) 
            continue;

        vector<int>cell_num;
        cell_num = GetCellNumber(scan_range, rad, map_data_info);
        for (int i = 0; i < cell_num.size() ; ++i)
            data[cell_num[i]] = 100;
    }

    occupancy_grid.data = data;;
    pub_grid_map.publish(occupancy_grid);

}

vector<int> LocalMap2D::GetCellNumber(double scan_range, double rad, nav_msgs::MapMetaData map_data_info){
    double scan_x = scan_range * cos(rad); 
    double scan_y = scan_range * sin(rad); 

    double dis_x = odom.pose.pose.position.x - map_data_info.origin.position.x + drift_x * cos(vehicle_yaw ) ;
    double dis_y = odom.pose.pose.position.y - map_data_info.origin.position.y + drift_y * sin(vehicle_yaw ) ;

    scan_x = scan_x + dis_x;
    scan_y = scan_y + dis_y;

    double cell_vehicle = int(floor(vehicle_size/cell_size));
    int cell_x = int(floor(scan_x / cell_size)) ;
    int cell_y = int(floor(scan_y / cell_size)) ;    

    vector<int>cell_list;
    for(int x = -cell_vehicle ; x<=cell_vehicle ; x++){
        for(int y = -cell_vehicle ; y<=cell_vehicle ; y++){
            double tmp_y = cell_y + y;
            double tmp_x = cell_x + x;
            int data = tmp_y * map_data_info.width + tmp_x;
            cell_list.push_back(data);
        }
    }
    return cell_list;
}

void LocalMap2D::cbOdomAndScan(const nav_msgs::OdometryConstPtr& msg_odom, const sensor_msgs::LaserScanConstPtr& msg_scan){
    odom = *msg_odom;
    scan = *msg_scan;
    first_data_arrive = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LocalMap2D");
    ros::NodeHandle nh("~");
    LocalMap2D lc(nh);
    
    ros::spin();
    return 0;
}
