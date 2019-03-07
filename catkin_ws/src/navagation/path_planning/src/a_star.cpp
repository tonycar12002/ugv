#include <iostream>
#include <math.h>
#include <vector.h>
#include <map.h>
#include <time.h>
//ros library
#include "ros/ros.h"
#include "nav_msgs/Path"
#include "nav_msgs/Odometry"
#include "nav_msgs/OccupancyGrid"
#include "geometry_msgs/PoseStamped"

using namespace std;
class AStar{
private:
    string node_name;


public:
    AStar(){node_name="a_star";
        ROS_INFO("[%s] Initializing ", node_name.c_str());}
    ~AStar(){}

    vector<set>
};


class PathPlanning{
private:
    string node_name;

    ros::NodeHandler nh;
    ros::Publisher  pub_path;

    ros::Subscriber sub_map;
    ros::Subscriber sub_goal;
    ros::Subscriber sub_odom

public:
    PathPlanning(ros::NodeHandle&);
    void cb_obj_list(const robotx_msgs::ObjectPoseListConstPtr&);
    void cal_normal_avg(pcl::PointCloud<pcl::PointXYZINormal> );
};
PathPlanning::PathPlanning(ros::NodeHandle& n){
    nh = n;
    node_name = ros::this_node::getName();

	ROS_INFO("[%s] Initializing ", node_name.c_str());


    //Publisher
    pub_path = nh.advertise<geome::MarkerArray>("global_path", 100);

    //Subscriber
    sub_map = nh.subscribe("/obj_list/odom", 1, &PathPlanning::cb_obj_list, this);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh("~");
    PathPlanning pp(nh);
    
    ros::spin();
    return 0;
}