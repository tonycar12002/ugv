#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include <time.h>
#include <algorithm>
#include <array>
#include "a_star.h"
//ros library
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

using namespace std;
class PathPlanning{
private:
    string node_name;
    bool receive_goal;
    double update_time;
    double vehicle_size;

    nav_msgs::Odometry odom;
    nav_msgs::OccupancyGrid map;
    geometry_msgs::PoseStamped goal;

    ros::NodeHandle nh;
    ros::Publisher  pub_path;

    ros::Subscriber sub_map;
    ros::Subscriber sub_goal;
    ros::Subscriber sub_odom;

    AStar a_star;

    vector<array<double, 2> >pose_array;

    ros::Timer timer;

public:
    PathPlanning(ros::NodeHandle&);
    void cbOdom(const nav_msgs::Odometry&);
    void cbMap(const nav_msgs::OccupancyGrid&);
    void cbGoal(const geometry_msgs::PoseStamped&);
    void Planning(const ros::TimerEvent&);
};
PathPlanning::PathPlanning(ros::NodeHandle& n){
    nh = n;
    node_name = ros::this_node::getName();
    receive_goal = false;
    update_time = 0.5;
    vehicle_size = 0.4;
    nh.getParam("update_time", update_time);
    nh.getParam("vehicle_size", vehicle_size);

	ROS_INFO("[%s] Initializing ", node_name.c_str());

    //Publisher
    pub_path = nh.advertise<nav_msgs::Path>("global_path", 1);

    timer = nh.createTimer(ros::Duration(0.5), &PathPlanning::Planning, this);

    //Subscriber
    sub_map = nh.subscribe("map", 1, &PathPlanning::cbMap, this);
    sub_odom = nh.subscribe("odom", 1, &PathPlanning::cbOdom, this);
    sub_goal = nh.subscribe("goal", 1, &PathPlanning::cbGoal, this);

}
void PathPlanning::Planning(const ros::TimerEvent& event){
    if(!receive_goal)
        return;
    
    cout << "Start planning" << endl; 
    clock_t t_start = clock();
    vector<Node> path = a_star.Planning(map, odom.pose.pose, goal.pose, vehicle_size);

    nav_msgs::Path  global_path;
    global_path.header.frame_id ="odom";
    global_path.header.stamp = ros::Time::now();

    for (int i = 0 ; i < path.size() ; i++){
        Node tmp = path[i];
        double x = tmp.x*map.info.resolution+map.info.origin.position.x;
        double y = tmp.y*map.info.resolution+map.info.origin.position.y;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id ="odom";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;
        global_path.poses.push_back(pose);

    }
    pub_path.publish(global_path);

    clock_t t_end = clock();
    cout << "A start planning time taken = " << (t_end-t_start)/(double)(CLOCKS_PER_SEC) << endl;
}
void PathPlanning::cbOdom(const nav_msgs::Odometry& msg_odom){
    odom = msg_odom;
}
void PathPlanning::cbMap(const nav_msgs::OccupancyGrid& msg_map){
    map = msg_map;
}
void PathPlanning::cbGoal(const geometry_msgs::PoseStamped& msg_goal){
    goal = msg_goal;
    receive_goal = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PathPlanning");
    ros::NodeHandle nh("~");
    PathPlanning pp(nh);
    
    ros::spin();
    return 0;
}