#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include <queue>
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
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

using namespace std;
class PathPlanning{
private:
    string node_name;
    bool receive_goal;
    double update_time;
    double vehicle_size;
    double local_cost;
    queue<geometry_msgs::PoseStamped> waypt_list;
    vector<array<double, 2> >pose_array;

    AStar a_star;

    nav_msgs::Odometry odom;
    nav_msgs::OccupancyGrid map;

    ros::NodeHandle nh;
    ros::Publisher  pub_path;
    ros::Publisher  pub_arrive;
    ros::Publisher  pub_marker;
    ros::Publisher  pub_marker_list;

    ros::Subscriber sub_map;
    ros::Subscriber sub_goal;
    ros::Subscriber sub_odom;

    ros::ServiceServer service_clear;
    ros::ServiceServer service_start;
    ros::Timer timer;

public:
    PathPlanning(ros::NodeHandle&);
    void cbOdom(const nav_msgs::Odometry&);
    void cbMap(const nav_msgs::OccupancyGrid&);
    void cbGoal(const geometry_msgs::PoseStamped&);
    void Planning(const ros::TimerEvent&);
    bool ClearWaypt(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
    bool StartWaypt(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
    void Visualize(queue<geometry_msgs::PoseStamped>);
};
PathPlanning::PathPlanning(ros::NodeHandle& n){
    nh = n;
    node_name = ros::this_node::getName();
    receive_goal = false;
    update_time = 0.5;
    vehicle_size = 0.4;
    local_cost = 2.0;
    nh.getParam("update_time", update_time);
    nh.getParam("vehicle_size", vehicle_size);
    nh.getParam("local_cost", local_cost);

    ROS_INFO("[%s] Initializing ", node_name.c_str());

    //Publisher
    pub_path = nh.advertise<nav_msgs::Path>("global_path", 1);
    pub_arrive = nh.advertise<std_msgs::Bool>("arrive", 1);
    pub_marker = nh.advertise< visualization_msgs::Marker >("goal_point", 1);
    pub_marker_list = nh.advertise< visualization_msgs::Marker >("waypoint_list", 1);

    service_clear = nh.advertiseService("/ClearWaypt", &PathPlanning::ClearWaypt, this);
    service_start = nh.advertiseService("/StartWaypt", &PathPlanning::StartWaypt, this);

    timer = nh.createTimer(ros::Duration(update_time), &PathPlanning::Planning, this);

    //Subscriber
    sub_map = nh.subscribe("map", 1, &PathPlanning::cbMap, this);
    sub_odom = nh.subscribe("odom", 1, &PathPlanning::cbOdom, this);
    sub_goal = nh.subscribe("goal", 1, &PathPlanning::cbGoal, this);

}
void PathPlanning::Planning(const ros::TimerEvent& event){
    if(waypt_list.empty() || !receive_goal){
        std_msgs::Bool arrive;
        arrive.data = true;
        pub_arrive.publish(arrive);
        receive_goal = false;
        return;
    }
    //cout << "Start planning" << endl; 
    clock_t t_start = clock();
    nav_msgs::Path  global_path;
    global_path.header.frame_id = map.header.frame_id;
    global_path.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped goal = waypt_list.front();
    Visualize(waypt_list);

    double tmp_x = odom.pose.pose.position.x - goal.pose.position.x;
    double tmp_y = odom.pose.pose.position.y - goal.pose.position.y;
    double dis = sqrt(tmp_x*tmp_x + tmp_y*tmp_y);
    if (dis <=  1.0){
        cout << "Arrive waypoint" << endl;
        waypt_list.pop();
        return;
    }

    vector<Node> path = a_star.Planning(map, odom.pose.pose, goal.pose, vehicle_size, local_cost);
    for (int i = 0 ; i < path.size() ; i++){
        Node tmp = path[i];
        double x = tmp.x*map.info.resolution+map.info.origin.position.x;
        double y = tmp.y*map.info.resolution+map.info.origin.position.y;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id =  map.header.frame_id;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;
        global_path.poses.push_back(pose);

    }
    pub_path.publish(global_path);

    clock_t t_end = clock();
    //cout << "A start planning time taken = " << (t_end-t_start)/(double)(CLOCKS_PER_SEC) << endl;
}
void PathPlanning::Visualize(queue<geometry_msgs::PoseStamped> waypoints){
    visualization_msgs::Marker points, points_list;
    points.id = 5;
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    points.pose.orientation.w = 1.0;
    points.ns = "points";
    points.header.stamp = ros::Time::now();
    points.header.frame_id = map.header.frame_id;

    points_list.id = 4;
    points_list.type = visualization_msgs::Marker::POINTS;
    points_list.action = visualization_msgs::Marker::ADD;
    points_list.scale.x = 0.2;
    points_list.scale.y = 0.2;
    points_list.color.b = 1.0f;
    points_list.color.a = 1.0;
    points_list.pose.orientation.w = 1.0;
    points_list.ns = "points_list";
    points_list.header.stamp = ros::Time::now();
    points_list.header.frame_id = map.header.frame_id;

    geometry_msgs::Point p1;
    p1.x = waypoints.front().pose.position.x;
    p1.y = waypoints.front().pose.position.y;
    p1.z = 0;
    points.points.push_back(p1);

    while (!waypoints.empty()){
        p1.x = waypoints.front().pose.position.x;
        p1.y = waypoints.front().pose.position.y;
        p1.z = 0;
        waypoints.pop();
        points_list.points.push_back(p1);
    }
  
    pub_marker.publish(points);
    pub_marker_list.publish(points_list);
}
bool PathPlanning::ClearWaypt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    res.success = 1;
    res.message = "Clear all waypoints";
    cout << "Clear all waypoints" << endl;
    receive_goal = false;
    queue<geometry_msgs::PoseStamped> empty;
    swap(waypt_list, empty );
    return true;
}
bool PathPlanning::StartWaypt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    res.success = 1;
    res.message = "Start waypoints";
    cout << "Start waypoints" << endl;
    receive_goal = true;

    std_msgs::Bool arrive;
    arrive.data = false;
    pub_arrive.publish(arrive);

    return true;
}

void PathPlanning::cbOdom(const nav_msgs::Odometry& msg_odom){
    odom = msg_odom;
}
void PathPlanning::cbMap(const nav_msgs::OccupancyGrid& msg_map){
    map = msg_map;
}
void PathPlanning::cbGoal(const geometry_msgs::PoseStamped& msg_goal){
    waypt_list.push(msg_goal);
    Visualize(waypt_list);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PathPlanning");
    ros::NodeHandle nh("~");
    PathPlanning pp(nh);
    
    ros::spin();
    return 0;
}
