#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include <time.h>
#include <algorithm>
#include <array>
#include <queue>
#include <stack>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
using namespace std;
struct Node
{
	int y;
	int x;
	int parent_x;
	int parent_y;
	float g_cost;
	float h_cost; 
	float f_cost;
    bool is_obstalce;
    bool is_closed;
    bool operator<(const Node& rhs) const
    {
        return f_cost > rhs.f_cost;
    }
    
};
std::ostream& operator << (std::ostream& o, const Node& a){
    o << "x = " << a.x << ", y = " << a.y << ", fcost = " << a.f_cost << endl;
    return o;
}
class AStar{
private:
    string node_name;
    vector<array<double, 2>> path;
    
    int map_width;
    int map_height;
    double map_resolution;

public:
    AStar(){node_name="a_star";
        ROS_INFO("[%s] Initializing ", node_name.c_str());}
    ~AStar(){}

    bool Compare(Node& left, Node& right); // True if right > left
    bool isValid(int x, int y, Node**);
    bool isDestination(int x, int y, Node& dest);
    double calculateHCost(int x, int y, Node& dest);
    int** MapToArray(nav_msgs::OccupancyGrid& map);


    vector<Node> Planning(nav_msgs::OccupancyGrid&, geometry_msgs::Pose&, geometry_msgs::Pose&);
    vector<Node> makePath(Node**, Node&);
    
};
bool AStar::Compare(Node& left, Node& right){
    return left.f_cost < right.f_cost;
}
bool AStar::isDestination(int x, int y, Node& dest){
    if (x == dest.x && y == dest.y)
        return true;
    else
        return false;
}
bool AStar::isValid(int x, int y, Node** map){
    if (x<0 || x >= map_width || y<0 || y >=map_height || map[y][x].is_obstalce)
        return false;
    else
        return true;
}
double AStar::calculateHCost(int x, int y, Node& dest) {
    double H = (sqrt((x - dest.x)*(x - dest.x)
        + (y - dest.y)*(y - dest.y)));
    return H;
}
vector<Node> AStar::makePath(Node** map, Node& dest){
    try {
        cout << "Found a path" << endl;
        int x = dest.x;
        int y = dest.y;
        stack<Node> path;
        vector<Node> final_path;

        while (!(map[y][x].parent_x == x && map[y][x].parent_y == y) && map[y][x].x != -1 && map[y][x].y != -1) {
            path.push(map[y][x]);
            int temp_x = map[y][x].parent_x;
            int temp_y = map[y][x].parent_y;
            x = temp_x;
            y = temp_y;
            
        }
        path.push(map[y][x]);

        while (!path.empty()) {
            Node top = path.top();
            path.pop();
            //cout << top.x << " " << top.y << endl;
            final_path.emplace_back(top);
        }

        // free
        for(int i = 0; i < map_height; ++i)
            delete [] map[i];
        delete [] map;

        return final_path;
    }
    catch(const exception& e){
        cout << e.what() << endl;
    }

}
vector<Node> AStar::Planning(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& odom, geometry_msgs::Pose& dest){
    map_width   = map.info.width;
    map_height  = map.info.height;
    map_resolution = map.info.resolution;

    Node start, goal;
    start.x = int( floor( (odom.position.x -  map.info.origin.position.x) / map.info.resolution)) ;
    start.y = int( floor( (odom.position.y -  map.info.origin.position.y) / map.info.resolution)) ;
    goal.x = int(  floor( (dest.position.x -  map.info.origin.position.x) / map.info.resolution)) ;
    goal.y = int(  floor( (dest.position.y -  map.info.origin.position.y) / map.info.resolution)) ;

    // Initial Map
    Node** all_map = new Node*[map_width];
    priority_queue<Node> queue;

    for(int i = 0; i < map_width; ++i)
        all_map[i] = new Node[map_height];

    for (int x = 0; x < map_width; x++) {
        for (int y = 0; y < map_height; y++) {
            all_map[y][x].f_cost = FLT_MAX;
            all_map[y][x].g_cost = FLT_MAX;
            all_map[y][x].parent_x = -1;
            all_map[y][x].parent_y = -1;
            all_map[y][x].x = x;
            all_map[y][x].y = y;
            all_map[y][x].is_closed = false;
            if (map.data[ y*map_width + x] >= 50)
                all_map[y][x].is_obstalce = true;
            else
                all_map[y][x].is_obstalce = false;
        }
    }
    vector<Node> empty;
    if (!isValid(goal.x, goal.y, all_map)){
        cout << "Goal point is obstacle or no map" << endl;
        return empty;
    }
    
    all_map[start.y][start.x].f_cost = 0.0;
    all_map[start.y][start.x].g_cost = 0.0;
    all_map[start.y][start.x].h_cost = 0.0;
    all_map[start.y][start.x].parent_x = start.x;
    all_map[start.y][start.x].parent_y = start.y;

    queue.push(all_map[start.y][start.x]); //priority queue
    bool destinationFound = false;
    

    while (!queue.empty() && queue.size()<map_width * map_height) {
 
        Node node = queue.top();
        queue.pop();

        int x = node.x;
        int y = node.y;
        all_map[y][x].is_closed = true;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                double g_new, h_new, f_new;
                int new_x = x+i;
                int new_y = y+j;
                if( isValid(new_x, new_y, all_map)){
                    if (isDestination(new_x, new_y, goal)){
                        //Destination found - make path
                        all_map[new_y][new_x].parent_x = x;
                        all_map[new_y][new_x].parent_y = y;
                        destinationFound = true;
                        return makePath(all_map, goal);
                    }
                    else if(all_map[new_y][new_x].is_closed == false){
                        g_new = node.g_cost + 1.0;
                        h_new = calculateHCost(new_x, new_y, goal);
                        f_new = g_new + h_new;

                        // Check if this path is better than the one already present
                        if (all_map[new_y][new_x].f_cost == FLT_MAX || all_map[new_y][new_x].f_cost > f_new){
                            // Update the details of this neighbour node
                            all_map[new_y][new_x].f_cost = f_new;
                            all_map[new_y][new_x].g_cost = g_new;
                            all_map[new_y][new_x].h_cost = h_new;
                            all_map[new_y][new_x].parent_x = x;
                            all_map[new_y][new_x].parent_y = y;
                            queue.push(all_map[new_y][new_x]);
                        }
                    }

                }
            }
        }
    }
    if (destinationFound == false) {
		cout << "Destination not found" << endl;
	}
    return empty;
    /*
    while (!queue.empty()){
        cout << queue.top() << endl;
        queue.pop();
    }
    
    for (int y = 0; y < map_height; y++) {
        for (int x = 0; x < map_width; x++) {
            if (isDestination(x, y, start))
                cout << "S" ;
            else if (isDestination(x, y, goal))
                cout << "G" ;
            else if (all_map[y][x].is_obstalce)
                cout << "X" ;
            else
                cout << "-";
        }
        cout << endl;
    }
    */

}

