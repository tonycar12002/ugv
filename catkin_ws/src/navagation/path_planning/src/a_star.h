#include <iostream>
#include<fstream>
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
	double g_cost;
	double h_cost; 
	double f_cost;
    double local_cost;
    bool is_obstalce;
    bool is_closed;
    double vehicle_range;
    bool operator<(const Node& rhs) const
    {
        /*
        double tmp_x, tmp_y, dis;
        tmp_x = x - rhs.x;
        tmp_y = y - rhs.y;
        dis = sqrt(tmp_x*tmp_x + tmp_y*tmp_y);
        if (dis<=vehicle_range)
            return local_cost > rhs.local_cost;
        else*/
            return f_cost > rhs.f_cost;
    }
    Node() : local_cost(0){}
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


    vector<Node> Planning(nav_msgs::OccupancyGrid&, geometry_msgs::Pose&, geometry_msgs::Pose&, double, double);
    vector<Node> makePath(Node**, Node&);
    void WriteCost(Node **all_map);
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
    double tmp_x = x - dest.x;
    double tmp_y = y - dest.y;
    return sqrt(tmp_x*tmp_x + tmp_y*tmp_y);
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
vector<Node> AStar::Planning(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& odom, geometry_msgs::Pose& dest, double vehicle_size, double local_cost){
    map_width   = map.info.width;
    map_height  = map.info.height;
    map_resolution = map.info.resolution;
    double range = vehicle_size/map_resolution;

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

    // Initial every node
    for (int x = 0; x < map_width; x++) {
        for (int y = 0; y < map_height; y++) {
            all_map[y][x].f_cost = FLT_MAX;
            all_map[y][x].g_cost = FLT_MAX;
            all_map[y][x].parent_x = -1;
            all_map[y][x].parent_y = -1;
            all_map[y][x].x = x;
            all_map[y][x].y = y;
            all_map[y][x].is_closed = false;
            all_map[y][x].vehicle_range = range;
            
            if (map.data[ y*map_width + x] >= 70){
                all_map[y][x].is_obstalce = true;
                
                //cout << range << endl;
                if (range>=1){
              
                    for (int i = -range; i <= range; i++) {
                        for (int j = -range; j <= range; j++) {
                            int new_x = x + i;
                            int new_y = y + j;
                            if (new_x<0 || new_x >= map_width || new_y<0 || new_y >=map_height)
                                continue;
                                
                            double tmp = max(abs(i), abs(j));
                            all_map[new_y][new_x].local_cost = max(all_map[new_y][new_x].local_cost , (range - tmp)*local_cost);

                        }
                    }
                }
                
            }
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
    
    // Start planning
    while (!queue.empty()) { // issue: queue.size()<map_width * map_height why
 
        Node node = queue.top(); //find the min fcost node
        queue.pop();

        int x = node.x;
        int y = node.y;
        all_map[y][x].is_closed = true;
     
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                double g_new, h_new, f_new;
                int new_x = x+i;
                int new_y = y+j;
                if( isValid(new_x, new_y, all_map)){ //check is obstacle and boundary
                    if (isDestination(new_x, new_y, goal)){
                        //Destination found - make path
                        all_map[new_y][new_x].parent_x = x;
                        all_map[new_y][new_x].parent_y = y;
                        destinationFound = true;
                        //WriteCost(all_map);
                        return makePath(all_map, goal); //arrive target => find path
                    }
                    else if(all_map[new_y][new_x].is_closed == false){
                        g_new = node.g_cost + 1.0;
                        h_new = calculateHCost(new_x, new_y, goal);
                        f_new = g_new + h_new + all_map[new_y][new_x].local_cost; //local cost for vehicle size

                        // Check if this path is better than the one already present
                        if (all_map[new_y][new_x].f_cost == FLT_MAX || all_map[new_y][new_x].g_cost > g_new){
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
		//WriteCost(all_map);
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

void AStar::WriteCost(Node **all_map){
    char filename[]="/root/ugv/f_cost.txt";
    fstream fp;
    fp.open(filename, ios::out);
    if(!fp){//如果開啟檔案失敗，fp為0；成功，fp為非0
        cout<<"Fail to open file: "<<filename<<endl;
    }
    for (int x = 0; x < map_width; x++) {
        for (int y = 0; y < map_height; y++) {  
            if (all_map[y][x].is_obstalce)
                fp<<"X"<<" ";
            else if (round(all_map[y][x].f_cost)>=10000)
                fp<<" "<<" ";
            else
                fp<<round(all_map[y][x].local_cost)<<" ";
        }
        fp<<endl;
    } 
    fp.close();
}
