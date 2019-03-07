#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry

import numpy as np
import heapq
import math
from time import time
import pyastar
class AStar:
    def __init__(self):
        self.node_name = "A_star"
        rospy.loginfo("[%s] Initializing " %(self.node_name))


    def planning(self, map, start, goal): # OccupancyGrid Pose Pose
        map_array = self.map_to_array(map)
        start = self.map_to_grid_cord(map.info, start.position.x, start.position.y)
        goal = self.map_to_grid_cord(map.info, goal.position.x, goal.position.y)

        weights = np.asarray(map_array, dtype=np.float32)
        tmp_path = pyastar.astar_path(weights, start, goal, allow_diagonal=True)
        
        '''
        for y in range(0, len(map_array)):
            for x in range(0, len(map_array[y])):
                if map_array[y][x] >=50:
                    print "X", 
                else:
                    print "-",
            print("")
        
        

        for y in range(0, len(map_array)):
            for x in range(0, len(map_array[y])):
                if x==start[0] and y == start[1]:
                    print "S",
                elif x==goal[0] and y == goal[1]:
                    print "G",
                elif [y, x] in tmp_path:
                    print "P",
                elif weights[y][x] >=50:
                    print "X", 
                else:
                    print "-",
            print("")
        print("================")
        '''

        # The start and goal coordinates are in matrix coordinates (i, j).
        
        path = []
        for i in tmp_path:
            tmp = (i[0]*map.info.resolution+map.info.origin.position.x, i[1]*map.info.resolution+map.info.origin.position.y)
            path.append(tmp)
        return path

    def trans_to_1d(self, width, x ,y):
        return y*width+x

    def map_to_grid_cord(self, map_info, x, y):
        cell_x = (x - map_info.origin.position.x) / map_info.resolution
        cell_y = (y - map_info.origin.position.y) / map_info.resolution

        cell_x = int(cell_x)
        cell_y = int(cell_y) 
      
        return (cell_x, cell_y)


    def heuristic(self, node1, node2):
        x = node1[0] - node2[0]
        y = node1[1] - node2[1]
        return x ** 2 + y ** 2

    def map_to_array(self, map):
        array_1d = np.array(map.data)
        array_2d = []
        for y in range(0, map.info.height):
            tmp_1d = [] 
            for x in range(0, map.info.width):
                cell = y * map.info.height + x
                if map.data[cell] >= 50:
                    tmp_1d.append(np.inf)
                else:
                    tmp_1d.append(1)
                
            array_2d.append(tmp_1d)

        return array_2d

if __name__ == '__main__':
    rospy.init_node("a_star")
    a_star = AStar()
    rospy.spin()
