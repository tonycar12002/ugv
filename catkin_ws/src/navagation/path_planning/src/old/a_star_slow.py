#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry

import numpy as np
import heapq
import math
from time import time
class AStar:
    def __init__(self):
        self.node_name = "A_star"
        rospy.loginfo("[%s] Initializing " %(self.node_name))


    def planning(self, map, start, goal): # OccupancyGrid Pose Pose
        start_time = time()
        map_array = self.map_to_array(map)
        #print(time()-start_time)
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

        start = self.map_to_grid_cord(map.info, start.position.x, start.position.y)
        goal = self.map_to_grid_cord(map.info, goal.position.x, goal.position.y)

        size = map.info.height*map.info.width
        width = map.info.width
        close_set = [0] * size
        g_score  = [10000] * size
        f_score = [10000] * size
        g_score[ self.trans_to_1d(width, start[0], start[1]) ] = 0
        f_score[ self.trans_to_1d(width, start[0], start[1]) ] = self.heuristic(start, goal)


        came_from = {}      # the parent of node
        close_set = np.zeros((map.info.height, map.info.width))   # already calculated node

        oheap = []
        heapq.heappush(oheap, (f_score[self.trans_to_1d(width, start[0], start[1])], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1] # node 

            if map_array[goal[1]][goal[0]] == 1:
                print("Goal is obstacles")
                return False

            if current == goal:
                path = []
                while current in came_from:
                    tmp = (current[0]*map.info.resolution+map.info.origin.position.x, current[1]*map.info.resolution+map.info.origin.position.y)
                    path.append(tmp)
                    current = came_from[current]

                return path  
            
            close_set[current[1]][current[0]] = 1

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j    
                neighbor_index = self.trans_to_1d(width, neighbor[0], neighbor[1])
                if neighbor[0] < 0 or neighbor[0] >= np.shape(map_array)[1]:
                    continue
                elif neighbor[1] < 0 or neighbor[1] >= np.shape(map_array)[0]:
                    continue
                elif map_array[neighbor[1]][neighbor[0]] == 1:
                    continue

                tentative_g_score = g_score[self.trans_to_1d(width, current[0], current[1])] + self.heuristic(current, neighbor) # calculate neighbors g score
                tentative_f_score = tentative_g_score + self.heuristic(neighbor, goal)

                if close_set[neighbor[1]][neighbor[0]] == 1 and tentative_g_score >= g_score[neighbor_index]:
                    continue

                if close_set[neighbor[1]][neighbor[0]] == 0 or tentative_g_score < g_score[neighbor_index]:
                    came_from[neighbor] = current  
                    g_score[neighbor_index] = tentative_g_score
                    f_score[neighbor_index] = tentative_f_score
                    heapq.heappush(oheap, (f_score[neighbor_index], neighbor))

        print("Unable to Arrive")
        return False
        '''
        for y in range(0, len(map_array)):
            for x in range(0, len(map_array[y])):
                if x==start[0] and y == start[1]:
                    print "S",
                elif x==goal[0] and y == goal[1]:
                    print "G",
                elif map_array[y][x] >=50:
                    print "X", 
                else:
                    print "-",
            print("")
        print("================")
        '''

        return 0

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
                    tmp_1d.append(1)
                else:
                    tmp_1d.append(0)
                
            array_2d.append(tmp_1d)

        return array_2d

if __name__ == '__main__':
    rospy.init_node("a_star")
    a_star = AStar()
    rospy.spin()
