#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData 
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
import tf
from math import cos ,sin, pi, sqrt

class LocalCost2d(object):
    def __init__(self):

        # variables
        self.node_name = rospy.get_name()
        self.odom = None
        self.scan = None
        self.vehicle_yaw = None

        # param
        self.cell_size  = rospy.get_param('~cell_size', 0.5) # meter
        self.map_length   = rospy.get_param("~map_length", 35) 
        self.wait_time  = rospy.get_param("~wait_time", 0.1)
        self.vehicle_size  = rospy.get_param("~vehicle_size", 0.1)

        self.cell_length = int(self.map_length/self.cell_size) * 2
        self.frame_id  = rospy.get_param("~frame_id", "odom")
        self.frame_rate = 0.5

        # Publisher
        self.pub_grid_map = rospy.Publisher("local_map", OccupancyGrid, queue_size=1)

        rospy.Timer(rospy.Duration(self.frame_rate), self.create_local_map)

        # Subscriber
        sub_odometry = Subscriber("odom", Odometry)
        sub_laser_scan = Subscriber("scan", LaserScan)
        ats = ApproximateTimeSynchronizer((sub_odometry, sub_laser_scan),queue_size = 1, slop = self.wait_time)
        ats.registerCallback(self.cb_sensor)

    def cb_sensor(self, msg_odom, msg_scan):
        self.odom  = msg_odom
        self.scan = msg_scan
       
    def create_local_map(self, event):
        if self.odom is None or self.scan is None:
            return 

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.header.frame_id = self.frame_id

        # Map data information
        map_data_info = MapMetaData()
        map_data_info.resolution = self.cell_size               # [m/cell]
        map_data_info.width = self.cell_length                  # [cells]
        map_data_info.height = self.cell_length

        
        q = (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        self.vehicle_yaw = tf.transformations.euler_from_quaternion(q)[2]
        '''
        x_orig = -self.map_length*cos(self.vehicle_yaw) + self.map_length*sin(self.vehicle_yaw)
        y_orig = -self.map_length*sin(self.vehicle_yaw) - self.map_length*cos(self.vehicle_yaw)
        map_data_info.origin.position.x = int( x_orig + self.odom.pose.pose.position.x )
        map_data_info.origin.position.y = int( y_orig + self.odom.pose.pose.position.y )
        map_data_info.origin.position.z = int(self.odom.pose.pose.position.z)
        map_data_info.origin.orientation = self.odom.pose.pose.orientation
        '''

        map_data_info.origin.position.x    = self.odom.pose.pose.position.x - self.map_length
        map_data_info.origin.position.y    = self.odom.pose.pose.position.y - self.map_length
        map_data_info.origin.position.z    = 0
        map_data_info.origin.orientation.x = 0
        map_data_info.origin.orientation.y = 0
        map_data_info.origin.orientation.z = 0
        map_data_info.origin.orientation.w = 1.0
        occupancy_grid.info = map_data_info 

        '''
        print (self.vehicle_yaw/3.14*180)
        print(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
        print(x_orig, y_orig)
        print(map_data_info.origin.position.x, map_data_info.origin.position.y)
        '''

        # initial cell 
        data = [0] * map_data_info.width * map_data_info.height
        # read laser scan to grid map 
        for laser in range(0, len(self.scan.ranges)):
            scan_range = self.scan.ranges[laser]
            rad = self.scan.angle_min + laser * self.scan.angle_increment + pi/2  + self.vehicle_yaw 

            # remove points out of range 
            if scan_range is None or scan_range < self.scan.range_min or scan_range > self.scan.range_max or scan_range > self.map_length - 5: # 
                continue

            cell_num = self.get_cell_number(scan_range, rad, map_data_info)
            for value in cell_num:
                value = map_data_info.width * map_data_info.height - value 
                data[value] = 100

        #print("==========================")
        occupancy_grid.data = data
        
        self.pub_grid_map.publish(occupancy_grid)

    def get_cell_number(self, scan_range, rad, map_data_info):
       
        scan_x = scan_range * cos(rad) 
        scan_y = scan_range * sin(rad) 

        dis_x = self.odom.pose.pose.position.x - map_data_info.origin.position.x
        dis_y = self.odom.pose.pose.position.y - map_data_info.origin.position.y

        scan_x = scan_x + dis_x
        scan_y = scan_y + dis_y

        scan_list = []
        cell_list = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                scan_list.append( [scan_x+i*self.vehicle_size , scan_y+j*self.vehicle_size ] )

        for scan in scan_list:
            cell_y = int(scan[0] / self.cell_size)
            cell_x = int(scan[1] / self.cell_size)
            cell_x = map_data_info.width - cell_x
            data = int(cell_y * map_data_info.width + cell_x)
            cell_list.append(data)

        return cell_list

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node("local_cost_map", anonymous = True)
    local_cost_map = LocalCost2d()
    rospy.on_shutdown(local_cost_map.onShutdown)
    rospy.spin()