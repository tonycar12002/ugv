#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from geometry_msgs.msg import PoseStamped
from a_star import AStar
from time import time

class ObsAvoidance:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # variables
        self.map = None
        self.goal = None
        self.a_star = AStar()
        self.vehicle_pose = None

        # service
        self.srv_clear = rospy.Service('~clear', Trigger, self.cb_srv_clear)

        # Publisher
        self.pub_path = rospy.Publisher("global_path", Path, queue_size = 1)

        rospy.Timer(rospy.Duration(0.5), self.planning)

        # Subscrbier
        self.sub_map = rospy.Subscriber("map", OccupancyGrid, self.cb_map, queue_size = 1)
        self.sub_goal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.cb_goal, queue_size = 1)
        self.sub_odom = rospy.Subscriber("gazebo/gazebo_odometry", Odometry, self.cb_odom, queue_size = 1)

    def planning(self, event):
        if self.goal is None:
            return
        elif self.map is None or self.goal is None or self.vehicle_pose is None:
            rospy.loginfo("[%s] Missing map or odomtery" %(self.node_name))
            return
        start_time = time()

        path_array = self.a_star.planning(self.map, self.vehicle_pose, self.goal.pose)
        
        global_path = Path()
        global_path.header.frame_id ="odom"
        global_path.header.stamp = rospy.Time.now()
        for path in path_array:
            pose = PoseStamped()
            pose.header.frame_id ="odom"
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = path[0]
            pose.pose.position.y = path[1]
            pose.pose.position.z = 0

            global_path.poses.append(pose)

        self.pub_path.publish(global_path)

        end_time = time()
        print("Planning time = ", end_time-start_time)
        #print(pose_array)
        #print("======")
        
    def cb_odom(self, msg_odom):
        self.vehicle_pose = msg_odom.pose.pose

    def cb_goal(self, msg_goal):
        self.goal = msg_goal

    def cb_map(self, msg_map):
        self.map = msg_map

    def cb_srv_clear(self):
        self.goal = None
    

if __name__ == '__main__':
    rospy.init_node("obs_avoid", anonymous=False)
    obs_avoid = ObsAvoidance()
    rospy.spin()
