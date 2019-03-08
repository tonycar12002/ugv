#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import numpy as np
from time import time
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class Control(self):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.odom = None
        self.path = None


        # Publisher
        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Timer(rospy.Duration(0.2), self.send_twist)

        # Subscriber
        sub_odometry = Subscriber("odom", Odometry)
        sub_gloal_path = Subscriber("global_path", Path)
        ats = ApproximateTimeSynchronizer((sub_odometry, sub_gloal_path),queue_size = 1, slop = 0.2)
        ats.registerCallback(self.cb_odom_path)


    def send_twist(self, event):
        if self.path is None or self.path is None:
            return 

        
    def cb_odom_path(self, msg_odom, msg_path):
        self.odom  = msg_odom
        self.path = msg_path



if __name__ == '__main__':
    rospy.init_node("control", anonymous=False)
    control = Control()
    rospy.spin()
