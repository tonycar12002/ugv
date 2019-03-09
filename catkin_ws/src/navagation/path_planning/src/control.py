#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
import numpy as np
from time import time
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
import math
import tf
import math
from visualization_msgs.msg import Marker

class Control:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.odom = None
        self.path = None
        self.update = False
        self.complete_dis = 0.5


        # Publisher
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_marker = rospy.Publisher("target_point", Marker, queue_size=11)

        rospy.Timer(rospy.Duration(0.2), self.send_twist)

        # Subscriber
        self.sub_odometry = rospy.Subscriber("odom", Odometry, self.cb_odom)
        self.sub_gloal_path = rospy.Subscriber("/path_planning/global_path", Path, self.cb_path)


    def distance(self, pose1, pose2):
        x = pose2.position.x - pose1.position.x
        y = pose2.position.y - pose1.position.y

        q1 = (pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w)
        yaw1 = tf.transformations.euler_from_quaternion(q1)[2]


        yaw2 = math.atan2(y, x)

        return math.sqrt(x*x+y*y), yaw2-yaw1

    def send_twist(self, event):
        if self.path is None or self.odom is None:
            return 

        self.update = False
        vehicle_pose = self.odom.pose.pose
        i = 0
        while(not self.update and i != len(self.path.poses)):
            pose = self.path.poses[i].pose
            dis, yaw = self.distance(vehicle_pose, pose)
            if dis <= self.complete_dis:
                i +=1
            else:
                cmd = Twist()
                cmd.linear.x = 0.18
                if yaw >= 0.8:
                    cmd.angular.z = yaw/2.5
                else:
                    cmd.angular.z = yaw/1.5
                #print(dis, yaw/math.pi*180)
                #print("x = ", cmd.linear.x, ", z = ", cmd.angular.z)
                #self.pub_twist.publish(cmd)

                marker = Marker(type=Marker.SPHERE, \
                    id=0, lifetime=rospy.Duration(), \
                    pose=Pose(Point(pose.position.x, pose.position.y, 0), Quaternion(0, 0, 0, 1)),\
                    scale=Vector3(0.3, 0.3, 0.3),\
                    header=Header(frame_id = "odom"),\
                    color=ColorRGBA(0.0, 11.0, 0.0, 1))

                self.pub_marker.publish(marker)


        
    def cb_odom(self, msg_odom):
        self.odom  = msg_odom

    def cb_path(self, msg_path):
        self.path = msg_path
        self.update = True

    def onShutdown(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.z = 0
        self.pub_twist.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("control", anonymous=False)
    control = Control()
    rospy.on_shutdown(control.onShutdown)
    rospy.spin()
