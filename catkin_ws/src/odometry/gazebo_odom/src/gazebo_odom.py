#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

def cb_states(msg):
    target_pose = None
    for i in range(0, len(msg.name)):
        if msg.name[i] == "X1":
            target_pose = msg.pose[i]
            target_pose.position.x -= 2
            target_pose.position.y -= 1


    gazebo_odom = Odometry()
    gazebo_odom.header.frame_id="X1/odom"
    gazebo_odom.header.stamp = rospy.Time.now()
    gazebo_odom.pose.pose = target_pose
    pub_odometry.publish(gazebo_odom)

if __name__ == "__main__":
    rospy.init_node("gazebo_odomtry",anonymous=False)

    rospy.Subscriber("/gazebo/model_states", ModelStates, cb_states, queue_size=1)

    pub_odometry = rospy.Publisher("/gazebo/X1/gazebo_odometry", Odometry, queue_size=1)

    rospy.spin()