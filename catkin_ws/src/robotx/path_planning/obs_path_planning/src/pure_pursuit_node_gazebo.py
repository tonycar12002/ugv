#!/usr/bin/env python

import rospy
from Pure_pursuit import Pure_pursuit, Pure_pursuit_gazebo

"""
This program utilizes pure pursuit to follow a given trajectory.
"""

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("pure_pursuit_node",anonymous=False)
    pure_pursuit = Pure_pursuit_gazebo()
    rospy.spin()
