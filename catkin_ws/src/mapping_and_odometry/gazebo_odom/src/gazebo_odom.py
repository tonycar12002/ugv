#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf
def cb_states(msg):
    target_pose = None
    global first, x, y, euler_offset, br
    for i in range(0, len(msg.name)):
        if msg.name[i] == "/":
            target_pose = msg.pose[i]
            if first == True:
                first = False
                x = target_pose.position.x
                y = target_pose.position.y
                q = (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
                euler_offset = tf.transformations.euler_from_quaternion(q)

            target_pose.position.x -= x
            target_pose.position.y -= y

            q = (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(q)
            q_after = tf.transformations.quaternion_from_euler(euler[0]-euler_offset[0], euler[1]-euler_offset[1], euler[2]-euler_offset[2])
            target_pose.orientation.x = q_after[0]
            target_pose.orientation.y = q_after[1]
            target_pose.orientation.z = q_after[2]
            target_pose.orientation.w = q_after[3]

    gazebo_odom = Odometry()
    gazebo_odom.header.frame_id="/odom"
    gazebo_odom.header.stamp = rospy.Time.now()
    gazebo_odom.pose.pose = target_pose

    br.sendTransform((gazebo_odom.pose.pose.position.x, \
                         gazebo_odom.pose.pose.position.y, gazebo_odom.pose.pose.position.z), \
                        (gazebo_odom.pose.pose.orientation.x, gazebo_odom.pose.pose.orientation.y, \
                        gazebo_odom.pose.pose.orientation.z, gazebo_odom.pose.pose.orientation.w), \
                        rospy.Time.now(),"/base_link","/odom")

    pub_odometry.publish(gazebo_odom)

if __name__ == "__main__":
    rospy.init_node("gazebo_odomtry",anonymous=False)

    first = False
    x = 0
    y = 0
    euler_offset = [0, 0, 0]
    br = tf.TransformBroadcaster()
    rospy.Subscriber("/gazebo/model_states", ModelStates, cb_states, queue_size=1)

    pub_odometry = rospy.Publisher("/gazebo/gazebo_odometry", Odometry, queue_size=1)

    rospy.spin()