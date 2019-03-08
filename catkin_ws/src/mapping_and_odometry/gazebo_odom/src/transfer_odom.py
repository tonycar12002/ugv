#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
def cb_odom(msg):
    target_pose = msg.pose.pose
    q = (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(q)


    #q_after = tf.transformations.quaternion_from_euler(euler[0]-euler_offset[0], euler[1]-euler_offset[1], euler[2]-euler_offset[2])
    
    target_pose = Pose()
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    br.sendTransform((msg.pose.pose.position.x, \
                         msg.pose.pose.position.y, msg.pose.pose.position.z), \
                        (target_pose.orientation.x, target_pose.orientation.y, \
                        target_pose.orientation.z, target_pose.orientation.w), \
                        rospy.Time.now(),"/base_link","/odom")

if __name__ == "__main__":
    rospy.init_node("transfer_odom",anonymous=False)

    br = tf.TransformBroadcaster()
    rospy.Subscriber("odom", Odometry, cb_odom, queue_size=1)

    rospy.spin()
