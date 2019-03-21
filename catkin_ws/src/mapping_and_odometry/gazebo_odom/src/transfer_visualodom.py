#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
import math

class VisualOdomTf(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.listener = tf.TransformListener()
        self.source_frame = "odom"
        self.target_frame = "camera_init"
        
        self.br = tf.TransformBroadcaster()

        self.pub_odom = rospy.Publisher("loam_odom", Odometry, queue_size=1)
        self.cb_odom = rospy.Subscriber("odom", Odometry, self.cb_odom, queue_size=1)

    def cb_odom(self, msg):
        dur = rospy.Duration(1.0)

        self.listener.waitForTransform(self.source_frame, self.target_frame, msg.header.stamp, dur)
        (trans, rot) = self.listener.lookupTransform(self.source_frame, self.target_frame, msg.header.stamp)
       
        trans1_mat = tf.transformations.translation_matrix(trans)
        rot1_mat   = tf.transformations.quaternion_matrix(rot)
        mat1 = np.dot(trans1_mat, rot1_mat)

        trans2 = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        rot2= [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        trans2_mat = tf.transformations.translation_matrix(trans2)
        rot2_mat   = tf.transformations.quaternion_matrix(rot2)
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3)
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        euler = tf.transformations.euler_from_quaternion(rot3)
        rot4 = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2]-math.pi/2)

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = trans3[0]
        odom.pose.pose.position.y = trans3[1]
        odom.pose.pose.position.z = trans3[2]

        odom.pose.pose.orientation.x = rot4[0]
        odom.pose.pose.orientation.y = rot4[1]
        odom.pose.pose.orientation.z = rot4[2]
        odom.pose.pose.orientation.w = rot4[3]
        self.pub_odom.publish(odom)

        self.br.sendTransform(
            trans3,
            rot3,
            rospy.Time.now(),
            "base_link",
            "odom")


if __name__ == '__main__':
    rospy.init_node('visual_odom', anonymous=False)
    visual_odom = VisualOdomTf()
    rospy.spin()