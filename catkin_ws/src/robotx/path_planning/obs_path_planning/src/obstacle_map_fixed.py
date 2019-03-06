#!/usr/bin/env python
import rospy
from tf import TransformListener, TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException
import roslib
from sensor_msgs.msg import PointCloud2
from robotx_msgs.msg import ObjectPoseList, Waypoint, WaypointList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

def call_back(msg):
    try:
        obj_list = ObjectPoseList()
        obj_list = msg
        position, quaternion = tf_.lookupTransform( "/odom", "/velodyne",rospy.Time(0))
        transpose_matrix = transformer.fromTranslationRotation(position, quaternion)

        if is_moos:
            dur = rospy.Duration(2)
            tf_.waitForTransform("/odom", "/wp_0", rospy.Time(), dur)
            (trans,rot) = tf_.lookupTransform("/odom", "/wp_0", rospy.Time(0))  

            for obs_index in range(obj_list.size):
                origin_p  = np.array([obj_list.list[obs_index].position.x, obj_list.list[obs_index].position.y, obj_list.list[obs_index].position.z, 1])
                new_p = np.dot(transpose_matrix, origin_p)
                obj_list.list[obs_index].position.x = new_p[0] - trans[0]
                obj_list.list[obs_index].position.y = new_p[1] - trans[1]
                obj_list.header.frame_id = "odom"
        
        else:
            for obs_index in range(obj_list.size):
                origin_p  = np.array([obj_list.list[obs_index].position.x, obj_list.list[obs_index].position.y, obj_list.list[obs_index].position.z, 1])
                new_p = np.dot(transpose_matrix, origin_p)
                obj_list.list[obs_index].position.x = new_p[0] 
                obj_list.list[obs_index].position.y = new_p[1] 
                obj_list.header.frame_id = "odom"         
                   
        pub_obs.publish(obj_list)

    except (LookupException, ConnectivityException, ExtrapolationException):
        print "Nothing Happen"

if __name__ == "__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("gmapping_odom",anonymous=False)
    tf_ = TransformListener()
    transformer = TransformerROS()
    is_moos = rospy.get_param("~moos", False)
    node_name  = rospy.get_name()

    rospy.loginfo("[%s] Initializing " %(node_name)) 
    rospy.loginfo("[%s] Param [is_moos] = %d" %(node_name, is_moos))  

    rospy.Subscriber("~obj_list", ObjectPoseList, call_back, queue_size=10)
    #rospy.Subscriber("/waypointList", WaypointList, call_back, queue_size=10)
    pub_obs = rospy.Publisher("~obj_list_odom", ObjectPoseList, queue_size=1)
    #pub_rviz = rospy.Publisher("/wp_path", Marker, queue_size = 1)
    rospy.spin()