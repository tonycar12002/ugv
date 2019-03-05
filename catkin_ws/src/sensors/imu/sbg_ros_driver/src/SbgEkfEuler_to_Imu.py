#!/usr/bin/env python
import numpy as np
import rospy
from sbg_driver.msg import SbgEkfEuler
from sensor_msgs.msg import Imu
import tf
pub = rospy.Publisher('/imu/data', Imu)

def callback(data):
    quaternion = tf.transformations.quaternion_from_euler(data.angle.x, - data.angle.y, - data.angle.z + np.pi/2)
    imu_data = Imu()
    imu_data.orientation.x = quaternion[0] 
    imu_data.orientation.y = quaternion[1]
    imu_data.orientation.z = quaternion[2]
    imu_data.orientation.w = quaternion[3]
    imu_data.header = data.header
    pub.publish(imu_data)
    
def listener():

    rospy.init_node('SbgEkfEuler_to_Imu', anonymous=True)

    rospy.Subscriber("/ekf_euler", SbgEkfEuler, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
