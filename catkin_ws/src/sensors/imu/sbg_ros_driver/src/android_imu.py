#!/usr/bin/env python
import socket, traceback
import math
import rospy
import tf
from sensor_msgs.msg import Imu
host = '192.168.2.101'
port = 5555
pub = rospy.Publisher('/imu/data', Imu, queue_size = 1)
rospy.init_node('Android_IMU', anonymous=True)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

while 1:
	try:
		message, address = s.recvfrom(8192)
		#print (message)
		arr = message.split(',')

		#print arr[10]
		#mag = arr[10].split(',')
		yaw = math.atan2(float(arr[10]), float(arr[11]))
		print yaw
		#print float(mag[0]), float(mag[1])
		#print math.atan2(float(mag[0]), float(mag[1]))
		imu_data = Imu()
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		imu_data.orientation.x = quaternion[0] 
		imu_data.orientation.y = quaternion[1]
		imu_data.orientation.z = quaternion[2]
		imu_data.orientation.w = quaternion[3]
		imu_data.header.frame_id = 'imu_link'
		imu_data.header.stamp = rospy.get_rostime()
		pub.publish(imu_data)
		#print "pub"

	except (KeyboardInterrupt, SystemExit):
		raise
	except:
		pass
		#traceback.print_exc()
