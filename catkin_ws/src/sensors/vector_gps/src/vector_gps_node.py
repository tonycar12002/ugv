#!/usr/bin/env python

# Revised by Sean based on nmea_navsat_driver
# Combined both GGA and RMC sentences and published 
# NavSatFix and Twist topics

import rospy
import serial
import time
import math

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from vector_gps import driver

def shutdown():
	gps_.close()
	rospy.loginfo("[%s] Shutdown..." %(rospy.get_name()))

def read_data(first, second):
	fix_valid = None 
	# Delete empty elements
	first = [x for x in first if x != '']
	second = [x for x in second if x != '']
	first_type = first[0][3:]
	second_type = second[0][3:]
	# GGA sentense length 15
	# Make sure type is 'GGA'
	# Print for debug
	print first
	print second
	if first_type == 'GGA' and len(first) >= 14 and second_type == 'RMC' and len(second) >= 14:
		fix_type, num_satellites, hdop, elevation = driver.handle_GGA(first)
		unix_time, latitude, latitude_direction, longitude, longitude_direction, \
		speed, true_course, fix_valid = driver.handle_RMC(second)
		rospy.loginfo("[%s] True course: %f" %(rospy.get_name(), true_course))
	else:
		rospy.loginfo("[%s] Wrong sentense length!  " %(rospy.get_name()))

	# Not valid
	if fix_valid == None:
		rospy.loginfo("[%s] Invalid state! " %(rospy.get_name()))
		return
	if fix_valid == False:
		rospy.loginfo("[%s] Reported position maybe wrong..." %(rospy.get_name())) 
	current_fix.status.status = fix_valid
	current_fix.status.service = NavSatStatus.SERVICE_GPS
	current_fix.header.stamp = rospy.Time.now()
	current_vel.header.stamp = rospy.Time.now()
	if latitude_direction == 'S':
		latitude *= -1
	if longitude_direction == 'W':
		longitude *= -1
	current_fix.latitude = latitude
	current_fix.longitude = longitude
	current_fix.altitude = elevation
	current_fix.position_covariance[0] = hdop ** 2
	current_fix.position_covariance[4] = hdop ** 2
	current_fix.position_covariance[8] = (2 * hdop) ** 2 #FIXME
	current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
	current_vel.twist.twist.linear.x =  speed #* math.sin(true_course)
	current_vel.twist.twist.linear.y =  0.0 #speed * math.cos(true_course)
	current_vel.twist.covariance[0] = 5.0 # TBD
	current_vel.twist.covariance[7] = 5.0 # TBD
	pub_fix.publish(current_fix)
	pub_vel.publish(current_vel)
	rospy.loginfo("[%s] Fix and vel published!" %(rospy.get_name()))

if __name__ == "__main__":
	rospy.init_node('vector_gps_node')
	port = rospy.get_param('~port', '/dev/ttyUSB0')
	baud = rospy.get_param('~baud', 19200)
	try:
		gps_ = serial.Serial(port, baud, timeout = 2)
		# Flush first data
		_ = gps_.readline()
		rospy.on_shutdown(shutdown)
		pub_fix = rospy.Publisher('/fix', NavSatFix, queue_size = 50)
		pub_vel = rospy.Publisher('/vel', TwistWithCovarianceStamped, queue_size = 50)
		current_fix = NavSatFix()
		current_fix.header.frame_id = 'gps'
		current_vel = TwistWithCovarianceStamped()
		current_vel.header.frame_id = 'gps'
	
		while not rospy.is_shutdown():
			data_1 = gps_.readline().strip()
			data_2 = gps_.readline().strip()
			data_1 = data_1.split(',')
			data_2 = data_2.split(',')
			# Make sure receive 'GGA' first since the transmitted data are 
			# GGA followed by RMC
			if data_1[0][3:] == 'RMC':
				_ = gps_.readline()
			else:
				read_data(data_1, data_2)
	except rospy.ROSInterruptException:
		gps.close() # Close GPS serial port
