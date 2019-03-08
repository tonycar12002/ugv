import math
import time
import calendar
import logging

logger = logging.getLogger('rosout')


def safe_float(field):
	try:
		return float(field)
	except ValueError:
		return float('NaN')

def safe_int(field):
	try:
		return int(field)
	except ValueError:
		return 0

def convert_latitude(field):
	return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0

def convert_longitude(field):
	return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0

def convert_time(nmea_utc):
	utc_struct = time.gmtime()
	utc_list = list(utc_struct)
	try:
		if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
			return float('NaN')
		else:
			hours   = int(nmea_utc[0:2])
			minutes = int(nmea_utc[2:4])
			seconds = int(nmea_utc[4:6])
			utc_list[3] = hours
			utc_list[4] = minutes
			utc_list[5] = seconds
			unix_time = calendar.timegm(tuple(utc_list))
			return unix_time

	except IndexError:
		print "UTC time formmat error, string is ", nmea_utc
		return float('NaN')

def convert_status_flag(status_flag):
	if status_flag == "A":
		return True
	elif status_flag == "V":
		return False
	else:
		return False

def convert_knots_to_mps(knots):
	return safe_float(knots) * 0.514444444444

def convert_deg_to_rads(degs):
	return math.radians(safe_float(degs))

# Return format:
# Total length: 12
#########################################################################################
# unix_time, latitude(in degrees), latitude_direction('N'/'S'), longitude(in degrees),  #
# longitude_direction('E'/ 'W'), elevation, speed(in m/s), true_course(in rad), hdop,   #
# fix_type, fix_valid, num_satellites                                                   #
#########################################################################################

def handle_GGA(sentense):
	# Get fix_type, num_satellites, hdop and elevation from GGA
	#sentense = sentense.split(',')
	fix_type = int(sentense[6]) # 9
	num_satellities = safe_int(sentense[7]) # 11
	hdop = safe_float(sentense[8]) # 8
	elevation = safe_float(sentense[9]) # 5
	
	result = [fix_type, num_satellities, hdop, elevation]
	
	return result

def handle_RMC(sentense):
	# Get remain from RMC
	#sentense = sentense.split(',')
	unix_time = convert_time(sentense[1]) # 0
	latitude = convert_latitude(sentense[3]) # 1
	latitude_direction = str(sentense[4]) # 2
	longitude = convert_longitude(sentense[5]) #3
	longitude_direction = str(sentense[6]) # 4
	speed = convert_knots_to_mps(sentense[7]) # 6
	true_course = convert_deg_to_rads(sentense[8]) # 7
	fix_valid = convert_status_flag(sentense[2]) # 10

	result = [unix_time, latitude, latitude_direction, longitude, \
		  longitude_direction, speed, true_course, fix_valid]
	
	return result

if __name__ == "__main__":
	res = [None] * 12
	sen_gga = "$GPGGA,121252.000,3937.3032,N,11611.6046,E,1,05,2.0,45.9,M,-5.7,M,,0000*77"
	sen_rmc = "$GPRMC,121252.000,A,3958.3032,N,11629.6046,E,15.15,359.95,070306,,,A*54"

	res[9], res[11], res[8], res[5] = handle_GGA(sen_gga)
	res[0], res[1], res[2], res[3], res[4], res[6], res[7] , res[10] = handle_RMC(sen_rmc)
	print res
