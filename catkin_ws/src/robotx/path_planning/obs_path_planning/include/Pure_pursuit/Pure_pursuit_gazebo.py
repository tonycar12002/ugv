#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Path, Odometry
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
from robotx_msgs.msg import ObstaclePoseList, Waypoint, WaypointList
import numpy as np
import math
import tf

class Pure_pursuit_gazebo(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.default_speed = 0.6
		self.speed = self.default_speed
		self.robot_pose = None #(x, y, heading)
		self.destination_pose = None
		#self.waypoints = [(7, 0),(7, 7),(0, 7),(0,0)]
		self.waypoints = []
		self.current_waypoint_index = 0
		self.distance_from_path = None
		self.lookahead_distance = rospy.get_param("~lookahead")
		self.threshold_proximity = 0.5      # How close the robot needs to be to the final waypoint to stop driving
		self.active = True
		self.start = True
		self.robot_go = False
		self.get_waypoint = True
		# Init subscribers and publishers
		#self.pub_cmd = rospy.Publisher('/car_cmd', Twist, queue_size=1)
		self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.pub_lookahead = rospy.Publisher('/pure_pursuit/lookahead', Point, queue_size=1)
		self.pub_finish = rospy.Publisher('/pure_pursuit/finished', Bool, queue_size=1)
		#self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.call_back, queue_size=10)
		self.sub_odom = rospy.Subscriber("/p3d_odom", Odometry, self.call_back, queue_size=10)
		self.sub_waypoint = rospy.Subscriber("/waypointList", WaypointList, self.waypoint_cb, queue_size = 10)	
		rospy.loginfo("[%s] Initialized ..." %(self.node_name))

	def initial_param(self):
		self.waypoints = []
		self.current_waypoint_index = 0
		self.distance_from_path = None
		self.active = True
		self.start = True
		self.get_waypoint = True

	# Waypoint List callback
	def waypoint_cb(self, msg):
		#print "way"
		self.initial_param()
		#print("Get waypoint list")
		if self.get_waypoint:
			for i in range(msg.size):
				self.waypoints.append([msg.list[i].x, msg.list[i].y])
			del self.waypoints[0] # remove current position
			#print "Waypoint: ", self.waypoints
			self.robot_go = True
			self.get_waypoint = False
		

	# Pose subscriber callback
	def call_back(self, msg):
		if not self.robot_go:
			return
		if not self.active:
			return 
		finish = Bool()
		finish.data = False
		self.pub_finish.publish(finish)
		# Get the orientation in quaternion and convert to euler angle
		quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		euler_angle = tf.transformations.euler_from_quaternion(quat)
		# Get robot pose, format in x, y, theta, and calculate the destination pose
		self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, euler_angle[2])
		self.destination_pose = self.pure_pursuit()
		# Reach
		if self.destination_pose == None:
			self.active = False
			rospy.loginfo("[%s]Approach destination" %(self.node_name))
			self.publish_cmd(0,0)
			msg = Bool()
			msg.data = True
			self.pub_finish.publish(msg)
		# Not reach yet
		else:
			self.publish_lookahead(self.destination_pose)
			self.speed = self.default_speed
			distance_to_destination= self.distanceBtwnPoints(self.robot_pose[0], self.robot_pose[1], \
															 self.destination_pose[0], self.destination_pose[1])
			angle_to_destination = -self.getAngle(self.robot_pose, self.destination_pose)
			w = ((angle_to_destination + math.pi) / (2 * math.pi) - 0.5)
			self.car_control(self.speed, w)
			

	def car_control(self, v, omega):
		#print omega
		'''if omega >= 0:
			w = omega * 0.85 + 0.15
		elif omega < 0:
			w = omega * 0.85 - 0.15
		#print w
		if abs(w) > 1:
			#print "Something Wrong"
			if w > 0:
				w = 1
			else:
				w = -1
		if w < abs(0.3):
			v = 0.07
		elif w < abs(0.4):
			v = 0.063
			#print "slightly slow down"
		elif w < abs(0.5):
			v = 0.058
			#print "slow down"
		else:
			v = 0.05
			#print "BIG TURN"
		#print w'''
		#print omega
		if abs(omega) > 0.2:
			v = 0.5
		if abs(omega) > 0.3:
			v = 0.4
		if abs(omega) > 0.4:
			v = 0.3
		print omega
		omega = omega*3.5
		self.publish_cmd(v, omega)

################################### Publish topic methods ###################################

	# Publish lookahead, type Point
	def publish_lookahead(self, lookahead):
		msg = Point()
		msg.x, msg.y = lookahead[:2]
		msg.z = 0
		self.pub_lookahead.publish(msg)

	# Publish car cmd, type Twist
	def publish_cmd(self, v, w):
		robot_twist_msg = Twist()
		robot_twist_msg.linear.x = v
		robot_twist_msg.linear.y = 0
		robot_twist_msg.linear.z = 0

		robot_twist_msg.angular.x = 0
		robot_twist_msg.angular.y = 0
		robot_twist_msg.angular.z = w
        
		self.pub_cmd.publish(robot_twist_msg)

############################## End  Publish topic methods ###################################

############################## Pure pursuit math methods ####################################

	# Calculate the distance between two points (x1, y1) and (x2, y2)
	def distanceBtwnPoints(self, x1, y1, x2, y2):
		return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

	# Tell if the point(x,y) is on the line segment (x_start,y_start) to (x_end,y_end)
	def isPointOnLineSegment(self, x, y, x_start, y_start, x_end, y_end):
		return round(self.distanceBtwnPoints(x_start, y_start, x, y) + self.distanceBtwnPoints(x, y, x_end, y_end), 4) \
												  == round(self.distanceBtwnPoints(x_start, y_start, x_end, y_end), 4)

	# Calculate the angle difference between robot heading and vector start from start_pose, end at end_pose and unit x vector of odom frame, 
	# in radian
	def getAngle(self, start_pose, end_pose):
		delta_x = end_pose[0] - start_pose[0]
		delta_y = end_pose[1] - start_pose[1]
		theta = start_pose[2]
		psi = np.arctan2(delta_y, delta_x)
		angle = theta - psi
		# Normalize in [-pi, pi)
		while angle >= np.pi:
			angle = angle - 2*np.pi
		while angle < -np.pi:
			angle = angle + 2*np.pi
		return angle

	# Find a point on the line which is closest to the point, robot poisition, and return the 
	# minimum distance
	def closestPoint(self, point, start, end):
		# Initialize values
		x = float(point[0])
		y = float(point[1])
		x_start = float(start[0])
		y_start = float(start[1])
		x_end = float(end[0])
		y_end = float(end[1])
		x_closest, y_closest = None, None
		shortest_distance = self.distanceBtwnPoints(x, y, self.waypoints[self.current_waypoint_index][0], self.waypoints[self.current_waypoint_index][1])

		# ======== Distance from a point to a line ========
		# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		# For line (segment) equation ax + by + c = 0
		a = y_start - y_end
		b = x_end - x_start
		if a**2 + b**2 == 0: # a=0 and b=0, or two points are the same
			return (None, None, None)
		c = -b*y_start - a*x_start  # Equivalently: x_start*y_end - x_end*y_start
		x_closest = (b*(b*x - a*y) - a*c)/(a**2 + b**2)
		y_closest = (a*(-b*x + a*y) - b*c)/(a**2 + b**2)
		distance = self.distanceBtwnPoints(x, y, x_closest, y_closest)
		# Check if the closest point is on the segment
		if not self.isPointOnLineSegment(x_closest, y_closest, x_start, y_start, x_end, y_end):
			return (None, None, None)
		return (x_closest, y_closest, distance)

	# Using lookahead to find out the waypoint
	def circleIntersect(self, point, start, end, lookahead_distance):
		# For circle equation (x - p)^2 + (y - q)^2 = r^2
		p = float(point[0])
		q = float(point[1])
		r = float(lookahead_distance)
		# Check line segments along path until intersection point is found or we run out of waypoints
		# For line (segment) equation y = mx + b
		x1 = float(start[0])
		y1 = float(start[1])
		x2 = float(end[0])
		y2 = float(end[1])
		# Not vertical line
		if x2 - x1 != 0:
			m = (y2 - y1)/(x2 - x1)
			b = y1 - m*x1
			# Quadratic equation to solve for x-coordinate of intersection point
			A = m**2 + 1
			B = 2*(m*b - m*q - p)
			C = q**2 - r**2 + p**2 - 2*b*q + b**2

			if B**2 - 4*A*C < 0:    # Circle does not intersect line
				return (None, None)
			# Points of intersection (could be the same if circle is tangent to line)
			x_intersect1 = (-B + math.sqrt(B**2 - 4*A*C))/(2*A)
			x_intersect2 = (-B - math.sqrt(B**2 - 4*A*C))/(2*A)
			y_intersect1 = m*x_intersect1 + b
			y_intersect2 = m*x_intersect2 + b
		# Vertical line
		else:
			x_intersect1 = x1
			x_intersect2 = x1
			y_intersect1 = q - math.sqrt(abs(-x1**2 + 2*x1*p - p**2 + r**2))
			y_intersect2 = q + math.sqrt(abs(-x1**2 + 2*x1*p - p**2 + r**2))

		# See if intersection points are on this specific segment of the line
		if self.isPointOnLineSegment(x_intersect1, y_intersect1, x1, y1, x2, y2):
			return (x_intersect1, y_intersect1)
		elif self.isPointOnLineSegment(x_intersect2, y_intersect2, x1, y1, x2, y2):
			return (x_intersect2, y_intersect2)
		return (None, None)

########################### End  Pure pursuit math methods ##################################

	# Pure pursuit process
	def pure_pursuit(self):
		x_robot, y_robot = self.robot_pose[:2]
		wp = self.waypoints
		cwpi = self.current_waypoint_index
		fake_robot_waypoint = (None, None)
		# If there is not waypoint, then end process
		if len(wp) == 0:
			rospy.loginfo("[%s]No target waypoint" %(self.node_name))
			return None
		# If there is only one waypoint left
		elif cwpi == len(wp)-1:
			# If the given lookahead distance is less than proximity threshold, change lookahead distance
			# to proximity threshold for safety, or the robot may circle around the final waypoint when 
			# almost reach
			if self.threshold_proximity < self.lookahead_distance:
				self.lookahead_distance = self.threshold_proximity
				rospy.loginfo("[%s]Change threshold distance for safety" %(self.node_name))
			x_endpoint, y_endpoint = wp[-1]
			# Distance from robot pose the the goal less than the given threshold value, then end the process
			if self.distanceBtwnPoints(x_endpoint, y_endpoint, x_robot, y_robot) <= self.threshold_proximity:
				return None
		# If distance from robot pose to next waypoint is less than the lookahead distance, then we reach this
		# waypoint
		if self.distanceBtwnPoints(x_robot, y_robot, wp[cwpi][0], wp[cwpi][1]) <= self.lookahead_distance :
			rospy.loginfo("[%s]Arrived waypoint: %d"%(self.node_name, cwpi))
			# If there are more than or equal to one waypoint left, then add the current_waypoint_index for next iteration
			if self.current_waypoint_index < len(self.waypoints)-1:
				self.current_waypoint_index = self.current_waypoint_index + 1
			# Else, return none
			else:
				return None
			# If this is the first time to run the process
			if self.start:
				self.start = False
		# If distance from robot pose to next waypoint is greater than the lookahead distance and start is True, 
		# then create a fake waypoint
		if self.start:
			fake_robot_waypoint = (x_robot, y_robot)
		# If start is False
		else:
			# Set fake waypoint as closest point
			fake_robot_waypoint = self.closestPoint(self.robot_pose, wp[cwpi-1], wp[cwpi])[:2]
			# If closest point is not on the line segment
			if fake_robot_waypoint == (None, None):
				# Set last waypoint as fake waypoint
				fake_robot_waypoint = (wp[cwpi-1][0], wp[cwpi-1][1])
		# Insert new fake waypoint, and remove waypoints which have been visited
		waypoints_to_search = [fake_robot_waypoint] + wp[cwpi : ]
		# Not use
		if self.lookahead_distance < self.distance_from_path:
			x_intersect, y_intersect = self.circleIntersect(self.robot_pose, waypoints_to_search[0], waypoints_to_search[1], self.distance_from_path)
		else:
			x_intersect, y_intersect = self.circleIntersect(self.robot_pose, waypoints_to_search[0], waypoints_to_search[1], self.lookahead_distance)
		if (x_intersect, y_intersect) == (None, None):
			if self.start:
				if self.distanceBtwnPoints(x_robot, y_robot, wp[cwpi][0], wp[cwpi][1]) <= self.lookahead_distance :
					rospy.loginfo("[%s]Arrived waypoint : %d" %(self.node_name, cwpi))
					self.current_waypoint_index = self.current_waypoint_index + 1
					self.start = False
			return fake_robot_waypoint
		return (x_intersect, y_intersect)