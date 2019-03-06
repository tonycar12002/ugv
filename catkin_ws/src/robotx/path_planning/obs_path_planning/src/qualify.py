#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import atan2, cos, sin, sqrt, tan, hypot, pi
from scipy.spatial import Delaunay
from robotx_msgs.msg import ObjectPoseList
from robotx_msgs.msg import Waypoint, WaypointList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from robotx_msgs.srv import *
from std_srvs.srv import *
from std_msgs.msg import Int32
from robotx_msgs.srv import *
from std_srvs.srv import *
#from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class qualify(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		self.debug = False
		self.frame_id = 'odom'
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		self.sub_obstacleList = rospy.Subscriber("/obj_list/map", ObjectPoseList, self.call_back, queue_size=1, buff_size = 2**24)
		self.pub_rviz = rospy.Publisher("/gate_middle", Marker, queue_size = 1)
		self.gate_dis_max = 15
		self.gate_dis_min = 7
		self.forward_dis = 3
		self.wp_dis = 45
		self.enter_gate_safe_dis = 8
		self.wp1_dis = 2
		self.wp2_dis = 5
		self.robot = None
		self.orientation = None
		self.gate_idx = 1
		self.wp = []
		self.state = 1
		print self.state

	def distance(self, p1, p2):
		return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def angle_range(self, angle):
		if angle > 180:
			angle = angle - 360
			angle = self.angle_range(angle)
		elif angle < -180:
			angle = angle + 360
			angle = self.angle_range(angle)
		return angle

	def angle_from_robot(self, p):
		robot_angle = np.degrees(self.orientation)
		p1 = [self.robot[0], self.robot[1]]
		p2 = [self.robot[0], self.robot[1]+1.]
		p3 = p
		angle = self.get_angle(p1, p2, p3)
		result = angle - robot_angle
		result = self.angle_range(result)
		return result

	def get_angle(self, p1, p2, p3):
		v0 = np.array(p2) - np.array(p1)
		v1 = np.array(p3) - np.array(p1)
		angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
		return np.degrees(angle)

	def is_front(self, p):
		angle = self.angle_from_robot(p)
		if abs(angle) < 90:
			return True
		else:
			return False

	def clock_side(self, p1, p2, p3):
		v1 = [p2[0] - p1[0], p2[1] - p1[1]]
		v2 = [p3[0] - p1[0], p3[1] - p1[1]]
		v1_v2 = np.cross(v1, v2)
		if v1_v2 < 0:
			return 1
		else:
			return -1
	
	def shift_point(self, p1, p2, robot, d, op_side):
		c = [(p1[0]+p2[0])/2., (p1[1]+p2[1])/2.]
		m = (p1[0] - p2[0]) / (p2[1] - p1[1])
		t = d / sqrt(1 + m**2)
		target1 = [c[0] + t, c[1] + m*t]
		target2 = [c[0] - t, c[1] - m*t]
		side1 = self.clock_side(p1, p2, target1)
		side2 = self.clock_side(p1, p2, target2)
		side = self.clock_side(p1, p2, robot)
		if op_side:
			if side * side1 < 0:
				return target1
			elif side * side2 < 0:
				return target2
		else:
			if side * side1 > 0:
				return target1
			elif side * side2 > 0:
				return target2

	def fsm(self):
		if self.wp == []:
			return
		if self.state == 1:	# ready to go
			if (self.distance(self.wp[0], self.robot) < 2):
				self.state = 2
				print self.state
		elif self.state == 2:	# enter first gate for safe distance
			if (self.distance(self.wp[1], self.robot) < 2):
				self.state = 3
				print self.state
		elif self.state == 3:	# enter first gate
			if (self.distance(self.wp[2], self.robot) < 3):
				self.state = 4
				print self.state
		elif self.state == 4:	# ready to go to second gate
			# update new self.wp
			if (self.distance(self.wp[0], self.robot) < 2):
				self.state = 5
				print self.state
		elif self.state == 5:	# enter second gate
			self.wp_dis = 8

	def call_back(self, msg):
		self.robot = [msg.robot.position.x, msg.robot.position.y]
		q = [msg.robot.orientation.x, msg.robot.orientation.y, msg.robot.orientation.z, msg.robot.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		yaw = euler[2]
		self.orientation = yaw
		self.obstacle_list = msg
		self.totems = []
		if self.state == 1:
			self.fsm()
			self.find_wp()
		elif self.state == 2:
			self.fsm()
		elif self.state == 3:
			self.find_wp()
			self.fsm()
		elif self.state == 4:
			self.fsm()
			self.find_wp()
		elif self.state == 5:
			self.fsm()
		
	def find_wp(self):
		gates = []
		for i in range(len(self.obstacle_list.list)):
			if self.obstacle_list.list[i].type == 'totem' or self.obstacle_list.list[i].type == 'light_buoy':
				self.totems.append(self.obstacle_list.list[i])
		for totem_i in self.totems:
			p_i = [totem_i.position.x, totem_i.position.y]
			for totem_j in self.totems:
				p_j = [totem_j.position.x, totem_j.position.y]
				is_gate = (self.distance(p_i, p_j) < self.gate_dis_max) and (self.distance(p_i, p_j) > self.gate_dis_min)
				if is_gate and not p_i == p_j:
					new_gate = True
					for gate in gates:
						if p_i in gate and p_j in gate:
							new_gate = False
					if new_gate:
						gates.append([p_i, p_j])
		middle_pts = []
		closet_gate = None
		closet_mid = None
		closet_dis = 10e5
		for p in gates:
			middle = [(p[0][0]+p[1][0])/2., (p[0][1]+p[1][1])/2.]
			if self.is_front(middle):
				if self.distance(middle, self.robot) < closet_dis:
					closet_dis = self.distance(middle, self.robot)
					closet_mid = middle
					closet_gate = p
				middle_pts.append(closet_mid)

		if closet_gate is not None:
			self.wp = []
			shft_pt1 = self.shift_point(closet_gate[0], closet_gate[1], self.robot, self.forward_dis, False)
			shft_pt2 = self.shift_point(closet_gate[0], closet_gate[1], self.robot, self.enter_gate_safe_dis, True)
			shft_pt3 = self.shift_point(closet_gate[0], closet_gate[1], self.robot, self.wp_dis, True)
			self.wp.append(shft_pt1)
			self.wp.append(shft_pt2)
			self.wp.append(shft_pt3)
			self.rviz(self.wp)
			self.call_controller()
			if self.state == 3:
				self.state = 4
				print self.state
	
	def call_controller(self):
		clear_wp = rospy.ServiceProxy("/clear_waypoints", Trigger)
		try:
			rospy.wait_for_service("/clear_waypoints")
			clear_wp()
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
		add_wp = rospy.ServiceProxy("/add_waypoint", waypoint)
		for i in range(len(self.wp)):
			if i != 0:
				wp_srv = waypointRequest()
				wp_srv.waypointx = self.wp[i][0]
				wp_srv.waypointy = self.wp[i][1]
				res = waypointResponse()
				try:
					rospy.wait_for_service("/add_waypoint")
					res.waypoint_len = add_wp(wp_srv)
				except (rospy.ServiceException, rospy.ROSException), e:
					rospy.logerr("Service call failed: %s" % (e,))
		start_motion = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
		start_motion()
		
	def rviz(self, points):
		marker = Marker()
		marker.header.frame_id = self.frame_id
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.points = []
		for point in points:
			p = Point()
			p.x = point[0]
			p.y = point[1]
			p.z = 0
			marker.points.append(p)
		self.pub_rviz.publish(marker)
	
if __name__ == "__main__":
	rospy.init_node("qualify", anonymous = False)
	qualify = qualify()
	rospy.spin()
