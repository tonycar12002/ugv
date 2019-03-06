#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2, cos, sin, sqrt
from scipy.spatial import Delaunay
from robotx_msgs.msg import ObstaclePoseList
from robotx_msgs.msg import Waypoint, WaypointList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from robotx_msgs.srv import *
from std_srvs.srv import *
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class PathPlanning(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		#self.sub_obstacleList = rospy.Subscriber("/obstacle_list", ObstaclePoseList, self.cb_obstacle, queue_size=1)
		#self.sub_odom		  = rospy.Subscriber("/odometry/filtered", Odometry, self.call_back, queue_size=10)
		sub_obstacleList = Subscriber("/obstacle_list/map", ObstaclePoseList)
		#sub_odom = Subscriber("/odometry/filtered", Odometry)
		sub_odom = Subscriber("/odometry/filtered", Odometry)
		ats = ApproximateTimeSynchronizer((sub_obstacleList, sub_odom),queue_size = 1, slop = 0.1)
		ats.registerCallback(self.call_back)
		#self.sub_goal		  = rospy.Subscriber("/goal", WaypointList, self.cb_wplist, queue_size = 1)
		self.pub_rviz		  = rospy.Publisher("/wp_line", Marker, queue_size = 1)
		self.start_point = [0, 0]
		self.old_wp_list_len = 0
		x = rospy.get_param("~x", 10)
		y = rospy.get_param("~y", 10)
		self.goal_point = [x, y]
		self.safe_dis = rospy.get_param("~safe_dis",5)
		self.init_param()
		print "The Goal Point is: ", self.goal_point
		print "Safe Distance is: ", self.safe_dis

	def init_param(self):
		self.iteration = 0
		self.lock = False
		self.plan_done = False
		self.p_now = self.start_point
		self.p_next = self.goal_point
		self.car_length = 1
		self.wp_index = 0
		self.change_side = False
		self.check_avoid_point = False
		self.frame_id = "odom"
		self.process_obs_list = []
		self.max_pos_angle = 0
		self.max_neg_angle = 0
		# Waypoint list
		self.wp_list = [self.start_point, self.goal_point]
		self.waypoint_size = 0
		self.waypoint_list = WaypointList()
		self.waypoint_list.header.frame_id = self.frame_id

	def call_back(self, obstacle_msg, odom_msg):
		self.start_point = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
		print "Recieve obstacle list"
		self.init_param()
		self.obstacle_list = obstacle_msg
		if not self.lock:
			self.plan_done = False
			self.lock = True
			print "Start Path Planning"
			while not self.plan_done:
				self._process()
				#self.rviz_debug()
				#rospy.sleep(3)
			self.lock = False
			#print self.wp_list
			

	def line(self, p1, p2):
		# y = Ax + B
		A = (p2[1] - p1[1]) / (p2[0] - p1[0])
		B = -A * p1[0] + p1[1]
		return A, B

	def distance(self, p1, p2):
		return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def point_on_both_line(self, a1, a2, b1, b2, ans):
		if round(self.distance(a1, a2), 4) == round(self.distance(ans, a1) + self.distance(ans, a2), 4):
			if round(self.distance(b1, b2), 4) == round(self.distance(ans, b1) + self.distance(ans, b2), 4):
				return True
			else:
				return None
		return False

	def check_intersect(self, a1, a2, b1, b2, get_intersection):
		# if two lines are both perpendicular
		if a1[0] - a2[0] == 0 and b1[0] - b2[0] == 0:
			if a1[0] == b1[0]:
				return True	# if two line coincedent
			return False	# if two lines parrellel

		###### UNDO: if they are on the same line ######

		if a1[0] - a2[0] == 0:	# a1a2 perpendicular
			L = self.line(b1, b2)
			x = a1[0]
			y = L[0]*x + L[1]
			ans = [x, y]
			if get_intersection:
				return ans
			return self.point_on_both_line(a1, a2, b1, b2, ans)
		if b1[0] - b2[0] == 0:	# b1b2 perpendicular
			L = self.line(a1, a2)
			x = b1[0]
			y = L[0]*x + L[1]
			ans = [x, y]
			if get_intersection:
				return ans
			return self.point_on_both_line(a1, a2, b1, b2, ans)
		# L1 : y = ax + b
		# L2 : y = cx + d
		# intersection(x, y):
		# x = -((b-d) / (a-c))
		# y = (ad-bc) / (a-c)
		L1 = self.line(a1, a2)
		L2 = self.line(b1, b2)
		if L1[0] == L2[0]:
			return False
			###### if they are on the same line ######
		x = -((L1[1] - L2[1]) / (L1[0] - L2[0]))
		y = (L1[0]*L2[1] - L1[1]*L2[0]) / (L1[0] - L2[0])
		ans = [x, y]
		if get_intersection:
			return ans
		return self.point_on_both_line(a1, a2, b1, b2, ans)

	def get_angle(self, p1, p2, p3):
		v0 = np.array(p2) - np.array(p1)
		v1 = np.array(p3) - np.array(p1)
		angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
		return np.degrees(angle)

	def dis_point2line(self, p1, p2, p3):
		p1 = np.array(p1)
		p2 = np.array(p2)
		p3 = np.array(p3)
		return np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)

	# dis -> shift distance, m -> slope, ,p -> shifted point
	# d > 0 right side, d < 0 left side, d = 0 on the line
	def shift_point(self, dis, m, d, p):
		if m == None:	# the slope is vertical
			if d > 0:	# point is above the line
				return [p[0], p[1] + dis]
			else:
				return [p[0], p[1] - dis]
		k = sqrt(dis**2 / (1 + m**2))
		if d < 0:
			k = -k
		new_point = [p[0]+k, p[1]+k*m]
		return new_point

	def find_shift_point(self, p_now, p_next, dis, p_new):
		m = None
		d = None
		if p_now[0] - p_next[0] == 0:	# if the line is vertical
			m = 0
			d = (p_new[0] - p_now[0]) - (p_new[1] - p_now[1])*(p_next[0] - p_now[0])/(p_next[1] - p_now[1])
		else:
			origin_m = self.line(p_now, p_next)
			#print origin_m[0]
			if origin_m[0] != 0:	# if the shift direction is not vertical
				# calculate the shift slope (vector)
				m = -(1.0 / origin_m[0])
				# determine the point is on which side of the line
				d = (p_new[0] - p_now[0]) - (p_new[1] - p_now[1])*(p_next[0] - p_now[0])/(p_next[1] - p_now[1])
				#print d
			else:	# if the shift direction is vertical
				if p_new[1] > p_now[1]:	# point is above the line
					d > 0
				else:
					d < 0
		if dis < 0:
			d = -d
			dis = -dis
		return self.shift_point(dis, m, d, p_new)

	'''def cb_wplist(self, wp_list_msg):
		length = len(wp_list_msg.list)
		self.X_DIM = wp_list_msg.list[length-1].x
		self.Y_DIM = wp_list_msg.list[length-1].y'''

	def _process(self):
		self.iteration = self.iteration + 1
		if self.iteration > 50:
			self.plan_done = True
			return
		#print self.p_now, self.p_next
		for obs_index in range(self.obstacle_list.size):
			if obs_index in self.process_obs_list:
				pass
			collision = False
			intersect_count = 0
			intersect_p1 = None
			intersect_p2 = None
			may_be_p1 = None
			may_be_p2 = None
			p_new = []
			p1 = [self.obstacle_list.list[obs_index].x_min_x, self.obstacle_list.list[obs_index].x_min_y]
			p2 = [self.obstacle_list.list[obs_index].y_min_x, self.obstacle_list.list[obs_index].y_min_y]
			p3 = [self.obstacle_list.list[obs_index].x_max_x, self.obstacle_list.list[obs_index].x_max_y]
			p4 = [self.obstacle_list.list[obs_index].y_max_x, self.obstacle_list.list[obs_index].y_max_y]
			obs_vertex = [p1]
			######## UNDO: deal with the situation that the line only intersect a single vertex ########
			if not p2 in obs_vertex :
				obs_vertex.append(p2)
			if not p3 in obs_vertex :
				obs_vertex.append(p3)
			if not p4 in obs_vertex :
				obs_vertex.append(p4)
			#Check every segment of the obstacle will intersect with the path or not
			for i,j in zip(obs_vertex[0::], obs_vertex[1::]):
				check_line_intersect = self.check_intersect(i, j, self.p_now, self.p_next, False)
				if check_line_intersect == True:
					collision = True
					intersect_count = intersect_count + 1
					intersect_p1 = i
					intersect_p2 = j
				if check_line_intersect is None:
					may_be_p1 = i
					may_be_p2 = j
			#This two vertex won't be consider in previous for loop
			check_line_intersect = self.check_intersect(obs_vertex[-1], obs_vertex[0], self.p_now, self.p_next, False)
			if check_line_intersect == True:
				collision = True
				intersect_count = intersect_count + 1
				intersect_p1 = i
				intersect_p2 = j
			if check_line_intersect is None:
				may_be_p1 = obs_vertex[-1]
				may_be_p2 = obs_vertex[0]

			# if self.p_next is in the obstacle!!!
			if intersect_count == 1 and len(obs_vertex) > 2:
				if self.check_avoid_point:
					self.wp_list.remove(self.p_next)
					self.p_next = self.wp_list[self.wp_index + 1]
					self.change_side = True
					return #Choose another path
				p_intersect = self.check_intersect(may_be_p1, may_be_p2, self.p_now, self.p_next, True)
				d = self.dis_point2line(intersect_p1, intersect_p2, p_intersect) + self.safe_dis
				self.p_next = self.find_shift_point(intersect_p1, intersect_p2, d, self.p_next)
				self.wp_list[self.wp_index + 1] = self.p_next
				return
			self.check_avoid_point = False

			if collision:
				obs_angle_list = []
				angle_list_positive = []
				angle_list_negetive = []
				second_angle = None
				for i in range(len(obs_vertex)):
					obs_angle_list.append([obs_vertex[i], self.get_angle(self.p_now, self.p_next, obs_vertex[i])])
					if obs_angle_list[i][1] >= 0:
						angle_list_positive.append(obs_angle_list[i][1])
					else:
						angle_list_negetive.append(obs_angle_list[i][1])
				angle_list_positive.sort()
				angle_list_negetive.sort()
				if abs(angle_list_positive[-1]) < abs(angle_list_negetive[0]):
					if self.change_side:
						second_angle = angle_list_negetive[0]
						print "change"
						self.change_side = False
						self.process_obs_list.append(obs_index)
						self.max_pos_angle = angle_list_negetive[0]
						self.max_neg_angle = angle_list_positive[-1]
					else:
						second_angle = angle_list_positive[-1]
				else:
					if self.change_side:
						second_angle = angle_list_positive[-1]
						print "change"
						self.change_side = False
						self.process_obs_list.append(obs_index)
						self.max_pos_angle = angle_list_negetive[0]
						self.max_neg_angle = angle_list_positive[-1]
					else:
						second_angle = angle_list_negetive[0]

				for i in range(len(obs_vertex)):
					if obs_angle_list[i][1] == second_angle:
						p_new = obs_angle_list[i][0]
						break

				p_new = self.find_shift_point(self.p_now, self.p_next, self.safe_dis + 0.1, p_new)
				#print "A"
				self.wp_list.insert(self.wp_index + 1, p_new)
				self.p_next = p_new
				self.plan_done = False
				self.check_avoid_point = True
				return
			min_dis = 1e6
			closest_p = None
			for i in obs_vertex:
				dis_P2L = self.dis_point2line(self.p_now, self.p_next, i)	# distance from a point to the line
				if dis_P2L < min_dis:
					min_dis = dis_P2L
					closest_p = i
			if min_dis < self.safe_dis:
				#print "------", min_dis, self.safe_dis
				if abs(self.get_angle(self.p_now, self.p_next, closest_p)) < 90 and abs(self.get_angle(self.p_next, self.p_now, closest_p)) < 90:
					# shift to another side
					#print self.get_angle(self.p_now, self.p_next, i), self.get_angle(self.p_next, self.p_now, i)
					#print "----", closest_p
					#print self.p_now, self.p_next
					self.check_avoid_point = True
					p_new = self.find_shift_point(self.p_now, self.p_next, -self.safe_dis - 0.1, closest_p)
					#print "B"
					self.wp_list.insert(self.wp_index + 1, p_new)
					self.p_next = p_new
					#print "it,s", self.p_next
					self.plan_done = False
					return
		if self.p_next != self.wp_list[-1]:
			self.wp_index = self.wp_index + 1
			self.p_now = self.wp_list[self.wp_index]
			self.p_next = self.wp_list[self.wp_index+1]
			self.plan_done = False
			#print "ok"
			return
		for i in self.wp_list:
			self.waypoint_size = self.waypoint_size + 1
			wp = Waypoint()
			wp.x = i[0]
			wp.y = i[1]
			wp.z = 0
			#print wp.x, wp.y
			self.waypoint_list.list.append(wp)
		self.waypoint_list.size = self.waypoint_size
		self.waypoint_list.header.frame_id = self.frame_id
		self.waypoint_list.header.stamp = rospy.Time.now()
		self.pub_waypointList.publish(self.waypoint_list)
		self.plan_done = True
		self.rviz()
		#if self.old_wp_list_len != len(self.waypoint_list.list):
		#print self.old_wp_list_len, len(self.waypoint_list.list)
		self.old_wp_list_len = len(self.waypoint_list.list)
		clear_wp = rospy.ServiceProxy("/clear_waypoints", Trigger)
		try:
			rospy.wait_for_service("/clear_waypoints")
			clear_wp()
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
		add_wp = rospy.ServiceProxy("/add_waypoint", waypoint)
		for i in range(self.waypoint_size):
			if i != 0:
				wp_srv = waypointRequest()
				wp_srv.waypointx = self.waypoint_list.list[i].x
				wp_srv.waypointy = self.waypoint_list.list[i].y
				res = waypointResponse()
				try:
					rospy.wait_for_service("/add_waypoint")
					res.waypoint_len = add_wp(wp_srv)
				except (rospy.ServiceException, rospy.ROSException), e:
					rospy.logerr("Service call failed: %s" % (e,))
		start_motion = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
		start_motion()
		print "Finish path planning"

	def rviz_debug(self):
		self.waypoint_list = WaypointList()
		self.waypoint_list.header.frame_id = self.frame_id
		self.waypoint_size = 0
		for i in self.wp_list:
			self.waypoint_size = self.waypoint_size + 1
			waypoint = Waypoint()
			waypoint.x = i[0]
			waypoint.y = i[1]
			waypoint.z = 0
			#print waypoint.x, waypoint.y
			self.waypoint_list.list.append(waypoint)
		self.waypoint_list.size = self.waypoint_size
		marker = Marker()
		marker.header.frame_id = self.frame_id
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.a = 1.0
		marker.color.r = 0
		marker.color.g = 1.0
		marker.color.b = 0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0
		marker.points = []
		for i in range(self.waypoint_size):
			p = Point()
			#print i
			#print self.waypoint_size, i, self.waypoint_list.list[i].x
			p.x = self.waypoint_list.list[i].x
			p.y = self.waypoint_list.list[i].y
			p.z = self.waypoint_list.list[i].z
			marker.points.append(p)
		self.pub_rviz.publish(marker)

	def rviz(self):
		marker = Marker()
		marker.header.frame_id = self.frame_id
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.a = 1.0
		marker.color.r = 0
		marker.color.g = 1.0
		marker.color.b = 0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0
		marker.points = []
		for i in range(self.waypoint_size):
			p = Point()
			#print i
			#print self.waypoint_size, i, self.waypoint_list.list[i].x
			p.x = self.waypoint_list.list[i].x
			p.y = self.waypoint_list.list[i].y
			p.z = self.waypoint_list.list[i].z
			marker.points.append(p)
		self.pub_rviz.publish(marker)
	
if __name__ == "__main__":
	rospy.init_node("Path_planning_node", anonymous = False)
	rrt_planning = PathPlanning()
	rospy.spin()
