#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import atan2, cos, sin, sqrt, tan, hypot, pi
from scipy.spatial import Delaunay
from robotx_msgs.msg import ObjectPoseList
from robotx_msgs.msg import Waypoint, WaypointList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from robotx_msgs.srv import *
from std_srvs.srv import *
from std_msgs.msg import Int32 
#from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class PathPlanning(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		self.debug = False
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		self.sub_obstacleList = rospy.Subscriber("/obj_list/map", ObjectPoseList, self.call_back, queue_size=1, buff_size = 2**24)
		self.sub_nav_state = rospy.Subscriber("/wp_nav_state", Int32, self.nav_state, queue_size=1)
		self.sub_new_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.call_new_goal, queue_size=1)
		self.new_goal = rospy.Service("/new_goal", waypoint, self.new_goal)
		self.pub_final_point = rospy.Publisher("/final_point", PoseStamped, queue_size = 1)
		self.pub_rviz = rospy.Publisher("/wp_line", Marker, queue_size = 1)
		self.pubish_p = rospy.Publisher("/ppp", Marker, queue_size = 1)
		self.pubish_pp = rospy.Publisher("/pppp", Marker, queue_size = 1)
		self.pubish_ppp = rospy.Publisher("/ppppp", Marker, queue_size = 1)
		self.pubish_pppp = rospy.Publisher("/pppppp", Marker, queue_size = 1)
		self.start_point = [0, 0]
		self.orientation = None
		self.old_wp_list_len = 0
		self.arrive_dis = rospy.get_param("~arrive_dis", 2)
		#x = rospy.get_param("~x", 100)
		#y = rospy.get_param("~y", 10)
		#self.goal_point = [x, y]
		self.count_call = 0
		self.goal_point = None
		self.goal_orientation = None
		self.get_goal = False
		self.station_keeping = True
		self.state = 0
		self.count_update_goal = 0
		self.update_goal = False
		self.arrived = False
		self.safe_dis = rospy.get_param("~safe_dis",4)
		self.cluster_dis = 2.*self.safe_dis + 2
		self.init_param()
		#print "The Goal Point is: ", self.goal_point
		print "Cluster distance is: ", self.cluster_dis
		print "Safe distance is: ", self.safe_dis
		print "======================"

	def init_param(self):
		self.is_gate = False
		self.key1 = [0, 0]
		self.key2 = [0, 0]
		self.count_safe = 0
		self.pre_cluster = []
		self.danger_point = False
		self.pub_point = Point()
		self.pub_pointt = Point()
		self.first = False
		self.already_first = False
		self.iteration = 0
		self.lock = False
		self.plan_done = False
		self.p_now = self.start_point
		self.p_next = self.goal_point
		self.car_length = 1
		self.wp_index = 0
		self.check_avoid_point = False
		self.frame_id = "odom"
		self.process_obs_list = []
		self.gate_point1 = None
		self.gate_point2 = None
		self.correct_waypoint = False
		self.switch = True
		self.process_obs = [None, None]
		self.pre_obs = None
		# Waypoint list
		self.wp_list = [self.start_point, self.goal_point]
		self.wp_obs_idx = [None, None]
		self.waypoint_size = 0
		self.waypoint_list = WaypointList()
		self.waypoint_list.header.frame_id = self.frame_id

	def nav_state(self, msg):
		self.state = msg.data

	def safe_goal_point(self):
		for i in range(self.obstacle_list.size):
			p = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
			dis = self.distance(p, self.goal_point)
			if dis < self.obs_size(i) + self.safe_dis:
				print('Shift goal point for safe')
				shift_dis = self.obs_size(i) + self.safe_dis + 0.5
				x = self.goal_point[0] - p[0]
				y = self.goal_point[1] - p[1]
				x_new = x*shift_dis/dis + p[0]
				y_new = y*shift_dis/dis + p[1]
				self.goal_point = [x_new, y_new]

	def call_new_goal(self, p):
		self.goal_point = [p.pose.position.x, p.pose.position.y]
		self.safe_goal_point()
		q = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.goal_orientation = euler[2]
		self.get_goal = True
		self.update_goal = True
		self.arrived = False
		self.state = 0

	def new_goal(self, req):
		self.goal_point = [req.waypointx, req.waypointy]
		self.safe_goal_point()
		self.goal_orientation = req.yaw
		self.get_goal = True
		self.update_goal = True
		self.arrived = False
		self.state = 0
		print "Update New Goal: ", self.goal_point
		return len(self.goal_point)

	def arrive_goal(self):
		if self.goal_point is None:
			return True
		dis = self.distance(self.start_point, self.goal_point)
		if dis <= self.arrive_dis:
			if not self.arrived:
				print "Arrived goal point: ", self.goal_point
			return True
		return False

	def call_back(self, msg):
		self.start_point = [msg.robot.position.x, msg.robot.position.y]
		q = [msg.robot.orientation.x, msg.robot.orientation.y, msg.robot.orientation.z, msg.robot.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		yaw = euler[2]
		self.orientation = yaw
		self.init_param()
		
		self.obstacle_list = msg
		#self._process()
		
		if self.state == 1:
			self.station_keeping = True
		if self.update_goal:
			self.count_update_goal = self.count_update_goal + 1
			self.station_keeping = False
			if self.count_update_goal > 10:
				self.count_update_goal = 0
				self.update_goal = False
		if self.station_keeping:
			rospy.loginfo('Station Keeping')
		#self.arrived = self.arrive_goal()
		if not self.lock and self.get_goal and not self.arrived and not self.station_keeping:
			#self.init_param()
			self.plan_done = False
			self.lock = True
			print "Start Path Planning"
			while not self.plan_done:
				self._process()
				if self.debug:
					self.rviz_debug()
					rospy.sleep(1.5)
			self.pub_p()
			self.lock = False

	def line(self, p1, p2):
		# y = Ax + B
		A = (p2[1] - p1[1]) / (p2[0] - p1[0])
		B = -A * p1[0] + p1[1]
		return A, B

	def distance(self, p1, p2):
		return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def point_within_line(self, a1, a2, p):
		if abs(self.get_angle(a1, a2, p)) < 90:
			if abs(self.get_angle(a2, a1, p)) < 90:
				return True
		return False

	def point_on_both_line(self, a1, a2, b1, b2, ans):
		if round(self.distance(a1, a2), 4) == round(self.distance(ans, a1) + self.distance(ans, a2), 4):
			if round(self.distance(b1, b2), 4) == round(self.distance(ans, b1) + self.distance(ans, b2), 4):
				return True
		return False


	#==========================================
	# False, None --> no intersect
	# True,  ans  --> Intersect with intersection
	# True,  None --> Coincedent
	#==========================================
	def check_intersect(self, a1, a2, b1, b2):
		#==========================================
		# if both lines are vertical
		#==========================================
		if a1[0] - a2[0] == 0 and b1[0] - b2[0] == 0:
			if a1[0] == b1[0]:
				return True, None	# if two line coincedent
			return False, None	# if two lines parrellel

		###### UNDO: if they are on the same line ######
		#==========================================
		# if one of the line is vertical
		#==========================================
		if a1[0] - a2[0] == 0:	# a1a2 perpendicular
			L = self.line(b1, b2)
			x = a1[0]
			y = L[0]*x + L[1]
			ans = [x, y]
			return self.point_on_both_line(a1, a2, b1, b2, ans), ans
		if b1[0] - b2[0] == 0:	# b1b2 perpendicular
			L = self.line(a1, a2)
			x = b1[0]
			y = L[0]*x + L[1]
			ans = [x, y]
			return self.point_on_both_line(a1, a2, b1, b2, ans), ans
		#==========================================
		# Check two unvertical lines
		#==========================================
		# L1 : y = ax + b
		# L2 : y = cx + d
		# intersection(x, y):
		# x = -((b-d) / (a-c))
		# y = (ad-bc) / (a-c)
		L1 = self.line(a1, a2)
		L2 = self.line(b1, b2)
		# It they have same slope
		if L1[0] == L2[0]:
			if L1[1] == L2[1]:
				return True, None # They are same line
			return False, None # They are parrellel
		x = -((L1[1] - L2[1]) / (L1[0] - L2[0]))
		y = (L1[0]*L2[1] - L1[1]*L2[0]) / (L1[0] - L2[0])
		ans = [x, y]
		return self.point_on_both_line(a1, a2, b1, b2, ans), ans

	def inside_vectors(self, p1, p2, obs):
		v1 = [p1[0] - self.p_now[0], p1[1] - self.p_now[1]]
		v2 = [p2[0] - self.p_now[0], p2[1] - self.p_now[1]]
		v_obs = [obs[0] - self.p_now[0], obs[1] - self.p_now[1]]
		v1_obs = np.cross(v1, v_obs)
		obs_v2 = np.cross(v_obs, v2)
		if v1_obs < 0 and obs_v2 < 0:
			return True
		else:
			return False

	def clock_side(self, p1, p2, p3):
		v1 = [p2[0] - p1[0], p2[1] - p1[1]]
		v2 = [p3[0] - p1[0], p3[1] - p1[1]]
		v1_v2 = np.cross(v1, v2)
		if v1_v2 < 0:
			return True
		else:
			return False

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
		p1 = [self.start_point[0], self.start_point[1]]
		p2 = [self.start_point[0], self.start_point[1]+1.]
		p3 = p
		angle = self.get_angle(p1, p2, p3)
		result = angle - robot_angle
		result = self.angle_range(result)
		#print result
		return result


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

	def dis_point2segment(self, p1, p2, p3):
		if self.point_within_line(p1, p2, p3):
			return self.dis_point2line(p1, p2, p3), True
		else:
			dis_1 = self.distance(p1, p3)
			dis_2 = self.distance(p2, p3)
			if dis_1 < dis_2:
				return dis_1, False
			return dis_2, False

	# dis -> shift distance, m -> slope, ,p -> shifted point
	# d > 0 right side, d < 0 left side, d = 0 on the line
	def shift_point(self, dis, m, d, p, p_now, move):
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

	def find_shift_point(self, p_now, p_next, dis, p_new, move):
		m = None
		d = None
		if p_now[0] - p_next[0] == 0:	# if the line is vertical
			m = 0
			d = (p_new[0] - p_now[0]) - (p_new[1] - p_now[1])*(p_next[0] - p_now[0])/(p_next[1] - p_now[1])
		else:
			origin_m = self.line(p_now, p_next)
			if origin_m[0] != 0:	# if the shift direction is not vertical
				# calculate the shift slope (vector)
				m = -(1.0 / origin_m[0])
				# determine the point is on which side of the line
				d = (p_new[0] - p_now[0]) - (p_new[1] - p_now[1])*(p_next[0] - p_now[0])/(p_next[1] - p_now[1])
			else:	# if the shift direction is vertical
				if p_new[1] > p_now[1]:	# point is above the line
					d = 1
				else:
					d = -1
		if move:
			d = -d
		return self.shift_point(dis, m, d, p_new, p_now, move)

	def cluster_obs(self):
		# if there isn't any obstacle
		if self.obstacle_list.size == 0 :
			return None
		cluster = []
		index = None
		cluster_index = None
		for i in range(self.obstacle_list.size):
			append_i = True
			for c in cluster:
				if i in c:
					append_i = False
			if append_i:
				cluster.append([i])
			i_being_clustered = False
			for j in range(self.obstacle_list.size):
				j_being_clustered = False
				p_i = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
				p_j = [self.obstacle_list.list[j].position.x, self.obstacle_list.list[j].position.y]
				dis = self.obs_size(i) + self.obs_size(j) + self.cluster_dis
				if i != j and self.distance(p_i, p_j) < dis:
					i_being_clustered = True
					cluster_index = None
					for c in cluster:
						# i in c --> make sure which cluster is
						# j not in c --> make sure not to add duplicate obstacle in same cluster
						if i in c and j not in c and not j_being_clustered:
							j_being_clustered = True
							cluster_index = cluster.index(c)
							cluster[cluster_index].append(j)
						elif i in c and j not in c and j_being_clustered:
							cluster[cluster_index] = cluster[cluster_index] + cluster[cluster.index(c)]
							del cluster[cluster.index(c)]
						elif i in c and j in c:
							cluster_index = cluster.index(c)
							j_being_clustered = True
						elif j in c and i not in c:
							j_being_clustered = True
							cluster_index = cluster.index(c)
							cluster[cluster_index] = cluster[cluster_index] + cluster[-1]
							del cluster[-1]
			#if i_being_clustered and len(cluster[-1])==1:
			#	del cluster[-1]
		count = 0
		for i in cluster:
			for c in i:
				count = count + 1
		#print(self.obstacle_list.size, count)
		#print(len(cluster))
		return cluster

	def clockwiseangle_and_distance(self, point):
		origin = self.p_next
		refvec = [0, 1]
		# Vector between point and the origin: v = p - o
		vector = [point[0]-origin[0], point[1]-origin[1]]
		# Length of vector: ||v||
		lenvector = hypot(vector[0], vector[1])
		# If length is zero there is no angle
		if lenvector == 0:
			return -np.pi, 0
		# Normalize vector: v/||v||
		normalized = [vector[0]/lenvector, vector[1]/lenvector]
		dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
		diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
		angle = atan2(diffprod, dotprod)
		# Negative angles represent counter-clockwise angles so we need to subtract them 
		# from 2*pi (360 degrees)
		if angle < 0:
			return 2*np.pi+angle, lenvector
		# I return first the angle because that's the primary sorting criterium
		# but if two vectors have the same angle then the shorter distance should come first.
		return angle, lenvector

	#==========================================
	# return: collision(Bool), wrong_waypoint(Bool), free_dis(Int)
	#==========================================
	def check_collision(self, obs):
		obs_p = []
		for i in obs:
			obs_p.append([self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y])
		obs_sort = sorted(obs_p, key=self.clockwiseangle_and_distance)
		collision = False
		self.is_gate = False # if point is in the obstacle
		self.gate_point = None
		for i in obs:
			obs_position = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
			obs_size = self.obs_size(i)
			dis, _ = self.dis_point2segment(self.p_now, self.p_next, obs_position)
			if dis < obs_size + self.safe_dis:
				#print('pos: ', obs_position, dis)
				collision = True
		count = 0
		for i,j in zip(obs_sort[0::], obs_sort[1::]):
			p1 = i
			p2 = j
			collis, intersection = self.check_intersect(p1, p2, self.p_now, self.p_next)
			if collis:
				collision = True
				count = count + 1
				self.gate_point1 = p1
				self.gate_point2 = p2
		p1 = obs_sort[-1]
		p2 = obs_sort[0]
		collis, intersection = self.check_intersect(p1, p2, self.p_now, self.p_next)
		if collis:
			collision = True
			count = count + 1
			self.gate_point1 = p1
			self.gate_point2 = p2
		if count == 1:
			print('In obs')
			obs_size = 0.5
			if self.distance(self.gate_point1, self.gate_point2) > 2*obs_size + self.cluster_dis:
				is_inter, inter_p = self.check_intersect(self.gate_point1, self.gate_point2, self.p_now, self.p_next)
				dis1 = self.distance(self.gate_point1, inter_p)
				dis2 = self.distance(self.gate_point2, inter_p)
				if dis1 > obs_size + self.safe_dis and dis2 > obs_size + self.safe_dis:
					return False
				if dis1 < dis2:
					self.gate_point = self.find_obs_index(self.gate_point1)
				else:
					self.gate_point = self.find_obs_index(self.gate_point2)
				self.is_gate = True
				print('GATE!!!')
				#return False
				
		return collision

	def find_obs_index(self, p):
		for i in range(len(self.obstacle_list.list)):
			if round(p[0], 4) == round(self.obstacle_list.list[i].position.x, 4):
				if round(p[1], 4) == round(self.obstacle_list.list[i].position.y, 4):
					return i

	def circles_intersection(self, p1, p2, r1, r2):
		d = self.distance(p1, p2)
		ex = (p2[0] - p1[0]) / d
		ey = (p2[1] - p1[1]) / d

		x = (r1*r1 - r2*r2 + d*d) / (2*d)
		y = sqrt(r1*r1 - x*x)

		cp1_x = p1[0] + x * ex - y * ey
		cp1_y = p1[1] + x * ey + y * ex
		cp2_x = p1[0] + x * ex + y * ey
		cp2_y = p1[1] + x * ey - y * ex

		cp1 = [cp1_x, cp1_y]
		cp2 = [cp2_x, cp2_y]

		return cp1, cp2

	def shift_p_from_p(self, p1, p2, d):
		v = [p2[0] - p1[0], p2[1] - p1[1]]
		v_len = sqrt(v[0]**2 + v[1]**2)
		v_unit = [v[0]/v_len, v[1]/v_len]
		return [p1[0] + v_unit[0]*d, p1[1] + v_unit[1]*d]

	def find_way(self, obs):
		obs_angle_list = []
		max_index1 = None
		max_index2 = None
		angle_list_positive = []
		angle_list_negetive = []
		chosen_angle = None
		obs_vertex = []
		p_new = None
		self.first = False
		nearest_p = None
		nearest_dis = 10e4
		#print(self.wp_list)
		#print(self.p_now, self.p_next)
		#print('======')
		if self.p_now == self.start_point:
			self.first = True
		if self.first:
			self.first_cluster = obs
		if self.pre_cluster != obs:
			self.already_first = False
		'''if self.first and self.already_first:
			if self.in_obs:
				return None'''
		for i in obs:
			p = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
			d = self.distance(p, self.p_now)
			obs_size = self.obs_size(i)
			p1, p2 = self.circles_intersection(p, self.p_now, obs_size, d)
			obs_vertex.append(p1)
			obs_vertex.append(p2)

		for i in range(len(obs_vertex)):
			p1 = [obs_vertex[i][0], obs_vertex[i][1]]
			for j in range(len(obs_vertex)):
				if i == j:
					continue
				p2 = [obs_vertex[j][0], obs_vertex[j][1]]
				count_inside = 0
				for k in range(len(obs_vertex)):
					if k == j:
						continue
					p3 = [obs_vertex[k][0], obs_vertex[k][1]]
					if self.inside_vectors(p1, p2, p3):
						count_inside = count_inside + 1
				if count_inside == 2*len(obs) - 2:
					max_index1 = i
					max_index2 = j
					self.key1 = p1
					self.key2 = p2
					break
			else:
				continue
			break
		
		for i in range(len(obs)):
			index = obs[i]
			p = [self.obstacle_list.list[index].position.x, self.obstacle_list.list[index].position.y]
			#for i in range(len(obs_vertex)):
			#p = obs_vertex[i]
			#print(p)
			dis, within = self.dis_point2segment(self.p_now, self.p_next, p)
			#if within:
				#k = [-0.615143025982317, -4.971630139868719]
				#a, b = self.dis_point2segment(self.p_now, self.p_next, k)
				#print(dis, p)
			if dis < nearest_dis and within:
				nearest_p = p
				nearest_dis = dis
			if self.first and not self.already_first:
				#print('p: ',p ,  self.angle_from_robot(p))
				obs_angle_list.append([p, self.angle_from_robot(p)])
			else:
				obs_angle_list.append([p, self.get_angle(self.p_now, self.p_next, p)])
			if obs_angle_list[i][1] >= 0:
				angle_list_positive.append(obs_angle_list[i][1])
			else:
				angle_list_negetive.append(obs_angle_list[i][1])

		angle_list_positive.sort()
		angle_list_negetive.sort()

		index1 = obs[max_index1/2]
		index2 = obs[max_index2/2]
		obs_size1 = self.obs_size(index1)
		obs_size2 = self.obs_size(index2)
		d1 = self.safe_dis + obs_size1 + 0.5
		d2 = self.safe_dis + obs_size2 + 0.5
		obs1 = [self.obstacle_list.list[index1].position.x, self.obstacle_list.list[index1].position.y]
		obs2 = [self.obstacle_list.list[index2].position.x, self.obstacle_list.list[index2].position.y]
		p1_shift = self.shift_p_from_p(obs1, self.key1, d1)
		p2_shift = self.shift_p_from_p(obs2, self.key2, d2)

		self.key1 = p1_shift
		self.key2 = p2_shift
		#self.danger_point = False
		first_dis = self.distance(self.key1, self.p_now)
		if first_dis > self.distance(self.key2, self.p_now):
			first_dis = self.distance(self.key2, self.p_now)
		#print(first_dis)
		if (self.first or self.danger_point) and first_dis < 2.*self.safe_dis:
			angle1 = self.angle_from_robot(p1_shift)
			angle2 = self.angle_from_robot(p2_shift)
		else:
			angle1 = self.get_angle(self.p_now, self.p_next, p1_shift)
			angle2 = self.get_angle(self.p_now, self.p_next, p2_shift)

		if abs(angle1) < abs(angle2):
			now_obs = p1_shift
			p_shift = p1_shift
			d = d1
		else:
			now_obs = p2_shift
			p_shift = p2_shift
			d = d2

		prevent_big_turn = False
		if not self.first:
			if self.danger_point:
				check1 = self.angle_from_robot(p1_shift)
				check2 = self.angle_from_robot(p2_shift)
				if p1_shift == p_shift and abs(check1) > 90:
					if abs(check2) < 90:
						prevent_big_turn = True
						now_obs = p2_shift
						p_shift = p2_shift
						d = d2
				elif p2_shift == p_shift and abs(check2) > 90:
					if abs(check1) < 90:
						prevent_big_turn = True
						now_obs = p1_shift
						p_shift = p1_shift
						d = d1
			else:
				check1 = self.get_angle(self.p_now, self.wp_list[self.wp_index-1], p1_shift)
				check2 = self.get_angle(self.p_now, self.wp_list[self.wp_index-1], p2_shift)
				if p1_shift == p_shift and abs(check1) < 90:
					if abs(check2) > 90:
						prevent_big_turn = True
						now_obs = p2_shift
						p_shift = p2_shift
						d = d2
				elif p2_shift == p_shift and abs(check2) < 90:
					if abs(check1) > 90:
						print(check1, check2)
						prevent_big_turn = True
						now_obs = p1_shift
						p_shift = p1_shift
						d = d1

		#self.danger_point = False

		'''if self.is_gate:
			print('Open Gate')
			is_inter, inter_p = self.check_intersect(self.gate_point1, self.gate_point2, self.p_now, self.p_next)
			dis = self.distance(inter_p, self.gate_point2)
			shift_dis = 0.5 + self.safe_dis + 0.5
			x = inter_p[0] - self.gate_point2[0]
			y = inter_p[1] - self.gate_point2[1]
			x_new = x*shift_dis/dis + self.gate_point2[0]
			y_new = y*shift_dis/dis + self.gate_point2[1]
			#return [x_new, y_new]'''

		#print p_shift, now_obs

		obs_side = self.clock_side(self.p_now, self.p_next, now_obs)
		p_shift_side = self.clock_side(self.p_now, self.p_next, p_shift)
		pass_by = False
		if obs_side*p_shift_side > 0:
			check_dis_obs, within1 = self.dis_point2segment(self.p_now, self.p_next, now_obs)
			check_dis_shift, within2 = self.dis_point2segment(self.p_now, self.p_next, p_shift)
			if within1 and within2:
				if check_dis_obs > check_dis_shift:
					pass_by = True
		else:
			check_dis_obs, within1 = self.dis_point2segment(self.p_now, self.p_next, now_obs)
			check_dis_shift, within2 = self.dis_point2segment(self.p_now, self.p_next, p_shift)
			if within1 and within2:
				if check_dis_obs > check_dis_shift:
					pass_by = True
		if self.pre_obs is None:
			self.pre_obs = now_obs
			return p_shift

		elif round(self.pre_obs[0], 4) == round(now_obs[0], 4) and round(self.pre_obs[1], 4) == round(now_obs[1], 4):
			self.pre_obs = now_obs
			#print(nearest_p
			return self.find_shift_point(self.p_now, self.p_next, d, nearest_p, True)

		elif pass_by and not prevent_big_turn:
			self.pre_obs = now_obs
			return self.find_shift_point(self.p_now, self.p_next, d, nearest_p, True)

		else:
			self.pre_obs = now_obs
			if not self.point_within_line(self.p_now, self.p_next, p_shift):
				print('skip')
				self.skip_waypoint()
				return None
			return p_shift
		
		'''elif self.pre_obs == now_obs and self.count_safe < 2:
			self.pre_obs = now_obs
			self.count_safe = self.count_safe + 1
			return p_shift
		else:
			print('Ha')
			self.pre_obs = now_obs
			return self.find_shift_point(self.p_now, self.p_next, d, now_obs, True)'''


		# If obstacle isn't cross on the path
		# But need to move some distance for safe
		#print(len(angle_list_negetive), len(angle_list_positive))
		'''
		#print self.pre_cluster
		if len(angle_list_negetive) == 0:
			chosen_angle = angle_list_positive[0]
			for i in range(len(obs)):
				if obs_angle_list[i][1] == chosen_angle:
					p_new = obs_angle_list[i][0]
					print('1')
					#print(self.obstacle_list.list[obs[i]].position)
					break
			change_side = abs(chosen_angle) > 90
			d = self.safe_dis + 1.0 + 0.5
			if self.first:
				if not self.already_first:
					shift_p = p_new
				else:
					shift_p = nearest_p
				self.already_first = True
				p_next = [self.p_now[0] + 10, self.p_now[1] + 10*tan(self.orientation - np.pi/2.)]
				if change_side:
					new_wp = self.find_shift_point(self.p_now, p_next, d, shift_p, False)
					return new_wp
				new_wp = self.find_shift_point(self.p_now, p_next, d, shift_p, True)
				return new_wp
			new_wp = self.find_shift_point(self.p_now, self.p_next, d, nearest_p, True)
			return new_wp
		elif len(angle_list_positive) == 0:
			chosen_angle = angle_list_negetive[-1]
			for i in range(len(obs)):
				if obs_angle_list[i][1] == chosen_angle:
					p_new = obs_angle_list[i][0]
					print('2')
					#print(self.obstacle_list.list[obs[i]].position)
					break
			change_side = abs(chosen_angle) > 90
			d = self.safe_dis + 1.0 + 0.5
			if self.first:
				if not self.already_first:
					shift_p = p_new
				else:
					shift_p = nearest_p
				self.already_first = True
				p_next = [self.p_now[0] + 10, self.p_now[1] + 10*tan(self.orientation + np.pi/2.)]
				if change_side:
					new_wp = self.find_shift_point(self.p_now, p_next, d, shift_p, False)
					return new_wp
				new_wp = self.find_shift_point(self.p_now, p_next, d, shift_p, True)
				return new_wp
			new_wp = self.find_shift_point(self.p_now, self.p_next, d, nearest_p, True)
			return new_wp

		else:
			if not self.already_first and abs(angle_list_positive[-1]) > 90 and abs(angle_list_negetive[0]) > 90:
				if abs(angle_list_positive[0]) < abs(angle_list_negetive[-1]):
					chosen_angle = angle_list_positive[0]
				else:
					chosen_angle = angle_list_negetive[-1]
			else:
				if abs(angle_list_positive[-1]) < abs(angle_list_negetive[0]):
					chosen_angle = angle_list_positive[-1]
				else:
					chosen_angle = angle_list_negetive[0]
			for i in range(len(obs)):
				if obs_angle_list[i][1] == chosen_angle:
					p_new = obs_angle_list[i][0]
					print('3')
					#print(self.obstacle_list.list[obs[i]].position)
					break
			change_side = abs(chosen_angle) > 90
			d = self.safe_dis + 1.0 + 0.5
			if self.first and not self.already_first:
				self.already_first = True
				if self.angle_from_robot(p_new) > 0:
					is_positive = True
				else:
					is_positive = False
				p_next = [self.p_now[0] + 10, self.p_now[1] + 10*tan(self.orientation + np.pi/2.)]
				if change_side:
					print('change')
					new_wp = self.find_shift_point(self.p_now, p_next, d, p_new, True)
					return new_wp
				new_wp = self.find_shift_point(self.p_now, p_next, d, p_new, True)
				return new_wp
			new_wp = self.find_shift_point(self.p_now, self.p_next, d, p_new, False)
			return new_wp'''

	def pub_waypoint_list(self):
		for i in self.wp_list:
			self.waypoint_size = self.waypoint_size + 1
			wp = Waypoint()
			wp.x = i[0]
			wp.y = i[1]
			wp.z = 0
			self.waypoint_list.list.append(wp)
		self.waypoint_list.size = self.waypoint_size
		self.waypoint_list.header.frame_id = self.frame_id
		self.waypoint_list.header.stamp = rospy.Time.now()
		self.pub_waypointList.publish(self.waypoint_list)

	def call_controller(self):
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
				if i == self.waypoint_size - 1: # give last point a certain orientation
					if self.goal_orientation is not None:
						wp_srv.yaw = self.goal_orientation
				res = waypointResponse()
				try:
					rospy.wait_for_service("/add_waypoint")
					res.waypoint_len = add_wp(wp_srv)
				except (rospy.ServiceException, rospy.ROSException), e:
					rospy.logerr("Service call failed: %s" % (e,))
		start_motion = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
		start_motion()

	def find_vertex(self, obs_index):
		p1 = [self.obstacle_list.list[obs_index].x_min_x, self.obstacle_list.list[obs_index].x_min_y]
		p2 = [self.obstacle_list.list[obs_index].y_min_x, self.obstacle_list.list[obs_index].y_min_y]
		p3 = [self.obstacle_list.list[obs_index].x_max_x, self.obstacle_list.list[obs_index].x_max_y]
		p4 = [self.obstacle_list.list[obs_index].y_max_x, self.obstacle_list.list[obs_index].y_max_y]
		obs_vertex = [p1]
		######## UNDO: deal with the situation that the line only intersect a single vertex ########
		# Make sure all vertexs won't duplicate
		if not p2 in obs_vertex :
			obs_vertex.append(p2)
		if not p3 in obs_vertex :
			obs_vertex.append(p3)
		if not p4 in obs_vertex :
			obs_vertex.append(p4)
		return obs_vertex
	
	def skip_waypoint(self):
		self.wp_index = self.wp_index + 1
		self.p_now = self.wp_list[self.wp_index]
		self.p_next = self.wp_list[self.wp_index+1]
		self.plan_done = False

	def add_waypoint(self, p_new):
		self.wp_list.insert(self.wp_index + 1, p_new)
		self.p_next = p_new
		self.plan_done = False

	def rm_waypoint(self, obs_index):
		idx = 10e5
		for i in self.process_obs_list:
			if obs_index in i:
				for j in i:
					if j in self.wp_obs_idx:
						idx_hold = self.wp_obs_idx.index(j)
						if idx_hold < idx:
							idx = idx_hold
		if idx == 10e5:
			del self.wp_list[1:-1]
			del self.wp_obs_idx[1:-1]
			self.wp_index = 0
			self.p_now = self.wp_list[self.wp_index]
			self.p_next = self.wp_list[self.wp_index+1]
		else:
			del self.wp_list[idx:-1]
			del self.wp_obs_idx[idx:-1]
			self.wp_index = idx - 1
			self.p_now = self.wp_list[self.wp_index]
			self.p_next = self.wp_list[self.wp_index+1]

	def add_obstacle(self, idx_pre, idx_pro):
		#print "#######", idx_pre, idx_pro, "#######"
		idx_pre_hold = None
		idx_pro_hold = None
		for i in self.process_obs_list:
			if idx_pre in i:
				idx_pre_hold = self.process_obs_list.index(i)
			if idx_pro in i:
				idx_pro_hold = self.process_obs_list.index(i)
		if idx_pre_hold is None and idx_pro_hold is not None:
			self.process_obs_list[idx_pro_hold].append(idx_pre)
		elif idx_pre_hold is not None and idx_pro_hold is None:
			self.process_obs_list[idx_pre_hold].append(idx_pro)
		elif idx_pre_hold is not None and idx_pro_hold is not None:
			if idx_pre_hold == idx_pro_hold:
				#del self.process_obs_list[idx_pro_hold]
				#print('--------')
				return True
			self.process_obs_list[idx_pre_hold] += self.process_obs_list[idx_pro_hold]
			del self.process_obs_list[idx_pro_hold]
		else:
			print "Something wrong"
		return False

	def process_obstacle(self, idx):
		add_obs = True
		for i in self.process_obs_list:
			if idx in i:
				add_obs = False
		if add_obs:
			self.process_obs_list.append([idx])

	def station_keep(self):
		print('Station keeping')
		self.goal_point = self.start_point
		self.init_param()
		self.plan_done = True
		self.pub_waypoint_list()
		self.rviz()
		self.call_controller()

	def escape_danger_place(self, p, danger, dis):
		print("DANGER!!!")
		x = p[0] - danger[0]
		y = p[1] - danger[1]
		std_d = self.distance(p, danger)
		x_new = x*dis/std_d + danger[0]
		y_new = y*dis/std_d + danger[1]
		p_new = [x_new, y_new]
		self.add_waypoint(p_new)
		self.wp_index = self.wp_index + 1
		self.p_now = self.wp_list[self.wp_index]
		self.p_next = self.wp_list[self.wp_index+1]
		self.plan_done = False

	def obs_size(self, i):
		max_dis = -10e5
		x_total = 0
		y_total = 0
		for pose in self.obstacle_list.list[i].pcl_points.poses:
			x_total = x_total + pose.position.x
			y_total = y_total + pose.position.y
		cx = x_total/len(self.obstacle_list.list[i].pcl_points.poses)
		cy = y_total/len(self.obstacle_list.list[i].pcl_points.poses)
		for pose in self.obstacle_list.list[i].pcl_points.poses:
			x = pose.position.x
			y = pose.position.y
			z = pose.position.z
			dis = self.distance([x,y], [cx, cy])
			if max_dis < dis:
				max_dis = dis
		#print(max_dis)
		return max_dis
		'''size = 1.
		if self.obstacle_list.list[i].type == 'totem':
			size = 1
		elif self.obstacle_list.list[i].type == 'buoy':
			size = 1.
		elif self.obstacle_list.list[i].type == 'dock':
			size = 6.
		return size'''

	def _process(self):
		self.iteration = self.iteration + 1
		rospy.loginfo(self.iteration)
		if self.iteration > 20:
			self.plan_done = True
			return
		for i in range(self.obstacle_list.size):
			p = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
			obs_size = self.obs_size(i)
			#print self.distance(p, self.p_now)
			#print('==: ', obs_size + self.safe_dis)
			if self.distance(p, self.p_now) < obs_size + self.safe_dis:
				dis = self.safe_dis + obs_size + 0.5
				self.escape_danger_place(self.p_now, p, dis)
				self.danger_point = True
				return
		obs_cluster = self.cluster_obs()
		#print(len(self.obstacle_list.list))
		#print(obs_cluster)
		for cluster in obs_cluster:
			if self.check_collision(cluster):
				#print('collision')
				if self.is_gate:
					print('OPEN GATE')
					p_new = self.find_way([self.gate_point])
				else:
					p_new = self.find_way(cluster)
				self.pre_cluster = cluster
				if p_new is None:
					break
				self.pub_point.x = p_new[0]
				self.pub_point.y = p_new[1]
				self.add_waypoint(p_new)
				return
		# If this is not the last path segment
		if self.p_next != self.wp_list[-1]:
			self.wp_index = self.wp_index + 1
			self.p_now = self.wp_list[self.wp_index]
			self.p_next = self.wp_list[self.wp_index+1]
			self.plan_done = False
			self.danger_point = False
			return
		self.pub_waypoint_list()
		self.plan_done = True
		if not self.debug:
			self.rviz()
		if self.count_call%10 == 0:
			if not self.debug:
				self.call_controller()
				pass
		self.count_call = self.count_call + 1
		rospy.loginfo("Finish path planning")

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
		marker.color.r = 1.0
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
			p.x = self.waypoint_list.list[i].x
			p.y = self.waypoint_list.list[i].y
			p.z = self.waypoint_list.list[i].z
			marker.points.append(p)
		self.pub_rviz.publish(marker)
		marker_p = Marker()
		marker_p.header.frame_id = self.frame_id
		marker_p.type = marker.POINTS
		marker_p.pose.orientation.w = 1
		marker_p.scale.x = 0.8
		marker_p.scale.y = 0.8
		marker_p.color.a = 1
		marker_p.color.r= 1.0
		marker_p.points = [self.pub_point]
		self.pubish_p.publish(marker_p)
		marker_pp = Marker()
		marker_pp.header.frame_id = self.frame_id
		marker_pp.type = marker.POINTS
		marker_pp.pose.orientation.w = 1
		marker_pp.scale.x = 1
		marker_pp.scale.y = 1
		marker_pp.color.a = 0.9
		marker_pp.color.g= 1.0
		p = Point()
		p.x = self.p_now[0]
		p.y = self.p_now[1]
		obs_cluster = self.cluster_obs()
		#p.x = self.key1[0]
		#p.y = self.key1[1]
		marker_pp.points = [p]
		self.pubish_pp.publish(marker_pp)
		marker_ppp = Marker()
		marker_ppp.header.frame_id = self.frame_id
		marker_ppp.type = marker.POINTS
		marker_ppp.pose.orientation.w = 1
		marker_ppp.scale.x = 1
		marker_ppp.scale.y = 1
		marker_ppp.color.a = 0.9
		marker_ppp.color.b= 1.0
		pp = Point()
		pp.x = self.p_next[0]
		pp.y = self.p_next[1]
		#pp.x = self.key2[0]
		#pp.y = self.key2[1]
		marker_ppp.points = [pp]
		self.pubish_ppp.publish(marker_ppp)

	def pub_p(self):
		marker = Marker()
		marker.header.frame_id = self.frame_id
		marker.type = marker.POINTS
		marker.pose.orientation.w = 1
		marker.scale.x = 1
		marker.scale.y = 1
		marker.color.a = 0.9
		marker.color.r= 1.0
		marker.color.g= 1.0
		marker.color.b= 1.0
		p = Point()
		p.x = self.wp_list[0][0]
		p.y = self.wp_list[0][1]
		marker.points = [p]
		self.pubish_pppp.publish(marker)

	def rviz(self):
		marker = Marker()
		marker.header.frame_id = self.frame_id
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.a = 1.0
		marker.color.r = 1.0
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
			p.x = self.waypoint_list.list[i].x
			p.y = self.waypoint_list.list[i].y
			p.z = self.waypoint_list.list[i].z
			marker.points.append(p)
		self.pub_rviz.publish(marker)
		p = PoseStamped()
		p.header.frame_id = self.frame_id
		p.pose.position.x = self.wp_list[-1][0]
		p.pose.position.y = self.wp_list[-1][1]
		q = tf.transformations.quaternion_from_euler(0, 0, self.goal_orientation)
		p.pose.orientation.x = q[0]
		p.pose.orientation.y = q[1]
		p.pose.orientation.z = q[2]
		p.pose.orientation.w = q[3]
		self.pub_final_point.publish(p)
	
if __name__ == "__main__":
	rospy.init_node("Path_planning_node", anonymous = False)
	rrt_planning = PathPlanning()
	rospy.spin()
