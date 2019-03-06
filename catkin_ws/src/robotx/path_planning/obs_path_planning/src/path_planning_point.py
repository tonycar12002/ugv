#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import atan2, cos, sin, sqrt
from scipy.spatial import Delaunay
from robotx_msgs.msg import ObjectPoseList
from robotx_msgs.msg import Waypoint, WaypointList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from robotx_msgs.srv import *
from std_srvs.srv import *
#from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class PathPlanning(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		self.debug = False
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		self.sub_obstacleList = rospy.Subscriber("/obj_list/map", ObjectPoseList, self.call_back, queue_size=1)
		#self.sub_new_goal = rospy.Subscriber("/clicked_point", PointStamped, self.call_new_goal, queue_size=1)
		self.sub_new_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.call_new_goal, queue_size=1)
		self.new_goal = rospy.Service("/new_goal", waypoint, self.new_goal)
		self.pub_rviz = rospy.Publisher("/wp_line", Marker, queue_size = 1)
		self.pubish_p = rospy.Publisher("/ppp", Marker, queue_size = 1)
		self.pubish_pp = rospy.Publisher("/pppp", Marker, queue_size = 1)
		self.pubish_ppp = rospy.Publisher("/ppppp", Marker, queue_size = 1)
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
		self.update_goal = False
		self.arrived = False
		self.safe_dis = rospy.get_param("~safe_dis",3.5)
		self.cluster_dis = 2.*self.safe_dis + 2
		self.init_param()
		#print "The Goal Point is: ", self.goal_point
		print "Cluster distance is: ", self.cluster_dis
		print "Safe distance is: ", self.safe_dis
		print "======================"

	def init_param(self):
		self.pre_cluster = []
		self.danger_start_point = False
		self.danger_point = None
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
		self.max_pos_angle = 0
		self.max_neg_angle = 0
		self.correct_waypoint = False
		self.switch = True
		self.process_obs = [None, None]
		self.pre_obs_idx = None
		# Waypoint list
		self.wp_list = [self.start_point, self.goal_point]
		self.wp_obs_idx = [None, None]
		self.waypoint_size = 0
		self.waypoint_list = WaypointList()
		self.waypoint_list.header.frame_id = self.frame_id

	def call_new_goal(self, p):
		self.goal_point = [p.pose.position.x, p.pose.position.y]
		q = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.goal_orientation = euler[2]
		self.get_goal = True
		self.update_goal = True
		self.arrived = False

	def new_goal(self, req):
		self.goal_point = [req.waypointx, req.waypointy]
		self.get_goal = True
		self.update_goal = True
		self.arrived = False
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
		self.arrived = self.arrive_goal()
		self.obstacle_list = msg
		#self._process()
		if not self.lock and self.get_goal and not self.arrived:
			#self.init_param()
			self.plan_done = False
			self.lock = True
			#print "Start Path Planning"
			while not self.plan_done:
				self._process()
				if self.debug:
					self.rviz_debug()
					rospy.sleep(1)
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
		new_point1 = [p[0]+k, p[1]+k*m]
		new_point2 = [p[0]-k, p[1]-k*m]
		if self.first:
			angle1 = -self.get_angle(self.p_now, self.p_next, new_point1)
			angle2 = -self.get_angle(self.p_now, self.p_next, new_point2)
			if move:
				if not self.is_positive:
					if angle1 < angle2:
						return new_point1
					else:
						return new_point2
				else:
					if angle2 < angle1:
						return new_point1
					else:
						return new_point2
			else:
				if not self.is_positive:
					if angle1 > angle2:
						return new_point1
					else:
						return new_point2
				else:
					if angle2 > angle1:
						return new_point1
					else:
						return new_point2
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


	#==========================================
	# return: collision(Bool), wrong_waypoint(Bool), free_dis(Int)
	#==========================================
	def check_collision(self, obs):
		collision = False
		for i in obs:
			obs_position = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
			obs_size = self.obs_size(i)
			dis, _ = self.dis_point2segment(self.p_now, self.p_next, obs_position)
			if dis < obs_size + self.safe_dis:
				#print('pos: ', obs_position, dis)
				collision = True
		if collision:
			return True
		for i,j in zip(obs[0::], obs[1::]):
			p1 = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
			p2 = [self.obstacle_list.list[j].position.x, self.obstacle_list.list[j].position.y]
			collis, intersection = self.check_intersect(p1, p2, self.p_now, self.p_next)
			if collis:
				collision = True
		p1 = [self.obstacle_list.list[obs[0]].position.x, self.obstacle_list.list[obs[0]].position.y]
		p2 = [self.obstacle_list.list[obs[-1]].position.x, self.obstacle_list.list[obs[-1]].position.y]
		collis, intersection = self.check_intersect(p1, p2, self.p_now, self.p_next)
		if collis:
			collision = True
		return collision

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

	def find_way(self, obs):
		obs_angle_list = []
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
		for i in obs:
			p = [self.obstacle_list.list[i].position.x, self.obstacle_list.list[i].position.y]
			d = self.distance(p, self.p_now)
			obs_size = self.obs_size(i)
			p1, p2 = self.circles_intersection(p, self.p_now, obs_size, d)
			obs_vertex.append(p1)
			obs_vertex.append(p2)
		#print('======')
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
			if self.p_now == self.start_point:
				#print('p: ',p ,  self.angle_from_robot(p))
				self.first = True
				obs_angle_list.append([p, self.angle_from_robot(p)])
			else:
				obs_angle_list.append([p, self.get_angle(self.p_now, self.p_next, p)])
			if obs_angle_list[i][1] >= 0:
				angle_list_positive.append(obs_angle_list[i][1])
			else:
				angle_list_negetive.append(obs_angle_list[i][1])
		#print('nnn: ', nearest_p, nearest_dis)
		#print('-------')
		angle_list_positive.sort()
		angle_list_negetive.sort()

		# If obstacle isn't cross on the path
		# But need to move some distance for safe
		#print(len(angle_list_negetive), len(angle_list_positive))
		if self.pre_cluster != obs:
			self.already_first = False
		if len(angle_list_negetive) == 0:
			chosen_angle = angle_list_positive[0]
			for i in range(len(obs)):
				if obs_angle_list[i][1] == chosen_angle:
					p_new = obs_angle_list[i][0]
					#print('1')
					#print(self.obstacle_list.list[obs[i]].position)
					break
			if self.first and not self.already_first:
				self.already_first = True
				return p_new, False
			return nearest_p, False
		elif len(angle_list_positive) == 0:
			chosen_angle = angle_list_negetive[-1]
			for i in range(len(obs)):
				if obs_angle_list[i][1] == chosen_angle:
					p_new = obs_angle_list[i][0]
					#print('2')
					#print(self.obstacle_list.list[obs[i]].position)
					break
			if self.first and not self.already_first:
				self.already_first = True
				return p_new, False
			return nearest_p, False

		else:
			if abs(angle_list_positive[-1]) < abs(angle_list_negetive[0]):
				chosen_angle = angle_list_positive[-1]
			else:
				chosen_angle = angle_list_negetive[0]
			for i in range(len(obs)):
				if obs_angle_list[i][1] == chosen_angle:
					p_new = obs_angle_list[i][0]
					#print('3')
					#print(self.obstacle_list.list[obs[i]].position)
					break
			#return p_new, True
			if self.first and not self.already_first:
				if self.angle_from_robot(p_new) > 0:
					self.is_positive = True
				else:
					self.is_positive = False
				d = self.safe_dis + 1.0 + 0.5
				if self.angle_from_robot(nearest_p) * chosen_angle < 0:
					new_nearest_p = self.find_shift_point(self.p_now, self.p_next, d, nearest_p, False)
					new_p_new = self.find_shift_point(self.p_now, self.p_next, d, p_new, False)
					#print(new_nearest_p)
					#print(new_p_new)
					if abs(self.angle_from_robot(new_nearest_p)) > abs(self.angle_from_robot(new_p_new)):
						return nearest_p, False
					else:
						return p_new, True
			return p_new, True

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
		size = 1.
		if self.obstacle_list.list[i].type == 'totem':
			size = 1
		elif self.obstacle_list.list[i].type == 'buoy':
			size = 1.
		elif self.obstacle_list.list[i].type == 'dock':
			size = 6.
		return size

	def _process(self):
		self.iteration = self.iteration + 1
		if self.iteration > 70:
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
				return
		obs_cluster = self.cluster_obs()
		#print(len(self.obstacle_list.list))
		#print(obs_cluster)
		for cluster in obs_cluster:
			if self.check_collision(cluster):
				#print('collision')
				p_new, change_side = self.find_way(cluster)
				#print p_new
				self.pre_cluster = cluster
				if p_new is None:
					break
				if self.angle_from_robot(p_new) > 0:
					self.is_positive = True
				else:
					self.is_positive = False
				d = self.safe_dis + 1.0 + 0.5
				#print(p_new)
				if change_side:
					p_new = self.find_shift_point(self.p_now, self.p_next, d, p_new, False)
				else:
					p_new = self.find_shift_point(self.p_now, self.p_next, d, p_new, True)
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
		marker_ppp.points = [pp]
		self.pubish_ppp.publish(marker_ppp)

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
	
if __name__ == "__main__":
	rospy.init_node("Path_planning_node", anonymous = False)
	rrt_planning = PathPlanning()
	rospy.spin()
