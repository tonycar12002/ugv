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
#from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class PathPlanning(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		self.sub_obstacleList = rospy.Subscriber("/obstacle_list/map", ObstaclePoseList, self.call_back, queue_size=1)
		self.pub_rviz		  = rospy.Publisher("/wp_line", Marker, queue_size = 1)
		self.add_wp = rospy.Service("/add_wp", waypoint, self.add_wp_handler)
		self.start_add_wp = rospy.Service("/start_add_wp", Trigger, self.start_add_wp)
		self.end_add_wp = rospy.Service("/end_add_wp", Trigger, self.end_add_wp)
		self.start_point = [0, 0]
		self.hold_wp_list = []
		self.new_wp_list = []
		self.srv_flag = False
		self.old_wp_list_len = 0
		x = rospy.get_param("~x", 100)
		y = rospy.get_param("~y", 10)
		self.goal_point = [x, y]
		self.safe_dis = rospy.get_param("~safe_dis",5)
		self.wp_list = [self.start_point, self.goal_point]
		self.init_param()
		print "The Goal Point is: ", self.goal_point
		print "Safe Distance is: ", self.safe_dis
		print "======================"

	def init_param(self):
		self.iteration = 0
		self.lock = False
		self.plan_done = False
		#self.wp_list = [self.start_point, self.goal_point]
		self.p_now = self.wp_list[0]
		self.p_next = self.wp_list[1]
		self.car_length = 1
		self.wp_index = 0
		self.change_side = False
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
		self.wp_obs_idx = [None, None]
		self.waypoint_size = 0
		self.waypoint_list = WaypointList()
		self.waypoint_list.header.frame_id = self.frame_id

	def start_add_wp(self, req):
		print "======= Start to add waypoints ======="
		self.hold_wp_list = []
		res = TriggerResponse()
		res.success = True
		res.message = "waypoint nav started"
		return res

	def end_add_wp(self, req):
		print "End to add waypoints"
		self.srv_flag = True
		self.new_wp_list = self.hold_wp_list[:]
		res = TriggerResponse()
		res.success = True
		res.message = "waypoint nav ended"
		return res

	def add_wp_handler(self, req):
		print "add waypoint:", req.waypointx, req.waypointy
		self.hold_wp_list.append([req.waypointx, req.waypointy])
		return len(self.hold_wp_list)

	def call_back(self, msg):
		self.start_point = [msg.robot.position.x, msg.robot.position.y]
		if self.srv_flag:
			#self.wp_list = self.new_wp_list[:]
			self.new_wp_list.insert(0, self.start_point)
		else:
			self.new_wp_list = [self.start_point, self.goal_point]
		self.init_param()
		self.obstacle_list = msg
		if not self.lock:
			self.plan_done = False
			self.lock = True
			print "Start Path Planning"
            for i in range(len(self.new_wp_list)-1):
                self.wp_list = [self.new_point[i], self.new_point[i+1]]
    			while not self.plan_done:
    				self._process()
    				#self.rviz_debug()
    				#rospy.sleep(0.5)
                self.final_wp_list
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
			return self.dis_point2line(p1, p2, p3)
		else:
			dis_1 = self.distance(p1, p3)
			dis_2 = self.distance(p2, p3)
			if dis_1 < dis_2:
				return dis_1
			return dis_2

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
			dis = abs(dis)
		return self.shift_point(dis, m, d, p_new)

	#==========================================
	# return: collision(Bool), wrong_waypoint(Bool), free_dis(Int)
	#==========================================
	def check_collision(self, obs_vertex):
		intersect_p1 = None
		intersect_p2 = None
		collision = False
		intersect_count = 0
		nearest_p = None
		wrong_waypoint = False
		#Check every segment of the obstacle will intersect with the path or not
		for i,j in zip(obs_vertex[0::], obs_vertex[1::]):
			# the next point is in dangerous place --> wrong waypoint
			dis = self.dis_point2segment(i, j, self.p_next)
			if dis < self.safe_dis:
				collision = True
				wrong_waypoint = True
				break
			vertex_colli, _ = self.check_intersect(i, j, self.p_now, self.p_next)
			if vertex_colli == True:
				collision = True
				intersect_count = intersect_count + 1
				intersect_p1 = i
				intersect_p2 = j

		#This two vertex won't be consider in previous for loop
		dis = self.dis_point2segment(obs_vertex[-1], obs_vertex[0], self.p_next)
		if dis < self.safe_dis:
			collision = True
			wrong_waypoint = True
		else:
			vertex_colli, _ = self.check_intersect(obs_vertex[-1], obs_vertex[0], self.p_now, self.p_next)
			if vertex_colli == True:
				collision = True
				intersect_count = intersect_count + 1
				intersect_p1 = i
				intersect_p2 = j
		#Although they aren't intersect, we still need to check whether they are too close
		if not collision:
			for i in obs_vertex:
				dis = self.dis_point2line(self.p_now, self.p_next, i)
				if dis < self.safe_dis :
					#The below "if" is a little bit weird
					if self.point_within_line(self.p_now, self.p_next, i):
						collision = True
						nearest_p = i
						break
		#If the next waypoint is in obstacle --> wrong waypoint
		if intersect_count == 1 and len(obs_vertex) > 2:
			wrong_waypoint = True

		return collision, wrong_waypoint, nearest_p

	def find_way(self, obs_vertex):
		obs_angle_list = []
		angle_list_positive = []
		angle_list_negetive = []
		chosen_angle = None
		nearest_p = None
		nearest_dis = 10e5
		nearest_angle = 0
		for i in range(len(obs_vertex)):
			dis = self.dis_point2segment(self.p_now, self.p_next, obs_vertex[i])
			if dis < nearest_dis:
				nearest_dis = dis
				nearest_p = obs_vertex[i]
				nearest_angle = self.get_angle(self.p_now, self.p_next, obs_vertex[i])
			obs_angle_list.append([obs_vertex[i], self.get_angle(self.p_now, self.p_next, obs_vertex[i])])
			if obs_angle_list[i][1] >= 0:
				angle_list_positive.append(obs_angle_list[i][1])
			else:
				angle_list_negetive.append(obs_angle_list[i][1])
		angle_list_positive.sort()
		angle_list_negetive.sort()

		# If obstacle isn't cross on the path
		# But need to move some distance for safe
		if len(angle_list_negetive) == 0 or len(angle_list_positive) == 0:
			return nearest_p, nearest_angle 
		else:
			if abs(angle_list_positive[-1]) < abs(angle_list_negetive[0]):
				chosen_angle = angle_list_positive[-1]
			else:
				chosen_angle = angle_list_negetive[0]

		for i in range(len(obs_vertex)):
			if obs_angle_list[i][1] == chosen_angle:
				p_new = obs_angle_list[i][0]
				break
		return p_new, chosen_angle

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

	def add_waypoint(self, p_new, obs_idx):
		self.wp_obs_idx.insert(self.wp_index + 1, obs_idx)
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
			self.process_obs_list[idx_pre_hold] += self.process_obs_list[idx_pro_hold]
			del self.process_obs_list[idx_pro_hold]
		else:
			print "Something wrong"

	def process_obstacle(self, idx):
		add_obs = True
		for i in self.process_obs_list:
			if idx in i:
				add_obs = False
		if add_obs:
			self.process_obs_list.append([idx])

	def _process(self):
		self.iteration = self.iteration + 1
		if self.iteration > 50:
			self.plan_done = True
			return
		for obs_index in range(self.obstacle_list.size):
			#print "=====", self.process_obs_list, "====="
			obs_vertex = self.find_vertex(obs_index)
			self.process_obstacle(obs_index)
			#Check every segment of the obstacle will intersect with the path or not
			collision, wrong_waypoint, nearest_p = self.check_collision(obs_vertex)
			if wrong_waypoint:
				self.add_obstacle(self.pre_obs_idx, obs_index)
				#self.pre_obs_idx = obs_index
				self.rm_waypoint(obs_index)
				obs_vertex_s = []
				vertex = []
				for idx in self.process_obs_list:
					if obs_index in idx:
						i = self.process_obs_list.index(idx)
						for j in self.process_obs_list[i]:
							obs_vertex_s = self.find_vertex(j)
							vertex = vertex + obs_vertex_s
				p_new, chosen_angle = self.find_way(vertex)
				d = self.safe_dis + 0.1
				if p_new is not None:
					p_new = self.find_shift_point(self.p_now, self.p_next, d, p_new, False)
					self.add_waypoint(p_new, obs_index)
					return
			
			if collision:
				self.pre_obs_idx = obs_index
				#print obs_index
				self.max_pos_angle = self.max_neg_angle = 0.
				p_new, chosen_angle = self.find_way(obs_vertex)
				if chosen_angle >= 0:
					self.max_pos_angle = chosen_angle
				else:
					self.max_neg_angle = chosen_angle
				if nearest_p is not None:
					d = self.safe_dis + 0.1
					p_new = self.find_shift_point(self.p_now, self.p_next, d, p_new, True)
				else:
					d = self.safe_dis + 0.1
					p_new = self.find_shift_point(self.p_now, self.p_next, d, p_new, False)
				self.add_waypoint(p_new, obs_index)
				if self.process_obs[0] is None:
					self.process_obs[0] = obs_index
					self.process_obs[1] = p_new
				'''elif obs_index != self.process_obs[0]:
					if obs_index not in self.process_obs_list or len(self.process_obs_list) == 1:
						self.switch = True
						self.process_obs[0] = obs_index
						self.process_obs[1] = p_new
				else:
					if self.switch:
						self.process_obs[1] = p_new'''
				return

		# If this is not the last path segment
		if self.p_next != self.wp_list[-1]:
			self.wp_index = self.wp_index + 1
			self.p_now = self.wp_list[self.wp_index]
			self.p_next = self.wp_list[self.wp_index+1]
			self.check_avoid_point = False
			self.plan_done = False
			return
		self.pub_waypoint_list()
		self.plan_done = True
		self.rviz()
		#self.call_controller()
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
