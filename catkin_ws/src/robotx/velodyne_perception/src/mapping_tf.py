#!/usr/bin/env python
import rospy
import roslib
import math
import scipy.stats
from sensor_msgs.msg import PointCloud2
from robotx_msgs.msg import ObjectPose, ObjectPoseList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseArray
import numpy as np
from std_srvs.srv import *

class mapping():
	def __init__(self): 
		# ======== Subscriber ========
		rospy.Subscriber("/obj_list/odom", ObjectPoseList, self.call_back, queue_size=1)
		#rospy.Subscriber("/waypointList", WaypointList, call_back, queue_size=10)

		# ======== Publisher ========
		self.pub_obj = rospy.Publisher("/obj_list/map", ObjectPoseList, queue_size=1)
		self.pub_marker = rospy.Publisher("/obj_marker/map", MarkerArray, queue_size = 1)
		#pub_rviz = rospy.Publisher("/wp_path", Marker, queue_size = 1)
		self.clear_map_service = rospy.Service("/clear_map", Trigger, self.clear_map_srv)

		# ======== Get ROS parameter ========
		self.visual = rospy.get_param('~visual', False)

		# ======== Declare Variable ========
		self.same_dis = 1.3
		self.first = True
		self.robot_pose = None
		self.map = ObjectPoseList()
		self.obj_list = None
		self.frame_id = "odom"
		self.matching = []
		self.remove_list = []
		self.remove_threshold = 0.09
		self.r_threshold = 6
		self.prob_threshold = 0.3
		self.update_range = 200.
		self.punish_range = 200.
		self.classify_range = 30.
		self.prior_mean = None
		self.prior_cov = None
		self.punish_no_detect = 2.2
		self.punish_unclassify = 1.5
		self.measurement_var = 1.
		self.init_varX = 1.
		self.init_varY = 1.
		self.kernel = scipy.stats.norm(loc = 0, scale = 0.5)
		# ======== Get from odometry =======
		#self.pos_covariance = np.diag([3., 3.])
		self.sensor_error = 1.

	def clear_map(self):
		self.first = True

	def clear_map_srv(self, req):
		print "Clear Map"
		self.clear_map()
		res = TriggerResponse()
		res.success = True
		res.message = "waypoint nav started"
		return res

	def cluster_dock(self):
		# if there isn't any obstacle
		if len(self.map.list) == 0 :
			return None
		cluster = []
		index = None
		cluster_index = None
		for i in range(len(self.map.list)):
			if not self.map.list[i].type == 'dock':
				continue
			append_i = True
			for c in cluster:
				if i in c:
					append_i = False
			if append_i:
				cluster.append([i])
			i_being_clustered = False
			for j in range(len(self.map.list)):
				if not self.map.list[j].type == 'dock' and not self.map.list[j] == 'None':
					continue
				j_being_clustered = False
				if i != j and self.distance(self.map.list[i], self.map.list[j]) < 7.5:
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
		count = 0
		for i in cluster:
			for c in i:
				count = count + 1
		print(cluster)
		docks = []
		docks_pcl = []
		for i in cluster:
			pcl_count = 0
			pcl_x = 0
			pcl_y = 0
			pcl = PoseArray()
			for j in i:
				pcl.poses = pcl.poses + self.map.list[j].pcl_points.poses
				for k in self.map.list[j].pcl_points.poses:
					pcl_x = pcl_x + k.position.x
					pcl_y = pcl_y + k.position.y
					pcl_count = pcl_count + 1
			pcl_x = pcl_x/pcl_count
			pcl_y = pcl_y/pcl_count
			docks.append([pcl_x, pcl_y, pcl])
		new_map = ObjectPoseList()
		new_map.header = self.map.header
		new_map.robot = self.map.robot
		for i in range(len(self.map.list)):
			skip = False
			for j in cluster:
				if i in j:
					skip = True
			if not skip:
				new_map.list.append(self.map.list[i])
		for dock in docks:
			obj = ObjectPose()
			obj.position.x = dock[0]
			obj.position.y = dock[1]
			obj.type = "dock"
			obj.pcl_points = dock[2]
			new_map.list.append(obj)
		self.map.size = len(self.map.list)
		self.map = new_map

	def points_dis(self, p1, p2):
		return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def class2index(self, cls):
		if cls == 'buoy':
			return 0
		elif cls == 'dock':
			return 1
		elif cls == 'totem':
			return 2
		elif cls == 'light_buoy':
			return 3
		else:
			return 4
	
	def index2class(self, idx):
		if idx == 0:
			return 'buoy'
		elif idx == 1:
			return 'dock'
		elif idx == 2:
			return 'totem'
		elif idx == 3:
			return 'light_buoy'
		else:
			return 'None'
		
	def class_filter(self, idx, new_cls):
		new_cls_idx = self.class2index(new_cls)
		if new_cls != "None":
			for i in range(len(self.map.list[idx].count)):
				if i == new_cls_idx:
					self.map.list[idx].count[i] = self.map.list[idx].count[i] + 1
					if self.map.list[idx].count[i] > 20:
						self.map.list[idx].count[i] = 20
				else:
					if self.map.list[idx].count[i] > 0:
						self.map.list[idx].count[i] = self.map.list[idx].count[i] - 1
		max_cnt = -1
		max_cls = None
		count = 0
		for i in range(len(self.map.list[idx].count) - 1):
			if self.map.list[idx].count[i] > max_cnt:
				max_cnt = self.map.list[idx].count[i]
				max_cls = self.index2class(i)
		if max_cls is None:
			max_cls = new_cls
		#print(self.map.list[idx].count)
		return max_cls

	def call_back(self, msg):
		#rospy.loginfo("Process Object List")
		self.obj_list = ObjectPoseList()
		self.obj_list = msg
		self.map_confidence = ObjectPoseList()
		self.map_confidence.header.frame_id = self.frame_id
		self.map.header.frame_id = self.frame_id
		self.map.robot = msg.robot
		self.matching = []
		self.remove_list = []
		robot_x = self.obj_list.robot.position.x
		robot_y = self.obj_list.robot.position.y
		robot_z = self.obj_list.robot.position.z
		self.robot_pose = [robot_x, robot_y, robot_z]
		self.obj_list.header.frame_id = self.frame_id
		if self.first:
			self.map = self.obj_list
			self.first = False
			for i in range(self.map.size):
				self.map.list[i].occupy = False
				self.map.list[i].count = [0, 0, 0, 0, 0]
				cls_idx = self.class2index(self.map.list[i].type)
				self.map.list[i].count[cls_idx] = self.map.list[i].count[cls_idx] + 1
				#print(self.map.list[i].varianceX)
				self.map.list[i].varianceX = self.init_varX
				self.map.list[i].varianceY = self.init_varY
		else:
			for i in range(self.map.size):
				self.map.list[i].occupy = False
			self.data_associate()
			self.update_map()
			for i in range(self.map.size):
				mean_x, mean_y = self.map.list[i].position.x, self.map.list[i].position.y
				var_x, var_y = self.map.list[i].varianceX, self.map.list[i].varianceY
				prob_x = scipy.stats.norm(mean_x, var_x).pdf(mean_x)
				prob_y = scipy.stats.norm(mean_y, var_y).pdf(mean_y)
				#print prob_x, prob_y
				if prob_x > self.prob_threshold and prob_y > self.prob_threshold:
					self.map_confidence.list.append(self.map.list[i])
				elif prob_x < self.remove_threshold and prob_y < self.remove_threshold:
					self.remove_list.append(i)
			remove_num = 0	#ensure the index are correct during removing
			for i in self.remove_list:
				del self.map.list[i - remove_num]
				remove_num = remove_num + 1
			self.map.size = len(self.map.list)
		self.map.header.stamp = rospy.Time.now()
		self.map_confidence.size = len(self.map_confidence.list)
		self.map.header.stamp = rospy.Time.now()
		self.map_confidence.header.stamp = rospy.Time.now()
		self.pub_obj.publish(self.map)
		self.drawRviz(self.map)

	def data_associate(self):
		'''self.matching = [None]*self.obj_list.size
		for i in range(self.map.size):
			min_dis = 10e5
			index = None
			for j in range(self.obj_list.size):
				dis = self.distance(self.map.list[i], self.obj_list.list[j])
				if dis < min_dis:
					index = j
					min_dis = dis
			if min_dis < self.r_threshold:
				self.matching[index] = i
		print self.matching'''
		for i in range(self.obj_list.size):
			min_dis = 10e5
			index = None
			for j in range(self.map.size):
				#if self.map.list[j].type == self.obj_list.list[i].type or self.obj_list.list[i].type == 'None':
				if not self.map.list[j].occupy:
					dis = self.distance(self.obj_list.list[i], self.map.list[j])
					if dis < self.same_dis or self.map.list[j].type == self.obj_list.list[i].type or self.obj_list.list[i].type == 'None':
						if dis < min_dis:
							index = j
							min_dis = dis
			if min_dis < self.r_threshold:
				self.map.list[index].occupy = True
			else:
				index = None
			self.matching.append(index)

	def update_map(self):
		for i in range(self.obj_list.size):
			index = self.matching[i]
			if self.distance_to_robot(self.obj_list.list[i]) < self.update_range:
				if index != None:
					# Kalman filter update position
					# ======= Kalman filter for x =======
					prior_mean = self.map.list[index].position.x
					prior_var = self.map.list[index].varianceX
					z_mean = self.obj_list.list[i].position.x
					z_var = self.measurement_var
					x_mean, x_var = self.kalman_filter_1D(prior_mean, prior_var, z_mean, z_var)
					self.map.list[index].position.x = x_mean
					self.map.list[index].varianceX = x_var
					# ======= Kalman filter for y =======
					prior_mean = self.map.list[index].position.y
					prior_var = self.map.list[index].varianceY
					z_mean = self.obj_list.list[i].position.y
					z_var = self.measurement_var
					y_mean, y_var = self.kalman_filter_1D(prior_mean, prior_var, z_mean, z_var)
					self.map.list[index].position.y = y_mean
					self.map.list[index].varianceY = y_var
					self.map.list[index].pcl_points = self.obj_list.list[i].pcl_points
					self.map.list[index].position_local = self.obj_list.list[i].position_local
					self.map.list[index].type = self.class_filter(index, self.obj_list.list[i].type)
					'''if self.obj_list.list[i].type != 'None':
						self.map.list[index].type = self.obj_list.list[i].type'''
				else:
					obj = ObjectPose()
					obj = self.obj_list.list[i]
					obj.varianceX = self.init_varX
					obj.varianceY = self.init_varY
					self.map.list.append(obj)
		for j in range(self.map.size):
			if not j in self.matching:
				dis2rob = self.distance_to_robot(self.map.list[j])
				if dis2rob < self.punish_range  and dis2rob > 4.:
					self.map.list[j].varianceX = self.map.list[j].varianceX*self.punish_no_detect
					self.map.list[j].varianceY = self.map.list[j].varianceY*self.punish_no_detect
		self.map.size = len(self.map.list)
		#print self.map.size
		#self.cluster_dock()

	def kalman_filter_1D(self, prior_mean, prior_var, z_mean, z_var):
		#======= Predict =======
		predict = scipy.stats.norm(loc = prior_mean + self.kernel.mean(), scale = np.sqrt(prior_var + self.kernel.var()))
		#======= Update step =======
		likelihood = scipy.stats.norm(loc = z_mean, scale = np.sqrt(z_var))
		posterior = self.gaussian_multiply(likelihood, predict)
		return posterior.mean(), posterior.var()

	def kalman_filter_2D(self, x, y, index):
		#======= Predict =======
		'''if self.prior_mean == None:	# Recieve first measurement
			self.prior_mean = np.array([x, y])		# State vector
			self.prior_cov = self.pos_covariance		# Covariance matrix'''
		prior_mean = np.array([self.map.list[index].position.x, self.map.list[index].position.y])
		#prior_cov = np.diag([0.5, 0.5])
		prior_cov = np.diag([self.map.list[index].varianceX, self.map.list[index].varianceY])
		F = np.array([[1., 0], [0, 1.]])			# State transition matrix
		predict_mean = np.dot(F, prior_mean)
		predict_cov = np.dot(F, prior_cov).dot(F.T)
		#predict_pos = np.random.multivariate_normal(predict_mean, predict_cov, 100)

		#======= Update step =======
		z = np.array([x, y])				# Measurement
		H = np.array([[1., 0]])				# Measurement function (Nothing to convert to measurement space)
		R = np.array([[self.sensor_error]]) # Measurement covariance (For sensor)
		S = np.dot(H, predict_cov).dot(H.T) + R 				# System uncertainty
		K = np.dot(predict_cov, H.T).dot(np.linalg.inv(S))	# Kalman gain
		residual = z - np.dot(H, predict_mean)					# Residual = measurement - prediction
		posterior_mean = predict_mean + np.dot(K.T, residual)
		posterior_cov = predict_cov - np.dot(K, H).dot(predict_cov)
		return posterior_mean, posterior_cov

	def gaussian_multiply(self, g1, g2):
		g1_mean, g1_var = g1.stats(moments='mv')
		g2_mean, g2_var = g1.stats(moments='mv')
		mean = (g1_var * g2_mean + g2_var * g1_mean) / (g1_var + g2_var)
		variance = (g1_var * g2_var) / (g1_var + g2_var)
		return scipy.stats.norm(loc = mean, scale = np.sqrt(variance))

	def distance(self, a, b): # caculate distance between two 3d points
		return math.sqrt((a.position.x-b.position.x)**2 + (a.position.y-b.position.y)**2 + (a.position.z-b.position.z)**2)

	def distance_to_robot(self, p): # caculate distance between two 3d points
		return math.sqrt((p.position.x - self.robot_pose[0])**2 + (p.position.y - self.robot_pose[1])**2 + (p.position.z - self.robot_pose[2])**2)

	def drawRviz(self, obj_list):
		marker_array = MarkerArray()
		# marker_array.markers.resize(obj_list.size)
		#print obj_list.size
		for i in range(obj_list.size):
			#print obj_list.list[i].varianceX, obj_list.list[i].varianceY
			marker = Marker()
			marker.header.frame_id = obj_list.header.frame_id
			marker.id = i
			marker.header.stamp = rospy.Time.now()
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.lifetime = rospy.Duration(3)
			marker.pose.position.x = obj_list.list[i].position.x
			marker.pose.position.y = obj_list.list[i].position.y
			marker.pose.position.z = obj_list.list[i].position.z
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 1
			marker.scale.y = 1
			marker.scale.z = 1
			# Buoy  -> BLUE
			# Totem -> GREEN
			# Dock  -> WHITE
			# Not been classified -> RED
			if obj_list.list[i].type == "buoy":
				marker.color.r = 0
				marker.color.g = 0
				marker.color.b = 1
				marker.color.a = 0.5
			elif obj_list.list[i].type == "totem":
				marker.color.r = 0
				marker.color.g = 1
				marker.color.b = 0
				marker.color.a = 0.5
			elif obj_list.list[i].type == "light_buoy":
				marker.color.r = 1
				marker.color.g = 1
				marker.color.b = 0
				marker.color.a = 0.5
			elif obj_list.list[i].type == "dock":
				marker.color.r = 1
				marker.color.g = 1
				marker.color.b = 1
				marker.color.a = 0.5
				marker.scale.x = 6
				marker.scale.y = 6
				marker.scale.z = 1
			else:
				marker.color.r = 1
				marker.color.g = 0
				marker.color.b = 0
				marker.color.a = 0.5
				marker.scale.x = 2
				marker.scale.y = 2
				marker.scale.z = 2
			marker_array.markers.append(marker)
		self.pub_marker.publish(marker_array)

if __name__ == "__main__":
	rospy.init_node('mapping')
	# Tell ROS that we're making a new node.
	rospy.init_node("mapping",anonymous=False)
	rospy.loginfo("Start Mapping")
	foo = mapping()
rospy.spin()
