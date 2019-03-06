#!/usr/bin/env python
import rospy
import roslib
import math
import scipy.stats
from sensor_msgs.msg import PointCloud2
from robotx_msgs.msg import ObstaclePose, ObstaclePoseList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class mapping():
	def __init__(self): 
		# ======== Subscriber ========
		rospy.Subscriber("/obstacle_list/odom", ObstaclePoseList, self.call_back, queue_size=1)
		#rospy.Subscriber("/waypointList", WaypointList, call_back, queue_size=10)

		# ======== Publisher ========
		self.pub_obj = rospy.Publisher("/obstacle_list/map", ObstaclePoseList, queue_size=1)
		self.pub_marker = rospy.Publisher("/obs_marker/map", MarkerArray, queue_size = 1)
		#pub_rviz = rospy.Publisher("/wp_path", Marker, queue_size = 1)

		# ======== Get ROS parameter ========
		self.visual = rospy.get_param('~visual', True)

		# ======== Declare Variable ========
		self.first = True
		self.robot_pose = None
		self.map = ObstaclePoseList()
		self.obj_list = None
		self.frame_id = "odom"
		self.matching = []
		self.remove_list = []
		self.remove_threshold = 0.01
		self.r_threshold = 5
		self.prob_threshold = 0.3
		self.update_range = 40.
		self.punish_range_max = 30.
		self.punish_range_min = 5.
		self.classify_range = 10.
		self.prior_mean = None
		self.prior_cov = None
		self.punish_no_detect = 1.1
		self.measurement_var = 1.
		self.init_varX = 1.
		self.init_varY = 1.
		self.kernel = scipy.stats.norm(loc = 0, scale = 0.5)
		# ======== Get from odometry =======
		#self.pos_covariance = np.diag([3., 3.])
		self.sensor_error = 1.

	def call_back(self, msg):
		#rospy.loginfo("Process Object List")
		self.obj_list = ObstaclePoseList()
		self.obj_list = msg
		self.map_confidence = ObstaclePoseList()
		self.map_confidence.header.frame_id = self.frame_id
		self.map.header.frame_id = self.frame_id
		self.matching = []
		self.remove_list = []
		self.map.robot = self.obj_list.robot
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
				#self.map.list[i].varianceX = self.init_varX
				#self.map.list[i].varianceY = self.init_varY
		else:
			for i in range(self.map.size):
				self.map.list[i].occupy = False
			self.data_associate()
			self.update_map()
			for i in range(self.map.size):
				mean_x, mean_y = self.map.list[i].x, self.map.list[i].y
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
		self.drawRviz(self.map_confidence)

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
				if not self.map.list[j].occupy:
					dis = self.distance(self.obj_list.list[i], self.map.list[j])
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
					prior_mean = self.map.list[index].x
					prior_var = self.map.list[index].varianceX
					z_mean = self.obj_list.list[i].x
					z_var = self.measurement_var
					x_mean, x_var = self.kalman_filter_1D(prior_mean, prior_var, z_mean, z_var)
					self.map.list[index].x = x_mean
					self.map.list[index].varianceX = x_var
					# ======= Kalman filter for y =======
					prior_mean = self.map.list[index].y
					prior_var = self.map.list[index].varianceY
					z_mean = self.obj_list.list[i].y
					z_var = self.measurement_var
					y_mean, y_var = self.kalman_filter_1D(prior_mean, prior_var, z_mean, z_var)
					self.map.list[index].y = y_mean
					self.map.list[index].varianceY = y_var
					self.map.list[index].x_min_x = self.obj_list.list[i].x_min_x
					self.map.list[index].x_min_y = self.obj_list.list[i].x_min_y
					self.map.list[index].y_min_x = self.obj_list.list[i].y_min_x
					self.map.list[index].y_min_y = self.obj_list.list[i].y_min_y
					self.map.list[index].x_max_x = self.obj_list.list[i].x_max_x
					self.map.list[index].x_max_y = self.obj_list.list[i].x_max_y
					self.map.list[index].y_max_x = self.obj_list.list[i].y_max_x
					self.map.list[index].y_max_y = self.obj_list.list[i].y_max_y
				else:
					obj = ObstaclePose()
					obj = self.obj_list.list[i]
					#obj.varianceX = self.init_varX
					#obj.varianceY = self.init_varY
					self.map.list.append(obj)
		for j in range(self.map.size):
			if not j in self.matching:
				if self.distance_to_robot(self.map.list[j]) < self.punish_range_max:
					if self.distance_to_robot(self.map.list[j]) > self.punish_range_min:
						self.map.list[j].varianceX = self.map.list[j].varianceX*self.punish_no_detect
						self.map.list[j].varianceY = self.map.list[j].varianceY*self.punish_no_detect
		self.map.size = len(self.map.list)

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
		prior_mean = np.array([self.map.list[index].x, self.map.list[index].y])
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
		return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

	def distance_to_robot(self, p): # caculate distance between two 3d points
		return math.sqrt((p.x - self.robot_pose[0])**2 + (p.y - self.robot_pose[1])**2 + (p.z - self.robot_pose[2])**2)

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
			marker.pose.position.x = obj_list.list[i].x
			marker.pose.position.y = obj_list.list[i].y
			marker.pose.position.z = obj_list.list[i].z
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 1
			marker.scale.y = 1
			marker.scale.z = 1
			marker.color.r = 1
			marker.color.g = 0
			marker.color.b = 0
			marker.color.a = 0.5
			marker_array.markers.append(marker)
		self.pub_marker.publish(marker_array)

if __name__ == "__main__":
	rospy.init_node('mapping')
	# Tell ROS that we're making a new node.
	rospy.init_node("mapping",anonymous=False)
	rospy.loginfo("Start Mapping")
	foo = mapping()
rospy.spin()