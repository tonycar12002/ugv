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
from std_msgs.msg import Int32, Bool
from robotx_msgs.srv import *
from std_srvs.srv import *
#from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class semi_obs(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		self.debug = False
		self.frame_id = 'odom'
		self.pub_arr = rospy.Publisher('/navobs/arrive', Bool, queue_size = 1)
		self.pub_arr = rospy.Publisher('/obs/arrive', Bool, queue_size = 1)
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		self.sub_nav_state = rospy.Subscriber("/wp_nav_state", Int32, self.nav_state, queue_size=1)
		self.sub_new_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.call_new_goal, queue_size=1)
		self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.call_back, queue_size=1)
		self.pub_rviz = rospy.Publisher("/multi_goal", Marker, queue_size = 1)
		self.start_obs_srv = rospy.Service("/start_obs", Trigger, self.start_obs)
		#self.start_nav_srv = rospy.Service("/start_nav", Trigger, self.start_nav)
		self.start_semi_srv = rospy.Service("/start_semi", Trigger, self.start_semi)
		self.robot = None
		self.orientation = None
		self.initial()
	
	def initial(self):
		self.state = 1
		self.pid_state = 0
		self.goal_points = []
		self.done_goal = False
		self.goal_count = 0
		self.arrived = False
		self.get_goal = False
		self.goal_point_idx = 0
		self.robot = []
		self.next_goal = False
		self.first_call = True
		self.arrived_last = False
		self.obs_start_flag = False
		self.obs_start_idx = None

	def call_back(self, msg):
		self.rviz()
		q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.robot = [msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2]]
		if self.arrived_last:
			ms = Bool()
			ms.data = True
			self.pub_arr.publish(ms)
			return
		if not self.done_goal or len(self.goal_points) == 0:
			return
		
		dis = self.distance(self.robot[:2], self.goal_points[self.goal_point_idx][:2])
		if dis <= 6. and self.pid_state == 1:
			print("Arrived goal point: '", self.goal_point_idx, "'")
			self.goal_point_idx = self.goal_point_idx + 1
			self.arrived = True
			self.next_goal = True
			#rospy.sleep(1)
		if self.next_goal and self.goal_point_idx < len(self.goal_points):
			if self.goal_point_idx >= self.obs_start_idx:
				self.add_obs_waypoint(self.goal_points[self.goal_point_idx])
			else:
				self.call_controller(self.goal_points[self.goal_point_idx])
		if self.first_call:
			self.add_obs_waypoint(self.goal_points[self.goal_point_idx])
			self.first_call = False
		elif self.next_goal and self.goal_point_idx >= len(self.goal_points):
			print("Arrived last goal point!!!")
		if self.goal_point_idx >= len(self.goal_points):
			self.arrived_last = True
			

	def call_new_goal(self, p):
		if self.done_goal:
			self.initial()
		self.goal_count = self.goal_count + 1
		q = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.goal_points.append([p.pose.position.x, p.pose.position.y, euler[2]])
		self.get_goal = True
		print('Get goal: ', [p.pose.position.x, p.pose.position.y, euler[2]])

	def start_obs(self, req):
		if not self.obs_start_flag:
			self.obs_start_idx = self.goal_count
			self.obs_start_flag = True
		print('Change to obstacle avoidance')
		print('===================')
		res = TriggerResponse()
		res.success = True
		res.message = "start"
		return res

	def start_semi(self, req):
		self.done_goal = True
		print('Start To Run!!!')
		print('Goal Numbers: ', self.goal_count)
		print('===================')
		res = TriggerResponse()
		res.success = True
		res.message = "start"
		return res

	def nav_state(self, msg):
		self.pid_state = msg.data

	def add_obs_waypoint(self, goal_point):
		try:
			send_waypoint = rospy.ServiceProxy("/new_goal", waypoint)
			wp_srv = waypointRequest()
			wp_srv.waypointx = goal_point[0]
			wp_srv.waypointy = goal_point[1]
			wp_srv.yaw = goal_point[2]
			
			res = waypointResponse()
			rospy.wait_for_service("/new_goal")
			res.waypoint_len = send_waypoint(wp_srv)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def distance(self, p1, p2):
		return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def onShutdown(self):
		rospy.loginfo("Node shutdown")

	def call_controller(self, goal_point):
		clear_wp = rospy.ServiceProxy("/clear_waypoints", Trigger)
		try:
			rospy.wait_for_service("/clear_waypoints")
			clear_wp()
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
		add_wp = rospy.ServiceProxy("/add_waypoint", waypoint)
		wp_srv = waypointRequest()
		wp_srv.waypointx = goal_point[0]
		wp_srv.waypointy = goal_point[1]
		wp_srv.yaw = goal_point[2]
		res = waypointResponse()
		try:
			rospy.wait_for_service("/add_waypoint")
			res.waypoint_len = add_wp(wp_srv)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
		start_motion = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
		start_motion()
		
		
	def rviz(self):
		if len(self.goal_points) == 0:
			return
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
		marker.points = []
		p = Point()
		p.x = self.robot[0]
		p.y = self.robot[1]
		p.z = 0
		marker.points.append(p)
		for i in range(len(self.goal_points) - self.goal_point_idx):
			idx = i + self.goal_point_idx
			if idx >= self.obs_start_idx:
				marker.color.r = 1.0
				marker.color.g = 0.0
				marker.color.b = 1.0
			else:
				marker.color.r = 0.0
				marker.color.g = 1.0
				marker.color.b = 1.0
			p = Point()
			p.x = self.goal_points[idx][0]
			p.y = self.goal_points[idx][1]
			p.z = 0
			marker.points.append(p)
		self.pub_rviz.publish(marker)
	
if __name__ == "__main__":
	rospy.init_node("semi_obs", anonymous = False)
	semi_obs = semi_obs()
	rospy.spin()
