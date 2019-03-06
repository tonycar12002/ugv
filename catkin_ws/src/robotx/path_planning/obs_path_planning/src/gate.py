#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import atan2, cos, sin, sqrt, tan, hypot, pi
from scipy.spatial import Delaunay
from robotx_msgs.msg import ObjectPoseList
from robotx_msgs.msg import Waypoint, WaypointList, roboteq_drive
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
		self.sub_obstacleList = rospy.Subscriber("/obj_list/map", ObjectPoseList, self.call_back, queue_size=1, buff_size = 2**24)
		self.sub_nav_state = rospy.Subscriber("/wp_nav_state", Int32, self.nav_state, queue_size=1)
		self.sub_new_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.call_new_goal, queue_size=1)
		self.pub_rviz = rospy.Publisher("/wp_line", Marker, queue_size = 1)
		self.start_point = [0, 0]
		self.orientation = None
		self.state = 0
		self.get_goal = False
		self.goal_point = []
		self.docking = []
		self.backward = False
		self.start_t = None
		self.end_t = None
		self.callc = False
		self.goal_count = 0
		print "======================"

	def nav_state(self, msg):
		print(self.state)
		print(self.backward)
		self.state = msg.data

	def call_new_goal(self, p):
		if self.goal_count >= 2:
                        self.arrived = False
                        self.goal_count = 0
			self.get_goal = False
                        self.goal_point = []
			self.callc = False
		if self.goal_count == 1:
			self.get_goal = True
		self.goal_point.append([p.pose.position.x, p.pose.position.y])
		q = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.goal_orientation = euler[2]
		self.arrived = False
		self.state = 0
		self.first_get_time = False
		self.goal_count = self.goal_count + 1

	def call_back(self, msg):
		self.start_point = [msg.robot.position.x, msg.robot.position.y]
		q = [msg.robot.orientation.x, msg.robot.orientation.y, msg.robot.orientation.z, msg.robot.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		yaw = euler[2]
		self.orientation = yaw
		if self.get_goal:
			print('get')
			dis = self.distance(self.start_point, self.goal_point[-1])
			print(dis)
			self.rviz()	
		if self.get_goal and not self.backward:
			if not self.callc:
				print(self.goal_point)
				self.call_controller()
				self.callc = True
			if self.state == 1 and dis < 1 and not self.first_get_time:
				self.start_t = rospy.get_time()
				self.first_get_time = False
			else:
				self.start_t = rospy.get_time()
			self.end_t = rospy.get_time()
			if (self.end_t-self.start_t) >= 20:
				self.backward = True
		elif self.get_goal and self.backward:
			clear_wp = rospy.ServiceProxy("/clear_waypoints", Trigger)
			try:
				rospy.wait_for_service("/clear_waypoints")
				clear_wp()
			except (rospy.ServiceException, rospy.ROSException), e:
				rospy.logerr("Service call failed: %s" % (e,))
			drive_msg = roboteq_drive()
			drive_msg.left = -900
			drive_msg.right = -900
			self.pub_back.publish(drive_msg)

	def distance(self, p1, p2):
		return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def call_controller(self):
		clear_wp = rospy.ServiceProxy("/clear_waypoints", Trigger)
		try:
			rospy.wait_for_service("/clear_waypoints")
			clear_wp()
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
		add_wp = rospy.ServiceProxy("/add_waypoint", waypoint)
		for i in range(len(self.goal_point)):
			print(self.goal_point[i])
			wp_srv = waypointRequest()
			wp_srv.waypointx = self.goal_point[i][0]
			wp_srv.waypointy = self.goal_point[i][1]
			wp_srv.yaw = self.goal_orientation
			res = waypointResponse()
			try:
				rospy.wait_for_service("/add_waypoint")
				res.waypoint_len = add_wp(wp_srv)
			except (rospy.ServiceException, rospy.ROSException), e:
				rospy.logerr("Service call failed: %s" % (e,))
		start_motion = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
		start_motion()


	def rviz(self):
		marker = Marker()
		marker.header.frame_id = 'odom'
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
		p = Point()
		p.x = self.start_point[0]
		p.y = self.start_point[1]
		pp = Point()
		pp.x = self.goal_point[0][0]
		pp.y = self.goal_point[0][1]
		p_dock = Point()
		p_dock.x = self.goal_point[1][0]
		p_dock.y = self.goal_point[1][1]
		marker.points.append(p)
		marker.points.append(pp)
		marker.points.append(p_dock)
		self.pub_rviz.publish(marker)
	
if __name__ == "__main__":
	rospy.init_node("Path_planning_node", anonymous = False)
	rrt_planning = PathPlanning()
	rospy.spin()
