#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date: 2018/10/01                                                                
Last update: 2018/10/01                                                              
Totem Circling
Issue:
    1. The condition which WAM-V doesn't detect any totem

'''
import rospy
from sensor_msgs.msg import Image, CompressedImage
from robotx_msgs.msg import ObjectPoseList, ObjectPose
from cv_bridge import CvBridge, CvBridgeError
from totem_circle.srv import SetTotemCircle, SetTotemCircleRequest, SetTotemCircleResponse
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from robotx_msgs.msg import Waypoint, WaypointList
from robotx_msgs.srv import waypoint
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf
import tf_conversions
import math
import imp
import rospkg
import sys
import os
import time
import numpy as np
import cv2
from collections import Counter

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('zed_perception') + "/src/"
sys.path.append(os.path.dirname(os.path.expanduser(pkg_path)))
from color_detect_totem import ColorDetectHSV
class Totem(object):
    def __init__(self, x, y, z, color):
        self.x = x
        self.y = y
        self.z = z
        self.color = color
        self.color_record = []

class TotemCircling(object):
    def __init__(self):
        # Variables
        self.node_name = rospy.get_name()
        self.img = None
        self.cv_bridge = CvBridge()
        self.totem_list = [] # (x, y, z, color)
        self.color_detect = ColorDetectHSV()
        self.wamv_position = None

        self.tf_listener = tf.TransformListener()
        self.tf_transformer = tf.TransformerROS()

        self.translation_lidar_camera =  [0.0, 0.13, 0.5]
        self.quaternion_lidar_camera =  [ 0, 0, -0.7068851, 0.7073284 ]
        self.camera_matrix = [1400.69, 0.0, 971.463, 0.0, 1400.69, 535.668, 0.0, 0.0, 1.0]
        #self.translation_lidar_camera =  [-0.350, 0.000, 0.291]
        #self.quaternion_lidar_camera =  [ 0, 0, -0.707, 0.707 ]
        #self.camera_matrix = [762.7249337622711, 0.0, 640.5, 0.0, 762.7249337622711, 360.5, 0.0, 0.0, 1.0]

        self.totem_circle_command = []
        self.start = False
        rospy.Timer(rospy.Duration(0.5), self.circle_path_planning)
        #self.add_wp = rospy.ServiceProxy("/add_waypoint", waypoint)
        #self.start_wp = rospy.ServiceProxy("/start_waypoint_nav", Trigger)


        # Ros service
        self.srv_totem_cirle = rospy.Service('~set_totem_cirle', SetTotemCircle, self.cb_srv_totem_circle)
        self.srv_totem_cirle = rospy.Service('~clear', Trigger, self.cb_srv_clear)
        self.srv_totem_cirle = rospy.Service('~start', Trigger, self.cb_srv_start)

        # Publisher
        self.pub_image_roi = rospy.Publisher("~image_roi", Image, queue_size=1)
        self.pub_image_roi_compressed = rospy.Publisher("~image_roi/compressed", CompressedImage, queue_size=1)
        self.pub_image_mask = rospy.Publisher("~image_mask", Image, queue_size=1)
        self.pub_marker = rospy.Publisher("~totem_marker", MarkerArray, queue_size=1)

        # Subscriber
        self.sub_image = rospy.Subscriber("~image_raw_compressed", CompressedImage, self.cb_image, queue_size=1)
        self.sub_obj_list = rospy.Subscriber("~obj_list", ObjectPoseList, self.cb_obj_list, queue_size=1)
        self.sub_odometry = rospy.Subscriber("/odometry/filtered", Odometry, self.cb_odometry, queue_size=1)


    def add_waypoint(self, x, y):
        rospy.wait_for_service("/add_waypoint")
        try:
            send_waypoint = rospy.ServiceProxy("/add_waypoint", waypoint)
            waypoint_len = send_waypoint(x, y, 0, 0)
            print "set way_point, x = ", x, ", y = ", y
            self.start_waypoint()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def start_waypoint(self):
        rospy.wait_for_service("/start_waypoint_nav")
        try:
            start_waypoint = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
            start_ = start_waypoint()
 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e        

    def clear_waypoint(self):
        rospy.wait_for_service("/clear_waypoints")
        try:
            clear_waypoint = rospy.ServiceProxy("/clear_waypoints", Trigger)
            clear_ = clear_waypoint()
            print "clear waypoints"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e            

    def calucalte_waypoint(self, target_totem, clockwise):
        ck = -1 if clockwise else 1
        waypoint_list = []
        angle = math.atan2(target_totem.y-self.wamv_position.y, target_totem.x-self.wamv_position.x)
        x_1 = target_totem.x - 10 * math.cos(angle) 
        y_1 = target_totem.y - 10 * math.sin(angle) 
        x_2 = target_totem.x + 10 * math.sin(angle) * ck
        y_2 = target_totem.y - 10 * math.cos(angle) * ck
        x_3 = target_totem.x + 10 * math.cos(angle) 
        y_3 = target_totem.y + 10 * math.sin(angle) 
        x_4 = target_totem.x - 10 * math.sin(angle) * ck
        y_4 = target_totem.y + 10 * math.cos(angle) * ck    
        print "target_totem x = ", target_totem.x, " y = ", target_totem.y
        waypoint_list.append([x_1, y_1])
        waypoint_list.append([x_2, y_2])
        waypoint_list.append([x_3, y_3])
        waypoint_list.append([x_4, y_4])
        waypoint_list.append([x_1, y_1])
        for way_point in waypoint_list:
            x = way_point[0]
            y = way_point[1]
            self.add_waypoint(x, y)   
            x_ = self.wamv_position.x - x
            y_ = self.wamv_position.y - y
            while math.sqrt(x_*x_ + y_*y_) >= 5 :
                x_ = self.wamv_position.x - x
                y_ = self.wamv_position.y - y                

    def circle_path_planning(self, event):
        if self.start is False or len(self.totem_circle_command) is 0:
            return

        # ================================================
        #   Cirling the target totem 
        # ================================================
        index = 0
        find_any_target = False
        random_totem_index = None
        arrive_random_totem = True
        while(len(self.totem_circle_command) != 0):

            if index == len(self.totem_circle_command):
                index = 0

                # Go to a totem which does not have color
                if not find_any_target and random_totem_index is not None and arrive_random_totem:
                    print ("Go random")
                    self.add_waypoint(self.totem_list[random_totem_index].x, self.totem_list[random_totem_index].y)
                    arrive_random_totem = False

                # Avoid keep sending waypoint and check totem find color
                if random_totem_index is not None:
                    #print self.totem_list[random_totem_index].x, self.totem_list[random_totem_index].y, self.totem_list[random_totem_index].color
                    
                    dis = self.distance(self.wamv_position, self.totem_list[random_totem_index]) 
                    if dis <= 5 or self.totem_list[random_totem_index].color is not "NONE":
                        self.clear_waypoint()
                        arrive_random_totem= True

                find_any_target = False

            # totem(x, y, z, color)  
            # totem_circle_command(color, clockwise)
           
            for i in range(0, len(self.totem_list)):
                
                if len(self.totem_circle_command) is 0:
                    break
                totem  = self.totem_list[i]

                if totem.color is "NONE" and arrive_random_totem:
                    nt = self.nearest_totem()
                    if nt == -1:
                        random_totem_index = i
                    else:   
                        random_totem_index = nt
                    #print self.totem_list[random_totem_index].x, self.totem_list[random_totem_index].y, self.totem_list[random_totem_index].color

                if totem.color.lower() == self.totem_circle_command[index][0].lower():
                    self.clear_waypoint()
                    self.calucalte_waypoint(totem, self.totem_circle_command[index][1])
                    print ("======================================")
                    del self.totem_circle_command[index]
                    index -= 1
                    find_any_target = True

            index += 1

        print "DONE"

    def nearest_totem(self):
        dis_min = 1000
        index = -1
        for i in range(0, len(self.totem_list)):
            if self.totem_list[i].color is "NONE":
                dis = self.distance(self.wamv_position, self.totem_list[i])
                if dis < dis_min:
                    dis_min = dis
                    index = i

        return index

    def cb_srv_totem_circle(self, request):
        self.totem_circle_command.append([request.color, request.clockwise])
        print ("Add totem circle [color] = " + request.color+ " [clockwise] = " + str(request.clockwise))
        return SetTotemCircleResponse()
    
    def cb_srv_clear(self, request):
        self.totem_circle_command = []
        print ("Clear")
        return TriggerResponse()
    
    def cb_srv_start(self, request):
        print ("Start")
        self.start = True
        return TriggerResponse()

    def cb_odometry(self, msg_odom):
        self.wamv_position = msg_odom.pose.pose.position

    def cb_obj_list(self, msg_obj_list):
        # Update totem_list
        for obj in msg_obj_list.list:
            if obj.type == "totem":
                index = self.totem_matching(obj, self.totem_list)
                if index == -1:
                    self.totem_list.append( Totem(obj.position.x, obj.position.y, obj.position.z, "NONE") )
                else:
                    self.totem_list[index].x = obj.position.x
                    self.totem_list[index].y = obj.position.y
                    self.totem_list[index].z = obj.position.z

    def totem_matching(self, obj, totem_list):
        # ================================================
        #   Find whether there has same totem in the list
        #   Return index or -1(None)
        # ================================================

        if totem_list is None:
            return -1

        for i in range(0, len(totem_list)):
            totem = totem_list[i]
            _x = totem.x - obj.position.x
            _y = totem.y - obj.position.y 
            dis = math.sqrt( _x*_x + _y*_y )
            if dis <= 3:
                return i

        return -1

    def distance(self, wamv_position, totem):
        x_ = wamv_position.x - totem.x
        y_ = wamv_position.y - totem.y    
        dis = math.sqrt(x_*x_ + y_*y_)
        return dis    

    def cb_image(self, msg_img):

        self.img = self.cv_bridge.compressed_imgmsg_to_cv2(msg_img, "bgr8")

        if self.totem_list is None:
            return

        # ================================================
        #   Get Transformation matrix 
        #   bewteen odom and velodyne, lidar and camera
        # ================================================
        trans = None
        while trans is None:
            try:
                (trans, qua) = self.tf_listener.lookupTransform("/velodyne", "/odom", rospy.Time(0))
            except (tf.LookupException):
                print "Nothing Happen"

        transform_matrix_odom_velodyne = self.tf_transformer.fromTranslationRotation(trans, qua)
        transform_matrix_lidar_camera = self.tf_transformer.fromTranslationRotation(self.translation_lidar_camera, self.quaternion_lidar_camera)


        # ================================================
        #   Find totem color and update the totem_list
        # ================================================
        
        for totem in self.totem_list:

            # Don't process the totem > threshold           
            if self.distance(self.wamv_position, totem) >= 15:
                pass
            else:
                # Transform pose from odom to velodyne
                totem_position = np.zeros((4, 1))
                totem_position[0, 0] = totem.x # x
                totem_position[1, 0] = totem.y # y
                totem_position[2, 0] = totem.z # z
                totem_position[3, 0] = 1
                lidar_totem_position = np.matmul(transform_matrix_odom_velodyne, totem_position)
                # Transform pose from velodyne to camera
                camera_totem_position = np.matmul(transform_matrix_lidar_camera, lidar_totem_position)
                
                
                totem_image_x =  self.camera_matrix[0]*(-camera_totem_position[1, 0]/camera_totem_position[0, 0]) + self.camera_matrix[2]
                totem_image_y =  self.camera_matrix[4]*(-camera_totem_position[2, 0]/camera_totem_position[0, 0]) + self.camera_matrix[5]
                totem_image_x = int(totem_image_x)
                totem_image_y = int(totem_image_y)


                if camera_totem_position[0, 0] >=0 and totem_image_x >= 140 and \
                    totem_image_y >= 150 and totem_image_y <= self.img.shape[0]-200 and totem_image_x <= self.img.shape[1]-140 :
                    #img_roi = self.img[totem_image_y-70:totem_image_y+50, totem_image_x-90:totem_image_x+90].copy()
                    img_roi = self.img[totem_image_y-140:totem_image_y+70, totem_image_x-120:totem_image_x+120].copy()

                    color, img_mask = self.color_detect.get_color_and_image_mask(img_roi)
                    
                    if len(totem.color_record) < 10:
                        msg_img_mask = Image()
                        msg_img_mask = self.cv_bridge.cv2_to_imgmsg(img_mask, "bgr8")
                        self.pub_image_mask.publish(msg_img_mask)

                        totem.color_record.append(color)
                        max_color = max(totem.color_record, key=totem.color_record.count)
                        totem.color = max_color

                        color_record_remove_none = list(totem.color_record)
                        color_record_remove_none = filter(lambda a: a != "NONE", color_record_remove_none)
                        second_max_color = "NONE"
                        
                        if len(color_record_remove_none)>0:
                            second_max_color = max(color_record_remove_none, key=color_record_remove_none.count)
                        if max_color is "NONE":
                            totem.color = second_max_color
                        if len(totem.color_record) is 10 and totem.color == "NONE":
                            totem.color = "White"
                        print totem.color

                    msg_img_roi = Image()
                    msg_img_roi_compressed = CompressedImage()
                    msg_img_roi = self.cv_bridge.cv2_to_imgmsg(img_roi, "bgr8")
                    msg_img_roi_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(img_roi)
                    self.pub_image_roi.publish(msg_img_roi)
                    self.pub_image_roi_compressed.publish(msg_img_roi_compressed)

        if len(self.totem_list) != 0:
            self.draw_totem()
            
    def draw_totem(self):
        marker_array = MarkerArray()
        
        for i in range(0, len(self.totem_list) ):
            totem = self.totem_list[i]
            marker = Marker()
            marker.header.frame_id = "/odom"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.id = i
            marker.pose.position.x = totem.x
            marker.pose.position.y = totem.y
            marker.pose.position.z = totem.z
            marker.color.a = 1.0
            if totem.color.lower() == "yellow":
                 marker.color.r = 1.0
                 marker.color.g = 1.0
                 marker.color.b = 0.0
            elif totem.color.lower() == "blue":
                 marker.color.r = 0.0
                 marker.color.g = 0.0
                 marker.color.b = 1.0
            elif totem.color.lower() == "green":
                 marker.color.r = 0.0
                 marker.color.g = 1.0
                 marker.color.b = 0.0
            elif totem.color.lower() == "red":
                 marker.color.r = 1.0
                 marker.color.g = 0.0
                 marker.color.b = 0.0
            elif totem.color.lower() == "black":
                 marker.color.r = 0.0
                 marker.color.g = 0.0
                 marker.color.b = 0.0    
            elif totem.color.lower() == "white":
                 marker.color.r = 1.0
                 marker.color.g = 1.0
                 marker.color.b = 1.0                   
            else:
                 marker.color.r = 0.0
                 marker.color.g = 1.0
                 marker.color.b = 1.0

            marker_array.markers.append(marker)
        self.pub_marker.publish(marker_array)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))        

if __name__ == '__main__':
    rospy.init_node('totem_cirling', anonymous=False)
    totem_cirling = TotemCircling()
    rospy.on_shutdown(totem_cirling.on_shutdown)
    rospy.spin()           