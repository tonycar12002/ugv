#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date: 2018/09/16                                                                
Last update: 2018/09/16                                                              
Move to Light buoy
Subscribe: 
  /obj_list             (robotx_msgs/ObjectPoseList)
  /odometry/filtered    (nav_msgs/Odometry)
Publish:
  /arrive_buoy          (std_msgs/Bool)
  /obj                  (robotx_msgs/ObjectPose)
  /motor_cmd            (robotx_gazebo/UsvDrive)
'''
import rospy
from robotx_msgs.msg import ObjectPose, ObjectPoseList, roboteq_drive
from robotx_gazebo.msg import UsvDrive
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import tf
import tf_conversions
from math import sqrt, atan2, pi

class MoveLightBuoy(object):
    def __init__(self):

        # Variables
        self.node_name  = rospy.get_name()
        self.light_buoy = None
        self.wamv_position = None
        self.wamv_orientation_angle = None
        self.arrive_buoy = Bool()
        self.arrive_buoy.data = False

        # Param
        self.visual = rospy.get_param("~visual", False)
        self.target_angle = rospy.get_param("~target_angle", 20.0)
        self.target_dis = rospy.get_param("~target_dis", 7.0)
        self.sim = rospy.get_param("~sim", False)
        self.linear_speed   = rospy.get_param('~linear_speed', 0.25)
        self.angular_speed  = rospy.get_param('~angular_speed', 0.3)

        # print param
        rospy.loginfo("[%s] Initializing " %(self.node_name))   
        rospy.loginfo("[%s] Param [visual] = %d" %(self.node_name, self.visual))
        rospy.loginfo("[%s] Param [target_angle] = %f" %(self.node_name, self.target_angle))
        rospy.loginfo("[%s] Param [target_dis] = %f" %(self.node_name, self.target_dis))
        rospy.loginfo("[%s] Param [sim] = %f" %(self.node_name, self.sim))
        rospy.loginfo("[%s] Param [linear_speed] = %f" %(self.node_name, self.linear_speed))
        rospy.loginfo("[%s] Param [angular_speed] = %f" %(self.node_name, self.angular_speed))

        rospy.Timer(rospy.Duration(0.3), self.update)

        # Publisher
        self.pub_obj = rospy.Publisher("~obj", ObjectPose, queue_size=1)
        self.pub_arrive_buoy = rospy.Publisher("~arrive_buoy", Bool, queue_size=1)
        if self.sim:
            self.pub_cmd = rospy.Publisher("/ccmd_drive", UsvDrive, queue_size=1)
        else:
            self.pub_cmd = rospy.Publisher("/ccmd_drive", roboteq_drive, queue_size=1)

        # Subscriber
        self.sub_obj_list = rospy.Subscriber("/obj_list/odom", ObjectPoseList, self.cb_obj, queue_size=1)
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.cb_odometry, queue_size=1)

    def update(self, event):
        if not self.light_buoy or not self.wamv_position:
            return
        (distance, angle) = self.info_between_ship_buoy()
        if angle <= -180:
            angle += 360
        if self.visual:
            print "Distance between ship and light buoy = ", distance, ", angle = ", angle
        
        # Check arrive buoy 
        self.arrive_buoy = Bool()
        if distance <= (self.target_dis+3.0) and abs(angle) <= (self.target_angle+3.0) and self.arrive_buoy.data:
            self.arrive_buoy.data = True
        elif distance <= self.target_dis and abs(angle) <= self.target_angle:
            self.arrive_buoy.data = True
        else:
            self.arrive_buoy.data = False
        self.pub_arrive_buoy.publish(self.arrive_buoy)
        self.pub_obj.publish(self.light_buoy)

        # Move to buoy
        if(self.sim):
            motor_msg = UsvDrive()
        else:
            motor_msg = roboteq_drive()
        motor_msg.left = 0
        motor_msg.right = 0
        
        # Backward
        if(distance<=self.target_dis - 3.0):
            motor_msg.left  = -self.linear_speed - self.angular_speed * angle * 0.01
            motor_msg.right = -self.linear_speed + self.angular_speed * angle * 0.01
            if self.visual:
                print 'Backward'
        # Pose keep
        elif(distance<=self.target_dis ):
            motor_msg.left  = - (self.angular_speed * angle * 0.01 * 4)
            motor_msg.right = (self.angular_speed * angle * 0.01 * 4) 
            if self.visual:
                print 'Pose keep'
        # Go ahead
        else:
            motor_msg.left  = self.linear_speed - self.angular_speed * angle * 0.01
            motor_msg.right = self.linear_speed + self.angular_speed * angle * 0.01
            if self.visual:
                print 'Go ahead'
        if not self.sim:
            motor_msg.left = motor_msg.left * 700
            motor_msg.right = motor_msg.right * 700
        #tmp = motor_msg.left
        #motor_msg.left = motor_msg.right
        #motor_msg.right = tmp
        self.pub_cmd.publish(motor_msg)


    def info_between_ship_buoy(self):
        # Get angle and distance between wamv and buoy
        x = self.light_buoy.position.x - self.wamv_position.x
        y = self.light_buoy.position.y - self.wamv_position.y
        distance = sqrt(x*x+y*y)
        angle = atan2(y, x) / pi * 180
            
        return distance, angle-self.wamv_orientation_angle

    def cb_odometry(self, odom_msg):
        # Get WAM-V position and yaw(degree)
        self.wamv_position = odom_msg.pose.pose.position

        q=[odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)  
        self.wamv_orientation_angle = yaw / pi * 180

    def cb_obj(self, obj_list_msg):
        find = True
        for obj in obj_list_msg.list:
            if obj.type == "totem":
                self.light_buoy = obj
                find = False
                return
        if (not self.light_buoy or find)  and obj_list_msg.size is not 0:
            min_dis = 100
            for obj in obj_list_msg.list:
                x = obj.position.x
                y = obj.position.y
                dis = sqrt(x*x+y*y)
                if dis < min_dis:
                    min_dis = dis
                    self.light_buoy = obj
        
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))


if __name__ == '__main__':
    rospy.init_node('move_lightbuoy', anonymous=False)
    move_light_buoy_node = MoveLightBuoy()
    rospy.on_shutdown(move_light_buoy_node.onShutdown)
    rospy.spin()        


