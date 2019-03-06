#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date: 2018/06/23                                                                
Last update: 2018/08/19                                                              
Move to Light buoy
Subscribe: 
  /obj_list             (robotx_msgs/ObjectPoseList)
Publish:
  /motion_arrow         (visualization_msgs/Marker)
  /arrive_buoy          (std_msgs/Bool)
  /obj                  (robotx_msgs/ObjectPose)
  /motor_cmd            (robotx_gazebo/UsvDrive)
'''
import rospy
import math
from robotx_msgs.msg import ObjectPoseList, ObjectPose, roboteq_drive
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from robotx_gazebo.msg import UsvDrive

class Motion2Buoy(object):
    def __init__(self):

        #Param
        self.node_name  = rospy.get_name()
        self.arrive_buoy = False
        self.linear_speed   = rospy.get_param('~linear_speed', 0.25)
        self.angular_speed  = rospy.get_param('~angular_speed', 0.3)
        self.visual = rospy.get_param('~visual', False)
        self.target_dis = rospy.get_param('~target_dis', 3.5)
        self.target_angle = rospy.get_param('~target_angle', 35.0)
        self.tolerance  = rospy.get_param('~tolerance', 2.0)
        self.sim = rospy.get_param('~sim', True)
        

        self.light_buoy_x = 0.0
        self.light_buoy_y = -1.0

        rospy.loginfo("[%s] Initializing " %(self.node_name))  
        rospy.loginfo("[%s] Param [visual] = %d" %(self.node_name, self.visual))  
        rospy.loginfo("[%s] Param [sim] = %d" %(self.node_name, self.sim))
        rospy.loginfo("[%s] Param [target_dis] = %f" %(self.node_name, self.target_dis))  
        rospy.loginfo("[%s] Param [target_angle] = %f" %(self.node_name, self.target_angle))
        rospy.loginfo("[%s] Param [tolerance] = %f" %(self.node_name, self.tolerance))  
        rospy.loginfo("[%s] Param [linear_speed] = %f" %(self.node_name, self.linear_speed)) 
        rospy.loginfo("[%s] Param [angular_speed] = %f" %(self.node_name, self.angular_speed)) 

        #Send motor cmd
        rospy.Timer(rospy.Duration(0.5), self.sendMotorCmd)

        #Publisher
        if self.sim:
            self.pub_motion         = rospy.Publisher("~motor_cmd", UsvDrive, queue_size=1)
        else:
            self.pub_motion         = rospy.Publisher("~motor_cmd", roboteq_drive, queue_size=1)

        self.pub_arrive_buoy    = rospy.Publisher("~arrive_buoy", Bool, queue_size=1)
        self.pub_markers        = rospy.Publisher("~motion_arrow", Marker, queue_size=1)
        self.pub_buoy           = rospy.Publisher("~obj", ObjectPose, queue_size=1)

        #Subscriber
        self.sub_ob_list = rospy.Subscriber("~obj_list", ObjectPoseList, self.cbObstacleList, queue_size=1)

    def sendMotorCmd(self, event):

        if(self.light_buoy_y < 0):
            return

        '''
        ***************************************************************  
            Caluculate distance between WAM-V and target, then transfer to motion
        ***************************************************************      
        '''     

        radius = math.atan2(self.light_buoy_y, self.light_buoy_x)
        radius = radius - math.pi/2 
        degree = radius*180/math.pi
        dis = self.light_buoy_x * self.light_buoy_x + self.light_buoy_y * self.light_buoy_y
        dis = math.sqrt(dis)
        if(self.visual):
            print "Buoy Distance = ", dis, ", degree = ", degree, ", radius = ", radius

        if(self.sim):
            motor_msg = UsvDrive()
        else:
            motor_msg = roboteq_drive()
        motor_msg.left = 0
        motor_msg.right = 0

        if(dis<=self.target_dis + self.tolerance and abs(degree)<= self.target_angle):
            self.arrive_buoy = True
        elif(dis>=self.target_dis + self.tolerance ) or abs(degree) >= self.target_angle:
            self.arrive_buoy = False
        
        # Backward
        if(dis<=self.target_dis - self.tolerance):
            motor_msg.left  = -self.angular_speed - self.linear_speed * radius
            motor_msg.right = -self.angular_speed + self.linear_speed * radius
            if self.visual:
                print 'Backward'
        # Pose keep
        elif(dis<=self.target_dis + self.tolerance):
            motor_msg.left  = - (self.linear_speed * radius * 4.0)
            motor_msg.right = (self.linear_speed * radius * 4.0) 
            if self.visual:
                print 'Pose keep'
        # Go ahead
        else:
            motor_msg.left  = self.angular_speed - self.linear_speed * radius
            motor_msg.right = self.angular_speed + self.linear_speed * radius
            if self.visual:
                print 'Go ahead'
        if not self.sim:
            motor_msg.left = motor_msg.left * 1000
            motor_msg.right = motor_msg.right * 1000
        self.pub_motion.publish(motor_msg)

        if self.visual:
            self.drawShipPose(motor_msg)

        arrive_msg = Bool()
        arrive_msg.data = self.arrive_buoy
        self.pub_arrive_buoy.publish(arrive_msg)

    def cbObstacleList(self, object_list):  
        '''
        ***************************************************************  
            Get light buoy position
        ***************************************************************      
        '''  
        dis_min = 99
        record = -1
        if object_list.size is not 0:
            for i in range(object_list.size):
                x = object_list.list[i].position.x
                y = object_list.list[i].position.y
                dis = math.sqrt(x*x+y*y)
                if dis < dis_min:
                    record = i
                    dis_min = dis
            self.light_buoy_x = object_list.list[record].position.x
            self.light_buoy_y = object_list.list[record].position.y
            self.pub_buoy.publish(object_list.list[record])
        
    def drawShipPose(self, motor):
        '''
        ***************************************************************  
            Using Arrow marker to visualize motion command
        ***************************************************************      
        '''  
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.header.stamp = rospy.get_rostime()
        marker.type     = marker.ARROW
        marker.action   = marker.ADD
        marker.scale.x  = 0.1
        marker.scale.y  = 0.1
        marker.scale.z  = 0.1
        marker.color.a  = 1.0
        marker.color.g  = 1.0
        st_pt = Point()
        st_pt.x = st_pt.y = st_pt.z = 0
        theta = motor.right - motor.left
        ed_pt = Point()
        ed_pt.z = 0
        ed_pt.x = 2*math.cos(theta)
        ed_pt.y = 2*math.sin(theta)
        marker.points.append(st_pt)
        marker.points.append(ed_pt)
        
        self.pub_markers.publish(marker)
        
    def onShutdown(self):
        motor_msg = UsvDrive()
        motor_msg.left = 0
        motor_msg.right = 0
        self.pub_motion.publish(motor_msg)
        rospy.loginfo("[%s] Shutdown." %(self.node_name))


if __name__ == '__main__':
    rospy.init_node('motion2buoy_node', anonymous=False)
    motion2buoy_node = Motion2Buoy()
    rospy.on_shutdown(motion2buoy_node.onShutdown)
    rospy.spin()