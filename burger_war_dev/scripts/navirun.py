#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)




    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def strategy(self):
        #r = rospy.Rate(5) # change speed 5fps
        r = rospy.Rate(1) # change speed 5fps
        num_circle = 0
        X_pointCE = 0
        X_pointBF = -X_pointCE
        Y_pointBC = 2.00
        Y_pointEF = -Y_pointBC

#test        num_circle = 0
#test        X_pointCE = 0.25
#test        X_pointBF = -X_pointCE
#test        Y_pointBC = 1.00
#test        Y_pointEF = -Y_pointBC


        self.setGoal(-1.00, 0.00, math.radians(   0))#O
        self.setGoal(-1.00, 0.00, math.radians(  55))#
        self.setGoal(-1.00, 0.00, math.radians( -55))#
        self.setGoal(-1.00, 0.00, math.radians(   0))#
        self.setGoal(-0.70, 0.00, math.radians(   0))#A
#test        self.setGoal(-0.70, 0.00, math.radians(   0))#A
        #self.setGoal(-0.70, 0.00, math.radians( -30))#A
        self.setGoal(-0.70, 0.00, math.radians(  30))#A

        while(1):
            self.setGoal(-0.3, 0.3, math.radians(  60))#AB

            self.setGoal( X_pointBF, Y_pointBC, math.radians(  60))#B
            self.setGoal( X_pointBF, Y_pointBC, math.radians(   0))#B

            self.setGoal( X_pointCE+0.05, Y_pointBC, math.radians(   0))#C

            self.setGoal( 0.3, 0.3, math.radians( -60))#CD

            #self.setGoal( 0.40, 0.00, math.radians( -90))#D

            if( num_circle >= 2 ):
                break

            self.setGoal( 0.3, -0.3, math.radians( -120))#DE

            self.setGoal( X_pointCE, Y_pointEF, math.radians(-120))#E
            self.setGoal( X_pointCE, Y_pointEF, math.radians( 180))#E

            self.setGoal( X_pointBF, Y_pointEF, math.radians( 180))#F
            self.setGoal( X_pointBF-0.05, Y_pointEF, math.radians( 120))#F

            self.setGoal( X_pointBF, -0.3, math.radians( 120))#EF

            self.setGoal(-0.40, 0.00, math.radians( 120))#A
            self.setGoal(-0.40, 0.00, math.radians(  60))#

            num_circle = num_circle +1


        self.setGoal( 1.00, 0.00, math.radians(   0))#敵
        #self.setGoal( 1.00, 0.00, math.radians( 180))#敵
        self.setGoal( 1.00, 0.00, math.radians( 135))#
        self.setGoal( 1.00, 0.00, math.radians(-135))#
        self.setGoal( 1.00, 0.00, math.radians( 180))#敵

        self.setGoal( 1.4, 0.00, math.radians( 180))#敵



        while(0):
            self.setGoal( 1.40, 0.00, math.radians( 135))#敵
            self.setGoal( 1.40, 0.00, math.radians(-135))#敵

   
if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()
