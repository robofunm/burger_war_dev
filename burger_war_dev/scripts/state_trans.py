#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is state transition node.
subscribe "??" topics.
publish '??' topics. 

by Moro.
'''

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from enum import IntEnum
basicrun_time = 50.0 #最初の50秒はBasicrunのみ

class IndexSearch(IntEnum):
    INDEX_DIS = 0
    INDEX_DIR = 1

class StateTrans():
    def __init__(self, node_name="StateTrans"):
        # node name 
        self.name = node_name
        # state publisher
        #self.msg_state = rospy.Publisher('Info_state', String,queue_size=1)
        self.msg_state = rospy.Publisher('Info_state', Int8,queue_size=1)
        self.start_time = rospy.get_time() 
        # object Subscriber
        sub = rospy.Subscriber('Info_enemy',Float32MultiArray,self.callback_enemy)
        sub = rospy.Subscriber('Info_obstacle',Float32MultiArray,self.callback_obstacle)

    def callback_enemy(self,Info_enemy):
        elapsed_time =  rospy.get_time() - self.start_time
        rospy.loginfo("ememy dis=%f",Info_enemy.data[IndexSearch.INDEX_DIS])
        rospy.loginfo("ememy dir=%f",Info_enemy.data[IndexSearch.INDEX_DIR])
        rospy.loginfo("経過時間=%f",elapsed_time)
        exist_data = Info_enemy.data[IndexSearch.INDEX_DIS]
        rospy.logwarn(exist_data)

        if exist_data == -1.0 or elapsed_time < basicrun_time:
            pub_info_1 = Int8(data=1)
            self.msg_state.publish(pub_info_1)
            rospy.logwarn('unknown')
        else:
            pub_info_3 = Int8(data=3)
            self.msg_state.publish(pub_info_3) 
            rospy.logwarn('Help me')

    def callback_obstacle(self,Info_obstacle):
        rospy.loginfo("obstacle dis=%f",Info_obstacle.data[IndexSearch.INDEX_DIS])
        rospy.loginfo("obstacle dir=%f",Info_obstacle.data[IndexSearch.INDEX_DIR])

    def StartStateTrans(self):
        rate = rospy.Rate(20) # change speed 5fps

        while not rospy.is_shutdown():
            #hello_str = String()
            #hello_str.data = "from StateTrans"
            #self.msg_state.publish(hello_str)
        #if et_data == -1.0:
         #   pub_info_1 = Int8(data=1)
          #  self.msg_state.publish(pub_info_1)
           # rospy.logwarn('unknown')
        #else:
         #   pub_info_3 = Int8(data=3)
          #  self.msg_state.publish(pub_info_3) 
           # rospy.logwarn('Help me')
     
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('StateTrans')
    node = StateTrans()
    node.StartStateTrans()

