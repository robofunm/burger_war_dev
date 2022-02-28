#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is search object node.
subscribe "??" topics.
publish '??' topics. 

by Moro.
'''
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from enum import IntEnum

class IndexColor(IntEnum):
    INDEX_CX = 0
    INDEX_CY = 1
    INDEX_AR = 2

class IndexSearch(IntEnum):
    INDEX_DIS = 0
    INDEX_DIR = 1

class SearchObject:
    def __init__(self, node_name="SearchObject"):
        rospy.loginfo("test")
        # node name 
        self.name = node_name
        # object publisher
        self.pub_enemy = rospy.Publisher('Info_enemy', Float32MultiArray,queue_size=1)
        self.pub_obstacle = rospy.Publisher('Info_obstacle', Float32MultiArray, queue_size=1)
         # state Subscriber
        self.sub_y = rospy.Subscriber('yellow_center',Int32MultiArray,self.callback_img_yellow)
        self.sub_r = rospy.Subscriber('red_center',Int32MultiArray,self.callback_img_red)
        self.sub_g = rospy.Subscriber('green_center',Int32MultiArray,self.callback_img_green)

        self.Info_enemy = [-1.0, 0.0]
        self.Info_obstacle = [-1.0, 0.0]

        self.ylw_cx = 0.0
        self.ylw_cy = 0.0
        self.red_cx = 0.0
        self.red_cy = 0.0
        self.grn_cx = 0.0
        self.grn_cy = 0.0
        self.grn_ar = 0.0

    def callback_img_yellow(self,Info_color):
        self.ylw_cx = Info_color.data[IndexColor.INDEX_CX]
        self.ylw_cy = Info_color.data[IndexColor.INDEX_CY]


    def callback_img_red(self,Info_color):
        self.red_cx = Info_color.data[IndexColor.INDEX_CX]
        self.red_cy = Info_color.data[IndexColor.INDEX_CY]


    def callback_img_green(self,Info_color):
        self.grn_cx = Info_color.data[IndexColor.INDEX_CX]
        self.grn_cy = Info_color.data[IndexColor.INDEX_CY]
        self.grn_ar = Info_color.data[IndexColor.INDEX_AR]

    def MainSearchObject(self):
        self.Info_enemy = [-1.0, 0.0]
        self.Info_obstacle = [-1.0, 0.0]
        #敵の距離を計算
        if self.red_cy > 0.0:
            self.Info_enemy[IndexSearch.INDEX_DIS] = (100.0-40.0)/160.0*self.red_cy+40.0#MAXを100に正規化
        elif self.grn_ar > 50000.0:
            self.Info_enemy[IndexSearch.INDEX_DIS] = 100*(4*(10**6)-self.grn_ar)/(4*(10**6)-50000)#MAXを100に正規化
            if self.grn_ar > 4*(10**6):
                self.Info_enemy[IndexSearch.INDEX_DIS] = 10*(4*(10**7)-self.grn_ar)/(4*(10**7)-4*(10**6))#MAXを100に正規化
                if self.grn_ar > 4*(10**7):
                    self.Info_enemy[IndexSearch.INDEX_DIS] = 0.0


        if self.Info_enemy[IndexSearch.INDEX_DIS] > 100.0:
            self.Info_enemy[IndexSearch.INDEX_DIS] = 100.0
        elif self.Info_enemy[IndexSearch.INDEX_DIS] < 0.0:
            self.Info_enemy[IndexSearch.INDEX_DIS] = -1.0

        #敵の位置を計算
        if self.red_cx > 0.0:
            self.Info_enemy[IndexSearch.INDEX_DIR] = (self.red_cx-320)*100/320#MAXを100に正規化
        elif self.grn_cx > 0.0:
            self.Info_enemy[IndexSearch.INDEX_DIR] = (self.grn_cx-320)*100/320#MAXを100に正規化


        #障害物の距離を計算
        if self.ylw_cy > 330.0:
            self.Info_obstacle[IndexSearch.INDEX_DIS] = (480.0-self.ylw_cy)/1.5#MAXを100に正規化
            if self.Info_obstacle[IndexSearch.INDEX_DIS] > 100.0:
                self.Info_obstacle[IndexSearch.INDEX_DIS] = 100.0

        #障害物の位置を計算
        if self.ylw_cx > 0.0:
            self.Info_obstacle[IndexSearch.INDEX_DIR] = (self.ylw_cx-320)*100/320#MAXを100に正規化

        pub_info_enemy = Float32MultiArray(data=self.Info_enemy)
        pub_info_obstacle = Float32MultiArray(data=self.Info_obstacle)
        self.pub_enemy.publish(pub_info_enemy)
        self.pub_obstacle.publish(pub_info_obstacle)


    def StartSearchObject(self):
        rate = rospy.Rate(20) # change speed 1fps
        while not rospy.is_shutdown():
            node_Search.MainSearchObject()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('SearchObject')
    node_Search = SearchObject()
    node_Search.StartSearchObject()
