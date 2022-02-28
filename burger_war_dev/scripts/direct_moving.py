#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is search object node.
subscribe "??" topics.
publish '??' topics. 

by Moro.
'''

import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32, Int8
from enum import IntEnum
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

#BasicRun
from geometry_msgs.msg import Twist
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import math as m
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import os


#周囲の状況
class Condition:
    
    def __init__(self):
        self.info_enemy_sub = rospy.Subscriber('Info_enemy', Float32MultiArray, self.callback_enemy)
        self.info_obstacle_sub = rospy.Subscriber('Info_obstacle', Float32MultiArray, self.callback_obstacle)
        #self.enemyCondition = EnemyCondition()
        self.attitude_pub = rospy.Publisher('attitude_enemy', String, queue_size=1)
        self.info_state_sub = rospy.Subscriber('Info_state', Int8, self.infostateCallback)  #String→Int8

    def update(self):
        return         
        
    def callback_enemy(self,Info_enemy):
        #rospy.loginfo("ememy dis=%f",Info_enemy.data[InfoEnemyIdx.INDEX_DIS])
        #rospy.loginfo("ememy dir=%f",Info_enemy.data[InfoEnemyIdx.INDEX_DIR])
        self.Info_enemy = Info_enemy
        self.update()

    def callback_obstacle(self,Info_obstacle):
        #rospy.loginfo("obstacle dis=%f",Info_obstacle.data[InfoEnemyIdx.INDEX_DIS])
        #rospy.loginfo("obstacle dir=%f",Info_obstacle.data[InfoEnemyIdx.INDEX_DIR])
        self.Info_obstacle = Info_obstacle
        self.update()

    def infostateCallback(self,Info_state):
        self.Info_state = Info_state
        rospy.loginfo("Info_state = {}".format(self.Info_state))
        self.update()

    #def getEnemyAttitude(self):
        #return self.enemyCondition.getAttitude()
    def getInfoState(self):
        return self.Info_state

    def getInfoEnemy(self):
        return self.Info_enemy

class DirectMoving(): #class Decision:置き換え
    def __init__(self, node_name="DirectMoving"):
        # node name 
        self.name = node_name
 
        self.condition = Condition()
        rospy.sleep(0.1)

        self.wayPoint = WayPoint(os.environ['HOME'] + \
                '/catkin_ws/src/burger_war_dev/burger_war_dev/scripts/course1.txt')
        rospy.sleep(0.1)

        self.algo = BasicRun(self.wayPoint)
        

    def run(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            result = self.algo.execute(self.condition) 
            state = self.condition.getInfoState()

            if not result:
                if state == Int8(data=2):
                    #逃げる?
                    pass
                elif state == Int8(data=3):#3
                    self.algo = ChaseRun()#追いかける?
                    pass
                else:
                    #わからない。とりあえずBasicRun
                    self.algo = BasicRun(self.wayPoint)
                    pass

            rate.sleep()
'''
    def DecideAction(self):
        twist = Twist()
        return twist

    def StartDirectMoving(self):
        rate = rospy.Rate(1) # change speed 1fps

        while not rospy.is_shutdown():
            twist = self.DecideAction()
            self.vel_pub.publish(twist)
            rate.sleep()

#decision_node.pyより
class Decision:
    def __init__(self):
        rospy.loginfo("Decision node")
        self.condition = Condition()
        self.algo = BasicRun()

    def run(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            self.algo.execute(self.condition)
            rate.sleep()
'''

class TurnDirection(IntEnum):
    RIGHT_TURN = 1
    LEFT_TURN = -1

class IndexSearch(IntEnum):
    INDEX_DIS = 0
    INDEX_DIR = 1

class WayPoint():
    def __init__(self,waypoint_file):
        self.idx = 0

        with open(waypoint_file,"r") as fp:
            lines = fp.read().splitlines()
            self.waypoints = [tuple(map(float,line.split(','))) for line in lines]

    def getNext(self,turn_direction):
        max_idx = len(self.waypoints)-1

        self.idx += turn_direction
        if self.idx > max_idx:
            self.idx = max_idx #0だとループ
        elif self.idx < 0:
            self.idx = max_idx
        return self.getCurrent()     

    def getCurrent(self):
        return self.waypoints[self.idx]

    #現在地から一番近いwaypointを探し、そのpointを返す
    def getNearest(self, x0, y0):
        cost = []
        for n,w in enumerate(self.waypoints):
            dist = m.sqrt((w[0] - x0)**2 + (w[1] - y0)**2)
            cost.append((dist, w, n))
        dist,point,idx = min(cost, key = lambda x: x[0])
        self.idx = idx
        return point

class ChaseRun():
    def __init__(self):
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        rospy.loginfo("ChaseRun")

    def execute(self, condition):
        #cmd_vel送る
        infoenemy = condition.getInfoEnemy()
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = infoenemy.data[IndexSearch.INDEX_DIR] / (-200)
        self.vel_pub.publish(twist)
        #conditionの中身みる　判定分岐
        #終わりだったらFalse
        if condition.getInfoState() == Int8(data=3):
            return True
        return False

class BasicRun():
    def __init__(self,wayPoint):
        '''self.wayPoint = WayPoint(os.environ['HOME'] + \
                '/catkin_ws/src/burger_war_dev/burger_war_dev/scripts/course1.txt') '''
        
        self.waypoint = wayPoint

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.amcl_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amclposeCallback)

        rospy.loginfo("BasicRun")

        #x,y,th = self.wayPoint.getCurrent()
        x,y,th = wayPoint.getCurrent()
        rospy.loginfo("waypoint : {} {} {}".format(x,y,th))
        result = self.setGoal(x,y,th)
    
    def amclposeCallback(self, data):
        self.my_pose_x = data.pose.pose.position.x
        self.my_pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        self.my_direction_th = rpy[2]
        rospy.loginfo("amclposeCallback update !! x,y,th={},{},{}".format(self.my_pose_x, self.my_pose_y, self.my_direction_th))

    def runActionlib(self):
        self.status = self.move_base_client.get_state()
        rospy.loginfo("actionlib status {}".format(self.status))

        if self.status == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("active")

        elif self.status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("load next goal")
            x,y,th = self.waypoint.getNext(1) #x,y,th = self.wayPoint.getNext(1)
            rospy.loginfo("waypoint : {} {} {}".format(x,y,th))
            result = self.setGoal(x,y,th)
        
        elif self.status == actionlib.GoalStatus.PENDING or\
             self.status == actionlib.GoalStatus.PREEMPTING or\
             self.status == actionlib.GoalStatus.PREEMPTED:
            x,y,th = self.waypoint.getCurrent() #x,y,th = self.wayPoint.getCurrent()
            self.setGoal(x,y,th)

        elif self.status == actionlib.GoalStatus.ABORTED:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.1
            self.direct_twist_pub.publish(cmd_vel)

    ## actionlib.GoalStatus.???
    ## https://docs.ros.org/en/noetic/api/actionlib_msgs/html/msg/GoalStatus.html
    def execute(self, condition):
        r = rospy.Rate(5) # change speed 5fps
        state = condition.getInfoState()

        if state == Int8(data=2) or\
           state == Int8(data=3): #2,3 
            rospy.loginfo("Enemy found.") #attitude = {}".format(attitude)
            self.cancel_goal()
            return False
        else: #state == Int8(data=1):
            rospy.loginfo("No enemy.")
            self.runActionlib()
            return True

    def setGoal(self,x,y,yaw):
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

        self.move_base_client.send_goal(goal)
        rospy.loginfo("sendGoal:{},{},{}".format(x,y,yaw))
    
    def cancel_goal(self):
        self.move_base_client.cancel_all_goals()
        return



'''
if __name__ == '__main__':
    rospy.init_node('DecisionNode')
    node = Decision()
    node.run()
'''

### unit test
#if __name__ == '__main__':
#    rospy.init_node('test')
#    way = WayPoint('course1.txt')
#    rospy.loginfo(way.getCurrent())
#
#    for i in range(4):
#        rospy.loginfo(way.getNext(TurnDirection.RIGHT_TURN))
#    for i in range(4):
#        rospy.loginfo(way.getNext(TurnDirection.LEFT_TURN))
#    
#    rospy.loginfo(way.getNearest(-0.51, 0.0))
#    rospy.loginfo(way.getNext(TurnDirection.LEFT_TURN))
#    rospy.loginfo(way.getNext(TurnDirection.LEFT_TURN))


if __name__ == '__main__':
    rospy.init_node('DirectMoving')
    node = DirectMoving()
    node.run()
