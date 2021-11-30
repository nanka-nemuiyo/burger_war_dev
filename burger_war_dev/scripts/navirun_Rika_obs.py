#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import rosparam
from math import radians, degrees, atan2

import tf

from geometry_msgs.msg import Twist

import tf
import tf

import numpy as np
import math
import requests

pi = math.pi
PI = math.pi

import actionlib
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry   

from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float32

from Enemy_detector_obs import EnemyDetector

import json
import requests

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


TARGET_TH = (
    (-PI/4, -PI/4, -PI/2, -PI/2, -PI*3/4, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/3, -PI/2, -PI*3/5, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/6,     0,   -PI/2, -PI*3/4,     -PI,  PI*3/4),
    (-PI/4, -PI/5,     0,     0,      PI,  PI*6/10,  PI*3/4,    PI/2),
    (    0,     0,  PI/2,  PI/2,      PI,  PI*3/4,  PI*3/4,    PI/2),
    (    0,  PI/4,  PI/3,  PI/2,  PI*5/6,  PI*3/4,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/3,  PI*5/6,    PI/2,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/4,    PI/3,    PI/2,  PI*3/4,  PI*3/4),
)
WIDTH = 1.2 * (2 **0.5) # [m]

def get_goals(my_color):
    if my_color == 'b':
        symbol = -1
        th     = pi
    else:
        symbol = 1
        th     = 0

    # rotation = 'CW'  # 回転方向を変える @en

    # if rotation == 'CW':
    #     symbol_2 = -1
    # else:
    #     symbol_2 = 1
    goal_xyyaw = np.array([ 
            [symbol * -1  , symbol * 0     , np.mod(pi/4 + th ,2*pi) ],
            [symbol * -1  , symbol * 0     , np.mod(0 + th ,2*pi) ],
            [symbol * -1  , symbol * 0  , np.mod(pi*7/4 + th ,2*pi) ], 
            [symbol * -0.9  , symbol * -0.5     , np.mod(pi*5/3 + th ,2*pi) ],
            [symbol * 0  , symbol *  -1  , np.mod(pi*3/4 + th ,2*pi) ],
            [symbol * 0  , symbol *  -1  , np.mod(pi/2 + th ,2*pi) ],
            [symbol * -0  , symbol *  -1  , np.mod(pi/4 + th ,2*pi) ],
            [symbol * 0.5     , symbol * -0.9   , np.mod(pi/6 + th ,2*pi) ], 
            [symbol * 1     , symbol * 0   , np.mod(pi*5/4  + th ,2*pi) ],
            [symbol * 1     , symbol * 0   , np.mod(pi  + th ,2*pi) ],
            [symbol * 1   , symbol * 0   , np.mod( pi*3/4     + th ,2*pi) ],
            [symbol * 0.9   , symbol * 0.5   ,np.mod(pi*2/3   + th ,2*pi) ],
            [symbol * 0   , symbol *   1   , np.mod(pi*7/4     + th ,2*pi) ],
            [symbol * 0   , symbol *   1   , np.mod(pi*3/2     + th ,2*pi) ], 
            [symbol * 0   , symbol *   1   , np.mod(pi*5/4  + th ,2*pi) ],
            [symbol * -0.5   , symbol * 0.9  , np.mod( pi*7/6    + th ,2*pi) ],
        ])        
    return goal_xyyaw 

def num2mvstate(i):
    return ["PENDING", "ACTIVE", "RECALLED", "REJECTED", "PREEMPTED", "ABORTED", "SUCCEEDED", "LOST"][i]
class NaviBot():
    def __init__(self):

        self.robot_namespace = rospy.get_param('~robot_namespace')
        print(self.robot_namespace)
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #self.tfBuffer = tf.Buffer()

        self.my_side = "r"

        self.enemy_detector = EnemyDetector()
        self.listener = tf.TransformListener()
        # robot state 'inner' or 'outer'
        self.state = 'inner' 
        # robot wheel rot 
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        self.k = 0.5
        self.near_wall_range = 0.2  # [m]
        self.speed = 0.07

        self.goals = get_goals('r')
        self.goalcounter = 0
        self.goalcounter_prev = -1

        self.pose_twist = Twist()
        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.


        self.is_near_wall = False

        self.is_second_lap = False

        self.is_near_enemy = False
        #self.enemy_direction = None
        self.enemy_dist = 0
        self.enemy_rad = 0
        self.near_enemy_twist = Twist()
        self.near_enemy_twist.linear.x = self.speed 
        self.near_enemy_twist.linear.y = 0.
        self.near_enemy_twist.linear.z = 0.
        self.near_enemy_twist.angular.x = 0. 
        self.near_enemy_twist.angular.y = 0.
        self.near_enemy_twist.angular.z = 0.
        self.is_initialized_pose = True
        self.enemy_x_pos = 0
        self.enemy_y_pos = 0
        self.my_x_pos = 0
        self.my_y_pos = 0
        
        # lidar scan
        self.scan = []
        self.odom = Odometry()
        self.odom_prev = Odometry()
        self.odom_diff = Odometry()
        

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size= 1)
        #self.vel_pub = rospy.Publisher('enemy_info', EnemyInfo, queue_size=1)
        # subscriber
        self.is_enemy_detecting = False
        rospy.Subscriber('enemy_detected', Bool, self.save_detected )
        rospy.Subscriber('enemy_dist', Float64, self.save_detected2 )
        rospy.Subscriber('enemy_rad', Float64, self.save_detected3 )
        rospy.Subscriber('odom', Odometry, self.save_detected4 )
        rospy.Subscriber('enemy_x_pos', Float32, self.save_detected5)
        rospy.Subscriber('enemy_y_pos', Float32, self.save_detected6)
        rospy.Subscriber('my_x_pos', Float32, self.save_detected7)
        rospy.Subscriber('my_y_pos', Float32, self.save_detected8)
    def getWarState(self):
        resp = requests.get(JUDGE_URL + "/warState")
        self.dic = resp.json()

        if self.my_side == "r": # red_bot
            self.my_score = int(self.dic["scores"]["r"])
            self.enemy_score = int(self.dic["scores"]["b"])
        else: # blue_bot
            self.my_score = int(self.dic["scores"]["b"])
            self.enemy_score = int(self.dic["scores"]["r"])
        self.targets = np.array([
            self.dic["targets"][11]["player"],self.dic["targets"][17]["player"],
            self.dic["targets"][13]["player"],self.dic["targets"][12]["player"],
            self.dic["targets"][15]["player"],self.dic["targets"][9]["player"],
            self.dic["targets"][8]["player"],self.dic["targets"][14]["player"],
            self.dic["targets"][6]["player"],self.dic["targets"][7]["player"],
            self.dic["targets"][16]["player"],self.dic["targets"][10]["player"]
        ])

    def save_detected(self, data):
        self.is_enemy_detecting = data.data
        #print("data_updated")
    def save_detected2(self, data):
        self.enemy_dist = data.data
        #print("data_updated")
    def save_detected3(self, data):
        self.enemy_rad = data.data
        #print("data_updated")
    def save_detected4(self, data):
        self.odom = data
        self.odom.pose.pose.position.x += self.odom_diff.pose.pose.position.x
        self.odom.pose.pose.position.y += self.odom_diff.pose.pose.position.y
        #print("data_updated")
    def save_detected5(self, data):
        self.enemy_x_pos_str = str(data.data)
        self.enemy_x_pos = float(self.enemy_x_pos_str)
        #print('enemy_x_pos : ' + str(self.enemy_x_pos))
    def save_detected6(self, data):
        self.enemy_y_pos_str = str(data.data)
        self.enemy_y_pos = float(self.enemy_y_pos_str)
        #print('enemy_y_pos : ' + str(self.enemy_y_pos))
    def save_detected7(self, data):
        self.my_x_pos_str = str(data.data)
        self.my_x_pos = float(self.my_x_pos_str)
        #print('my_x_pos : ' + str(self.my_x_pos))
    def save_detected8(self, data):
        self.my_y_pos_str = str(data.data)
        self.my_y_pos = float(self.my_y_pos_str)
        #print('my_y_pos : ' + str(self.my_y_pos))

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

        #self.client.send_goal(goal)
        #wait = self.client.wait_for_result()
        #if not wait:
        #    rospy.logerr("Action server not available!")
        #    rospy.signal_shutdown("Action server not available!")
        #else:
        #    return self.client.get_result()        
        def active_cb():
            # rospy.loginfo("active_cb. Goal pose is now being processed by the Action Server...")
            return

        def feedback_cb( feedback):
            #To print current pose at each feedback:
            #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
            # rospy.loginfo("feedback_cb. Feedback for goal pose received{}".format(feedback))
            return

        def done_cb(status, result):
            if status is not GoalStatus.PREEMPTED:
                self.goalcounter += 1
                if self.goalcounter == len(self.goals):
                    self.is_second_lap = True
                self.goalcounter %= len(self.goals)
            #rospy.loginfo("done_cb. status:{} result:{}".format(num2mvstate(status), result))
        self.client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        return

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        map_name=self.robot_namespace+'/map'
        is_enemy_detected, x, y = self.getEnemyPos(map_name)
        print(is_enemy_detected)

        #goal_xyyaw = self.goals

        is_patrol_mode_prev = False
        is_patrol_mode = True
        rotate_ccw = True
        cnt = 0
        flg = True
        symbol = 1
        th = 0
        self.enemy_pos = np.array([0.0, 0.0])
        self.my_pos = np.array([0.0, 0.0])

        ccw_goal_xyyaw = np.array([ 
                [symbol * -0.8  , symbol * 0     , np.mod(0      + th ,2*pi) ], # (1） 
                [symbol * -0.8  , symbol * -0.2  ,-np.mod(-pi/4  + th ,2*pi) ], 
                [symbol * -0.8  , symbol * 0     , np.mod(pi/2   + th ,2*pi) ],
                [symbol * -0.8  , symbol *  0.2  , np.mod(pi/3   + th ,2*pi) ],
                [symbol * -0.5  , symbol *  0.2  , np.mod(0      + th ,2*pi) ],
                [symbol * 0     , symbol * 0.4   , np.mod(pi*4/5 + th ,2*pi) ], # (2)
                [symbol * 0     , symbol * 0.4   , np.mod(-pi/2  + th ,2*pi) ],
                [symbol * 0.2   , symbol * 0.4   , np.mod( 0     + th ,2*pi) ],
                [symbol * 0.2   , symbol * 0.4   ,-np.mod(pi/3   + th ,2*pi) ],
                [symbol * 0.5   , symbol *   0   , np.mod(pi     + th ,2*pi) ], #（3）
                [symbol * 0.5   , symbol *   0   , np.mod(-pi/2  + th ,2*pi) ],
                [symbol * 0.2   , symbol * -0.4  , np.mod( pi    + th ,2*pi) ],
                [symbol * 0.2   , symbol * -0.4  , np.mod(-pi/3  + th ,2*pi) ],
                [symbol * 0     , symbol * -0.4  , np.mod( 0     + th ,2*pi) ], # (4)
                [symbol * 0     , symbol * -0.4  , np.mod(-pi*4/5+ th ,2*pi) ],
                [symbol * 0     , symbol * -0.4  , np.mod( pi    + th ,2*pi) ],
                [symbol * -0.5  , symbol * 0     , np.mod( pi    + th ,2*pi) ], # (5) 
                [symbol * -1.40 , symbol * 0     , np.mod( 0     + th ,2*pi) ]
            ])
        # idx=0
        cw_goal_xyyaw = np.array([ 
                [symbol * -1  , symbol * 0     , np.mod(pi/4 + th ,2*pi) ],
                [symbol * -1  , symbol * 0     , np.mod(0 + th ,2*pi) ],
                [symbol * -1  , symbol * 0  , np.mod(pi*7/4 + th ,2*pi) ], 
                [symbol * -0.9  , symbol * -0.5     , np.mod(pi*5/3 + th ,2*pi) ],#[3]:右
                [symbol * 0  , symbol *  -1  , np.mod(pi*3/4 + th ,2*pi) ],
                [symbol * 0  , symbol *  -1  , np.mod(pi/2 + th ,2*pi) ],
                [symbol * -0  , symbol *  -1  , np.mod(pi/4 + th ,2*pi) ],
                [symbol * 0.5     , symbol * -0.9   , np.mod(pi/6 + th ,2*pi) ], #[7]:上
                [symbol * 1     , symbol * 0   , np.mod(pi*5/4  + th ,2*pi) ],
                [symbol * 1     , symbol * 0   , np.mod(pi  + th ,2*pi) ],
                [symbol * 1   , symbol * 0   , np.mod( pi*3/4     + th ,2*pi) ],
                [symbol * 0.9   , symbol * 0.5   ,np.mod(pi*2/3   + th ,2*pi) ],
                [symbol * 0   , symbol *   1   , np.mod(pi*7/4     + th ,2*pi) ],
                [symbol * 0   , symbol *   1   , np.mod(pi*3/2     + th ,2*pi) ], 
                [symbol * 0   , symbol *   1   , np.mod(pi*5/4  + th ,2*pi) ],
                [symbol * -0.5   , symbol * 0.9  , np.mod( pi*7/6    + th ,2*pi) ],
            ])

        while(True):
            r.sleep()
            self.getWarState()
            my_get_keys, enemy_get_keys = self.score_check()
            field_score_diff = len(my_get_keys) - len(enemy_get_keys)
            is_patrol_mode = True
            detect_inner_th = 1.0
            detect_outer_th = 1.1

            #print('enemy_pos : ' + str(self.enemy_x_pos),str(self.enemy_y_pos))
            #print('enemy_y_pos : ' + str(self.enemy_y_pos))
            #print('my_pos : ' + str(self.my_x_pos),str(self.my_y_pos))
            #print('my_y_pos : ' + str(self.my_y_pos))
            if self.is_enemy_detecting:
                # 古い座標値を記録
                self.old_enemy_pos = self.enemy_pos
                self.old_my_pos = self.my_pos 
                # 新しい座標値を記録
                self.enemy_pos = np.array([self.enemy_x_pos, self.enemy_y_pos])
                self.my_pos = np.array([self.my_x_pos, self.my_y_pos])
                # ベクトルの向きを計算
                vec_enemy = self.enemy_pos - self.old_enemy_pos
                vec_my = self.my_pos - self.old_my_pos
                #self.enemy_direction = np.arctan2(vec_enemy[0], vec_enemy[1])
                self.my_direction = np.arctan2(vec_my[0], vec_my[1])
                # ベクトルの向きが自分と相手でちょうどpi違うとき、相手はロボットではないとしてenemy判定解除
                # 内積を計算
                inner = np.inner(vec_enemy, vec_my)
                # ベクトルの長さを計算
                dis_enemy = np.linalg.norm(vec_enemy)
                dis_my = np.linalg.norm(vec_my)
                # 2つのベクトルの成す角を計算
                theta = np.arccos(inner/(dis_enemy*dis_my))
                #print('角度'+str(theta))
                # 2ベクトルの成す角がpi+-2.5%ならenemy判定解除
                if pi-(pi*0.05) < theta and theta < pi+(pi*0.05):
                    self.is_enemy_detecting = False
                    print('これは敵ではない？')
                elif theta-(theta*0.05) < self.my_direction and self.my_direction < theta+(theta*0.05):
                    self.is_enemy_detecting = False
                    print('これは敵ではない？')
                elif self.enemy_x_pos-(self.enemy_x_pos*0.015) < self.old_enemy_pos[0] and self.old_enemy_pos[0] < self.enemy_x_pos+(self.enemy_x_pos*0.015):
                    self.is_enemy_detecting = False
                    print('これは敵ではない？敵の座標が動いていない？')

            if not self.is_enemy_detecting:
                is_patrol_mode = True
            elif self.is_enemy_detecting and flg and detect_inner_th > self.enemy_dist:
                is_patrol_mode = False
            elif not is_patrol_mode and detect_outer_th < self.enemy_dist:
                is_patrol_mode = True

            
            if is_patrol_mode and (not is_patrol_mode_prev or (self.goalcounter is not self.goalcounter_prev)):
                print("次のゴールに行くぜ〜")
                # 新たに巡回モードに切り替わった瞬間及びゴール座標が変わった時
                # goalcounterのゴール座標をセット
                #self.setGoal(goal_xyyaw[self.goalcounter][0], goal_xyyaw[self.goalcounter][1], goal_xyyaw[self.goalcounter][2])
                # goal_val0 = goal_xyyaw[idx%len(goal_xyyaw)][0]
                # goal_val1 = goal_xyyaw[idx%len(goal_xyyaw)][1]
                # goal_val2 = goal_xyyaw[idx%len(goal_xyyaw)][2]
                self.setGoal(ccw_goal_xyyaw[self.goalcounter][0], ccw_goal_xyyaw[self.goalcounter][1], ccw_goal_xyyaw[self.goalcounter][2])
                #rospy.loginfo( num2mvstate(self.client.get_state()))
                # self.goalcounter_prev = idx%len(goal_xyyaw)
                self.goalcounter_prev = self.goalcounter  
                self.odom_pub.publish(self.odom )   
            elif is_patrol_mode:
                print("巡回するぜ〜")
                # 巡回モード最中。CBが来るまで何もしない。
                targetscounter = self.goalcounter // 4
                if (self.goalcounter %4) != 3 and self.targets[self.goalcounter - targetscounter] == "r":
                    self.goalcounter += 1
            else:
                #print("Yeah!! Enemy Det!!敵を見るぜ〜")
                #print('自分の向き'+str(self.my_direction))
                #print('角度'+str(theta))
                # 敵の方向を向くモード
                self.client.cancel_all_goals()
                # print("POSE TWIST: {}, {}".format(self.pose_twist.linear.x, self.pose_twist.angular.z))
                # print("ENEMY TWIST: {}, {}".format(self.near_enemy_twist.linear.x, self.near_enemy_twist.angular.z))
                # print("wall: {}, Enemy: {}, X: {}, Z: {}".format(self.is_near_wall, self.is_near_enemy, twist.linear.x, twist.angular.z))
                '''
                twist = Twist()
                #twist.linear.x = -0.1
                twist.angular.z = self.enemy_rad
                print(self.enemy_rad)
                self.vel_pub.publish(twist)
                '''
                if cnt < 1 :
                        self.odom_prev = self.odom
                self.client.cancel_all_goals()
                
                if cnt > 50 :
                    print("Yeah!! Enemy Det!!敵を見るぜ〜")
                    #flg = False
                    self.odom_diff.pose.pose.position.x = self.odom_prev.pose.pose.position.x - self.odom.pose.pose.position.x
                    self.odom_diff.pose.pose.position.y = self.odom_prev.pose.pose.position.y - self.odom.pose.pose.position.y
                    
                    twist = Twist()
                    #twist.linear.x = 0.1
                    twist.angular.z = self.enemy_rad
                    self.vel_pub.publish(twist)
                    continue

                # import pdb; pdb.set_trace()
                
                if self.enemy_dist < 0.5 :
                    print("Yeah!! Enemy Det!!敵に攻撃だ！")
                    cnt += 1
                    self.odom_pub.publish(self.odom_prev)
                # print("POSE TWIST: {}, {}".format(self.pose_twist.linear.x, self.pose_twist.angular.z))
                # print("ENEMY TWIST: {}, {}".format(self.near_enemy_twist.linear.x, self.near_enemy_twist.angular.z))
                # print("wall: {}, Enemy: {}, X: {}, Z: {}".format(self.is_near_wall, self.is_near_enemy, twist.linear.x, twist.angular.z))
                    twist = Twist()
                    twist.linear.x = 0.1
                    twist.angular.z = self.enemy_rad
                    self.vel_pub.publish(twist)

                
                # pass
                
            is_patrol_mode_prev = is_patrol_mode

    def getEnemyPos(self, frame):
        try:
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.listener.lookupTransform(frame, enemy_closest_name, rospy.Time())
            #trans = trans_stamped.transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            return False, 0, 0
        return True, trans_stamped[0], trans_stamped[1]

    def getEnemyDistRad(self):
        try:
            # <class 'geometry_msgs.msg._TransformStamped.TransformStamped'>
            base_footprint_name=self.robot_namespace+'/base_footprint'
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.listener.lookupTransform(base_footprint_name, enemy_closest_name, rospy.Time())
            #trans = trans_stamped.transform
            # trans = self.tfBuffer.lookup_transform('enemy_closest', "base_footprint", rospy.Time(), rospy.Duration(4))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # rate.sleep()
            # rospy.logwarn(e)
            return False, 0, 0
        # rospy.loginfo(trans)
        #print(trans_stamped)
        
        dist = (trans_stamped[0][0]**2 + trans_stamped[0][1]**2)**0.5
        rad = atan2(trans_stamped[0][1], trans_stamped[0][0])
        # print ("trans.translation.x:{}, trans.translation.y:{}".format(trans.translation.x, trans.translation.y))

        # rot = trans.rotation
        # rad = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]

        return True, dist, rad

    def score_check(self):
        # 点数計算
        resp = requests.get("http://127.0.0.1:5000/warState")
        dic = resp.json()
        
        my_color = "r"
        if my_color == "r": # red_bot
            self.my_score = int(dic["scores"]["r"])
            self.enemy_score = int(dic["scores"]["b"])
            # フィールド上の点数を誰が取っているか取得
            field_key = ["top_left_N", "top_left_S", "top_right_N", "top_right_S", "bottom_left_N", "bottom_left_S", "bottom_right_N", "bottom_right_S", "center_N", "center_E", "center_W", "center_S"]
            field_value = []
            for num in range(6,18):
                field_value.append(dic["targets"][num]["player"])
            field_dic = dict(zip(field_key, field_value))
            my_get_keys = [k for k, v in field_dic.items() if v == 'r']
            enemy_get_keys = [k for k, v in field_dic.items() if v == 'b']
            #print("フィールド上の点数を表示する！")
            #print("自分の取得点数：" + str(len(my_get_keys)) + ", 自分の取得箇所：" + str(my_get_keys))
            #print("敵の取得点数：" + str(len(enemy_get_keys)) + ", 敵の取得箇所：" + str(enemy_get_keys))
        
        return my_get_keys, enemy_get_keys
    
if __name__ == '__main__':
    rospy.init_node('navirun')
    JUDGE_URL = rospy.get_param('~judge_url', 'http://127.0.0.1:5000')
    
    bot = NaviBot()
    bot.strategy()
