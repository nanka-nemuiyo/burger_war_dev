#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
import numpy as np
pi = math.pi
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from math import radians, degrees, atan2

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
from actionlib import SimpleActionClient, GoalStatus

from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
import requests

PI = math.pi
# 8x8  [rad]
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

target_idx_r = np.array([
    [ 13, 17, 11],
    [ 10, 16,  7],
    [  6, 14,  8],
    [  9, 15, 12]
])
target_idx_b = np.array([
    [  6, 14,  8],
    [  9, 15, 12],
    [ 13, 17, 11],
    [ 10, 16,  7]
])



# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

# respect is_point_enemy freom team rabbit
# https://github.com/TeamRabbit/burger_war
class EnemyDetector:
    '''
    Lidarのセンサ値から簡易的に敵を探す。
    obstacle detector などを使ったほうがROSらしいがそれは参加者に任せます。
    いろいろ見た感じ Team Rabit の実装は綺麗でした。
    方針
    実測のLidarセンサ値 と マップと自己位置から期待されるLidarセンサ値 を比較
    ズレている部分が敵と判断する。
    この判断は自己位置が更新された次のライダーのコールバックで行う。（フラグで管理）
    0.7m 以上遠いところは無視する。少々のズレは許容する。
    '''
    def __init__(self):
        self.max_distance = 0.7
        self.thresh_corner = 0.25
        self.thresh_center = 0.35

        self.pose_x = 0
        self.pose_y = 0
        self.th = 0    
    
    def findEnemy(self, scan, pose_x, pose_y, th):
        '''
        input scan. list of lider range, robot locate(pose_x, pose_y, th)
        return is_near_enemy(BOOL), enemy_direction[rad](float)
        '''
        if not len(scan) == 360:
            return False
        
        # update pose
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.th = th

        # drop too big and small value ex) 0.0 , 2.0 
        near_scan = [x if self.max_distance > x > 0.1 else 0.0 for x in scan]

        enemy_scan = [1 if self.is_point_emnemy(x,i) else 0 for i,x in  enumerate(near_scan)]

        is_near_enemy = sum(enemy_scan) > 5  # if less than 5 points, maybe noise
        if is_near_enemy:
            idx_l = [i for i, x in enumerate(enemy_scan) if x == 1]
            idx = idx_l[len(idx_l)/2]
            enemy_direction = idx / 360.0 * 2*PI
            enemy_dist = near_scan[idx]
        else:
            enemy_direction = None
            enemy_dist = None

        # print("Enemy: {}, Direction: {}".format(is_near_enemy, enemy_direction))
        # print("enemy points {}".format(sum(enemy_scan)))
        return is_near_enemy, enemy_direction, enemy_dist
        

    def is_point_emnemy(self, dist, ang_deg):
        if dist == 0:
            return False

        ang_rad = ang_deg /360. * 2 * PI
        point_x = self.pose_x + dist * math.cos(self.th + ang_rad)
        point_y = self.pose_y + dist * math.sin(self.th + ang_rad)

        #フィールド内かチェック
        if   point_y > (-point_x + 1.53):
            return False
        elif point_y < (-point_x - 1.53):
            return False
        elif point_y > ( point_x + 1.53):
            return False
        elif point_y < ( point_x - 1.53):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        len_p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        len_p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        len_p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
        len_p5 = math.sqrt(pow( point_x        , 2) + pow( point_y        , 2))

        if len_p1 < self.thresh_corner or len_p2 < self.thresh_corner or len_p3 < self.thresh_corner or len_p4 < self.thresh_corner or len_p5 < self.thresh_center:
            return False
        else:
            #print(point_x, point_y, self.pose_x, self.pose_y, self.th, dist, ang_deg, ang_rad)
            #print(len_p1, len_p2, len_p3, len_p4, len_p5)
            return True

def get_goals(my_color, rotation):
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

    #print("get_goals", my_color, symbol, th, symbol_2)
         
    # 12x3 (x,y,yaw) 
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
        # velocity publisher 
        self.vel_pub  = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client   = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # self.my_color = rospy.get_param('~rside')
        self.my_side = "r"
        
        self.goalcounter = 0
        self.goalcounter_prev = -1
        self.goals = get_goals('r', 'CCW')
        # robot wheel rot 
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        self.k = 0.5
        self.near_wall_range = 0.2  # [m]

        # speed [m/s]
        self.speed = 0.1
        # self.speed = 0.5

        # lidar scan
        self.scan = []

        # subscriber
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        self.pose_twist = Twist()
        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.

        self.is_second_lap = False

        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None
        self.near_enemy_twist = Twist()
        self.near_enemy_twist.linear.x = self.speed 
        self.near_enemy_twist.linear.y = 0.
        self.near_enemy_twist.linear.z = 0.
        self.near_enemy_twist.angular.x = 0. 
        self.near_enemy_twist.angular.y = 0.
        self.near_enemy_twist.angular.z = 0.

        self.is_initialized_pose = False
        self.enemy_detector = EnemyDetector()
        self.symbol_2 = 1

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


        # for zone in range(4):
        #     for target in range(3):
        #         if self.my_side == "r":
        #             target_idx = target_idx_r[zone][target]
        #         else:
        #             target_idx = target_idx_b[zone][target]

        #         s = dic["targets"][target_idx]["player"]
        #         self.score_prev[zone][target] = self.score[zone][target]
        #         if s == "n":
        #             self.score[zone][target] = 1
        #         elif s == self.my_side:
        #             self.score[zone][target] = 0
        #         else:
        #             self.score[zone][target] = 2
        #             if self.score_prev[zone][target] < 2:	# When the enemy get a marker,
        #                 self.enemy_zone = zone
        #                 self.enemy_stamp = rospy.Time.now()
        #                 print("#"),
        #         print(self.score[zone][target])
        #     print("|")
    
    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

        self.th = rpy[2]
        th_xy = self.calcTargetTheta(self.pose_x,self.pose_y)
        
        self.updatePoseTwist(self.th, th_xy)
        self.is_initialized_pose = True

    def updatePoseTwist(self, th, th_xy):
        # update pose twist
        th_diff = th_xy - th
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI
        
        delta_th = self.calcDeltaTheta(th_diff)
        new_twist_ang_z = max(-0.8, min((th_diff + delta_th) * self.k , 0.8))
        
        self.pose_twist.angular.z = new_twist_ang_z
        self.pose_twist.linear.x = self.speed
        #print("th: {}, th_xy: {}, delta_th: {}, new_twist_ang_z: {}".format(th, th_xy, delta_th, new_twist_ang_z))

    def calcTargetTheta(self, pose_x, pose_y):
        x = self.poseToindex(pose_x)
        y = self.poseToindex(pose_y)
        th = TARGET_TH[x][y]
        #print("POSE pose_x: {}, pose_y: {}. INDEX x:{}, y:{}".format(pose_x, pose_y, x, y))
        return th

    def calcDeltaTheta(self, th_diff):
        if not self.scan:
            return 0.
        R0_idx = self.radToidx(th_diff - PI/8)
        R1_idx = self.radToidx(th_diff - PI/4)
        L0_idx = self.radToidx(th_diff + PI/8)
        L1_idx = self.radToidx(th_diff + PI/4)
        R0_range = 99. if self.scan[R0_idx] < 0.1 else self.scan[R0_idx]
        R1_range = 99. if self.scan[R1_idx] < 0.1 else self.scan[R1_idx]
        L0_range = 99. if self.scan[L0_idx] < 0.1 else self.scan[L0_idx]
        L1_range = 99. if self.scan[L1_idx] < 0.1 else self.scan[L1_idx]

        #print("Ranges R0: {}, R1: {}, L0: {}, L1: {}".format(R0_range, R1_range, L0_range, L1_range))
        if R0_range < 0.3 and L0_range > 0.3:
            return PI/4
        elif R0_range > 0.3 and L0_range < 0.3:
            return -PI/4
        elif R1_range < 0.2 and L1_range > 0.2:
            return PI/8
        elif R1_range > 0.2 and L1_range < 0.2:
            return -PI/8
        else:
            return 0.

    def radToidx(self, rad):
        deg = int(rad / (2*PI) * 360)
        while not 360 > deg >= 0:
            if deg > 0:
                deg -= 360
            elif deg < 0:
                deg += 360
        return deg

    def poseToindex(self, pose):
        i = 7 - int((pose + WIDTH) / (2 * WIDTH) * 8)
        i = max(0, min(7, i))
        return i

    def lidarCallback(self, data):
        '''
        lidar scan use for bumper , and find enemy
        controll speed.x
        '''
        scan = data.ranges
        self.scan = scan
        self.is_near_wall = self.isNearWall(scan)
        
        # enemy detection
        if self.is_initialized_pose:
            self.is_near_enemy, self.enemy_direction, self.enemy_dist = self.enemy_detector.findEnemy(scan, self.pose_x, self.pose_y, self.th)
        
        if self.is_near_enemy:
            self.updateNearEnemyTwist()

    def updateNearEnemyTwist(self):
        # update pose twist
        th_diff = self.enemy_direction
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI
        new_twist_ang_z = max(-0.5, min((th_diff) * self.k , 0.5))

        if self.enemy_dist > 0.3:
            speed = self.speed
        else:
            speed = -self.speed
        #print("enemy_dist {}".format(self.enemy_dist))
        
        self.near_enemy_twist.angular.z = new_twist_ang_z
        self.near_enemy_twist.linear.x = speed
        #print("Update near Enemy Twist")

    def isNearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:15] + scan[-15:]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        if min(forword_scan) < 0.2:
            return True
        return False

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q = tf.transformations.quaternion_from_euler(0,0,yaw)
        # import pdb; pdb.set_trace()
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        '''
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        
        '''
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
                if self.pre_rotation != self.now_rotation:
                    self.symbol_2 = self.symbol_2 * (-1)
                self.goalcounter += self.symbol_2
                if self.goalcounter == len(self.goals):
                    self.is_second_lap = True
                self.goalcounter %= len(self.goals)

                # 2周目の場合は、最初の方のマーカーをとばす
                # if self.is_second_lap and (self.goalcounter in [0, 1, 2]):
                #     self.goalcounter = 3
            rospy.loginfo("done_cb. status:{} result:{}".format(num2mvstate(status), result))

        self.client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        return

    def detect_from_camera(self, data):
        red_angle, green_angle, blue_angle = self.camera_detector.detect_enemy(
            data)
        if red_angle != -360:
            self.is_camera_detect = True
            self.camera_detect_angle = red_angle
            return
        else:
            if green_angle != -360:
                self.is_camera_detect = True
                self.camera_detect_angle = green_angle
            else:
                self.is_camera_detect = False
                self.camera_detect_angle = -360

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        goal_xyyaw = self.goals
        # import pdb; pdb.set_trace()
        
        is_patrol_mode_prev = False
        is_patrol_mode = True
        detect_cnt = 0
        start_cnt = 0

        self.setGoal(self.goals[0][0], self.goals[0][1], self.goals[0][2])

        while not rospy.is_shutdown():
            r.sleep()

            self.getWarState()
            # モード推定
            # 敵追跡モードと巡回モードの分岐条件判定
            is_patrol_mode = True
            detect_inner_th = 0.9
            detect_outer_th = 1.0
            if not self.is_near_enemy:
                is_patrol_mode = True
                detect_cnt = 0
            elif self.is_near_enemy and detect_inner_th > self.enemy_dist:
                is_patrol_mode = False
                detect_cnt += 1
            elif not is_patrol_mode and detect_outer_th < self.enemy_dist:
                is_patrol_mode = True
                detect_cnt = 0

            # Debug
            # is_patrol_mode = False

            if start_cnt < 90:
                is_patrol_mode = True
                start_cnt += 1
            
            # 移動実施
            if is_patrol_mode and (not is_patrol_mode_prev or (self.goalcounter is not self.goalcounter_prev)):
                print("次のゴールに行くぜ〜")
                # 新たに巡回モードに切り替わった瞬間及びゴール座標が変わった時
                # goalcounterのゴール座標をセット
                self.setGoal(goal_xyyaw[self.goalcounter][0], goal_xyyaw[self.goalcounter][1], goal_xyyaw[self.goalcounter][2])
                rospy.loginfo( num2mvstate(self.client.get_state()))
                self.goalcounter_prev = self.goalcounter              
            elif detect_cnt > 2 and is_patrol_mode == False:
                print("Yeah!! Enemy Det!!敵を見るぜ〜")
                # 敵の方向を向くモード
                self.client.cancel_all_goals()
                twist = Twist()
                if detect_cnt < 20:
                    twist.angular.z = 1.5*radians(degrees(self.near_enemy_twist.angular.z))
                else :
                    twist.angular.z = 1.3*radians(degrees(self.near_enemy_twist.angular.z))
                self.vel_pub.publish(twist)
            else :
                print("巡回するぜ〜")
                # 巡回モード最中。CBが来るまで何もしない。
                targetscounter = self.goalcounter // 4
                if (self.goalcounter %4) != 3 and self.targets[self.goalcounter - targetscounter] == "r":
                    self.goalcounter += 1
                
            is_patrol_mode_prev = is_patrol_mode
            
            '''
            for ii in range(goal_xyyaw.shape[0]):
                goal_tmp =goal_xyyaw[ii,:]
                print('self.setGoal(' +str(goal_tmp) + ')')
                self.setGoal(goal_tmp[0],goal_tmp[1],goal_tmp[2])
                if self.is_near_enemy:
                    print("Enemy Det!!!!")
                    # import pdb; pdb.set_trace()
                else:
                    pass
            print('END')        
            '''
            
            
if __name__ == '__main__':
    rospy.init_node('navirun')
    JUDGE_URL = rospy.get_param('~judge_url', 'http://127.0.0.1:5000')

    bot = NaviBot()
    bot.strategy()