#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
import numpy as np
pi = math.pi
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient, GoalStatus

import tf

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
from visualization_msgs.msg import Marker


# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2
def get_goals(my_color, rotation="CW"):
    if my_color == 'b':
        symbol = -1
        th     = pi
    else:
        symbol = 1
        th     = 0

    # rotation = 'CW'  # 回転方向を変える @en

    if rotation == 'CW':
        symbol_2 = -1
    else:
        symbol_2 = 1

    #print("get_goals", my_color, symbol, th, symbol_2)
         
    # 12x3 (x,y,yaw) 
    goal_xyyaw = np.array([ 
            [symbol * -1  , symbol * 0     , np.mod(pi/4 + th ,2*pi) ],
            [symbol * -1  , symbol * 0  , np.mod(pi*7/4 + th ,2*pi) ], 
            [symbol * -0.9  , symbol * -0.5     , np.mod(pi*5/3 + th ,2*pi) ],
            [symbol * 0  , symbol *  -1  , np.mod(pi*3/4 + th ,2*pi) ],
            [symbol * -0  , symbol *  -1  , np.mod(pi/4 + th ,2*pi) ],
            [symbol * 0.5     , symbol * -0.9   , np.mod(pi/6 + th ,2*pi) ], 
            [symbol * 1     , symbol * 0   , np.mod(pi*5/4  + th ,2*pi) ],
            [symbol * 1   , symbol * 0   , np.mod( pi*3/4     + th ,2*pi) ],
            [symbol * 0.9   , symbol * 0.5   ,np.mod(pi*2/3   + th ,2*pi) ],
            [symbol * 0   , symbol *   1   , np.mod(pi*7/4     + th ,2*pi) ], 
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
        self.my_color = rospy.get_param('~rside')
        self.goalcounter = 0
        self.goalcounter_prev = -1
        self.goals = get_goals(self.my_color)
        self.is_second_lap = False

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

            rospy.loginfo("done_cb. status:{} result:{}".format(num2mvstate(status), result))

        self.client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb) 

        return

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        # import pdb; pdb.set_trace()
        cnt = 0
        enemy_dist = 1.0
        is_patrol_mode = True
        is_patrol_mode_prev = False
        is_enemy_detected =  False

        self.setGoal(self.goals[0][0], self.goals[0][1], self.goals[0][2])

        while not rospy.is_shutdown():
            r.sleep()

            if cnt < 50:
                enemy_dist = 0.9
            elif cnt < 60:
                enemy_dist = 0.3
                is_enemy_detected = True
            elif cnt >= 60:
                cnt = 0

            cnt += 1  
            
            detect_inner = 0.5
            detect_outer = 0.6

            if not is_enemy_detected:
                is_patrol_mode = True
            elif is_patrol_mode and detect_inner > enemy_dist:
                is_patrol_mode = False
            elif not is_patrol_mode and detect_outer < enemy_dist:
                is_patrol_mode = True

            # 移動実施
            if is_patrol_mode and (not is_patrol_mode_prev or (self.goalcounter is not self.goalcounter_prev)):
                # 新たに巡回モードに切り替わった瞬間及びゴール座標が変わった時
                # goalcounterのゴール座標をセット
                self.setGoal(self.goals[self.goalcounter][0], self.goals[self.goalcounter][1], self.goals[self.goalcounter][2])
                print("切り替わるタイミング")
                self.goalcounter_prev = self.goalcounter
            elif is_patrol_mode:
                # 巡回モード最中。CBが来るまで何もしない。
                print("巡回モード")
                #pass
            else : 
                # 敵の方向を向くモード
                print("敵向くモード")
                self.client.cancel_all_goals()
                twist = Twist()
                twist.angular.z = 0
                self.vel_pub.publish(twist)
            is_patrol_mode_prev = is_patrol_mode

            print(is_patrol_mode)

            

        print('END')        

if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()