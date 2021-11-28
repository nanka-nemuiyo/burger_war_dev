#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import rosparam
from math import radians, degrees, atan2

import tf

from geometry_msgs.msg import Twist

import tf

import numpy as np
import math

pi = math.pi
PI = math.pi

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float64

from Enemy_detector_obs import EnemyDetector

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

class EnermyDetectLoop():
    def __init__(self):

        self.robot_namespace = rospy.get_param('~robot_namespace')
        print(self.robot_namespace)
        # velocity publisher
        #self.tfBuffer = tf.Buffer()

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


        self.pose_twist = Twist()
        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.


        self.is_near_wall = False

        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None
        self.near_enemy_twist = Twist()
        self.near_enemy_twist.linear.x = self.speed; self.near_enemy_twist.linear.y = 0.; self.near_enemy_twist.linear.z = 0.
        self.near_enemy_twist.angular.x = 0.; self.near_enemy_twist.angular.y = 0.; self.near_enemy_twist.angular.z = 0.

        self.is_initialized_pose = True

        # lidar scan
        self.scan = []

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.enm_pub = rospy.Publisher('enemy_detected', Bool ,queue_size=1)
        self.enm_pub2 = rospy.Publisher('enemy_dist', Float64 ,queue_size=1)
        self.enm_pub3 = rospy.Publisher('enemy_rad', Float64 ,queue_size=1)
        # subscriber



    def scanEnemy(self):
        r = rospy.Rate(5) # change speed 5fps
        map_name=self.robot_namespace+'/map'
        #is_enemy_detected, x, y = self.getEnemyPos(map_name)
        
        while(1):
            # 敵の検出
            is_enemy_detected, enemy_dist, enemy_rad = self.getEnemyDistRad()
            # rospy.loginfo("is_enemy_detected:{} enemy_dist{} enemy_rad:{}".format(is_enemy_detected, enemy_dist, enemy_rad))
            #print("test")
            self.enm_pub.publish(is_enemy_detected)
            self.enm_pub2.publish(enemy_dist)
            self.enm_pub3.publish(enemy_rad)
            #map_name=self.robot_namespace+'/map'
            r.sleep()
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
    
if __name__ == '__main__':
    rospy.init_node('enemy_detect_loop')
    enemyloop = EnermyDetectLoop()
    enemyloop.scanEnemy()
