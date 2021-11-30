#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import math
import tf
import roslib.packages
from obstacle_detector.msg import Obstacles
from std_msgs.msg          import Float32

class EnemyDetector:
    def __init__(self):
        #self.map_data#このクラスが持つ「num」変数に引数を格納
        self.tf_broadcaster  = tf.TransformBroadcaster()
        self.tf_listener     = tf.TransformListener()
        self.sub_obstacles   = rospy.Subscriber('obstacles', Obstacles, self.obstacles_callback)
        self.pub_robot2enemy = rospy.Publisher('robot2enemy', Float32, queue_size=10)
        self.robot_name      = rospy.get_param('~robot_namespace', '')
        self.pub_enemy_x_pos = rospy.Publisher('enemy_x_pos', Float32, queue_size=1)
        self.pub_enemy_y_pos = rospy.Publisher('enemy_y_pos', Float32, queue_size=1)
        self.pub_my_x_pos = rospy.Publisher('my_x_pos', Float32, queue_size=1)
        self.pub_my_y_pos = rospy.Publisher('my_y_pos', Float32, queue_size=1)
        
    def obstacles_callback(self, msg):

        closest_enemy_len = sys.float_info.max
        closest_enemy_x   = 0
        closest_enemy_y   = 0

        for num in range(len(msg.circles)):

            temp_x = msg.circles[num].center.x
            temp_y = msg.circles[num].center.y

            #フィールド内のオブジェクトであればパス
            if self.is_point_emnemy(temp_x, temp_y) == False:
                #print("not det")
                continue
            #print("enemy_x = " + str(temp_x))
            #print("enemy_y = " + str(temp_y))
            #敵の座標をTFでbroadcast
            enemy_frame_name = self.robot_name + '/enemy_' + str(num)
            map_frame_name   = self.robot_name + "/map"
            self.tf_broadcaster.sendTransform((temp_x,temp_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            #ロボットから敵までの距離を計算
            try:
                target_frame_name = self.robot_name + '/enemy_' + str(num)
                source_frame_name = self.robot_name + "/base_footprint"
                (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            #print("my_x(m-e) = " + str(trans[0] + temp_x))
            #print("my_y(m-e) = " + str(trans[1] + temp_y))
            #print("my_x(e-m) = " + str(temp_x - trans[0]))
            #print("my_y(e-m) = " + str(temp_y - trans[1]))
            my_x, my_y = 0,0
            if -1 < trans[0] + temp_x and trans[0] + temp_x < 1:
                my_x = trans[0] + temp_x
            elif -1 < temp_x - trans[0] and temp_x - trans[0] < 1:
                my_x = temp_x - trans[0]

            if -1 < trans[1] + temp_y and trans[1] + temp_y < 1:
                my_y = trans[1] + temp_y
            elif -1 < temp_y - trans[1] and temp_y - trans[1] < 1:
                my_y = temp_y - trans[1]

            len_robot2enemy = math.sqrt(pow(trans[0],2) + pow(trans[1],2))

            if closest_enemy_len > len_robot2enemy:
                closest_enemy_len = len_robot2enemy
                closest_enemy_x   = temp_x
                closest_enemy_y   = temp_y

        #敵を検出している場合、その座標と距離を出力
        if closest_enemy_len < sys.float_info.max:

            map_frame_name   = self.robot_name + "/map"
            enemy_frame_name = self.robot_name + "/enemy_closest"
            self.tf_broadcaster.sendTransform((closest_enemy_x,closest_enemy_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            #ロボットから敵までの距離をpublish
            self.pub_robot2enemy.publish(closest_enemy_len)
            #敵の座標と自分の座標？をpublish
            self.pub_enemy_x_pos.publish(closest_enemy_x)
            self.pub_enemy_y_pos.publish(closest_enemy_y)
            self.pub_my_x_pos.publish(my_x)
            self.pub_my_y_pos.publish(my_y)


    def is_point_emnemy(self, point_x, point_y):
        #フィールド内の物体でない、敵と判定する閾値（半径）
        thresh_corner = 0.180
        thresh_center = 0.280

        #フィールド内かチェック
        if   point_y > (-point_x + 1.55):
            return False
        elif point_y < (-point_x - 1.55):
            return False
        elif point_y > ( point_x + 1.55):
            return False
        elif point_y < ( point_x - 1.55):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        len_p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        len_p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        len_p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
        len_p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2))

        if len_p1 < thresh_corner or len_p2 < thresh_corner or len_p3 < thresh_corner or len_p4 < thresh_corner or len_p5 < thresh_center:
            return False
        else:
            return True


if __name__ == '__main__':

    rospy.init_node('enemy_detector')
    ed = EnemyDetector()
    rospy.loginfo("Enemy Detector Start.")
    rospy.spin()
