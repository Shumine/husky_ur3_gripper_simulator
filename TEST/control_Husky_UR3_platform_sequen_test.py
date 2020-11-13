#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
import sys
import rospy
import copy, math
import threading
import time
import tf, signal

from threading import Thread
import multiprocessing
from multiprocessing import Process
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list, list_to_pose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pi
from ar_track_alvar_msgs.msg import AlvarMarkers

import numpy as np
from numpy import linalg


import numpy as np
import tf.transformations as tf
from math import *
import cmath
from geometry_msgs.msg import Pose, Quaternion

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def ar_position(msg):

    global ar_x
    global ar_y
    global ar_z
    ar_topic_pose = Pose()
    try:    
        ar_topic_pose.position.x = msg.markers[0].pose.pose.position.x
        ar_topic_pose.position.y = msg.markers[0].pose.pose.position.y
        ar_topic_pose.position.z = msg.markers[0].pose.pose.position.z
        ar_x = ar_topic_pose.position.x
        ar_y = ar_topic_pose.position.y
        ar_z = ar_topic_pose.position.z
    except:
        return



def jmove_to_pose_goal(pose_goal):

    move_group.set_pose_target(pose_goal)
    move_group.go(wait=False)
    #tf_display_position = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]      
    #tf_display_orientation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
    #ii = 0
    #while ii < 5:
    #    ii += 1
    #    br = tf.TransformBroadcaster()
    #    br.sendTransform(
    #        tf_display_position,
    #        tf_display_orientation,
    #        rospy.Time.now(),
    #        "Target_pose",
    #        "base_link")
    #    rospy.sleep(1)
        

def move_Joint(q1,q2,q3,q4,q5,q6):
    joint_goal = move_group.get_current_joint_values()

    joint_goal_list = [q1,q2,q3,q4,q5,q6] 

    #매니퓰레이터 관절 value 설정
    joint_goal[0] = joint_goal_list[0]
    joint_goal[1] = joint_goal_list[1]
    joint_goal[2] = joint_goal_list[2]
    joint_goal[3] = joint_goal_list[3]
    joint_goal[4] = joint_goal_list[4]
    joint_goal[5] = joint_goal_list[5]
    #제어시작
    move_group.go(joint_goal, wait=False)




if __name__=='__main__':
    signal.signal(signal.SIGINT, signal_handler)

    #GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
    roscpp_initialize(sys.argv)
    rospy.init_node('control_Husky_UR3', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()



    ##모바일 파트 관련 변수 선언
    x = 0.0
    y = 0.0 
    theta = 0.0

    ## 매니퓰레이터 변수 선언



    group_name = "ur3_manipulator"
    move_group = MoveGroupCommander(group_name)
    FIXED_FRAME = 'world'

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   DisplayTrajectory,
                                                   queue_size=20)





    # ar마커 변수 선언
    ar_x = 0
    ar_y = 0
    ar_z = 0





    # 사용자입력 : 목표 eef 위치 지정 (x,y,z,r,p,yaw)
    goal_eef = [0,0,0,0,0,0,1]
    goal_eef_quat = list_to_pose(goal_eef)


    # ar 마커 및 eef pose data 선언.
    ar_pose = Pose()
    eef_pose = Pose()

    






    sub1 = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_position)

    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    speed = Twist()
    rospy.sleep(2) 

    r = rospy.Rate(20)

    ur_reco_reach = 0.5

    prev_t = time.time() 
    while not rospy.is_shutdown():

        ar_pose = Pose()
        ar_pose.position.x = sqrt(ur_reco_reach**2-(ar_z-0.39)**2)+0.25-0.05
        ar_pose.position.y = 0
        ar_pose.position.z = ar_z
        ar_pose.orientation.x = 0.707
        ar_pose.orientation.y = 0.0
        ar_pose.orientation.z = 0.707
        ar_pose.orientation.w = 0.0
        print ar_pose.position.x 
        rospy.sleep(0.1) 

        dist_x = 0
        prev_x = x

        dist_th = 0
        prev_th = theta

        dist_to_rotate = math.atan2(ar_y,ar_x)

        dist_to_go = ar_x/(math.cos(dist_to_rotate/2)) - ar_pose.position.x #모바일 로봇이 가야하는 경로

        ratio_lin_and_ang = dist_to_go/dist_to_rotate


        while  dist_to_go-dist_x > 0.05 or abs(dist_to_rotate-dist_th)>0.01 : 

            #ee_state = move_group.get_current_pose()
            #ee_x = ee_state.pose.position.x

            #dist_ar = abs(ar_x-ee_x)

            curr_x = x
            dist_x = curr_x-prev_x

            curr_th = theta
            dist_th = curr_th-prev_th

            last_dist = dist_to_go-dist_x
            desired_speed = 0.45/(np.exp(-12*last_dist+6)+1)+0.05
            
            desired_ang_speed = desired_speed/ratio_lin_and_ang

            speed.linear.x = desired_speed
            speed.angular.z = desired_ang_speed

            #print ar_x,ee_x
            print "desired speed:",desired_speed
            print "distance:",last_dist

            pub.publish(speed)
            r.sleep()    
        
        ar_pose.position.x = ar_x
        ar_pose.position.y = 0
        ar_pose.position.z = ar_z
        ar_pose.orientation.x = 0.707
        ar_pose.orientation.y = 0.0
        ar_pose.orientation.z = 0.707
        ar_pose.orientation.w = 0.0
        jmove_to_pose_goal(ar_pose)

    
    
        print 'complete! press enter to next task..'
        now_t = time.time()
        dur = now_t-prev_t
        print "time duration:",dur,"sec"
        raw_input()





 

    








    









  

























 