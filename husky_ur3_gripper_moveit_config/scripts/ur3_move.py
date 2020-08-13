#!/usr/bin/env python

# Author: Hyeonjun Park, Ph.D. candidate
# Affiliation: Human-Robot Interaction LAB, Kyung Hee University, South Korea
# koreaphj91@gmail.com
# init: 9 Apr 2019
# revision: 17 Feb 2020


import sys
import rospy
import tf
import moveit_commander  # https://answers.ros.org/question/285216/importerror-no-module-named-moveit_commander/
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move',anonymous=True)
group = [moveit_commander.MoveGroupCommander("ur3_manipulator")]  # ur3 moveit group name: manipulator





xx = 1

while not rospy.is_shutdown():  
  if(xx%3 == 1):        
    pose_goal.position.x = 0.5555 # red line      0.2   0.2
    pose_goal.position.y = -0.456  # green line  0.15   0.15
    pose_goal.position.z = 0.3164  # blue line   # 0.35   0.6
    
    pose_goal.orientation.x = 0.99
    pose_goal.orientation.y = 0.99
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0


    group[0].set_pose_target(pose_goal)
    group[0].go(True)
    
    print xx%3,"th action"

    rospy.sleep(1)

  elif(xx%3 == 2):
    pose_goal.position.x = 0.785994671063 # red line      0.2   0.2
    pose_goal.position.y = -0.0229786933013  # green line  0.15   0.15
    pose_goal.position.z = 0.314996895388  # blue line   # 0.35   0.6

    pose_goal.orientation.x = 0.318900610382
    pose_goal.orientation.y = 0.735222859634
    pose_goal.orientation.z = -0.282179386008
    pose_goal.orientation.w = 0.527375143027


    group[0].set_pose_target(pose_goal)
    group[0].go(True)
    print xx%3,"th action"
    rospy.sleep(1)


      
  else:
    pose_goal.position.x = 0.525450383153 # red line      0.2   0.2
    pose_goal.position.y = -0.000374391970629  # green line  0.15   0.15
    pose_goal.position.z = 0.945649709494  # blue line   # 0.35   0.6

    pose_goal.orientation.x = 0.000843811179718
    pose_goal.orientation.y = 3.578436014e-07
    pose_goal.orientation.z = 0.00041607644225
    pose_goal.orientation.w = 0.999999557431


    group[0].set_pose_target(pose_goal)
    group[0].go(True)
      
    print xx%3,"th action"
    rospy.sleep(1) 
    xx = xx + 1 

  
'''
pose_goal.orientation.w = 0.0
pose_goal.position.x = 0.4 # red line      0.2   0.2
pose_goal.position.y = 0.15  # green line  0.15   0.15
pose_goal.position.z = 0.5  # blue line   # 0.35   0.6
group[0].set_pose_target(pose_goal)
group[0].go(True)
'''
moveit_commander.roscpp_shutdown()