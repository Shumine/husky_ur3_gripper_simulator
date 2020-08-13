#!/usr/bin/env python

# Author: Sangheum Lee
# Affiliation: Human-Robot Interaction LAB, Kyung Hee University, South Korea
# humine@khu.ac.kr



import sys
import rospy
import tf
import moveit_commander  # https://answers.ros.org/question/285216/importerror-no-module-named-moveit_commander/
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi
from moveit_msgs.msg import DisplayTrajectory







print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('control_UR3',
                anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur3_manipulator")




def move_joint(q1,q2,q3,q4,q5,q6):
    joint_goal = group.get_current_joint_values()

    mobile_joints = [-pi/3, 0.5]
    joint_goal_list = [q1,q2,q3,q4,q5,q6] 

    joint_goal[0] = joint_goal_list[0]
    joint_goal[1] = joint_goal_list[1]
    joint_goal[2] = joint_goal_list[2]
    joint_goal[3] = joint_goal_list[3]
    joint_goal[4] = joint_goal_list[4]
    joint_goal[5] = joint_goal_list[5]

    group.go(joint_goal, wait=True)


if __name__=='__main__':
    #move_mobile_manipulator(1,1,pi/2,-pi/2,0,0,0,0)

    move_joint(pi/2,0,0,-pi/2,-pi/2,0) 


    end_flag = 0
    listener = tf.TransformListener()
    while end_flag ==0:
        try:
            (rom_trans,rom_rot) = listener.lookupTransform('/base_link','/ee_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        end_flag = 1
    rom_radius_x = rom_trans[0]/2
    print 'x_max_radius:',rom_radius_x,rom_trans[0]



    move_joint(0, -pi/2,0,-pi/2,0,0) 

    
    end_flag = 0
    listener = tf.TransformListener()
    while end_flag ==0:
        try:
            (rom_trans,rom_rot) = listener.lookupTransform('/base_link','/ee_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        end_flag = 1
    
    rom_radius_z = rom_trans[2]/2
    print 'z_max_radius:',rom_radius_z,rom_trans[2]
    
    
    
    
    
    print "============ Generating plan 1"
    pose_target = Pose()
    pose_target.position.x = 0.5
    pose_target.position.y = -0.2
    pose_target.position.z = 0.8
    group.set_pose_target(pose_target)
    
    #group.go(True)
    print "============ plan 1 complete!"
    print "============ ee pose : "
    #print group.get_current_pose()
    print "============"
    
    
    

