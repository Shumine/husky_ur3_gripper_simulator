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


def get_TF(a,b):
  end_flag = 0
  listener = tf.TransformListener()
  while end_flag ==0:
      try:
          (trans,rot) = listener.lookupTransform(a,b, rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      end_flag = 1

  return trans,rot
  
    
def move_ee(Px,Py,Pz,Ox,Oy,Oz,Ow):


  trans,rot = get_TF('/odom','/base_link')
  print('TF from odom to base link :',trans)
  x = Px-trans[0]
  y = Py-trans[1]
  z = Pz-trans[2]
  Ox = Ox-rot[0]
  Oy = Oy-rot[1]
  Oz = Oy-rot[2]
  Ow = Ow-rot[3]
  print "============ Generating plan 1"
  pose_target = Pose()
  pose_target.position.x = x
  pose_target.position.y = y
  pose_target.position.z = z
  pose_target.orientation.x = Ox
  pose_target.orientation.y = Oy
  pose_target.orientation.z = Oz
  pose_target.orientation.w = Ow
  group.set_pose_target(pose_target)
  group.go(True)
  print "============ plan 1 complete!"

  trans,rot = get_TF('odom','/ee_link')
  print "============ ee pose : "
  print group.get_current_pose()
  print group.get_planning_frame()
  print 'odom_TF',trans,rot
  print "============"





if __name__=='__main__':
  move_joint(pi/2,0,0,-pi/2,-pi/2,0) 

 

  #move_ee(0.65,0,0.65,0,0,0,0)
  #move_joint(pi/2,0,0,-pi/2,-pi/2,0) 






    
    




