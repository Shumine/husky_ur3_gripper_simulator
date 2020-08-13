#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 10 11:16:00 2014
@author: Sam Pfeiffer
Snippet of code on how to send a navigation goal and how to get the current robot position in map
Navigation actionserver: /move_base/goal
Type of message: move_base_msgs/MoveBaseActionGoal
Actual robot pose topic: /amcl_pose
Type of message: geometry_msgs/PoseWithCovarianceStamped
"""
import sys
import rospy
import copy, math
from math import pi, radians, degrees
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String



#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
roscpp_initialize(sys.argv)
rospy.init_node('joint_control_UR', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()



group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'world'

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)

                                               # We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print "============ Planning frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

# We can get the joint values from the group and adjust some of the values:
q1 = pi*90/180
q2 =  -pi*130/180
q3 = pi*111/180
q4 =  -pi*68/180
q5 =  -pi*90/180
q6 =  0

joint_goal = move_group.get_current_joint_values()
#joint_goal_list = [pi*(90/180), -pi*(130/180) , pi*(111/180), -pi*(68/180), -pi*(90/180), 0] #home pose
joint_goal_list = [q1,q2,q3,q4,q5,q6]
joint_goal[0] = joint_goal_list[0]
joint_goal[1] = joint_goal_list[1]
joint_goal[2] = joint_goal_list[2]
joint_goal[3] = joint_goal_list[3]
joint_goal[4] = joint_goal_list[4]
joint_goal[5] = joint_goal_list[5]

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
#move_group.stop()




