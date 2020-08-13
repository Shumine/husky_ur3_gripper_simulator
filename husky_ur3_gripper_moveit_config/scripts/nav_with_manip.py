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
from math import pi
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees


GROUP_NAME_ARM = "ur3_manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"


class TestMove():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur3_move',anonymous=True)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(5)
        self.robot_arm.set_num_planning_attempts(5)
        self.pose_goal = Pose()


        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)        

    def move_code(self):
          
        self.robot_arm.set_named_target("home")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to home 1 ======")        
        rospy.sleep(0.5)        
        
#        print("====== move plan go to up ======")
        self.robot_arm.set_named_target("up")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to up ======")        
        rospy.sleep(0.5)       

        self.robot_arm.set_named_target("home")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to home 1 ======")        
        rospy.sleep(0.5)      

#        robot_arm.set_named_target("up")
#        robot_arm.go(wait=True)

        robot_state = self.robot_arm.get_current_pose()
        robot_angle = self.robot_arm.get_current_joint_values()

        print(robot_state)

    def move_TF(self):
        

        self.pose_goal.position.x = 0.84105
        self.pose_goal.position.y = -0.169917  
        self.pose_goal.position.z = 0.41419  

        self.pose_goal.orientation.x = -0.34484
        self.pose_goal.orientation.y = -0.000192
        self.pose_goal.orientation.z = 0.0003138
        self.pose_goal.orientation.w = 0.93866

        self.robot_arm.set_pose_target(self.pose_goal)
        self.robot_arm.go(True)




def create_nav_goal(x, y, yaw):
    """Create a MoveBaseGoal with x, y position and yaw rotation (in degrees).
    Returns a MoveBaseGoal"""
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = "map" # Note: the frame_id must be map
    mb_goal.target_pose.pose.position.x = x
    mb_goal.target_pose.pose.position.y = y
    mb_goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)
    
    # Orientation of the robot is expressed in the yaw value of euler angles
    angle = radians(yaw) # angles are expressed in radians
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
    
    return mb_goal

def callback_pose(data):
    """Callback for the topic subscriber.
       Prints the current received data on the topic."""
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
    rospy.loginfo("Current robot pose: x=" + str(x) + "y=" + str(y) + " yaw=" + str(degrees(yaw)) + "º")

if __name__=='__main__':

    tf_x = 0
    tf_y = 0
    tf_yaw = 0
    tm = TestMove()
    tm.__init__()
    #rospy.init_node("navigation_snippet")
    
    # Read the current pose topic
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
    
    # Connect to the navigation action server
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Connecting to /move_base AS...")
    nav_as.wait_for_server()
    rospy.loginfo("Connected.")
    


    while not rospy.is_shutdown():  

        rospy.loginfo("Enter the position where the robot reach.(X,Y,Yaw)")
        tf_x = float(input("X:"))
        tf_y = float(input("Y:"))
        tf_yaw = float(input("Yaw:"))
        nav_goal = create_nav_goal(tf_x, tf_y, tf_yaw)
        rospy.loginfo("Sending goal to x= %.2f, y= %.2f, yaw= %.2f" %(tf_x,tf_y,tf_yaw))
        nav_as.send_goal(nav_goal)
        rospy.loginfo("Waiting for result...")
        nav_as.wait_for_result()
        nav_res = nav_as.get_result()
        nav_state = nav_as.get_state()

        rospy.loginfo("Done!")

        rospy.sleep(0.5)
        print 
        print "Result: ", str(nav_res) # always empty, be careful
        print "Nav state: ", str(nav_state) # use this, 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)


    
    rospy.spin()
    roscpp_shutdown()