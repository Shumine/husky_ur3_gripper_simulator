#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Sangheum Lee

Navigation actionserver: /move_base/goal
Type of message: move_base_msgs/MoveBaseActionGoal
Actual robot pose topic: /amcl_pose
Type of message: geometry_msgs/PoseWithCovarianceStamped
"""
import sys
import rospy
import copy, math
import threading
import time
import tf
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
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

#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
roscpp_initialize(sys.argv)
rospy.init_node('control_Husky_UR3', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()



## 매니퓰레이터 변수 선언

group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'world'





#모바일 로봇 변수
x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def move_base(a,b):             #x,y좌표를 받아와서 그곳으로 platform을 움직이는 코드 
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b
    goal_angle = atan2(goal.y,goal.x)

    while abs(goal.x-x) >0.01 : #가까의 범위가 0.01이내로 들어오면 break.

        if(goal.x-x) >0:
            speed.linear.x = 0.1
            speed.angular.z = 0.0
            if(abs(goal.x-x)<0.05):
                speed.linear.x = 0.01
                speed.angular.z = 0.0
        else :
            speed.linear.x = -0.1
            speed.angular.z = 0.0
            if(abs(goal.x-x))<0.05:
                speed.linear.x = -0.01
                speed.angular.z = 0.0


        pub.publish(speed)
        r.sleep() 
        print goal.x, x, abs(goal.x-x)

    return x,y






def move_Joint(q1,q2,q3,q4,q5,q6):
    joint_goal = move_group.get_current_joint_values()

    mobile_joints = [-pi/3, 0.5]
    joint_goal_list = [q1,q2,q3,q4,q5,q6] 

    #매니퓰레이터 관절 value 설정
    joint_goal[0] = joint_goal_list[0]
    joint_goal[1] = joint_goal_list[1]
    joint_goal[2] = joint_goal_list[2]
    joint_goal[3] = joint_goal_list[3]
    joint_goal[4] = joint_goal_list[4]
    joint_goal[5] = joint_goal_list[5]
    #제어시작
    move_group.go(joint_goal, wait=True)



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
  
    
def move_ee(Px,Py,Pz,Rx,Ry,Rz,Rw):


  trans,rot = get_TF('/odom','/base_link')
  print('TF from odom to base link :',trans)
  x = Px-trans[0]
  y = Py-trans[1]
  z = Pz-trans[2]


  Ox = Rx
  Oy = Ry
  Oz = Rz-rot[2]
  Ow = Rw

  print 'real_planning_pose',x,y,z,Ox,Oy,Oz,Ow
  print "============ Generating plan 1"
  pose_target = Pose()
  pose_target.position.x = x
  pose_target.position.y = y
  pose_target.position.z = z
  pose_target.orientation.x = Ox
  pose_target.orientation.y = Oy
  pose_target.orientation.z = Oz
  pose_target.orientation.w = Ow
  move_group.set_pose_target(pose_target)
  move_group.go(True)
  print "============ plan 1 complete!"

  trans_1,rot_1 = get_TF('odom','/ee_link')
  print "============ ee pose : "
  print move_group.get_current_pose()
  print move_group.get_planning_frame()
  print 'odom_TF',trans_1,rot_1
  print "============"




    
def cartesian_path_planner(a,b,c):
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z +=  a  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    wpose = move_group.get_current_pose().pose
    wpose.position.x +=  b  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    wpose = move_group.get_current_pose().pose
    wpose.position.y +=  c  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))    

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,        # eef_step
                                       0.0)         # jump_threshold


def x_path_planner(a,pose_tar):


    pose_target = pose_tar
    rospy.sleep(1)
    pose_target.position.x +=  a  # First move up (z)
    move_group.set_pose_target(pose_target)
    
    tf_display_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]      
    tf_display_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    ii = 0
    while ii < 5:
        ii += 1
        br = tf.TransformBroadcaster()
        br.sendTransform(
            tf_display_position,
            tf_display_orientation,
            rospy.Time.now(),
            "Target_pose",
            "base_link")
        rate.sleep()

    raw_input()
    move_group.go(True)

def y_path_planner(c,pose_tar):
    pose_target = pose_tar
    rospy.sleep(1)
    pose_target.position.y +=  c  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)  

    tf_display_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]      
    tf_display_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    ii = 0
    while ii < 5:
        ii += 1
        br = tf.TransformBroadcaster()
        br.sendTransform(
            tf_display_position,
            tf_display_orientation,
            rospy.Time.now(),
            "Target_pose",
            "base_link")
        rate.sleep()
    
    raw_input()    
    move_group.go(True)



def z_path_planner(b,pose_tar):
    pose_target = pose_tar
    rospy.sleep(1)
    pose_target.position.z +=  b  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)  

    tf_display_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]      
    tf_display_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    ii = 0
    while ii < 5:
        ii += 1
        br = tf.TransformBroadcaster()
        br.sendTransform(
            tf_display_position,
            tf_display_orientation,
            rospy.Time.now(),
            "Target_pose",
            "base_link")
        rate.sleep()
   
    raw_input()  
    move_group.go(True)

    
def down_demo():
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #down pose 
    print "Down demo is ready to start!, press enter..!"
    raw_input()
    z_path_planner(0.1)
    y_path_planner(-0.112)  
    x_path_planner(0.1)    
    z_path_planner(-0.2)
    rospy.sleep(3)
    print "Down demo complete!, Go to home pose..!"

def Grasp_object(x_dir,y_dir,z_dir): # Grasping할 목표 위치 지정 x,y,z

    x_dir_distance_mm = 0.6#(-3/7*(z_dir-13/6))

    go_to_nav_goal(x_dir-x_dir_distance_mm,y_dir,0) #매니퓰레이터 뻗는 길이에 의해 고려된 모바일 위치 입력 Yaw방향은 항상 0으로 고정함.
    current_mobile_x,current_mobile_y =  move_base(x_dir-x_dir_distance_mm,y_dir)

    #move_group.set_named_target("up")  #go to goal state.
    #move_group.go(wait=True)
    #rospy.sleep(0.5)       

    
    curr_pose = move_group.get_current_pose().pose
    z_path_planner(0.1,curr_pose)
    print "Grasping is ready to start!, press enter..!"
    #raw_input()
    
    x_distance = current_mobile_x+curr_pose.position.x - x_dir  
    y_distance = current_mobile_y+curr_pose.position.y -  y_dir  
    z_distance = curr_pose.position.z -  z_dir  

    print curr_pose.position.x
    print 'x_dir =',x_dir,'y_dir=',y_dir,'z_dir=',z_dir

    print 'x =',x_distance,'y=',y_distance,'z=',z_distance
    x_path_planner(-x_distance,curr_pose)    
    y_path_planner(-y_distance,curr_pose)  
    z_path_planner(-z_distance,curr_pose)
    rospy.sleep(3)
    (result_xyz,result_rot) = get_TF('/odom','ee_link')
    print 'xyz_result=',result_xyz[0],result_xyz[1],result_xyz[2]
    print "Grasping complete!, Go to home pose..!"






    ## ## ## show how to move on the Rviz
    #coke_waypoints = []
    #coke_waypoints.append(copy.deepcopy(self.calculed_coke_pose))
    #(coke_plan, coke_fraction) = self.robot_arm.compute_cartesian_path(coke_waypoints, 0.01, 0.0)
    #self.display_trajectory(coke_plan)
    ### ## #
    #print "============ Press `Enter` to if plan is correct!! ..."
    #raw_input()
    #self.robot_arm.go(True)






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


def go_to_nav_goal(tf_x,tf_y,tf_yaw): #엔드이펙터 좌표에의해 구해진 모바일 베이스 (네비게이션 기반) 위치 제어기
    
    # Read the current pose topic
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
    
    # Connect to the navigation action server
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Connecting to /move_base AS...")
    nav_as.wait_for_server()
    rospy.loginfo("Connected.")


    nav_goal = create_nav_goal(tf_x, tf_y, tf_yaw)
    rospy.loginfo("Sending goal to x= %.2f, y= %.2f, yaw= %.2f" %(tf_x,tf_y,tf_yaw))
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()
    nav_res = nav_as.get_result()
    nav_state = nav_as.get_state()
    rospy.loginfo("Done!")
    rospy.sleep(1)




    print "Result: ", str(nav_res) # always empty, be careful
    print "Nav state: ", str(nav_state) # use this, 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
    

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

    rate = rospy.Rate(10.0)

    
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose 


    while not rospy.is_shutdown():  

        (result_xyz,result_rot) = get_TF('/odom','ee_link')
        print 'initial_xyz=',result_xyz[0],result_xyz[1],result_xyz[2]

        rospy.loginfo("Enter the position where the end-effector reach.(X,Y,Z)")
        x_input = float(input("X:"))
        y_input = float(input("Y:"))
        z_input = float(input("Z(range:0.3~0.5):"))
        
        Grasp_object(x_input,y_input,z_input)
        print "Grasp finished, press Enter."
        raw_input()
        move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose 




    
    rospy.spin()
    roscpp_shutdown()