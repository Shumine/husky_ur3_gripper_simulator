#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import tf
from geometry_msgs.msg import Point, Twist
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
import std_msgs.msg
from moveit_commander.conversions import pose_to_list, list_to_pose
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import pi, radians, degrees, atan2, sqrt
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry



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


def move_base(a,b):             
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b
    goal_angle = atan2(goal.y,goal.x)

    while abs(goal.x-x) >0.01 : 

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

    tr,ro = get_TF('base_link','odom')
    return tr[0],tr[1]



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

def callback_pose(data):
    """Callback for the topic subscriber.
       Prints the current received data on the topic."""
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])


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


def go_to_nav_goal(tf_x,tf_y,tf_yaw): 
    
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


class PickNPlaceTutorial():
    """PickNPlaceTutorial"""
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_n_place_tutorial', anonymous=True)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander('ur3_manipulator')

        # Misc variables
        self.box_name = ''
        
    def jmove_to_pose_goal(self, pose_goal):
        self.robot_group.set_pose_target(pose_goal)
        self.robot_group.go(wait=True)

        tf_display_position = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]      
        tf_display_orientation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        ii = 0
        while ii < 5:
            ii += 1
            br = tf.TransformBroadcaster()
            br.sendTransform(
                tf_display_position,
                tf_display_orientation,
                rospy.Time.now(),
                "Target_pose",
                "odom")
            rospy.sleep(1)

    def jmove_to_joint_goal(self, joint_goal):
        self.robot_group.go(joint_goal, wait=True)
        
    
    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_box(self, name, pose_stamp, size=(0.05, 0.05, 0.05)):
        self.scene.add_box(name, pose_stamp, size=size)
        self.wait_for_state_update(box_name=name, box_is_known=True)

def calc_dist_and_go(goal_pose):
        tr,ro = get_TF('base_link','odom')
        curr_mob_x = tr[0]
        curr_mob_y = tr[1]

        goal_ee_x = goal_pose.position.x
        goal_ee_y = goal_pose.position.y

        dist_x = goal_ee_x  - 0.5
        dist_y = goal_ee_y -0.5
        go_to_nav_goal(dist_x,dist_y,0)
        


def main():
    try:
        pnp = PickNPlaceTutorial()
        pnp.jmove_to_joint_goal([1.57,-2.27,1.93,-1.19,-1.57,0]) #home pose
        
        tr,ro = get_TF('base_link','odom')
        curr_mob_x = tr[0]
        curr_mob_y = tr[1]

    
        table = geometry_msgs.msg.PoseStamped()
        table.header.frame_id = "odom"
        table.pose.position.x =0.75
        table.pose.position.y =0.1
        table.pose.position.z =0.7
        table.pose.orientation.w = 1                          
        pnp.add_box(name='table', pose_stamp=table, size=(0.1, 0.1, 0.3))
        

        i = 0
        while(i<5):
            
            test_pose =  list_to_pose([0.75-0.01*i,0.1,0.7,0,0,0])

            table.pose.position.x =test_pose.position.x
            table.pose.position.y =test_pose.position.y
            table.pose.position.z =test_pose.position.z

            table.pose.orientation.x = test_pose.orientation.x
            table.pose.orientation.y = test_pose.orientation.y
            table.pose.orientation.z = test_pose.orientation.z
            table.pose.orientation.w = test_pose.orientation.w  
            table.pose.position.x = table.pose.position.x - 0.1
            table.pose.position.y = table.pose.position.y -0.1
            table.pose.position.z = table.pose.position.z -0.1       
                                    
            pnp.add_box(name='table', pose_stamp=table, size=(0.1, 0.1, 0.1))
            rospy.sleep(0.1)
            print(table.pose)
            
            #calc_dist_and_go(table.pose)
    
            pnp.jmove_to_pose_goal(table.pose)
            rospy.sleep(0.1)
            i+=1     

   




        pnp.jmove_to_joint_goal([1.57,-2.27,1.93,-1.19,-1.57,0]) #home pose
#
        #pnp.jmove_to_pose_goal(list_to_pose([0.35, -0.10, 0.48, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.35, -0.10, 0.18, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.35, -0.10, 0.28, 0, -pi, -pi/2]))
#
#
        #pnp.jmove_to_pose_goal(list_to_pose([0.65, -0.10, 0.48, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.65, -0.10, 0.18, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.65, -0.10, 0.28, 0, -pi, -pi/2]))
#
#
        #pnp.jmove_to_pose_goal(list_to_pose([0.35, 0.10, 0.48, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.35, 0.10, 0.18, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.35, 0.10, 0.28, 0, -pi, -pi/2]))
#
#
        #pnp.jmove_to_pose_goal(list_to_pose([0.65, 0.10, 0.48, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.65, 0.10, 0.18, 0, -pi, -pi/2]))
        #pnp.tmove_to_pose_goal(list_to_pose([0.65, 0.10, 0.28, 0, -pi, -pi/2]))
#
        #pnp.jmove_to_joint_goal([0, 0, -pi/2, 0, -pi/2, 0])

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("============ Python pick place demo complete!")
if __name__ == '__main__':
  main()