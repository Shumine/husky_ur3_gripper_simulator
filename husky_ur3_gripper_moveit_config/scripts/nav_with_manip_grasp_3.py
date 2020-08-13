#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
import std_msgs.msg
from moveit_commander.conversions import pose_to_list, list_to_pose

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

def main():
    try:
        pnp = PickNPlaceTutorial()
        pnp.jmove_to_joint_goal([1.57,-2.27,1.93,-1.19,-1.57,0]) #home pose

        table = geometry_msgs.msg.PoseStamped()
        table.header.frame_id = "odom"
        table.pose.position.x =0.75
        table.pose.position.y =0.1
        table.pose.position.z =0.7
        table.pose.orientation.w = 1                          
        pnp.add_box(name='table', pose_stamp=table, size=(0.1, 0.1, 0.3))
        

        
        i = 0
        while(i<10):
            
            test_pose =  list_to_pose([0.75,0.1,0.7,0,9*pi/180*i,0])

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
            pnp.jmove_to_pose_goal(table.pose)
            rospy.sleep(0.1)
            i+=1

        i = 0
        while(i<10):
            
            test_pose =  list_to_pose([0.75,0.1,0.7,0,pi/2-i*9*pi/180,0])

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
            pnp.jmove_to_pose_goal(table.pose)
            rospy.sleep(0.1)
            i+=1


        i = 0
        while(i<5):
            
            test_pose =  list_to_pose([0.75+0.01*i,0.1,0.7,0,0,0])

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
            pnp.jmove_to_pose_goal(table.pose)
            rospy.sleep(0.1)
            i+=1        

        i = 0
        while(i<5):
            
            test_pose =  list_to_pose([0.85-0.01*i,0.1,0.7,0,0,0])

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
            pnp.jmove_to_pose_goal(table.pose)
            rospy.sleep(0.1)
            i+=1     

        i = 0
        while(i<5):
            
            test_pose =  list_to_pose([0.75,0.1+0.01*i,0.7,0,0,0])

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
            pnp.jmove_to_pose_goal(table.pose)
            rospy.sleep(0.1)
            i+=1      
        i = 0
        while(i<10):
            
            test_pose =  list_to_pose([0.75,0.15-0.01*i,0.7,0,0,0])

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
            pnp.jmove_to_pose_goal(table.pose)
            rospy.sleep(0.1)
            i+=1   
        i = 0
        while(i<5):
            
            test_pose =  list_to_pose([0.75,0.05+0.01*i,0.7,0,0,0])

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