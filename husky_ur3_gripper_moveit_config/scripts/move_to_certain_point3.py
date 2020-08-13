#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

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
    rospy.init_node("speed_controller")

    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    speed = Twist()

    r = rospy.Rate(4)


    speed.linear.x = a
    speed.angular.z = b   
       
      
    pub.publish(speed)
    r.sleep()    


if __name__=='__main__':
    
    
    i = 0
    while(True):
        
        x_val = 0.02*i
        yaw_val = 0.01 *i
        move_base(x_val,yaw_val)

        i +=1

        print i 