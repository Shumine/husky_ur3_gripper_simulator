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

rospy.init_node("speed_controller")


def move_base(a,b):             #x,y좌표를 받아와서 그곳으로 platform을 움직이는 코드 theta는 x와 y좌표에 의해 정해짐.(원점기준)
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b
    goal_angle = atan2(goal.y,goal.x)

    while abs(goal.x-x) >0.1 or abs(goal.y-y) >0.1 or abs(goal_angle-theta) >0.1 : #가까의 범위가 0.1이내로 들어오면 break.
        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            if abs(angle_to_goal - theta) < 0.5:        # 0.5이내로 들어오면 속도를 매우 줄여서 목표점을 지나쳐버리는 일이 없도록함.
                speed.angular.z = 0.05
        
        else:
            speed.linear.x = 0.2
            speed.angular.z = 0.0
            if abs(goal.x-x) <0.5 and abs(goal.y-y):
                speed.angular.x = 0.05

        print goal.x-x, goal.y-y, goal_angle-theta

        pub.publish(speed)
        r.sleep()    


if __name__=='__main__':
    move_base(1,1)
    print 'complete!'
    rospy.sleep(1)
    move_base(1,2)
    print 'complete!'
    rospy.sleep(1)
    move_base(3,3)
    print 'complete!'
    rospy.sleep(1)
