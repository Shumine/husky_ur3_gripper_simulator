#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2,pi,sqrt

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

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)


goal_x_dist = 3 #meter
deg_to_goal = 90 #degree
angle_to_goal = (pi/180)*deg_to_goal
goal_distance = 0


sum_vel = 0

r.sleep()

print 'current_th :' ,theta

while abs(theta-angle_to_goal) >0.01:
    
    if abs(theta-angle_to_goal) < 0.2:
        speed.angular.z = 0.02
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.2
    
    pub.publish(speed)
    r.sleep()
    print "theta:",theta,"goal_th:",angle_to_goal

    
init_x = x
init_y = y

while abs(goal_x_dist - goal_distance) >0.01:

    current_dist = abs(sqrt((x-init_x) ** 2 + (y-init_y) ** 2))
    goal_distance = current_dist

    if abs(goal_x_dist - goal_distance) < 0.2:
        speed.linear.x = 0.02
        speed.angular.z = 0
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0
    

    pub.publish(speed)
    r.sleep()
    goal_distance = current_dist
    print "distance:",goal_distance





 