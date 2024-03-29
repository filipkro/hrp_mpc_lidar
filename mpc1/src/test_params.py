#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as euler_fq
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import atan2


x = 0.0
y = 0.0
theta = 0.0

u_0 = [0.0, 0.0]

x_odom = 0.0
y_odom = 0.0


# Callbak function
def newPos(msg) :

    print("newPos Callback")
    global x
    global y
    global theta

    x = msg.pose.position.x
    y = msg.pose.position.y

    rot_q = msg.pose.orientation
    (roll, pitch, theta) = euler_fq([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #theta = yaw;

# Callbak function
def newOdom(msg) :
    global x_odom
    global y_odom
    global theta_odom

    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_odom) = euler_fq([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #theta = yaw;

rospy.init_node("position_controller")

sub_pos = rospy.Subscriber("/slam_out_pose", PoseStamped, newPos)
sub = rospy.Subscriber("/odom", Odometry, newOdom)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

h = 1
rate = rospy.Rate(1/h)

while not rospy.is_shutdown():
    try:
        # if x > 2:
        #     speed.linear.x = 0.0
        # else:
        #     
        speed.linear.x = 0.0
        speed.angular.z = 1.0
    except(ValueError, TypeError):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    pub.publish(speed)
    rate.sleep()