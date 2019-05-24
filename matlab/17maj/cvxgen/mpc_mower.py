#!/usr/bin/env python#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as euler_fq
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import solver_interface as si
from math import atan2

print "12"

si.set_parameters()
si.setup_indexing()

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

h = 0.1
rate = rospy.Rate(1/h)

#Reference     Kolla sign på pos - ref[0] & pos - ref[1] ? om samma; behåll ref annars flytta index??
#t = np.linspace(0,6.28,100/h)
# xref = np.cos(t)
# yref = np.sin(t)

t = np.linspace(0,10,100/h)
yref = t * 0
for i in range(10):
    yref[10+i] = 0.1*i
    yref[30-i] = 0.1*i


# Set parameters
u_0 = [0.0, 0.0]

#Q
si.set_parameters(1,0,100.0)
si.set_parameters(1,4,100.0)
si.set_parameters(1,8,10.0)

#R
si.set_parameters(2,0,1.0)
si.set_parameters(2,3,1.0)

#u_max
si.set_parameters(5,0,10.0)
si.set_parameters(5,1,2.0)

#deltau_max
si.set_parameters(12,0,1.0)
si.set_parameters(12,1,1.0)

while not rospy.is_shutdown():
    #x_0
    si.set_parameters(0,0,x)
    si.set_parameters(0,1,y)
    si.set_parameters(0,2,theta)

    #u_prev
    si.set_parameters(13,0,u_0[0])
    si.set_parameters(13,1,u_0[1])

    A = np.array([[1.0,0.0,-u_0[0]*h*np.sin(theta)], [0.0,1.0,u_0[0]*h*np.cos(theta)], [0.0,0.0,1.0]])
    for i in range(3):
        for j in range(3):
            si.set_parameters(3,i+3*j,float(A[i,j]))

    B = np.array([[h*np.cos(theta),-u_0[0]*np.sin(theta)*0.5*h**2], [h*np.sin(theta), u_0[0]*np.cos(theta)*0.5*h**2], [0,h]])
    for i in range(3):
        for j in range(2):
            si.set_parameters(4,i+3*j,float(B[i,j]))
    
    try :
        
        deltay = yref[5] - y
        deltax = xref[5] - x
        theta_ref = atan2(deltay, deltax)

        si.set_parameters(7,0,xref[0])
        si.set_parameters(7,1,yref[0])
        si.set_parameters(7,2,theta_ref)

        si.set_parameters(8,0,xref[1])
        si.set_parameters(8,1,yref[1])
        si.set_parameters(8,2,theta_ref)

        si.set_parameters(9,0,xref[2])
        si.set_parameters(9,1,yref[2])
        si.set_parameters(9,2,theta_ref)

        si.set_parameters(10,0,xref[3])
        si.set_parameters(10,1,yref[3])
        si.set_parameters(10,2,theta_ref)

        si.set_parameters(11,0,xref[4])
        si.set_parameters(11,1,yref[4])
        si.set_parameters(11,2,theta_ref)

        si.solve()

        contr = np.array(si.get_control_horizon())
        u1 = np.array(cont[::2])
        u2 = np.array(cont[1::2])

        speed.linear.x = u1[0]
        speed.angular.z = u2[0]

        u_0 = [u1[0], u2[0]]

    except:
        print "except"
        speed.linear.x = 0.0
        speed.angular.z = 0.0

    pub.publish(speed)
    rate.sleep()
