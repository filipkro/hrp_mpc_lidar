#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from math import atan2

# Store the current position of robot in intertial frame

x = 0.0
y = 0.0
theta = 0.0
x_odom = 0.0
y_odom = 0.0

# Callbak function
def newGoal(msg) :

    print("newGoal callback")
    global x_goal
    global y_goal
    global theta_goal

    x_goal = msg.pose.position.x
    y_goal = msg.pose.position.y

    rot_q = msg.pose.orientation
    (roll_goal, pitch_goal, theta_goal) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #theta = yaw;

# Callbak function
def newPos(msg) :

    print("newPos Callback")
    global x
    global y
    global theta

    x = msg.pose.position.x
    y = msg.pose.position.y

    rot_q = msg.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #theta = yaw;

# Callbak function
def newOdom(msg) :
    global x_odom
    global y_odom
    global theta_odom

    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_odom) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #theta = yaw;

rospy.init_node("position_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, newGoal)
sub_pos = rospy.Subscriber("/slam_out_pose", PoseStamped, newPos)
sub = rospy.Subscriber("/odom", Odometry, newOdom)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

rate = rospy.Rate(4)

goal = Point()
goal.x = 4.48;
goal.y = 0.92;


while not rospy.is_shutdown():

    print("x: ", x_odom)
    print("y: ", y_odom)

    try :
        print("try")
        speed.linear.x = 0.0
        speed.angular.z = 0.1
        print("Theta", theta)

    except :
        print("except")
        speed.linear.x = 0.0
        speed.angular.z = 0.0

    pub.publish(speed)
    rate.sleep()
