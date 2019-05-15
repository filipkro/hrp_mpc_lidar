#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point;
from geometry_msgs.msg import Twist
from math import atan2

# Store the current position of robot in intertial frame

x = 0.0
y = 0.0
theta = 0.0

# Callbak function
def newOdom(msg) :
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #theta = yaw;

rospy.init_node("speed_controller")

#sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
sub = rospy.Subscriber("/odom", Odometry, newOdom)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

rate = rospy.Rate(4)

goal = Point()
goal.x = 4.48;
goal.y = 0.92;


while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x);

    print(theta)

    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0
        print("else")

    print("x: ", x)
    print("y: ", y)
    
    pub.publish(speed)
    rate.sleep()
