#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as euler_fq


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


# Callbak function
def wheelEnc_callback(msg) :
    global lWheel
    global rWheel
    print "wheel encoder callback"

    lWheel = msg.lWheel
    rWheel = msg.rWheel

    lwheelAccum = msg.lWheelAccum
    rwheelAccum = msg.rWheelAccum

    print("lwheelAccum: ", lwheelAccum)
    print("rwheelAccum: ", rwheelAccum)

rospy.init_node("ekf")
sub_wheelEnc = rospy.Subscriber("/wheel_encoder", WheelEncoder, wheelEnc_callback)
rate = rospy.Rate(1/h)
