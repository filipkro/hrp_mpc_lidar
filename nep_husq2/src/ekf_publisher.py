#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from am_driver.msg import WheelEncoder
import ekf
from geometry_msgs.msg import PoseStamped
import numpy as np

x = 0.0
y = 0.0
theta = 0.0

lwheel = 0.0
rwheel = 0.0
lwheelAccum = 0.0
rwheelAccum = 0.0

lwheelAccumPrev = 0.0
rwheelAccumPrev = 0.0

# Callbak function
def newPos(msg) :

    #print("newPos Callback")
    global x
    global y
    global theta

    x = msg.pose.position.x
    y = msg.pose.position.y

    rot_q = msg.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


# Callbak function
def wheelEnc_callback(msg) :
    global lwheel
    global rwheel
    global lwheelAccum
    global rwheelAccum
    global lwheelAccumPrev
    global rwheelAccumPrev
    #print "wheel encoder callback"

    #lwheel = msg.lwheel
    #rwheel = msg.rwheel

    lwheelAccum = msg.lwheelAccum
    rwheelAccum = msg.rwheelAccum

    #print("lwheelAccum: ", lwheelAccum)
    #print("rwheelAccum: ", rwheelAccum)

# Callbak function
def ekf_callback(msg) :

    print "ekf callback"

    pose_point.pose.position.x = m[0]
    pose_point.pose.position.y = m[1]

    print("x-estimate: ", pose_point.position.x)
    print("y-estimate: ", pose_point.position.y)


rospy.init_node("ekf")
sub_pos = rospy.Subscriber("/slam_out_pose", PoseStamped, newPos)
sub_wheelEnc = rospy.Subscriber("/wheel_encoder", WheelEncoder, wheelEnc_callback)
pub_ekf = rospy.Publisher("/ekf_estimate", PoseStamped, queue_size=1)
h = 1/25.0
rate = rospy.Rate(1/h)
pose_point = PoseStamped()

Q = np.diag([0.001, 0.001, 0.01, 0.01, 0.01])
#Qsq = h*sl.cholesky(Q)
R = np.diag([0.01, 0.01, 1.0, 1.0])
#Rsq = sl.cholesky(R)

# EKF STUFF
my_ekf = ekf.ekf(Q,R)
## Run the estimation on the generated data
#m  = np.zeros((N,5,1))
#P  = np.zeros((N,5,5))

m0 = np.zeros((5,1))+1.0
P0 = 0.1*np.eye(5)
yk0= np.zeros((4,1))
yk = np.zeros((4,1))

mk, Pk = my_ekf.filter_update(yk, m0, P0)
# END EKF STUFF

while not rospy.is_shutdown():
    print("ekf'in")

    yk[0,0] = x
    yk[1,0] = y
    yk[2,0] = (rwheelAccum - rwheelAccumPrev)/h
    yk[3,0] = (lwheelAccum - lwheelAccumPrev)/h


    mk, Pk = my_ekf.filter_update(yk, mk, Pk)
    pose_point.header.frame_id = "/map"
    pose_point.pose.position.x = mk[0,0]
    pose_point.pose.position.y = mk[1,0]

    (pose_point.pose.orientation.x, pose_point.pose.orientation.y, pose_point.pose.orientation.z, pose_point.pose.orientation.w) = \
     quaternion_from_euler(0.0, 0.0, mk[2,0])
    print("THETA", mk[2,0])


    print("x-estimate: ", pose_point.pose.position.x)
    print("y-estimate: ", pose_point.pose.position.y)


    #m[k] = mk
    #P[k] = Pk

    pub_ekf.publish(pose_point)

    lwheelAccumPrev = lwheelAccum
    rwheelAccumPrev = rwheelAccum

    rate.sleep()
