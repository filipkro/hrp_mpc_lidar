#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion as euler_fq
from geometry_msgs.msg import Point, Twist, PointStamped, PoseStamped
import numpy as np
import solver_interface as si
from math import atan, atan2, cos, sin
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray

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

# Callbak function - for path
def path_cb(msg) :
    global xValue
    global yValue
    global thetaValue
    xValue = []
    yValue = []
    thetaValue = []
    for i in msg.poses:
        xValue.append(i.pose.position.x)
        yValue.append(i.pose.position.y)
        (roll, pitch, theta_path) = euler_fq([i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w])
        thetaValue.append(theta_path)

# SETUP

si.set_defaults()
si.setup_indexing()

x = 0.0
y = 0.0
theta = 0.0

u_0 = [0.0, 0.0]

x_odom = 0.0
y_odom = 0.0
theta_odom = 0.0

xref = [0.0, 0.0, 0.0, 0.0, 0.0]
yref = [0.0, 0.0, 0.0, 0.0, 0.0]
theta_ref = [0.0, 0.0, 0.0, 0.0, 0.0]

xValue = []
yValue = []
thetaValue = []

h = 1

#Q
si.set_parameters(1,0,10000.0)
si.set_parameters(1,4,10000.0)
si.set_parameters(1,8,0.0)

#R
si.set_parameters(2,0,1.0)
si.set_parameters(2,3,10.0)

#u_max
si.set_parameters(5,0,0.5)
si.set_parameters(5,1,1.0)

#deltau_max
si.set_parameters(12,0,0.2)
si.set_parameters(12,1,0.2)


# ROS SETUP
sub_pos = rospy.Subscriber("/slam_out_pose", PoseStamped, newPos)
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
mark_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
sub_path = rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, path_cb)

rospy.init_node("position_controller")

markerArray = MarkerArray()
speed = Twist()
rate = rospy.Rate(1/h)

# PATH

# x_len = 6
# run_time = 40
# temp = np.linspace(0, x_len, run_time/h)
# xref_long = np.array(temp)
# for i in range(5):
#     np.append(xref_long, x_len)
#    # xref_long.append(x_len)
# yref_long = 0*xref_long

# for i in range(8):
#     yref_long[i+5] = 0.15*i
#     yref_long[34-i] = 0.15*i
# yref_long[13:27] = 0.15*8


# x_len = 1
# run_time = 40
# xref_long = np.array(np.linspace(0, x_len, run_time/h))
# yref_long = 0 * xref_long
# for i in range(len(xref_long)):
#     yref_long[i] = xref_long[i] * xref_long[i]

#     marker = Marker()
#     marker.header.frame_id = "/map"
#     marker.type = marker.SPHERE
#     marker.action = marker.ADD
#     marker.scale.x = 0.05
#     marker.scale.y = 0.05
#     marker.scale.z = 0.05
#     marker.color.a = 1.0
#     marker.color.r = 0.0 
#     marker.color.g = 0.0
#     marker.color.b = 1.0 # make it blue

#     marker.pose.orientation.w = 0.0
#     marker.pose.position.x = xref_long[i]
#     marker.pose.position.y = yref_long[i]

#     markerArray.markers.append(marker)

# id = 0
# for m in markerArray.markers:
#     m.id = id
#     id += 1

# mark_pub.publish(markerArray)




while not rospy.is_shutdown():
    try:
        x_used = x_odom
        y_used = y_odom
        theta_used = theta_odom

        # PLOT TRAVELLED PATH RVIZ

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0 # make it red
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.orientation.w = theta_used
        marker.pose.position.x = x_used
        marker.pose.position.y = y_used

        markerArray.markers.append(marker)
        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        mark_pub.publish(markerArray)

        # UPDATE PARAMS

        #x_0
        si.set_parameters(0,0,x_used)
        si.set_parameters(0,1,y_used)
        si.set_parameters(0,2,theta_used)

        #u_prev
        si.set_parameters(13,0,u_0[0])
        si.set_parameters(13,1,u_0[1])

        A = np.array([[1.0,0.0,-u_0[0]*h*np.sin(theta_used)], [0.0,1.0,u_0[0]*h*np.cos(theta_used)], [0.0,0.0,1.0]])
        for i in range(3):
            for j in range(3):
                si.set_parameters(3,i+3*j,float(A[i,j]))
        
        B = np.array([[h*np.cos(theta_used),-u_0[0]*np.sin(theta_used)*0.5*h**2], [h*np.sin(theta_used), u_0[0]*np.cos(theta_used)*0.5*h**2], [0,h]])
        for i in range(3):
            for j in range(2):
                si.set_parameters(4,i+3*j,float(B[i,j]))

        if len(xValue) < 5:
            print "Finished"
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            u_0 = [0.0, 0.0]
        else:
            xref[0] = xValue[len(xValue)/3]
            xref[1] = xValue[2*len(xValue)/3]
            yref[0] = yValue[len(yValue)/3]
            yref[1] = yValue[2*len(yValue)/3]
            xref[2] = xValue[len(xValue)-1]
            yref[2] = yValue[len(yValue)-1]
            theta_ref[0] = thetaValue[len(xValue)/3]
            theta_ref[1] = thetaValue[2*len(xValue)/3]
            theta_ref[2] = thetaValue[len(thetaValue)-1]
            for ii in range(2):
                xref[ii+3] = xref[ii+2] + 0.1*cos(theta_ref[2])
                yref[ii+3] = yref[ii+2] + 0.1*sin(theta_ref[2])

            print "xref: " + str(xref)
            print "yref: " + str(yref)
            print "theta_ref: " + str(theta_ref)
            # xref_long = np.delete(xref_long, 0)
            # yref_long = np.delete(yref_long, 0)
            # for angle - when calculated
            deltay = yref[4] - y_used
            deltax = xref[4] - x_used
            # theta_ref = atan2(deltay, deltax)
            # when angle from reference
            #theta_ref = ...

            # SET PATH FOR CVXGEN
            si.set_parameters(7,0,xref[0])
            si.set_parameters(7,1,yref[0])
            si.set_parameters(7,2,theta_ref[0])

            si.set_parameters(8,0,xref[1])
            si.set_parameters(8,1,yref[1])
            si.set_parameters(8,2,theta_ref[1])

            si.set_parameters(9,0,xref[2])
            si.set_parameters(9,1,yref[2])
            si.set_parameters(9,2,theta_ref[2])

            si.set_parameters(10,0,xref[3])
            si.set_parameters(10,1,yref[3])
            si.set_parameters(10,2,theta_ref[3])

            si.set_parameters(11,0,xref[4])
            si.set_parameters(11,1,yref[4])
            si.set_parameters(11,2,theta_ref[4])

            # SOLVE OPTIMIZATION
            si.solve()

            # get control signals
            cont = np.array(si.get_control_horizon())
            u1 = np.array(cont[::2])
            u2 = np.array(cont[1::2])

            speed.linear.x = u1[0]
            speed.angular.z = u2[0]

            u_0 = [u1[0], u2[0]]

    except (ValueError, TypeError):
        print "except"
        speed.linear.x = 0.0
        speed.angular.z = 0.0

    print "u1: " + str(speed.linear.x)
    print "u2: " + str(speed.angular.z)

    pub_vel.publish(speed)
    rate.sleep()    
