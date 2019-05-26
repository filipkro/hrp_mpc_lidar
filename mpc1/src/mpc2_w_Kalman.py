#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as euler_fq
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import solver_interface as si
from math import atan
from math import atan2
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy

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

# Jacobians of the flow
def get_flow_jacobians(m, h, r, d):
    # Unpack the state
    x, y, t, p1, p2 = m[0,0],m[1,0],m[2,0],m[3,0],m[4,0]
    #Evaluate the drift term 
    f = np.array([
        [x + (h*r*cos(t)*(p1 + p2))/2.0],
        [y + (h*r*sin(t)*(p1 + p2))/2.0],
        [   t + (h*r*(p1 - p2))/(2.0*d)],
        [                            p1],
        [                            p2]])
    # Compute continuous time jacobians
    dfdx = np.array([
        [ 1.0, 0, -h*(r*sin(t)*(p1 + p2))/2.0, h*(r*cos(t))/2.0, h*(r*cos(t))/2.0],
        [ 0, 1.0,  h*(r*cos(t)*(p1 + p2))/2.0, h*(r*sin(t))/2.0, h*(r*sin(t))/2.0],
        [ 0,   0,                         1.0,      h*r/(2.0*d),     -h*r/(2.0*d)],
        [ 0,   0,                           0,              1.0,                0],
        [ 0,   0,                           0,                0,            1.0]])
    # Compute hessians of f[0] and f[1], all other f[n]=0
    ddfddx1 = np.array([
        [ 0, 0,                           0,                 0,                 0],
        [ 0, 0,                           0,                 0,                 0],
        [ 0, 0, -(h*r*cos(t)*(p1 + p2))/2.0, -(h*r*sin(t))/2.0, -(h*r*sin(t))/2.0],
        [ 0, 0,           -(h*r*sin(t))/2.0,                 0,                 0],
        [ 0, 0,           -(h*r*sin(t))/2.0,                 0,                 0]])
    ddfddx2 = np.array([
        [ 0, 0,                           0,                 0,                 0],
        [ 0, 0,                           0,                 0,                 0],
        [ 0, 0, -(h*r*sin(t)*(p1 + p2))/2.0,  (h*r*cos(t))/2.0,  (h*r*cos(t))/2.0],
        [ 0, 0,            (h*r*cos(t))/2.0,                 0,                 0],
        [ 0, 0,            (h*r*cos(t))/2.0,                 0,                 0]])
    return f, dfdx, ddfddx1, ddfddx2


#Measurement jacobians
# Jacobians of the state - Behövs ej här??
def get_measurement_jacobians(m, c1, c2):
    # Unpack the state
    x, y, t, p1, p2 = m[0,0], m[1,0], m[2,0], m[3,0], m[4,0]
    #Evaluate the drift term 
    hx = np.array([
         [x],
         [y],
         [p1 / c1],
         [p2 / c2]])
    
    # Compute continuous time jacobians
    dhdx = np.array([
        [ 1.0,   0, 0,      0,      0],
        [   0, 1.0, 0,      0,      0],
        [   0,   0, 0, 1.0/c1,      0],
        [   0,   0, 0,      0, 1.0/c2]])
    return hx, dhdx

# UPDATE KALMAN FILTER
def filter_update(ym, m, P, h,r, d, c1, c2, Q, R):
    # Update the current state estimate
    #
    e1, e2, e3, e4, e5 = np.eye(5)
    e1 = np.reshape(e1,[5,1])
    e2 = np.reshape(e2,[5,1])
    
    # Prediction
    fx, dfdx, ddfddx1, ddfddx2 = get_flow_jacobians(m, h, r, d)

    Pf1 = np.dot(ddfddx1, P)
    Pf2 = np.dot(ddfddx2, P)
    F11 = 0.5*(np.trace(np.dot(Pf1,Pf1)))*np.dot(e1,e1.T)
    F12 = 0.5*(np.trace(np.dot(Pf1,Pf2)))*np.dot(e1,e2.T)
    F21 = 0.5*(np.trace(np.dot(Pf2,Pf1)))*np.dot(e2,e1.T)
    F22 = 0.5*(np.trace(np.dot(Pf2,Pf2)))*np.dot(e2,e2.T)
    F1  = 0.5*np.trace(Pf1)*e1
    F2  = 0.5*np.trace(Pf2)*e2
    
    mkf = fx + F1 + F2
    Pkf = np.dot(np.dot(dfdx, P), dfdx.T) + Q + F11 + F21 + F12 + F22

    # Correction
    e1, e2, e3, e4 = np.eye(4)
    e1 = np.reshape(e1,[4,1])
    e2 = np.reshape(e2,[4,1])

    hx, dhdx = get_measurement_jacobians(mkf, c1, c2)

    Ph1 = np.dot(ddfddx1, P)
    Ph2 = np.dot(ddfddx2, P)
    H1  = 0.5*np.trace(Ph1)*e1
    H2  = 0.5*np.trace(Ph2)*e2

    e = ym - (hx + H1 + H2)
    Sk = R + np.dot(np.dot(dhdx,Pkf),dhdx.T)
    Kk = np.dot(np.dot(Pkf, dhdx.T),sl.inv(Sk))
    IKC = (np.eye(5)-np.dot(Kk,dhdx))
    
    mk = mkf + np.dot(Kk,e)
    Pk = np.dot(np.dot(IKC,Pkf),IKC.T) + np.dot(np.dot(Kk,R),Kk.T)
    return mk, Pk


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

h = 1

d = 0.235    # half distance between the wheels (m)
r = 0.14      # radius of the wheel (m)

#For Kalman filter, process noise
Q   = np.diag([0.001, 0.001, 0.01, 0.01, 0.01])

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

x_len = 1
run_time = 40
xref_long = np.array(np.linspace(0, x_len, run_time/h))
yref_long = 0 * xref_long
for i in range(len(xref_long)):
    yref_long[i] = xref_long[i] * xref_long[i]

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0.0 
    marker.color.g = 0.0
    marker.color.b = 1.0 # make it blue

    marker.pose.orientation.w = 0.0
    marker.pose.position.x = xref_long[i]
    marker.pose.position.y = yref_long[i]

    markerArray.markers.append(marker)

id = 0
for m in markerArray.markers:
    m.id = id
    id += 1

mark_pub.publish(markerArray)




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

        if len(xref_long) < 5:
            print "Finished"
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            u_0 = [0.0, 0.0]
        else:
            xref = xref_long[0:5]
            yref = yref_long[0:5]
            print "xref: " + str(xref)
            print "yref: " + str(yref)
            xref_long = np.delete(xref_long, 0)
            yref_long = np.delete(yref_long, 0)
            # for angle - when calculated
            deltay = yref[4] - y_used
            deltax = xref[4] - x_used
            theta_ref = atan2(deltay, deltax)
            # when angle from reference
            #theta_ref = ...

            # SET PATH FOR CVXGEN
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
