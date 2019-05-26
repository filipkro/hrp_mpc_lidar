#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as euler_fq
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import solver_interface as si
from math import atan
from math import atan2
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from math import floor

si.set_defaults()
si.setup_indexing()

x = 0.0
y = 0.0
theta = 0.0

u_0 = [0.0, 0.0]

x_odom = 0.0
y_odom = 0.0
theta_odom = 0.0

xValue = [0.0, 0.0, 0.0, 0.0, 0.0]
yValue = [0.0, 0.0, 0.0, 0.0, 0.0]
thetaValue = [0.0, 0.0, 0.0, 0.0, 0.0]

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

rospy.init_node("position_controller")

sub_pos = rospy.Subscriber("/slam_out_pose", PoseStamped, newPos)
sub = rospy.Subscriber("/odom", Odometry, newOdom)
sub_path = rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, path_cb)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

h = 1

MARKERS_MAX = 100
count = 0

#PATH
#funkar:
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
for i in range(5):
    np.append(xref_long, x_len)
   # xref_long.append(x_len)
yref_long = 0*xref_long



# xref_long = np.linspace(0, x_len, run_time/h)
# yref_long = []
# for i in range(len(xref_long)):
#     yref_long.append(0.25*sigmoid(10*(xref_long[i]-x_len/2)))
# # diff = yref_long[0]

# for i in range(len(xref_long)):
#     yref_long.append(0.25-0.25*sigmoid(10*(xref_long[i]-x_len/2)))

# for i in range(10):
#     yref_long.insert(0,0)

# xref_long = np.linspace(0, 2*x_len+10, (2*run_time+10)/h)

#PATH END

rate = rospy.Rate(1/h)

#Reference
# t = np.linspace(0,2*3.14,100/h)
# xref_long = 1*np.cos(t) - 1
# yref_long = 1*np.sin(t)

# xref_long = np.linspace(0,0.5,100)
# yref_long = 0 * xref_long

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)
markerArray = MarkerArray()

# Set parameters
u_0 = [0.0, 0.0]

#Q
si.set_parameters(1,0,1000.0)
si.set_parameters(1,4,1000.0)
si.set_parameters(1,8,0.0)

#R
si.set_parameters(2,0,1.0)
si.set_parameters(2,3,1000.0)

#u_max
si.set_parameters(5,0,0.5)
si.set_parameters(5,1,1.0)

#deltau_max
si.set_parameters(12,0,0.2)
si.set_parameters(12,1,0.2)

while not rospy.is_shutdown():
    print "x_odom: " + str(x_odom)
    print "y_odom: " + str(y_odom)
    print "theta_odom" + str(theta_odom)

    print "x: " + str(x)
    print "y: " + str(y)
    print "theta" + str(theta)

    x_used = x_odom
    y_used = y_odom
    theta_used = theta_odom

    print "xValue: " + str(xValue)
    print "yValue: " + str(yValue)

    print "xLen: " + str(len(xValue))
    print "yLen: " + str(len(yValue))
    #viz pos & path Rviz

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

    

     # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
    if(count > MARKERS_MAX):
        markerArray.markers.pop(0)

    markerArray.markers.append(marker)

   # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
    
   # Publish the MarkerArray
    publisher.publish(markerArray)

    count += 1
    try :
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

        #cnt = 0
        # while xref_long & cnt < 5:
        #     xref[cnt] = xref_long.pop(0)
        #     yref[cnt] = yref_long.pop(0)
        #     cnt += 1
        if len(xref_long) < 5 | len(xValue) < 5:
            si.set_parameters(13,0,u_0[0])


            si.set_parameters(7,0,x_used)
            si.set_parameters(7,1,y_used)
            si.set_parameters(7,2,0.0)

            si.set_parameters(8,0,x_used)
            si.set_parameters(8,1,y_used)
            si.set_parameters(8,2,0.0)

            si.set_parameters(9,0,x_used)
            si.set_parameters(9,1,y_used)
            si.set_parameters(9,2,0.0)

            si.set_parameters(10,0,x_used)
            si.set_parameters(10,1,y_used)
            si.set_parameters(10,2,0.0)

            si.set_parameters(11,0,x_used)
            si.set_parameters(11,1,y_used)
            si.set_parameters(11,2,0.0)

            si.set_parameters(2,3,0.0)

            si.solve()


        else:
            # if len(xValue) > 49:
            #     xref = xValue[0:50:10]
            #     yref = yValue[0:50:10]
            # else:
            
            xref = xValue[0:len(xValue):floor(len(xValue)/5)]
            yref = yValue[0:len(yValue):floor(len(yValue)/5)]
            print "287"
            print "xref: " + str(xref)
            print "yref: " + str(yref) 
            # xref_long = np.delete(xref_long, 0)
            # yref_long = np.delete(yref_long, 0)
            # deltay = yref[4] - y_used
            # deltax = xref[4] - x_used
            # print "deltax: " + str(deltax)
            # print "deltay: " + str(deltay)
            theta_ref = thetaValue[0:5]
            
            #= atan2(deltay, deltax)

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
            
            si.solve()



        cont = np.array(si.get_control_horizon())
        u1 = np.array(cont[::2])
        u2 = np.array(cont[1::2])

        speed.linear.x = u1[0]
        speed.angular.z = u2[0]

        u_0 = [u1[0], u2[0]]
        print "u1, u2: " + str(u_0)
    
        # print "xref: " + str(xref)
        # print "yref: " + str(yref)


    except (ValueError, TypeError):
        print "except"
        speed.linear.x = 0.0
        speed.angular.z = 0.0

    # speed.linear.x = 0.0
    # speed.angular.z = 0.0
    pub.publish(speed)
    rate.sleep()
