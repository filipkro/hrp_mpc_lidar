#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Create vector
vector = np.zeros([10,3])
print("Shape relative_ref: ", vector.shape)

x_odom = 0.0
y_odom = 0.0

# Callbak function
def newOdom(msg) :
    global x_odom
    global y_odom
    global theta_odom

    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y
    print(x_odom)

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_odom) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #theta = yaw;

#rate = rospy.Rate(10)

# Subscriber
sub = rospy.Subscriber("/odom", Odometry, newOdom)

# Publisher
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)


rospy.init_node('register')
markerArray = MarkerArray()

count = 0
MARKERS_MAX = vector.shape[0]

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

for i in range(10) :
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x_odom
    marker.pose.position.y = y_odom
    marker.pose.position.z = 0
    markerArray.markers.append(marker)

while not rospy.is_shutdown():

   marker.pose.orientation.w = 1.0
   marker.pose.position.x = x_odom
   marker.pose.position.y = y_odom
   marker.pose.position.z = 0


   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
   #if(count > MARKERS_MAX):
   #    markerArray.markers.pop(0)

   markerArray.marke§rs.append(marker)
   markerArray.markers.pop(0)

   # Renumber the marker IDs
   id = 0
   for m in markerArray.markers:
       m.id = id
       id += 1

   # Publish the MarkerArray
   publisher.publish(markerArray)

   count += 1
   count = count % MARKERS_MAX


   #rate.sleep()
   rospy.sleep(0.1)
