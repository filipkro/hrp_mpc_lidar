#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

rospy.init_node('register')

markerArray = MarkerArray()


# Create vector
lin = np.linspace(0.0, 1.0, num=100)
relative_ref = np.transpose(np.matrix([np.power(lin,2), lin, np.zeros(lin.size)]))
print("Shape relative_ref: ", relative_ref.shape)
print("Lin size: ", lin.size)

count = 0
MARKERS_MAX = lin.size - 1

while not rospy.is_shutdown():

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
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = relative_ref[count, 0]
   marker.pose.position.y = relative_ref[count, 1]
   marker.pose.position.z = relative_ref[count, 2]

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
   count = count % MARKERS_MAX

   rospy.sleep(0.01)
