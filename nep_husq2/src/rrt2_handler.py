#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

import matplotlib.pyplot as plt
#from matplotlib.pyplot import plot, draw, show

from PIL import Image
from math import atan2
import numpy as np
import math
import sys

import rrt_test2
import rrt_test

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class rrt_handler :
    def __init__(self):
        self.ogrid = None
        self.ogrid_origin = None
        self.ogrid_cpm = None
        self.rostime = lambda: rospy.Time.now().to_sec()

        self.counter = 0

        # Create vector
        self.lin = np.linspace(0.0, 100.0, num=10)
        self.vec = np.transpose(np.array([self.lin, self.lin, np.zeros(self.lin.size)]))
        #vec = np.transpose(np.matrix([np.power(lin,2), lin, np.zeros(lin.size)]))
        print("Shape vec: ", self.vec.shape)

        self.x_scat = np.array([])
        self.y_scat = np.array([])

        # Subscriber
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.ogrid_callback)
        #sub_map_meta = rospy.Subscriber("/slam_out_pose", MapMetaData, ogrid_emta_callback)

        # Publisher
        self.markerArray = MarkerArray()
        self.topic = 'visualization_marker_array'
        self.publisher = rospy.Publisher(self.topic, MarkerArray, queue_size=1)


        #speed = Twist()
        self.rate = rospy.Rate(2)

        self.x_goal = 10
        self.y_goal = 10

        self.path = None
        self.path_shape = None



    def ogrid_callback(self, msg):

            self.flag = 1;
            self.counter = self.counter + 1

            start = self.rostime()
            self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            print("Shape ogrid", self.ogrid.shape)
            print("Ogrid patch", self.ogrid[0:50, 0:50])


            self.ogrid[self.ogrid < 0] = 50
            print(self.ogrid[100:150,100:150])

            RRT = rrt_test2.rrt(self.ogrid)
            #RRT = rrt_test.rrt()
            RRT.build_tree()
            """
            plt.figure(1)
            imgplot = plt.imshow(self.ogrid)
            plt.show(block=False)
            plt.figure(2)
            """
            path = RRT.get_path()
            path_shape = path.shape
            path = path.reshape(path_shape[0]/2,2)
            path = path*0.01
            print("PATH", path)
            print("Path shape", path_shape[0])
            self.path = path
            self.path_shape = path_shape

            for i in range(path_shape[0]/2) :
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.SPHERE
                #marker.type = marker.LINE_STRIP
                marker.action = marker.ADD
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0 # make it red
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = path[i, 0]
                marker.pose.position.y = path[i, 1]
                marker.pose.position.z = 0
                self.markerArray.markers.append(marker)

                print("append marker")

            id = 0
            for m in self.markerArray.markers:
                m.id = id
                id += 1

            self.publisher.publish(self.markerArray)
            self.markerArray = MarkerArray()

                #rospy.sleep(0.01)


            """
            plt.scatter(path[:,0], path[:,1], s=150, c='k', marker='o',)
            plt.show(block=False)
            """


            self.ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
            self.ogrid_cpm = 1 / msg.info.resolution

            try:
                print("TRY")
            except:
                print("\n(WARNING: something went wrong in reevaluate_plan)\n")
            elapsed = abs(self.rostime() - start)
            if elapsed > 1:
                print("\n(WARNING: ogrid callback is taking {} seconds)\n".format(np.round(elapsed, 2)))


if __name__ == "__main__":
    rospy.init_node("rrt_handler_node")
    RRT_handler = rrt_handler()

    print("Starting")

    while not rospy.is_shutdown() :
        print("Running")

        """
        self.counter = self.counter + 1
        if self.counter > 10 :
            self.markerArray.markers.pop(0)
            self.counter = 0

        self.publisher.publish(self.markerArray)
        """
        """
        try :
            print("Path shape main", RRT_handler.path_shape[0])
            for i in range(RRT_handler.path_shape[0]/2) :
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
                marker.pose.position.x = RRT_handler.path[i, 0]
                marker.pose.position.y = RRT_handler.path[i, 1]
                marker.pose.position.z = 0
                self.markerArray.markers.append(marker)

            RRT_handler.publisher.publish(RRT_handler.markerArray)

        except :
            print("No path")
        """

        """
        RRT_handler.counter = RRT_handler.counter + 1
        if RRT_handler.counter > 10 :
            RRT_handler.markerArray.markers.pop(0)
            RRT_handler.counter = 0

        RRT_handler.publisher.publish(RRT_handler.markerArray)
        """

        RRT_handler.rate.sleep()
