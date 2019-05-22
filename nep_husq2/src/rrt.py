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
from matplotlib import animation

from PIL import Image
from math import atan2
import numpy as np
import math
import sys

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent


"""
def ogrid_callback(self, msg):

        #Expects an OccupancyGrid message.
        #Stores the ogrid array and origin vector.
        #Reevaluates the current plan since the ogrid changed.


        start = self.rostime()
        self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.ogrid_cpm = 1 / msg.info.resolution
        try:
            self.reevaluate_plan()
        except:
            # print("(WARNING: something went wrong in reevaluate_plan)")
        elapsed = abs(self.rostime() - start)
        if elapsed > 1:
            # print("(WARNING: ogrid callback is taking seconds)".format(np.round(elapsed, 2)))
"""
class rrt :
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

        #self.f1 = plt.figure()
        #self.f2 = plt.figure()
        #rospy.init_node("RRT")

        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.ogrid_callback)
        #sub_map_meta = rospy.Subscriber("/slam_out_pose", MapMetaData, ogrid_emta_callback)

        #speed = Twist()
        self.rate = rospy.Rate(2)

        self.flag = 0

        self.nodes = []

        self.x_goal = 10
        self.y_goal = 10



    def ogrid_callback(self, msg):

            self.flag = 1;
            """
            Expects an OccupancyGrid message.
            Stores the ogrid array and origin vector.
            Reevaluates the current plan since the ogrid changed.
            """
            #global ogrid
            #global ogrid_origin
            #global ogrid_cpm
            #global x_sact
            #global y_scat

            #global counter
            self.counter = self.counter + 1

            start = self.rostime()
            self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            print("Shape ogrid", self.ogrid.shape)
            print("Ogrid patch", self.ogrid[0:50, 0:50])



            # This opens new windows for displaying images every interation -> super anoying
            #img = ogrid + 1
            #img = Image.fromarray(img.astype(np.uint8))
            #img.show()
            self.ogrid[self.ogrid < 0] = 50

            #if self.counter == 1 :
            #plt.figure(self.f1.number)

            #plt.imshow(self.ogrid, cmap='hot', interpolation='nearest')
            #plt.show()
            #plt.show(block=False)
            #plt.figure(1) # throws error
            plt.figure()
            plt.ion()
            imgplot = plt.imshow(self.ogrid)

            #plt.show(block=False)

            #plt.show()
            #plt.figure(self.f2.number)

            np.random.seed(19680801)
            N = 50
            N = 10
            x = 256*np.random.rand(N)
            print("RRT.vec", np.transpose(RRT.vec[:,0]))
            print("RRT.vec shape", np.transpose(RRT.vec[:,0]).shape)
            y = 256*np.random.rand(N)
            colors = np.random.rand(N)
            area = (30 * np.random.rand(N))**2  # 0 to 15 point radii

            #plt.scatter(x, y, s=area, c=colors, alpha=0.5)
            plt.scatter(self.vec[:,0], self.vec[:,1])
            print("vec counted", self.vec[self.counter,0])
            plt.show(block=False)


            """
            x_scat = np.append(x_scat, vec[counter,0])
            y_scat = np.append(y_scat, vec[counter,1])
            plt.scatter(x_scat,y_scat)
            plt.draw()
            """
            """
            #plt.scatter(vec[counter,0], vec[counter,1])
            imgplot = plt.imshow(self.ogrid, cmap='hot', interpolation='nearest')
            plt.show(block=False)
            """

            """
            self.x_scat = np.append(self.x_scat, self.vec[self.counter,0])
            self.y_scat = np.append(self.y_scat, self.vec[self.counter,1])

            #plt.figure(2)
            plt.figure(self.f2.number)
            plt.scatter(self.x_scat,self.y_scat)
            plt.draw()
            """


            #print(self.vec[self.counter,:])
            print("Counter", self.counter)
            #plt.plot([100,200,300],[200,150,200],'o')
            #plt.show(block=False)

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
    rospy.init_node("rrt_node")
    RRT = rrt()

    """
    plt.ion()
    plt.figure(2)
    axes = plt.gca()
    axes.set_xlim([-5,260])
    axes.set_ylim([-5,260])
    """

    print("HEEEEEY")

    #f1 = plt.figure()
    #f2 = plt.figure()

    #plt.figure(f2.number)
    #plt.scatter(np.transpose(RRT.vec[:,0]),np.transpose(RRT.vec[:,1]))
    #plt.plot(np.transpose(RRT.vec[:,0]),np.transpose(RRT.vec[:,1]), 'bo')
    #plt.show(block=False)
    #plt.draw()


    """
    np.random.seed(19680801)
    N = 50
    x = np.random.rand(N)
    print("RRT.vec", np.transpose(RRT.vec[:,0]))
    print("RRT.vec shape", np.transpose(RRT.vec[:,0]).shape)
    y = np.random.rand(N)
    colors = np.random.rand(N)
    area = (30 * np.random.rand(N))**2  # 0 to 15 point radii

    plt.scatter(x, y, s=area, c=colors, alpha=0.5)
    #plt.plot(x,y,'ro')
    #plt.draw()
    #plt.show(block=False)
    plt.show()
    """



    while not rospy.is_shutdown() :
        print("Running")
        RRT.rate.sleep()
        """
        if(RRT.flag == 1) :
            try :
                print("ogrid-shape", RRT.ogrid().shape)
            except :
                print("no ogrid yet")
        """
