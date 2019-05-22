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

class rrt :
    def __init__(self, ogrid):
        self.ogrid = ogrid

        self.nodes = []
        self.initial_point = Node(np.array([0,0]), False)
        self.nodes.append(self.initial_point)

        self.goalPoint = np.array([100.0,100.0])
        #self.goalPoint = np.round(np.random.uniform(0,256,2))

        self.NUMNODES = 500
        self.node_counter = 0
        self.delta = 15

        self.all_nodes = np.array([0,0])
        self.all_random = np.array([0,0])

        self.goalNode = None
        self.goalFound = False
        self.GOAL_RADIUS = 5.0
        print("DISTABCE TEST", self.dist(self.goalPoint, np.array([20.0,200.0])))

        self.path = np.array([0,0])


    def build_tree(self) :
        while self.node_counter < self.NUMNODES and self.goalFound == False:
            foundNext = False
            while foundNext == False:
                rand = self.get_random_clear()
                parentNode = self.nodes[0]
                for p in self.nodes:
                    if self.dist(p.point,rand) <= self.dist(parentNode.point,rand):
                        newPoint = self.step_from_to(p.point,rand)
                        if self.collides(newPoint) == False:
                            parentNode = p
                            foundNext = True

            newnode = self.step_from_to(parentNode.point,rand)
            #print("Newpoint", newnode)
            #print("Goal point", self.goalPoint)

            self.all_nodes = np.append(self.all_nodes, newnode,  axis=0)
            self.nodes.append(Node(newnode, parentNode))

            if self.dist(newnode, self.goalPoint) <= self.GOAL_RADIUS :
                self.goalFound = True
                size = self.get_size()
                self.goalNode = self.nodes[size-1]

            print("DIST", self.dist(newnode, self.goalPoint))

            self.node_counter = self.node_counter + 1

        if(self.goalFound == True) :
            self.path = self.goalNode.point
            par = self.goalNode.parent
            while par != False :
                self.path = np.append(self.path, par.point, axis=0)
                par = par.parent
                print("pathing2")


    def dist(self,p1,p2):    #distance between two points
        return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

    def get_random_clear(self) :
        rand_point = np.random.uniform(0,256,2)
        #rand_point = np.round(np.random.uniform(0,256,2))

        #rand_point = np.array([np.round(np.random.uniform(0,256,1)),np.round(np.random.uniform(0,256,1))])
        #print("Random point generation", rand_point)
        self.all_random = np.append(self.all_random, rand_point,  axis=0)
        return rand_point

    def step_from_to(self,p1,p2):
        if self.dist(p1,p2) < self.delta:
            return p2
        else:
            theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
            return np.round(p1 + np.array([self.delta*math.cos(theta), self.delta*math.sin(theta)]))
    def get_all_nodes(self):
        shape = self.all_nodes.shape
        return self.all_nodes.reshape(shape[0]/2, 2)

    def get_all_random(self):
        shape = self.all_random.shape
        return self.all_random.reshape(shape[0]/2, 2)

    def collides(self, point):
        #print("point", point)
        #print(point[0])
        #print(int(point[0]))
        if self.ogrid[int(point[0]),int(point[1])] < 49 :
            return True
        return False

    def get_goal_node(self) :
        return self.goalNode
    def get_goal_point(self) :
        return self.goalPoint
    def get_tree(self) :
        return self.nodes
    def get_size(self) :
        return len(self.nodes)
    def get_path(self) :
        return self.path

if __name__ == "__main__":
    RRT = rrt()
    RRT.build_tree()
    all_nodes = RRT.get_all_nodes()
    print(all_nodes)
    all_random = RRT.get_all_random()

    print("ALL_NODES_SHAPE", RRT.get_all_nodes().shape)

    plt.scatter(all_nodes[:,0], all_nodes[:,1])
    plt.scatter(all_nodes[-1,0], all_nodes[-1,1], s=100, c='g', marker='o',)
    plt.scatter(all_nodes[0,0], all_nodes[0,1], s=100, c='g', marker='o',)

    goal = RRT.get_goal_point()
    plt.scatter(goal[0], goal[1], s=100, c='r', marker='o',)


    tree = RRT.get_tree()
    tree_size = RRT.get_size()
    print("Tree size", tree_size)

    for i in range(tree_size) :
        node = tree[tree_size-1-i]

        if tree_size-1-i != 0 :
            parent = node.parent
            plt.plot([node.point[0], parent.point[0]],[node.point[1],parent.point[1]], 'ro-')

        print("Working", i)
    #plt.plot([0, 100],[0,100], 'ro-')

    path = RRT.path

    path_shape = path.shape
    path = path.reshape(path_shape[0]/2,2)
    print("PATH", path)
    plt.scatter(path[:,0], path[:,1], s=150, c='k', marker='o',)

    plt.show()

    """
    goalNode = tree[tree_size -1]
    path = goalNode.point

    par = goalNode.parent
    print("Parent point", par)

    #while parent != False :
    for i in range(100) :
        np.append(path, par.point)
        parent = goalNode.parent
        print("pathing", par)

    np.append(path, tree[0].point)

    print("PATH", path)
    """
