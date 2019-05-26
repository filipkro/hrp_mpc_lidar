#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose

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

        # Subscribers
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.ogrid_callback)
        #sub_map_meta = rospy.Subscriber("/slam_out_pose", MapMetaData, ogrid_emta_callback)
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.newGoal)
        self.sub_pos = rospy.Subscriber("/slam_out_pose", PoseStamped, self.newPos)
        self.sub_pos2 = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.newPos2)
        self.sub_path2 = rospy.Subscriber("/poseArrayTopic", PoseArray, self.path_cb2)


        # Marker Publisher
        self.markerArray = MarkerArray()
        self.topic = 'visualization_marker_array'
        self.publisher = rospy.Publisher(self.topic, MarkerArray, queue_size=1)
        # PoseArray Publisher
        self.poseArray = PoseArray()
        self.pose_array_pub = rospy.Publisher("/poseArrayTopic", PoseArray)

        #speed = Twist()
        self.rate = rospy.Rate(2)

        self.x_goal = 10
        self.y_goal = 10

        self.path = None
        self.path_shape = None

        self.x_pos = 0.0
        self.y_pos = 0.0

        self.x_goal = 0.0
        self.y_goal = 0.0

        self.prev_theta = 0.0

        # Vactorn names to be used in the MPC program.
        self.xValue = None
        self.yValue = None
        self.thetaValue = None

    def ogrid_callback(self, msg):

            self.flag = 1;
            self.counter = self.counter + 1

            start = self.rostime()
            self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            print("Shape ogrid", self.ogrid.shape)
            #print("Ogrid patch", self.ogrid[0:50, 0:50])


            self.ogrid[self.ogrid < 0] = 50
            #print("Ogrid patch2" self.ogrid[100:150,100:150])
            #print("MAX OGRID", np.max(self.ogrid))
            #print("MIN OGRID", np.min(self.ogrid))

            RRT = rrt_test2.rrt(self.ogrid)
            RRT.startPoint = np.array([self.x_pos, self.y_pos])
            RRT.goalPoint = np.array([self.x_goal, self.y_goal])
            RRT.build_tree()
            print("START POINT", RRT.startPoint)
            print("GOAL POINT", RRT.goalPoint)


            # GET PATH FOR PLOTTING

            path = RRT.get_path()
            path_shape = path.shape
            path = path.reshape(path_shape[0]/2,2)
            print("PATH1", path)
            path = np.flip(path, axis = 0)
            print("PATH2", path)
            path = path*0.05
            print("Path shape", path_shape[0])

            #self.path = path
            #self.path_shape = path_shape

            for i in range(path_shape[0]/2) :
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.SPHERE
                #marker.type = marker.LINE_STRIP
                marker.action = marker.ADD
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
                marker.color.a = 1.0
                marker.color.r = 1.0 # make it red
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = path[i, 0]
                marker.pose.position.y = path[i, 1]
                marker.pose.position.z = 0.0
                self.markerArray.markers.append(marker)

                # make the PoseArray
                #self.poseArray.header.stamp = self.rostime()
                self.poseArray.header.frame_id = "/map"
                somePose = Pose()
                #somePose.header.frame_id = "/map" ###
                somePose.position.x = path[i, 0]
                somePose.position.y = path[i, 1]
                somePose.position.z = 0.0

                try :
                #if (i < path_shape[0]/2 -1) :
                    #print("try")
                    #ang_i_cur = math.atan2(path[i, 0], path[i, 1])
                    #ang_i_next = math.atan2(path[i+1, 0], path[i+1, 1])
                    ang_i_cur = math.atan2(path[i, 1], path[i, 0])
                    ang_i_next = math.atan2(path[i+1, 1], path[i+1, 0])
                    #angle_to_next = (ang_i_next + ang_i_cur + math.pi) % (2.0*math.pi) - math.pi
                    angle_to_next = math.atan2(path[i+1, 1]-path[i, 1], path[i+1, 0]-path[i, 0])
                    self.prev_theta = angle_to_next
                    print("ang_i_cur", ang_i_cur)
                    print("ang_i_next", ang_i_next)
                    print("Angle to next", ang_i_next)
                    #(roll, pitch, theta_pos) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
                    (somePose.orientation.x, somePose.orientation.y, somePose.orientation.z, somePose.orientation.w) = \
                     quaternion_from_euler(0.0, 0.0, angle_to_next)
                except :
                    print("except")
                    somePose.orientation.x = 0.0
                    somePose.orientation.y = 0.0
                    somePose.orientation.z = 0.0
                    somePose.orientation.w = 1.0

                self.poseArray.poses.append(somePose)

            # GET POINTS FOR PLOTTING
            all_points = RRT.get_all_nodes()
            all_points_shape = all_points.shape
            print("All points shape", all_points_shape)
            #all_points = all_points.reshape(all_points_shape[0]/2,2)
            all_points = all_points*0.05
            #print("PATH", all_points)
            #self.path = path
            #self.path_shape = path_shape

            for i in range(all_points_shape[0]/2) :
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.SPHERE
                #marker.type = marker.LINE_STRIP
                marker.action = marker.ADD
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 0.0 # make it red
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = all_points[i, 0]
                marker.pose.position.y = all_points[i, 1]
                marker.pose.position.z = 0
                self.markerArray.markers.append(marker)


            id = 0
            for m in self.markerArray.markers:
                m.id = id
                id += 1

            self.publisher.publish(self.markerArray)
            self.markerArray = MarkerArray()

            self.pose_array_pub.publish(self.poseArray)
            self.poseArray = PoseArray()

            self.ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
            self.ogrid_cpm = 1 / msg.info.resolution

            try:
                print("TRY")
            except:
                print("\n(WARNING: something went wrong in reevaluate_plan)\n")
            elapsed = abs(self.rostime() - start)
            if elapsed > 1:
                print("\n(WARNING: ogrid callback is taking {} seconds)\n".format(np.round(elapsed, 2)))


    def newGoal(self, msg) :
        print("newGoal Callback")
        self.x_goal = round(20*msg.pose.position.x)
        self.y_goal = round(20*msg.pose.position.y)

        rot_q = msg.pose.orientation
        (roll_goal, pitch_goal, theta_goal) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        #theta = yaw;

    # Callbak function
    def newPos(self, msg) :

        print("newPos Callback")
        self.x_pos = round(20*msg.pose.position.x)
        self.y_pos = round(20*msg.pose.position.y)

        print("X-POS", self.x_pos)
        print("Y-POS", self.y_pos)

        rot_q = msg.pose.orientation
        (roll, pitch, theta_pos) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # Callbak function for pose estimation from Rviz
    def newPos2(self, msg) :

        print("newPos2 Callback")
        self.x_pos = round(20*msg.pose.pose.position.x)
        self.y_pos = round(20*msg.pose.pose.position.y)

        print("X-POS", self.x_pos)
        print("Y-POS", self.y_pos)

        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta_pos) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # Test callback functioin for the path published
    def path_cb2(self, msg) :

        print("path_cb2 Callback")
        poses = msg.poses
        self.xValue = poses[0].position.x
        self.yValue = poses[0].position.y
        rot_q = poses[0].orientation
        (roll, pitch, theta_pos) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.thetaValue = theta_pos

        for i in range(1, len(poses)) :
            self.xValue = np.append(self.xValue, poses[i].position.x)
            self.yValue = np.append(self.yValue, poses[i].position.y)
            rot_q = poses[i].orientation
            (roll, pitch, theta_pos) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
            self.thetaValue = np.append(self.thetaValue, theta_pos)

        print("xValue", self.xValue)
        print("yValue", self.yValue)
        print("thetaValue", self.thetaValue)

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
