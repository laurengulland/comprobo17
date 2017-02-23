#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

from dynamic_reconfigure.server import Server
from neato_soccer.cfg import SoccerConfig
import dynamic_reconfigure.client



class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.r_lower = 0
        self.r_upper = 91
        self.g_lower = 94
        self.g_upper = 255
        self.b_lower = 0
        self.b_upper = 21
        self.center_x = 0
        self.center_y = 0
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0))

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        cv2.namedWindow('video_window_2')
        self.binary_image = None
        srv = Server(SoccerConfig, self.configCallback)


    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(self.cv_image, (self.b_lower,self.g_lower,self.r_lower), (self.b_upper,self.g_upper,self.r_upper))
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            print int(self.center_x), int(self.center_y)
        cv2.circle(self.cv_image,(int(self.center_x), int(self.center_y)), 20, (0,0,255), 3)

    def configCallback(self,config,level):
          rospy.loginfo("""Reconfigure Request: {r_lower}, {r_upper}, {g_lower}, {g_upper}, {b_lower}, {b_upper}""".format(**config))
          self.r_lower = config.r_lower
          self.r_upper = config.r_upper
          self.b_lower = config.b_lower
          self.b_upper = config.b_upper
          self.g_lower = config.g_lower
          self.g_upper = config.g_upper
          return config

    def move_robot(self):
        if not (self.center_x, self.center_y):
            self.moves.linear.x=0
            self.moves.angular.z=0
        elif self.center_x in range(640/4,640/2+640/4):
            self.moves.linear.x = .75
            self.moves.angular.z = 0
        # elif self.center_x in range(0,640/4):
        #     self.moves.linear.x = 1
        #     self.moves.angular.z = .2*self.center_x/640
        # elif self.center_x in range(3*640/4,640):
        #     self.moves.linear.x = 1
        #     self.moves.angular.z = -.2*self.center_x/640
        else:
            self.moves.linear.x = .2
            self.moves.angular.z = -(self.center_x-320)/320
        self.pub.publish(self.moves)

    def stop_moving(self):
        self.moves.linear.x=0
        self.moves.angular.z=0
        self.pub.publish(self.moves)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        rospy.on_shutdown(self.stop_moving)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # print self.cv_image.shape
                cv2.imshow('video_window', self.cv_image)
                if not self.binary_image is None:
                    cv2.imshow('video_window_2', self.binary_image)
                self.move_robot()
                #Find center of mass pixel
                cv2.waitKey(5)
            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
