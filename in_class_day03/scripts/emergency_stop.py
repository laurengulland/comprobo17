#!/usr/bin/env python


#Didn't get to try running successfully!!

""" Emergency Stop script for the Neatos. In class, day 3 """
from geometry_msgs.msg import Twist
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import rospy

class Emergency_Stop(object):
    def __init__(self):
        #initialize node
        rospy.init_node('e_stop')
        #initiate Subs/Pubs
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #initalize flags
        self.is_bumped = False
        self.see_object = False
        self.r = rospy.Rate(10) #execute at 10 Hz
        self.vel = 1 #???

	def process_bump(self,msg):
		if (msg.leftFront==1 or msg.leftFront==1 or msg.rightFront ==1 or msg.rightSide==1):
			self.is_bumped = True
		else:
			self.is_bumped = False
    def process_scan(self,msg):
        for dist in msg.ranges[0:10]:
			if dist == 0.0:
				continue
			if dist < 1.0:
				self.see_obstacle = True
				return
		for dist in msg.ranges[350:361]:
			if dist == 0.0:
				continue
			if dist < 1.0:
				self.see_obstacle = True
				return
		self.see_obstacle = False

    def run(self):
        while not rospy.is_shutdown():
            linear_msg = Vector3(x = 0)
            angular_msg = Vector3(z = 0)
            twist_msg = Twist(linear=linear_msg, angular = angular_msg)
            if self.see_obstacle or self.is_bumped:
                twist.msg.linear.x = 0.0
            else:
                twist_msg.linear.x = 1
            self.pub.publish(twist_msg)
            print twist_msg
            self.r.sleep()

            r.sleep()
        print "Node is finished"
