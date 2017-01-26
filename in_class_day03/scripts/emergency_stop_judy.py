#!/usr/bin/env python

"""In class day 3 stopping the robot if an obstacle is detected"""
# from geometry_msgs.msg import PointStamped,Point
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg	import Bump
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

class Emergency_Stop(object):

	def __init__(self):
		rospy.init_node('e_stop')
		rospy.Subscriber('/bump',Bump, self.process_bump)
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
		self.is_bumped = False
		self.see_obstacle = False
		self.r = rospy.Rate(10)

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
			if self.see_obstacle:
				twist.msg.angular.z = 0.3
			else:
				twist_msg.linear.x = 1
			self.pub.publish(twist_msg)
			print twist_msg
			self.r.sleep()

my_stop = Emergency_Stop()
my_stop.run()
