#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Twist, PoseStamped
from math import atan2
from tf import TransformListener
import tf
import math
import numpy as np
import time

class Robot(object):
	def __init__(self, pub_topic_speed, sub_topic_odom, link1, link2):
		self.link_1 = link1
		self.link_2 = link2
		self.goal_x = 0
		self.goal_y = 0
		self.goal_orientation = 0
		self.pose_x = 0.0
		self.pose_y = 0.0
		self.pose_orientation = 0.0
		self.speed = Twist()
		self.step = 0
		print("Step 0")
				
		self.listener = tf.TransformListener() 
		self.sub = rospy.Subscriber(sub_topic_odom, PoseStamped, self.getOdom)
		self.pub = rospy.Publisher(pub_topic_speed, Twist, queue_size = 1)

	def getOdom(self, msg):		
		rot = msg.pose.orientation
		(roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
		self.pose_orientation = yaw
		self.pose_x = msg.pose.position.x
		self.pose_y = msg.pose.position.y
		#print("initial pos_x ==", self.pose_x)
		#print("initial pos_y ==", self.pose_y)

	def calculateSpeed(self):
		
		if self.step == 0:
			self.step_0()	
		elif self.step == 1:
			self.step_1()
		elif self.step == 2:
			self.step_2()
		elif self.step == 3:
			self.step_3()

		self.sendSpeed()

	def step_0(self):
		
		goal_x, goal_y, goal_rot = self.getGoal()
		if abs(self.goal_x - goal_x) > 0.5 or abs(self.goal_y - goal_y) > 0.5:
			print("====  In step 0  =====")
			time.sleep(5)
			#goal_x, goal_y, goal_rot = self.getGoal()
			self.goal_x = goal_x
			self.goal_y = goal_y
			self.goal_orientation = goal_rot
			if abs(self.pose_x - self.goal_x) > 0.5:
				self.step = 1
				print("Step 1")
			elif abs(self.pose_y - self.goal_y) > 0.5:
				self.step = 1
				print("Step 1")
	
	def step_1(self):
		print("goal_x", self.goal_x)
		print("goal_y", self.goal_y)
		print("goal orientation", np.degrees(self.goal_orientation))

		print("pose_x", self.pose_x)
		print("pose_y", self.pose_y)
		print("pose orientation", np.degrees(self.pose_orientation))


		inc_x = self.goal_x - self.pose_x
		inc_y = self.goal_y - self.pose_y
		angle_to_goal = atan2(inc_y, inc_x)
		angle_diff = np.degrees(angle_to_goal - self.pose_orientation)
		print("====  In step 1  =====")
		print("angle_diff ==",np.degrees(angle_to_goal))
		if abs(angle_diff) < 5:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			self.speed.angular.z = 0
			self.step = 2
			print("Step 2")
		elif angle_diff < 5:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			# self.speed.angular.z = -127 # clockwise for lizard
			self.speed.angular.z = 127 #for the bridge
		elif angle_diff > 5:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			# self.speed.angular.z = 127  # counter clockwise for lizard
			self.speed.angular.z = -127 # for bridge
		else:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			self.speed.angular.z = 0
			self.step = 2
			print("Step 2")

	def step_2(self):
		print("====  In step 2  =====")
		inc_x = self.goal_x - self.pose_x
		if inc_x > 0.05:
			self.speed.linear.x = -127 #forward direction
			self.speed.linear.y = 0
			self.speed.angular.z = 0
		elif inc_x < -0.05:
			self.speed.linear.x = 127 #backward direction
			self.speed.linear.y = 0
			self.speed.angular.z = 0
		else:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			self.speed.angular.z = 0
			self.step = 3
			print("Step 3")

	def step_3(self):
                print("====  In step 3  =====")
		angle_diff = np.degrees(self.goal_orientation - self.pose_orientation)
		print("Step 3 Angle Diff", angle_diff)
		print("goal_orientation", np.degrees(self.goal_orientation))
		print("pose_orientation", np.degrees(self.pose_orientation))

		if abs(angle_diff) < 5:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			self.speed.angular.z = 0
			self.step = 0
			print("Step 0")
			print("!!!=======  goal reached  ======!!!")
		elif angle_diff < -5:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			self.speed.angular.z = -127
		elif angle_diff > 5:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			self.speed.angular.z = 127
		else:
			self.speed.linear.x = 0
			self.speed.linear.y = 0
			self.speed.angular.z = 0
			self.step = 0
			print("Step 0")

	def sendSpeed(self):
		self.pub.publish(self.speed)
		print("Velocity X: ", self.speed.linear.x)
		print("Velocity Y: ", self.speed.linear.y)
		print("Velocity Z: ", self.speed.angular.z)
		print("----------------------------")

	def getGoal(self):
		(trans,rot) = self.listener.lookupTransform(self.link_1, self.link_2, rospy.Time(0))
		(roll, pitch, yaw) = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
		#print("GOAL X: ", trans[0])
		#print("GOAL Y: ", trans[1])
		#print("GOAL Rot: ", yaw)
		return trans[0], trans[1], yaw

def main():
	rospy.init_node("speedcontroller")

	bw = Robot("/bw_cmd_vel", "bw_tf_pose", "/map", "/back_cylinder")
	#mc = Robot("/mc_cmd_vel", "mc_tf_pose", "/map", "/end_eff")     
	time.sleep(2)

	print "Starting loop"
	while not rospy.is_shutdown():
		#try:
			#mc.calculateSpeed()
		        bw.calculateSpeed()
		#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			#print "No goal received"
			#continue    
		        time.sleep(0.2)	
	print "Program stopped"


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print "Speed controller stopped"
