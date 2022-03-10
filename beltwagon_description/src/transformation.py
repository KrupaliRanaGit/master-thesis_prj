#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import rospy

class Transformation(object):
	def __init__(self, pub_topic, sub_topic, trans_vector, initialpose):
		self.trans_vector = trans_vector
		self.sub = rospy.Subscriber(sub_topic, PoseStamped, self.getPose)
		self.pub = rospy.Publisher(pub_topic, PoseStamped, queue_size = 1)
		self.initialpose = initialpose

	def tf_rot(self, angle):
		rot = np.array([[math.cos(angle), -math.sin(angle)],
				[math.sin(angle), math.cos(angle)]])
		return rot.dot(self.trans_vector)

	def tf_trans(self, vector, translation):
		return vector + translation

	def transformation(self, vector, angle):
		rotTranslation = self.tf_rot(angle)
		return self.tf_trans(vector, rotTranslation) + self.initialpose

	def getPose(self, msg):
		rot = msg.pose.orientation
		(roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
		x = msg.pose.position.x
		y = msg.pose.position.y
		vector_pose = np.array([[x],[y]])
		vector_tf_pose = self.transformation(vector_pose, yaw)		
		tf_pose = PoseStamped()				
		tf_pose.pose.position.x = vector_tf_pose[0][0]
		tf_pose.pose.position.y = vector_tf_pose[1][0]
		tf_pose.pose.orientation = msg.pose.orientation
		self.sendPose(tf_pose)
	
	def sendPose(self, pose):
		pose.header.frame_id = 'map'
		self.pub.publish(pose)


def main():
	rospy.init_node("tf_node")
	r = rospy.Rate(1)
	translation_bw = np.array([[0],[-1.3]]) #[[0],[-1.605]] sensor position from middle point of kinematic chain
	initialpose_bw = np.array([[0],[0]]) #[[0],[2.505]] pose arrow position from kinematic chain(rviz)
	#translation_mc = np.array([[0],[0]])      
	#initialpose_mc = np.array([[0],[0]])
	tf_bw = Transformation("bw_tf_pose", "/bw_slam/slam_out_pose", translation_bw, initialpose_bw)
	#tf_mc = Transformation("mc_tf_pose", "/mc_slam/slam_out_pose", translation_mc, initialpose_mc)
	print("Progress")
	rospy.spin()	


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print "Transformation stopped"
