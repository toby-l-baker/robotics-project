#!/usr/bin/env python
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped as PS
import time

class Turtlebot:
	def __init__(self, name, type_, queue_size=1):
		# name is all lowercase color of turtlebot
		self.map = "/map"
		self.name = name
		self.type_ = type_
		self.publish_topic = '/' + self.name + '/move_base_simple/goal'
		self.queue_size=queue_size
		self.publisher = rospy.Publisher(self.publish_topic, PS, queue_size=queue_size)

		self.tf = tf.TransformListener()
		self.frame = '/' + self.name + '/base_link'

		
	def move(self, x, y, theta):
		pose = xytheta_to_pose(x, y, theta)
		self.publisher.publish(pose)

# Turns an x, y, theta coordinate into a pose
def xytheta_to_pose(x, y, theta):
	pose = PS()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	pose.pose.position.x = x
	pose.pose.position.y = y
	quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
	pose.pose.orientation.x = quaternion[0]
	pose.pose.orientation.y = quaternion[1]
	pose.pose.orientation.z = quaternion[2]
	pose.pose.orientation.w = quaternion[3]
	return pose
