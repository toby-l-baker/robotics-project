#!/usr/bin/env python
import numpy as np
import rospy 
import tf
from geometry_msgs.msg import PoseStamped as PS
import time

Node = "path_planner"

class Turtlebot:
	def __init__(self, name, queue_size=1):
		# name is all lowercase color of turtlebot
		self.name = name
		self.publish_topic = '/' + self.name + '/move_base_simple/goal/'
		self.queue_size=queue_size
		self.publisher = rospy.Publisher(self.publish_topic, PS, queue_size=queue_size)



		self.tf = tf.TransformListener()
		self.tf_topic = '/' + self.name + '/base_link' 
		rospy.Subscriber(self.tf_topic , PS, self.pos_callback) 

	def pos_callback(data):
		if self.tf.frameExists(self.tf_topic) and self.tf.frameExists("/map"):
			t = self.tf.getLatestCommonTime(self.tf_topic, "/map")
			pos, q = self.tf.lookupTransform(self.tf_topic, "/map", t)
			
			self.x = pos.x
			self.y = pos.y
			euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
			self.theta = euler[2]

	def move(self, x, y, theta):
		pose = xytheta_to_pose(x, y, theta)
		self.pub.publish(pose)
		


def xytheta_to_pose(x, y, theta):
	pose = PoseStamped()
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





def main():
	rospy.init_node(Node, anonymous=True)
	red = Turtlebot('black')
	while not rospy.is_shutdown():
		print(red.x, red.y, red.theta)
		time.sleep(1)
	



if __name__ == "__main__":
	main()