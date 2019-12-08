#!/usr/bin/env python
import numpy as np
import rospy 
import tf
from geometry_msgs.msg import PoseStamped as PS
import time

Node = "path_planner"

def Test():
	print("Works prettry good")

class Turtlebot:
	def __init__(self, name, queue_size=1):
		# name is all lowercase color of turtlebot
		self.map = "/map"
		self.name = name
		self.publish_topic = '/' + self.name + '/move_base_simple/goal'
		self.queue_size=queue_size
		self.publisher = rospy.Publisher(self.publish_topic, PS, queue_size=queue_size)

		self.x = None
		self.y = None
		self.theta = None

		self.tf = tf.TransformListener()
		self.frame = '/' + self.name + '/base_link'

		angle = 0
		while self.x == None or self.y == None or self.theta == None:
			self.position()

	# Gets position of Turtlebot
	def position(self):
		# Pose published on "amcl_pose"
		# TODO: make this a callback
		try:
			now = rospy.Time.now()
			
			self.tf.waitForTransform(self.map, self.frame, now, rospy.Duration(2.0))
			# t = self.tf.getLatestCommonTime(self.frame, "/map")

			pos, q = self.tf.lookupTransform(self.map, self.frame, rospy.Time())
			
			self.x = pos[0]
			self.y = pos[1]
			euler = tf.transformations.euler_from_quaternion(q)
			self.theta = euler[2]
		except Exception as e:
			print(e)
	# Moves turtlebot to given x,y position with heading theta
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





# def main():
# 	rospy.init_node(Node, anonymous=True)
# 	black = Turtlebot('black')
# 	green = Turtlebot('green')
# 	print(black.x, black.y)
# 	print(green.x, green.y)
# 	# red.move(0, 0, 0)
# 	# black.move(0, 0, 0)

# 	# while not rospy.is_shutdown():
# 	# 	tb.position()
# 	# 	print(tb.x, tb.y, tb.theta)
		

# if __name__ == "__main__":
# 	main()