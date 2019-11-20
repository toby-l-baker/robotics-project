#!/usr/bin/env python

import tf2_ros
import rospy
from geometry_msgs.msg import Twist


class VelocityController:
	"""
	This class will subscribe to the transform from the slaves
	USB_Cam and the AR Tag on the master turtlebot. Using this
	transform it will alter the velocities in the path to align 
	each turtlebot.
	"""
	def __init__(self, marker_num, turtlebot_num, master_turtlebot_num):
		"""Setup the names for the transform"""
		self.turtlebot = "/mobile_base" # "/mobile_base_"+turtlebot_num
		self.master_turtlebot = str("/mobile_base_"+master_turtlebot_num) # mobile_base_0
		self.marker_frame = str("ar_marker_"+marker_num) # self.master_turtlebot+"/ar_marker_"+marker_num
		self.camera_frame = "camera_rgb_frame" # self.turtlebot+"/usb_cam_frame"

		"""Setup the publisher and tf buffers"""
		self.tfBuffer = tf2_ros.Buffer()
		self.tfListener = tf2_ros.TransformListener(self.tfBuffer)


	def update_velocity(self):
		"""Get transform from the camera frame to the marker frame"""
		#print("About to transform")
		self.trans = self.tfBuffer.lookup_transform(self.camera_frame, self.marker_frame, rospy.Time())
		current_x = self.trans.transform.translation.x
		#print("Got transform")
		print(self.trans)
		self.velocity = Twist();
		target_x = 0.3

		
