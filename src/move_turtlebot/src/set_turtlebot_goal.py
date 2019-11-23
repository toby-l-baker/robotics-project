#!/usr/bin/python

import rospy
import numpy as np
import sys
from velocity_controller import VelocityController, vector
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
import tf.transformations as tft

def controller(marker_num, self_num, master_num):
	vel_control = VelocityController(marker_num, self_num, master_num)
	pub_target = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
	#pub_rel_vel = rospy.Publisher(vel_control.turtlebot+'/relative_velocity', Twist, queue_size=10)
	rate = rospy.Rate(1) #subject to change
	pose_target = Pose()
	pose_target.position.x = 0.55
	pose_target.position.z = -0.35
	# pose_target.position.y = 0
	# post_target.position.z = 0
	# post_target.orientation.x 
	pose_target.orientation.y = 1/np.sqrt(2)
	# post_target.orientation.z
	pose_target.orientation.w = 1/np.sqrt(2)

	target_stamped = PoseStamped()
	target_stamped.header.stamp = rospy.Time.now()
	target_stamped.header.frame_id = "map"
	target_stamped.pose = None

	while not target_stamped.pose:
		target_stamped.pose = vel_control.transform_pose(pose_target, "map", vel_control.marker_frame)

	target_stamped.pose.position.z = 0
	angles = tft.euler_from_quaternion(vector(target_stamped.pose.orientation))
	quatern = tft.quaternion_from_euler(0, 0, angles[2])
	target_stamped.pose.orientation.x = quatern[0]
	target_stamped.pose.orientation.y = quatern[1]
	target_stamped.pose.orientation.z = quatern[2]
	target_stamped.pose.orientation.w = quatern[3]

        #position = Point(1.266, 0.0, 0.0)
        #orientation = Quaternion(0, 0, 0.127, 0.991)
        #target_stamped.pose.position = position
        #target_stamped.pose.orientation = orientation

	print(target_stamped)
	pub_target.publish(target_stamped)
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('turtlebot_controller', anonymous=True)
	try:
		controller(sys.argv[1], sys.argv[2], sys.argv[3])
	except Exception as e:
		print(e)
		pass
