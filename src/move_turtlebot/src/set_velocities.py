#!/usr/bin/env python

import rospy
import sys
from velocity_controller import VelocityController
from geometry_msgs.msg import Twist

def controller(marker_num, self_num, master_num):
	vel_control = VelocityController(marker_num, self_num, master_num)
	pub = rospy.Publisher(vel_control.turtlebot+'/commands/velocity', Twist, queue_size=10)
	rate = rospy.Rate(10) #subject to change


	while not rospy.is_shutdown():
		try:
			vel_control.update_velocity()
			#pub.publish(vel_control.cmd_vel)
		except Exception as e:
			print(e)
			pass

		rate.sleep()



if __name__ == '__main__':
	rospy.init_node('turtlebot_controller', anonymous=True)
	try:
		controller(sys.argv[1], sys.argv[2], sys.argv[3])
	except Exception as e:
		print(e)
		pass