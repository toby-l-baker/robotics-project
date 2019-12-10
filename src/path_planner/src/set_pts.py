#!/usr/bin/env python
import json
import rospy
from std_msgs.msg import String


Subscriber_Topic = "/way_points"



class File_Writer:
	def __init__(self, filename = "default.json"):
		self.file = open(filename, 'w')
		self.subscriber = rospy.Subscriber(Subscriber_Topic, String, self.new_pt_cb) 
		# Order of new points is: leader start, leader goal, follower start, follower goal
		self.way_points = {
			"leader_start": None, 
			"leader_goal": None, 
			"follower_start": None, 
			"follower_goal": None
		}
		self.current_way_point = 0

		print("\nStarting point collection.\nEnter points as 2d Pose Estimate in RVIZ in the order:")
		print("\tLeader Start Postion ")
		print("\tLeader Goal Postion ")
		print("\tFollower Start Postion ")
		print("\tFollower Goal Postion ")


	def new_pt_cb(self, msg):
		
		# if not None in self.way_points.values():
		# 	# json_msg = self.create_json()
		# 	self.file.write(json.dumps(self.way_points))
		# else:
		if self.current_way_point == 0:
			self.way_points["leader_start"] = msg
		elif self.current_way_point == 1:
			self.way_points["leader_goal"] = msg
		elif self.current_way_point == 2:
			self.way_points["follower_start"] = msg
		elif self.current_way_point == 3:
			self.way_points["follower_goal"] = msg
			self.file.write(json.dumps(self.way_points))
		self.current_way_point = self.current_way_point + 1
def main():
	rospy.init_node("set_pts", anonymous=True)

	fw = File_Writer()

	fw.new_pt_cb(0)
	fw.new_pt_cb(1)
	fw.new_pt_cb(2)
	fw.new_pt_cb(3)

	# rospy.spin()

if __name__ == "__main__":
	main()