#!/usr/bin/env python
import json
import yaml
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped as PS
import rospkg





Subscriber_Topic = "/way_points"



class File_Writer:
	def __init__(self, filename):
		rospack = rospkg.RosPack()
		rospack.list() 
		self.filename = filename
		self.filepath = rospack.get_path('path_planner') + '/waypoints/' + filename + '.json'
		
		self.subscriber = rospy.Subscriber(Subscriber_Topic, PS, self.new_pt_cb) 
		# Order of new points is: leader start, leader goal, follower start, follower goal
		self.way_points = {
			"leader_start": None, 
			"leader_goal": None, 
			"follower_start": None, 
			"follower_goal": None
		}
		self.current_way_point = 0

		print("\nStarting point collection.\nEnter points as 2d Nav Goal in RVIZ in the following order:")
		print("\tLeader Start Postion ")
		print("\tLeader Goal Postion ")
		print("\tFollower Start Postion ")
		print("\tFollower Goal Postion ")


	def new_pt_cb(self, msg):
		self.current_way_point = self.current_way_point + 1
		if self.current_way_point == 1:
			self.way_points["leader_start"] = yaml.load(str(msg))
			print("Read Leader Start Position")
		elif self.current_way_point == 2:
			self.way_points["leader_goal"] = yaml.load(str(msg))
			print("Read Leader Goal Position")
		elif self.current_way_point == 3:
			self.way_points["follower_start"] = yaml.load(str(msg))
			print("Read Follower Start Position")
		elif self.current_way_point == 4:
			self.way_points["follower_goal"] = yaml.load(str(msg))
			print("Read Follower Goal Position")
			self.file = open(self.filepath, 'w')
			self.file.write(json.dumps(self.way_points, indent=4))
			# self.file.write("Test")
			self.file.close()
			print(self.filename + ".json file written.\n")
			self.current_way_point = 0
			self.way_points = {
				"leader_start": None, 
				"leader_goal": None, 
				"follower_start": None, 
				"follower_goal": None
			}


def main():
	rospy.init_node("set_pts", anonymous=True)
	filename = rospy.get_param("~filename")

	fw = File_Writer(filename)


	rospy.spin()

if __name__ == "__main__":
	main()