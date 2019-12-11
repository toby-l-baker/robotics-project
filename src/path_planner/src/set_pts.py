#!/usr/bin/env python
import json
import yaml
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped as PS
from geometry_msgs.msg import Quaternion
import rospkg
from path_planner.msg import NavigationTargets
import tf
import numpy as np




Subscriber_Topic = "/way_points"
Publisher_Topic = "/path_plan_temp"


class File_Reader:
	def __init__(self, filename):
		rospack = rospkg.RosPack()
		rospack.list() 
		self.filename = filename
		self.filepath = rospack.get_path('path_planner') + '/waypoints/' + filename + '.json'
		self.nav_targets = NavigationTargets()
		self.has_nav_targets = False
		
		# with open(self.filepath) as json_file:
		json_file = open(self.filepath, "r")

		json_str = json_file.read()

		self.waypoints = json.loads(json_str)


		q = self.waypoints['leader_start']['pose']['orientation']
		quaternion = Quaternion(*[q['x'], q['y'], q['z'], q['w']])
		vec_q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		angle = vec_q[2]
		self.nav_targets.leader.initial = [
			self.waypoints['leader_start']['pose']['position']['x'],
			self.waypoints['leader_start']['pose']['position']['y'],
			angle
		]

		q = self.waypoints['leader_goal']['pose']['orientation']
		quaternion = Quaternion(*[q['x'], q['y'], q['z'], q['w']])
		vec_q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		angle = vec_q[2]
		self.nav_targets.leader.goal = [
			self.waypoints['leader_goal']['pose']['position']['x'],
			self.waypoints['leader_goal']['pose']['position']['y'],
			angle
		]

		q = self.waypoints['follower_start']['pose']['orientation']
		quaternion = Quaternion(*[q['x'], q['y'], q['z'], q['w']])
		vec_q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		angle = vec_q[2]
		self.nav_targets.follower.initial = [
			self.waypoints['follower_start']['pose']['position']['x'],
			self.waypoints['follower_start']['pose']['position']['y'],
			angle
		]

		q = self.waypoints['follower_goal']['pose']['orientation']
		quaternion = Quaternion(*[q['x'], q['y'], q['z'], q['w']])
		vec_q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		angle = vec_q[2]
		self.nav_targets.leader.initial = [
			self.waypoints['follower_goal']['pose']['position']['x'],
			self.waypoints['follower_goal']['pose']['position']['y'],
			angle
		]

		self.has_nav_targets = True


	
	def get_nav_targets(self):
		if self.has_nav_targets:
			return self.nav_targets
		else:
			return None


class File_Writer:
	def __init__(self, filename):
		rospack = rospkg.RosPack()
		rospack.list() 
		self.filename = filename
		self.filepath = rospack.get_path('path_planner') + '/waypoints/' + filename + '.json'
		
		self.subscriber = rospy.Subscriber(Subscriber_Topic, PS, self.new_pt_cb) 
		self.nav_targets = NavigationTargets()
		self.has_nav_targets = False
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
		position = yaml.load(str(msg))['pose']['position']
		q = yaml.load(str(msg))['pose']['orientation']
		quaternion = Quaternion(*[q['x'], q['y'], q['z'], q['w']])

		vec_q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		angle = vec_q[2]
		pos = [position['x'], position['y']]
		if self.current_way_point == 1:
			self.way_points["leader_start"] = yaml.load(str(msg))
			self.nav_targets.leader.initial = [pos[0], pos[1], angle]
			print("Read Leader Start Position")
		elif self.current_way_point == 2:
			self.way_points["leader_goal"] = yaml.load(str(msg))
			self.nav_targets.leader.goal = [pos[0], pos[1], angle]
			print("Read Leader Goal Position")
		elif self.current_way_point == 3:
			self.way_points["follower_start"] = yaml.load(str(msg))
			self.nav_targets.follower.initial = [pos[0], pos[1], angle]
			print("Read Follower Start Position")
		elif self.current_way_point == 4:
			self.way_points["follower_goal"] = yaml.load(str(msg))
			self.nav_targets.follower.goal = [pos[0], pos[1], angle]
			print("Read Follower Goal Position")
			self.file = open(self.filepath, 'w')
			self.file.write(json.dumps(self.way_points, indent=4))
			# self.file.write("Test")
			self.file.close()
		
			self.has_nav_targets = True
			print(self.filename + ".json file written and /start_goal targets published.\n")
			
			self.current_way_point = 0
			self.way_points = {
				"leader_start": None, 
				"leader_goal": None, 
				"follower_start": None, 
				"follower_goal": None
			}
	
	def get_nav_targets(self):
		if self.has_nav_targets:
			return self.nav_targets
		else:
			return None

def create_nav_targets(x, y, theta):
	nt = NavigationTargets()

def main():
	rospy.init_node("set_pts", anonymous=True)

	publisher = rospy.Publisher(Publisher_Topic, NavigationTargets, queue_size=10, latch=True)

	filename = rospy.get_param("~filename")
	gui_flag = rospy.get_param("~gui")

	targets = None

	if gui_flag:
		fw = File_Writer(filename)
		while not rospy.is_shutdown() and targets is None:
			targets = fw.get_nav_targets()
			rospy.sleep(0.5)
	else:
		fr = File_Reader(filename)
		while not rospy.is_shutdown() and targets is None:
			targets = fr.get_nav_targets()
			rospy.sleep(0.5)

 	publisher.publish(targets)
 
	rospy.spin()

if __name__ == "__main__":
	main()