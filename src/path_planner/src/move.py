#!/usr/bin/env python
import rospy
from turtle_bot_init import *
import numpy as np
from std_msgs.msg import String
import json
from path_planner.msg import NavigationTargets
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

# master_start = (0, 0, 0)
# master_goal = (1, 0, 0)
# # slave_start = (0.2, 1, 0)
# # slave_goal = (1, 1, 0)

# master_start = (0, -1.3, 0)
# master_goal = (0, 0.9, 0)
# slave_start = (1, -1.25, 0)
# slave_goal = (1, 1, 0)


# min_drop_dist = 1.0


class TB_Move:
	def __init__(self, name, type_):
		self.tb = Turtlebot(name, type_)
		rospy.Subscriber("path_plan", String, self.path_plan_cb)
		rospy.Subsrciber("move_base/status", GoalStatusArray, self.move_base_status_cb)

	def start(self):
		self.tb.move()

	def move_base_status_cb(self, data):
		if data.goal_id.status = GolaStatus.SUCCEEDED:
			pass # do state transition
		# http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html


	def done(self):
		# TODO check if move_base is finished
		return True

	# def path_plan_cb(self, plan_json):
	# 	path_plan = json.loads(plan_json.data)
	# 	self.start_pos = path_plan[self.tb.type_]['Start']
	# 	self.end_pos = path_plan[self.tb.type_]['End']
	# 	self.transfer_start_pos = path_plan[self.tb.type_]['Transfer_Start']
	# 	self.transfer_end_pos = path_plan[self.tb.type_]['Transfer_End']
	# 	self.tb.move(*self.transfer_start_pos)
	def path_plan_cb(self, data):
		if self.type_ == "Follower":
			self.start_pos = data.follower.initial
			self.end_pos = data.follower.goal
			self.transfer_start_pos = data.follower.line_start
			self.transfer_end_pos = data.follower.line_start
		elif self.type_ == "Leader":
			self.start_pos = data.leader.initial
			self.end_pos = data.leader.goal
			self.transfer_start_pos = data.leader.line_start
			self.transfer_end_pos = data.leader.line_start


# def create_path(lead_follow, name):
# 	red = Turtlebot("red")
# 	black = Turtlebot("black")
#
# 	path_start, path_end = path_planner(master_start, master_goal, slave_start, slave_goal, min_drop_dist)
#
#
# 	# print(get_path_angle(path_start, path_end))
# 	# plot_path(master_start, master_goal, slave_start, slave_goal, path_start, path_end)
# 	publish_path_plan(path_start, path_end)

	# print(path)

# def move(data):


# def ghetto_tb_move_test(lead, follower, path_start, path_end):
# 	lead_start = path_start
# 	lead_end = path_end
# 	angle = get_path_angle(path_start, path_end)
# 	follower_start = offset_coords(path_start, angle)
# 	follower_end = offset_coords(path_end, angle)

# 	leader.move(*lead_start)
# 	follwer.move(*follower_start)

def main():
	rospy.init_node("move", anonymous=True)

	name = rospy.get_param("~robot_name")
	type_ = rospy.get_param("~type")
	move = TB_Move(name, type_)
	rospy.spin()
	# leader or follower
	# turtlebot color
	#


if __name__ == "__main__":
	main()
