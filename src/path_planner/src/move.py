#!/usr/bin/env python
import rospy
from turtle_bot_init import *
# How to import from diff folder? 
import sys
# insert at 1, 0 is the script path (or '' in REPL)
import state_names
import numpy as np
from std_msgs.msg import String
import json
from path_planner.msg import NavigationTargets
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

class TB_Move:
	def __init__(self, name, type_):
		self.name = name
		self.type_ = type_
		self.tb = Turtlebot(name, type_)
		self.last_done = False
		self.start_pos = [0, 0, 0]
		self.end_pos = [0, 0, 0]
		self.transfer_start_pos = [0, 0, 0]
		self.transfer_end_pos = [0, 0, 0]
		rospy.Subscriber("/path_plan", NavigationTargets, self.path_plan_cb)
		rospy.Subscriber("move_base/status", GoalStatusArray, self.move_base_status_cb)

		# Subscribes to state changes of State Machine
		rospy.Subscriber('/state', String, self.state_change_callback)
		self.initial_ack = rospy.Publisher('Initial_ack', String, queue_size=1)
		self.follow_ack = rospy.Publisher('Follow_ack', String, queue_size=1)
		self.final_ack = rospy.Publisher('Final_ack', String, queue_size=1)
		# Publishes ready check to State Machine
		ready_pub = rospy.Publisher('/node_ready', String, queue_size=1)		
		if self.type_ == 'Leader':
		# Sets Leader_move ready
			ready_pub.publish("LEADER")
		elif self.type_ == 'Follower':
		# Sets follower_move ready
			ready_pub.publish("FOLLOWER")

	def move_base_status_cb(self, msg):
		print(msg)
		for i in msg.status_list:
			if i.status == GoalStatus.SUCCEEDED:
				self.last_done = True
				print("Woot")
		#if msg.status_list[0].goal_id.status == GoalStatus.SUCCEEDED:
		#	print("WOOT")
		# http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html


	def done(self):
		# TODO check if move_base is finished
		return True

	def path_plan_cb(self, data):
		# Gets path plan from path_planner node
		if self.type_ == "Follower":
			self.start_pos = data.follower.initial
			self.end_pos = data.follower.goal
			self.transfer_start_pos = data.follower.line_start
			self.transfer_end_pos = data.follower.line_end
		elif self.type_ == "Leader":
			self.start_pos = data.leader.initial
			self.end_pos = data.leader.goal
			self.transfer_start_pos = data.leader.line_start
			self.transfer_end_pos = data.leader.line_end
		print("Path plan received")
		# self.tb.move(*self.transfer_start_pos)
	def move_transfer_start(self):
		# moves to start of transfer 
		self.tb.move(*self.transfer_start_pos)
	def move_transfer_end(self):
		# moves to end of transfer. Note if TB is follower, the follow node takes over.
		if(self.type_ == "Leader"):
			self.tb.move(*self.transfer_end_pos)
	def move_end(self):
		# moves to final goal
		self.tb.move(*self.end_pos)
	def state_change_callback(self, msg):
		if(msg.data == state_names.IDLE):
			print("In idle state")
		elif(msg.data == state_names.INITIAL):
			print("In initial state")
			self.move_transfer_start()
			while(self.last_done == False):
				pass
			initial_ack.publish("DONE " + self.name)
			self.last_done = False
		elif(msg.data[0:6] == state_names.FOLLOW):
			print("In follow state")
			self.move_transfer_end()
			while(self.last_done == False):
				pass
			follow_ack.publish("DONE " + self.name)
			self.last_done = False
		elif(msg.data == state_names.FINAL):
			print("In final state")
			self.move_end()
			while(self.last_done == False):
				pass
			final_ack.publish("DONE " + self.name)
			self.last_done = False


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

if __name__ == "__main__":
	main()
