#!/usr/bin/env python
import rospy 
from turtle_bot_init import *
import numpy as np
from path_planner import *
Node = "move"

# master_start = (0, 0, 0)
# master_goal = (1, 0, 0)
# slave_start = (0.2, 1, 0)
# slave_goal = (1, 1, 0)

master_start = (0, -1.3, 0)
master_goal = (0, 0.9, 0)
slave_start = (1, -1.25, 0)
slave_goal = (1, 1, 0)


min_drop_dist = 1.0

def create_path(lead_follow, name):
	# tb = Turtlebot(name)
	path_start, path_end = path_planner(master_start, master_goal, slave_start, slave_goal, min_drop_dist)
	

	# print(get_path_angle(path_start, path_end))
	# plot_path(master_start, master_goal, slave_start, slave_goal, path_start, path_end)
	publish_path_plan(path_start, path_end)

	# print(path)


def main():
	rospy.init_node(Node, anonymous=True)
	create_path("lead", "black")
	rospy.spin()
	# leader or follower
	# turtlebot color
	# 


if __name__ == "__main__":
	main()