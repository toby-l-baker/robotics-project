#!/usr/bin/env python
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from scipy.optimize import NonlinearConstraint
import json
import rospy
from std_msgs.msg import String
from path_planner.msg import NavigationTargets


# Path_Plan_Topic = 'path_plan'
TB_Seperation_Dist = 0.75
#
# master_start = (0, -1.3, 0)
# master_goal = (0, 0.9, 0)
# slave_start = (1, -1.25, 0)
# slave_goal = (1, 1, 0)
#
# min_drop_dist = 1.0

Subscriber_Topic = "/path_plan"


def total_dist(x, p):
	# All points are of form (x,y)
	# x is a flattened list of:
	# 		[drop start, drop end]
	# p is a flattened list of:
	#		[master start, master end, slave start, slave end]
	x1 = np.array(x[0:2])
	x2 = np.array(x[2:4])
	p1 = np.array(p[0:2])
	p2 = np.array(p[2:4])
	p3 = np.array(p[4:6])
	p4 = np.array(p[6:8])
	return dist(p1, x1) + dist(p3, x1) + dist(p2, x2) + dist(p4, x2) + dist(x1,x2)

def dist_constraint(x, d):
	# All points are of form (x,y)
	# x is a flattened list of:
	# 		[drop start, drop end]
	# d is the distance needed for the drop
	# equality: dist(drop_start, drop_end) = d
	x1 = np.array(x[0:2])
	x2 = np.array(x[2:4])
	return dist(x1, x2) - d

def non_linear_dist_constraint(x):
	# All points are of form (x,y)
	# x is a flattened list of:
	# 		[drop start, drop end]
	# d is the distance needed for the drop
	# equality: dist(drop_start, drop_end) = d
	x1 = np.array(x[0:2])
	x2 = np.array(x[2:4])
	return dist(x1, x2)

def non_linear_jacobian_constraint(x):
	# x is a flattened list of:
	# 		[drop start, drop end]
	x_start = [x[0], x[1]]
	x_goal = [x[2], x[3]]
	return [x[0]-x[2], x[1]-x[3], x[0]-x[2], x[1]-x[3]] / dist(np.array(x_start), np.array(x_goal))

def dist(p, q):
	# Euclidean distance between two points of the form (x, y)
	return np.linalg.norm(p - q)


def get_path_angle(start, end):
	path_vector = np.subtract(end, start)
	return np.round(np.arctan2(path_vector[1], path_vector[0])*180.0/np.pi, 1)

# def path_planner(master_start, master_goal, slave_start, slave_goal, drop_distance):
# 	# All locations are of the form (x,y,theta) or (x,y)
# 	# Drop distance is in meters
# 	drop_start = [(master_start[0] - slave_start[0])/2, (master_start[1] - slave_start[1])/2]
# 	drop_end = [drop_start[0] + drop_distance, drop_start[1]]
# 	X = points_to_list(drop_start, drop_end)
# 	P = points_to_list(master_start[0:2], master_goal[0:2], slave_start[0:2], slave_goal[0:2])
#
# 	nl_cons = NonlinearConstraint(non_linear_dist_constraint, drop_distance, 100)
# 	options = {'disp':False}
# 	ans = minimize(fun=total_dist, args=P, x0=X, method='trust-constr', constraints=nl_cons, options=options)
# 	# print(total_dist(ans.x,P))
# 	angle = get_path_angle(ans.x[0:2], ans.x[2:4])
#
# 	return (ans.x[0], ans.x[1], angle), (ans.x[2], ans.x[3], angle)

def plot_path(master_start, master_goal, slave_start, slave_goal, drop_start, drop_end):
	# Plotting utility to visualize the 2d path calulated
	plt.plot(master_start[0], master_start[1], 'ro')
	plt.text(master_start[0], master_start[1], 'Master Start')

	plt.plot(master_goal[0], master_goal[1], 'ro')
	plt.text(master_goal[0], master_goal[1], 'Master Goal')

	plt.plot(slave_start[0], slave_start[1], 'ro')
	plt.text(slave_start[0], slave_start[1], 'Slave Start')

	plt.plot(slave_goal[0], slave_goal[1], 'ro')
	plt.text(slave_goal[0], slave_goal[1], 'Slave Goal')

	# plt.plot(drop_start[0], drop_start[1], 'go')
	# plt.text(drop_start[0], drop_start[1], 'Drop Start')

	# plt.plot(drop_end[0], drop_end[1], 'go')
	# plt.text(drop_end[0], drop_end[1], 'Drop End')

	plt.plot([drop_start[0], drop_end[0]], [drop_start[1], drop_end[1]], 'g--', label='Transfer path')
	plt.legend()
	plt.show()

def offset_coords(point, angle, tb_separation):
	# Calculates the offset for two points to seperate the turtlebots
	# Point is a coordinate in the form (x, y, theta)
	delta = (tb_separation * np.cos(angle * np.pi/180), tb_separation * np.sin(angle * np.pi/180), 0)
	return np.subtract(point, delta).tolist()


def points_to_list(*pts):
	# Flattens an arbitrary number of points (x,y) into a list
	return [i for sub in pts for i in sub]



class PathPlanner():
	def __init__(self):
		rospy.init_node("path_planner", anonymous=True)

		
		self.nav_targets = NavigationTargets()

		# Get the length of the straight line path
		self.min_drop_dist = float(rospy.get_param("~min_drop_dist"))
		self.tb_separation = float(rospy.get_param("~tb_separation"))

		# Get the topic to publish the plans to
		self.path_topic = rospy.get_param("~path_topic")
		self.pub = rospy.Publisher(self.path_topic, NavigationTargets, queue_size=1, latch=True)
		self.sub = rospy.Subscriber(self.path_topic, NavigationTargets, self.path_plan_cb) 




		# Initialise message to published and setup initial and goal poses
	




	def path_planner(self):
		# drop_start = [(self.master_start_pos[0] - self.follower_start_pos[0])/2, (self.master_start_pos[1] - self.follower_start_pos[1])/2]
		drop_start = [(self.nav_targets.leader.initial[0] - self.nav_targets.follower.initial[0])/2, 
			(self.nav_targets.leader.initial[1] - self.nav_targets.follower.initial[1])/2]
		drop_end = [drop_start[0] + self.min_drop_dist, drop_start[1]]
		X = points_to_list(drop_start, drop_end)
		P = points_to_list(self.nav_targets.leader.initial[0:2], self.nav_targets.leader.goal[0:2], 
			self.nav_targets.follower.initial[0:2], self.nav_targets.follower.goal[0:2])

		nl_cons = NonlinearConstraint(non_linear_dist_constraint, self.min_drop_dist, 100)
		options = {'disp':False}
		ans = minimize(fun=total_dist, args=P, x0=X, method='trust-constr', constraints=nl_cons, options=options)
		# print(total_dist(ans.x,P))
		angle = get_path_angle(ans.x[0:2], ans.x[2:4])

		# Store the path start and end
		path_start = (ans.x[0], ans.x[1], angle)
		path_end = (ans.x[2], ans.x[3], angle)

		# Generate start and end of line for both turtlebots and store in nav message
		self.nav_targets.follower.line_start = offset_coords(path_start, angle, self.tb_separation)
		self.nav_targets.follower.line_end = offset_coords(path_end, angle, self.tb_separation)
		self.nav_targets.leader.line_start = path_start
		self.nav_targets.leader.line_end = path_end

		self.publish_path_plan()

	def path_plan_cb(self, msg):
		self.nav_targets.leader.initial = msg.leader.initial
		self.nav_targets.leader.goal = msg.leader.goal
		self.nav_targets.follower.initial = msg.follower.initial
		self.nav_targets.follower.goal = msg.follower.goal
		self.path_planner
	
	def publish_path_plan(self):
		plot_path(self.master_start_pos, self.master_goal_pos, self.follower_start_pos, self.follower_goal_pos, self.nav_targets.leader.line_start, self.nav_targets.leader.line_end)
		self.pub.publish(self.nav_targets)
		print("Publshed Navigation Plan")




if __name__ == "__main__":
	planner = PathPlanner()


	# planner.publish_path_plan()
	rospy.spin()
