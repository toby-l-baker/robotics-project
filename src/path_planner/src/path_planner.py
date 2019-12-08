#!/usr/bin/env python
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from scipy.optimize import NonlinearConstraint
import json
import rospy
from std_msgs.msg import String

Node = "path_planner"
Path_Plan_Topic = 'path_plan'
TB_Seperation_Dist = 0.75

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

def path_planner(master_start, master_goal, slave_start, slave_goal, drop_distance):
	# All locations are of the form (x,y,theta) or (x,y)
	# Drop distance is in meters
	drop_start = [(master_start[0] - slave_start[0])/2, (master_start[1] - slave_start[1])/2]
	drop_end = [drop_start[0] + drop_distance, drop_start[1]]
	X = points_to_list(drop_start, drop_end)
	P = points_to_list(master_start[0:2], master_goal[0:2], slave_start[0:2], slave_goal[0:2])

	nl_cons = NonlinearConstraint(non_linear_dist_constraint, drop_distance, 100)
	options = {'disp':False}
	ans = minimize(fun=total_dist, args=P, x0=X, method='trust-constr', constraints=nl_cons, options=options)
	# print(total_dist(ans.x,P))
	angle = get_path_angle(ans.x[0:2], ans.x[2:4])

	return (ans.x[0], ans.x[1], angle), (ans.x[2], ans.x[3], angle)

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

	plt.plot(drop_start[0], drop_start[1], 'go')
	plt.text(drop_start[0], drop_start[1], 'Drop Start')

	plt.plot(drop_end[0], drop_end[1], 'go')
	plt.text(drop_end[0], drop_end[1], 'Drop End')

	plt.plot([drop_start[0], drop_end[0]], [drop_start[1], drop_end[1]], 'go-')

	plt.show()

def offset_coords(point, angle):
	# Calculates the offset for two points to seperate the turtlebots
	# Point is a coordinate in the form (x, y, theta)
	delta = (TB_Seperation_Dist * np.cos(angle * np.pi/180), TB_Seperation_Dist * np.sin(angle * np.pi/180), 0)
	return np.subtract(point, delta).tolist()
	

def points_to_list(*pts):
	# Flattens an arbitrary number of points (x,y) into a list
	return [i for sub in pts for i in sub]

def publish_path_plan(path_start, path_end):
	# rospy.init_node(Node, anonymous=True)
	angle = get_path_angle(path_start, path_end)
	lead_start = path_start
	lead_end = path_end
	follower_start = offset_coords(path_start, angle)
	follower_end = offset_coords(path_end, angle)
	
	msg_json_str = json.dumps({"Leader":{'Start':lead_start, 'End':lead_end}, 
								"Follower":{'Start': follower_start, 'End': follower_end}})
	pub = rospy.Publisher(Path_Plan_Topic, String, queue_size=1, latch=True)
	pub.publish('Test')
	rospy.spin()

	print("JSON message published.")

	