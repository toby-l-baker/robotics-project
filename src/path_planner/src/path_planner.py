import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from scipy.optimize import NonlinearConstraint

D = 2.0


master_start = (0, 0)
master_goal = (5, 2)
slave_start = (-1, -3)
slave_goal = (4, -5)

# master_start = (0, 0)
# master_goal = (1, 0)
# slave_start = (0, -1)
# slave_goal = (1, -1)


def total_dist(x, p):
	# All points are of form (x,y)
	# x is a flattened list of:
	# 		[drop start, drop end]
	# p is a flattened list of: 
	#		[master start, master end, slave start, slave end]
	x1 = np.array(x[0:2])
	x2 = np.array(x[2:4])
	p1 = np.array(p[0:2])
	p2 = np.array(p[2:3])
	p3 = np.array(p[4:6])
	p4 = np.array(p[6:8])
	return dist(p1, x1) + dist(p3, x1) + dist(p2, x2) + dist(p4, x2)

def dist_constraint(x, d):
	# All points are of form (x,y)
	# x is a flattened list of:
	# 		[drop start, drop end]
	# d is the distance needed for the drop
	# equality: dist(drop_start, drop_end) = d
	x1 = np.array(x[0:2])
	x2 = np.array(x[2:4])
	return dist(x1, x2) - d

def dist(p, q):
	return np.linalg.norm(p - q)

def path_planner(master_start, master_goal, slave_start, slave_goal, drop_distance):
	
	drop_start = [(master_start[0] - slave_start[0])/2, (master_start[1] - slave_start[1])/2]
	drop_end = [drop_start[0] + drop_distance, drop_start[1]]
	X = points_to_list(drop_start, drop_end)
	P = points_to_list(master_start, master_goal, slave_start, slave_goal)
	cons = [{'type':'eq', 'fun':dist_constraint, 'args':[D]}]
	ans = minimize(fun=total_dist, args=P, x0=X, constraints=cons)
	return ans

def plot_path(master_start, master_goal, slave_start, slave_goal, drop_start, drop_end):
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

	# plt.plot(drop_start, drop_end, 'ro-')

	plt.show()


def main():
	ans = path_planner(master_start, master_goal, slave_start, slave_goal, D)
	start = ans.x[0:2]
	end = ans.x[2:4]
	print(ans)

	
	plot_path(master_start, master_goal, slave_start, slave_goal, start, end)

def points_to_list(*pts):
	return [i for sub in pts for i in sub]

if __name__ == "__main__":
	main()
