
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *


node_name = 'nav_test'
frame_id = "map"

def shutdown():
	rospy.loginfo("Stop")

def move(x = 0, y = 0, w = 0):
	rospy.init_node(node_name, anonymous=False)
	rospy.on_shutdown(shutdown)
	#tell the action client that we want to spin a thread by default
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("wait for the action server to come up")
	#allow up to 5 seconds for the action server to come up
	move_base.wait_for_server(rospy.Duration(5))
	

	#we'll send a goal to the robot to move 3 meters forward
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = frame_id
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x #3 meters
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.orientation.w = w #go forward
	print("Goal is set up")
	#start moving
	move_base.send_goal(goal)

	#allow TurtleBot up to 60 seconds to complete task
	success = move_base.wait_for_result(rospy.Duration(60)) 


	if not success:
                move_base.cancel_goal()
                rospy.loginfo("The base failed to move forward 3 meters for some reason")
    	else:
		# We made it!
		state = move_base.get_state()
		if state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Hooray, the base moved 3 meters forward")

if __name__ == "__main__":
	try:
		move(1, 0, 1)
	except rospy.ROSInterruptException:
		rospy.loginfo("Exception thrown")