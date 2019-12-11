#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from advanced_turtlebot_follower import TurtlebotFollower
from turtle_bot_init import Turtlebot
from geometry_msgs.msg import Twist, Point

# Point.x = leader state, Point.y = follower state

class LeaderFollower():

    def __init__(self):
        self.type_ = rospy.get_param("~type")
        self.name = rospy.get_param("~name")
        self.speed = rospy.get_param("~speed")
        self.follower = TurtlebotFollower()
        self.tb = Turtlebot(self.name, self.type_)

        # Setup velociy publisher
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=1)
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = float(self.speed)
        self.transfer_done = False # For seeing if follower is complete
        self.move_base_complete = False

        # Setup state topics
        self.state_topic = "/state"
        self.state = Point()
        self.state_publisher = rospy.Publisher(self.state_topic, Point, queue_size=1)
        self.state_subscriber = rospy.Subscriber(self.state_topic, Point, self.state_callback)

        #Subsribe to path plan and move base
        rospy.Subscriber("/path_plan", NavigationTargets, self.path_plan_cb)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.move_base_status_cb)

        # Dummy variables for plan
        self.start_pos = [0, 0, 0]
        self.end_pos = [0, 0, 0]
        self.transfer_start_pos = [0, 0, 0]
        self.transfer_end_pos = [0, 0, 0]

    def get_state(self):
        return (int(self.state.x), int(self.state.y))

    def move_base_status_cb(self, msg):
        print(msg)
        for i in msg.status_list:
            if i.status == GoalStatus.SUCCEEDED:
            	if not self.move_base_complete:
	                if "Leader" in self.type_:
	                    self.state.x += 1
	                elif "Follower" in self/type_:
	                    self.state.y += 1
	                self.state_publisher.publish(self.state)
	                self.move_base_complete = True
            else:
            	self.move_base_complete = False

    def state_callback(self, msg):
        if msg.z == 1:
            # force update
            self.state = msg
            self.state.z = 0
        else:
            if "Leader" in self.type_:
                self.state.y = int(msg.y) # only update follower state
            elif "Follower" in self.type_:
                self.state.x = int(msg.x) # only update leader state

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
        if "Leader" in self.type_:
            self.state.x += 1
        elif "Follower" in self/type_:
            self.state.y += 1
        self.state_publisher.publish(self.state)
        print("Path plan received")
        # pose_stamped = xytheta_to_pose(*self.start_pos)
        # msg = PoseWithCovarianceStamped()
        # msg.header = pose_stamped.header
        # msg.pose.pose = pose_stamped.pose


def main():
    rospy.init_node("move_tb", anonymous=True)
    move_tb = LeaderFollower()

    while not rospy.is_shutdown():

        if "Leader" in move_tb.type_:
            '''Move turtlebot to initial post when both ready'''
            if (move_tb.state.x == 1 and move_tb.state.y == 1):
            	'''Move to position'''
                move_tb.tb.move(*move_tb.transfer_start_pos)
                '''Wait'''
                while not move_tb.move_base_complete:
                    pass
                move_tb.timestamp_1 = rospy.Time.now()

            elif (move_tb.state.x == 2 and move_tb.state.y == 2): # TODO setup a timer here
            	'''Publish commended velocity for 5 seconds'''
                move_tb.cmd_vel_pub.publish(move_tb.cmd_vel)
                move_tb.timestamp_2 = rosp.Time.now()
                if (move_tb.timestamp_2.secs - move_tb.timestamp_1.secs) > rospy.Duration(4.0):
                    move_tb.state.x += 1
                    move_tb.state_publisher.publish(move_tb.state)

            elif (move_tb.state.x == 3 and move_tb.state.y == 3):
            	'''Move to final position'''
                move_tb.move_tb.move.tb(move_tb.end_pos) 
                '''Wait'''
                while move_tb.move_base_complete:
                    pass

        elif "Follower" in move_tb.type_:
            if (move_tb.state.x == 1 and move_tb.state.y == 1) and not (move_tb.move_base_complete):
            	'''Move to position'''
                move_tb.tb.move(*move_tb.transfer_start_pos)
                '''Wait'''
                while move_tb.move_base_complete:
                    pass

            elif (move_tb.state.x == 2 and move_tb.state.y == 2) or move_tb.move_base_complete:
            	'''Follow the other robot until transfer done'''
                move_tb.transfer_done = move_tb.follower.run_to_completion()
                '''Update state once complete'''
                if move_tb.transfer_done:
                    move_tb.state.y+= 1
                    move_tbmove_tb.state_publisher.publish(move_tb.state)

            elif (move_tb.state.x == 3 and move_tb.state.y == 3):
            	'''Moev to final position'''
                move_tb.tb.move(move_tb.end_pos)
                '''Wait'''
                while move_tb.move_base_complete:
                    pass


if __name__ == "__main__":
    main()

	    	

