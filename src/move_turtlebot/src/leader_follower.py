#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from turtle_bot_init import Turtlebot
from geometry_msgs.msg import Twist, Point
from path_planner.msg import NavigationTargets

# Point.x = leader state, Point.y = follower state

class LeaderFollower():

    def __init__(self):
        self.type_ = rospy.get_param("~type")
        self.name = rospy.get_param("~name")
        self.speed = rospy.get_param("~speed")
        # Only yellow red or green Turtlebots have the Raspberry Pi to use GPIO pins
        if "yellow" or "red" or "green" in self.name:
            from advanced_turtlebot_follower import TurtlebotFollower
            self.follower = TurtlebotFollower()
        self.tb = Turtlebot(self.name, self.type_)

        # Setup velociy publisher
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = float(self.speed)
        self.transfer_done = False # For seeing if follower is complete
        self.move_base_complete = False

        # Setup state topics
        self.state_topic = "/state"
        self.state = Point()
        self.transfer_done = False
        self.state_subscriber = rospy.Subscriber(self.state_topic, Point, self.state_callback)

        #Subsribe to path plan and move base
        rospy.Subscriber("/path_plan", NavigationTargets, self.path_plan_cb)

        # Initial variables for plan
        self.start_pos = [0, 0, 0]
        self.end_pos = [0, 0, 0]
        self.transfer_start_pos = [0, 0, 0]
        self.transfer_end_pos = [0, 0, 0]

    def get_state(self):
        return (int(self.state.x), int(self.state.y))

    def state_callback(self, msg):
        if msg.z == 1:
            # force update
            self.state = msg
            self.transfer_done = False

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


def main():
    rospy.init_node("move_tb", anonymous=True)
    move_tb = LeaderFollower()

    while not rospy.is_shutdown():

        if "Leader" in move_tb.type_:
            '''Move turtlebot to initial post when both ready'''
            if (move_tb.state.x == 1 and move_tb.state.y == 1):
                '''Move to position'''
                print("State 1")
                move_tb.tb.move(*move_tb.transfer_start_pos)
                '''Wait'''
                while move_tb.state.x == 1:
                    pass
                move_tb.timestamp_1 = rospy.Time.now()

            elif (move_tb.state.x == 2 and move_tb.state.y == 2):
                '''Publish commended velocity for 5 seconds'''
                move_tb.timestamp_2 = rospy.Time.now()
                if (move_tb.timestamp_2.secs - move_tb.timestamp_1.secs) < 6.0:
                    print("Sending Commands")
                    move_tb.cmd_vel_pub.publish(move_tb.cmd_vel)

            elif (move_tb.state.x == 3 and move_tb.state.y == 3):
                '''Move to final position'''
                print("State 3")
                move_tb.tb.move(*move_tb.end_pos)
                '''Wait'''
                move_tb.move_base_complete = False
                while move_tb.state.x == 3:
                    pass

        elif "Follower" in move_tb.type_:
            if (move_tb.state.x == 1 and move_tb.state.y == 1) and not (move_tb.move_base_complete):
            	'''Move to position'''
                print("State 1")
                move_tb.tb.move(*move_tb.transfer_start_pos)
                move_tb.transfer_done = False
                '''Wait'''
                while move_tb.state.y == 1:
                    pass

            elif (move_tb.state.x == 2 and move_tb.state.y == 2) or move_tb.move_base_complete:
            	'''Follow the other robot until transfer done'''
                # print("State 2")
                if move_tb.transfer_done == False:
                    print("State 2")
                    move_tb.transfer_done = move_tb.follower.run_to_completion()
                    print("State 2 Done")

            elif (move_tb.state.x == 3 and move_tb.state.y == 3):
            	'''Moev to final position'''
                print("State 3")
                move_tb.tb.move(*move_tb.end_pos)
                '''Wait'''
                while move_tb.state.y == 3:
                    pass

if __name__ == "__main__":
    main()
