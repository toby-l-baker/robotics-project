#!/usr/bin/env python
import rospy
from advanced_turtlebot_follower import TurtlebotFollower
from turtle_bot_init import Turtlebot
from move_turtlebot.msgs import state_msg
from geometry_msgs.msg import Twist

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
        self.state = state_msg()
        self.state_publisher = rospy.Publisher(self.state_topic, state_msg, queue_size=1)
        self.state_subscriber = rospy.Subscriber(self.state_topic, state_msg)
        #Subsribe to path plan and move base
        rospy.Subscriber("/path_plan", NavigationTargets, self.path_plan_cb)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.move_base_status_cb)
        # Dummy variables for plan
        self.start_pos = [0, 0, 0]
        self.end_pos = [0, 0, 0]
        self.transfer_start_pos = [0, 0, 0]
        self.transfer_end_pos = [0, 0, 0]

    def move_base_status_cb(self, msg):
        print(msg)
        for i in msg.status_list:
            if i.status == GoalStatus.SUCCEEDED:
                if "Leader" in self.type_:
                    self.state.states[0] += 1
                elif "Follower" in self/type_:
                    self.state.states[1] += 1
                self.state_publisher.publish(self.state)
                self.move_base_complete = True

    def state_callback(self, msg):
        if "Leader" in self.type_:
            self.state.states[1] = msg.states[1] # only update follower state
        elif "Follower" in self.type_:
            self.state.states[0] = msg.states[0] # only update leader state

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
            self.state.states[0] += 1
        elif "Follower" in self/type_:
            self.state.states[1] += 1
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
                if (move_tb.state.states[0] == 1 and move_tb.state.states[1] == 1) and not (move_tb.move_base_complete):
                    move_tb.tb.move(*move_tb.transfer_start_pos)
                    while move_tb.state.states[0] <= 1:
                        pass
                    move_tb.timestamp_1 = rospy.Time.now()

                elif (move_tb.state.states[0] == 2 and move_tb.state.states[1] == 2) or (move_tb.move_base_complete): # TODO setup a timer here
                    move_tb.cmd_vel_pub.publish(move_tb.cmd_vel)
                    move_tb.timestamp_2 = rosp.Time.now()
                    if (move_tb.timestamp_2.secs - move_tb.timestamp_1.secs) > 5:
                        move_tb.state.states[0] += 1
                        move_tb.state_publisher.publish(move_tb.state)

                elif move_tb.state.states[0] == 3 and move_tb.state.states[1] == 3:
                    move_tb.move_tb.move.tb(move_tb.end_pos) 
                    while move_tb.state.states[0] <= 3:
                        pass
            elif "Follower" in move_tb.type_:
                if (move_tb.state.states[0] == 1 and move_tb.state.states[1] == 1) and not (move_tb.move_base_complete):
                    move_tb.tb.move(*move_tb.transfer_start_pos)
                    while move_tb.state.states[1] <= 1:
                        pass

                elif (move_tb.state.states[0] == 2 and move_tb.state.states[1] == 2) or move_tb.move_base_complete:
                    move_tb.transfer_done = move_tb.follower.run_to_completion()
                    if move_tb.transfer_done:
                        move_tb.state.states[1] += 1
                        move_tbmove_tb.state_publisher.publish(move_tb.state)

                elif move_tb.state.states[0] == 3 and move_tb.state.states[1] == 3:
                    move_tb.tb.move(move_tb.end_pos)
                    while move_tb.state.states[1] <= 3:
                        pass


if __name__ == "__main__":
    main()

	    	

