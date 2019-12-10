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
from geometry_msgs.msg import PoseWithCovarianceStamped
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
        self.state_initial = True
        initial_ack_topic = rospy.get_param('~initial_ack')
        follow_ack_topic = rospy.get_param('~follow_ack')
        final_ack_topic = rospy.get_param('~final_ack')
        node_ready_topic = rospy.get_param('~node_ready')
        self.initial_ack = rospy.Publisher(initial_ack_topic, String, queue_size=1, latch=True)
        self.follow_ack = rospy.Publisher(follow_ack_topic, String, queue_size=1, latch=True)
        self.final_ack = rospy.Publisher(final_ack_topic, String, queue_size=1, latch=True)
        self.initial_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher(node_ready_topic, String, queue_size=1, latch=True)


        rospy.Subscriber("/path_plan", NavigationTargets, self.path_plan_cb)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.move_base_status_cb)

        # Subscribes to state changes of State Machine
        rospy.Subscriber('/state', String, self.state_change_callback)
        # Publishes ready check to State Machine

        if self.type_ == 'Leader':
            # Sets Leader_move ready
                self.ready_pub.publish("LEADER {}".format(name))
        if self.type_ == 'Follower':
            # Sets Leader_move ready
                self.ready_pub.publish("FOLLOWER {}".format(name))

    def move_base_status_cb(self, msg):
        print(msg)
        for i in msg.status_list:
            if i.status == GoalStatus.SUCCEEDED:
                self.last_done = True
                print("Woot")
                if "initial" in self.my_state:
                    self.initial_ack.pub("DONE")
                elif "follow" in self.my_state:
                    self.follow_ack.pub("DONE")
                elif "final" in self.my_state:
                    self.final_ack.pub("DONE")

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
        pose_stamped = xytheta_to_pose(*self.start_pos)
        msg = PoseWithCovarianceStamped()
        msg.header = pose_stamped.header
        msg.pose.pose = pose_stamped.pose
        self.initial_pose.publish(msg)

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
            self.my_state = "initial"
            self.move_transfer_start()
            while(self.last_done == False):
                pass
            self.initial_ack.publish("DONE " + self.name)
            self.last_done = False
        elif(msg.data[0:6] == state_names.FOLLOW):
            print("In follow state")
            self.move_transfer_end()
            self.my_state = "follow"
            while(self.last_done == False):
                pass
            self.follow_ack.publish("DONE " + self.name)
            self.last_done = False
        elif(msg.data == state_names.FINAL):
            print("In final state")
            self.my_state = "final"
            self.move_end()
            while(self.last_done == False):
                pass
            self.final_ack.publish("DONE " + self.name)
            self.last_done = False

def main():
    rospy.init_node("move", anonymous=True)

    name = rospy.get_param("~robot_name")
    type_ = rospy.get_param("~type")
    move = TB_Move(name, type_)
    rospy.spin()

if __name__ == "__main__":
    main()
