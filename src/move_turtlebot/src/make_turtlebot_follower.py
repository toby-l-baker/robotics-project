#!/usr/bin/env python

import rospy
import sys
from turtlebot_follower import TurtlebotFollower
from geometry_msgs.msg import Twist
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Turtlebot following')
    parser.add_argument('ar_marker_frame')
    parser.add_argument('turlebot_base_frame')
    parser.add_argument('turtlebot_to_follow_frame')
    args = parser.parse_args()

    follower = TurtlebotFollower(args.ar_marker_frame, args.turtlebot_base_frame, args.turtlebot_to_follow_frame)
