#!/usr/bin/env python

import rospy
import sys
from naive_turtlebot_follower import TurtlebotFollower
from geometry_msgs.msg import Twist
import argparse

if __name__ == '__main__':
    follower = TurtlebotFollower()
