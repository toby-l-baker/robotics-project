#!/usr/bin/python

import rospy

class LinePath():
    """
    Subscribes to the start goal pose, and generates a line of
    waypoints of constant offset and count.
    """

    def __init__(self, offset, count):
        self.offset = offset
        self.count = count
        self.length = offset * count
