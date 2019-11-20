#!/usr/bin/python

import rospy
from line_path import LinePath

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node("make_line_path")
    wait_for_time()
    linePath = LinePath(0.05, 40)
    rospy.spin()

if __name__ == "__main__":
    main()
