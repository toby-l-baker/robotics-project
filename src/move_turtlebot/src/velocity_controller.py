#!/usr/bin/env python

import tf2_ros
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
import numpy as np
import tf.transformations as tft

def vector(obj):
    result = []
    attributes = ["x", "y", "z", "w"]
    for attr in attributes:
        if hasattr(obj, attr):
            result.append(getattr(obj, attr))
    return result

class VelocityController:
    """
    This class will subscribe to the transform from the slaves
    USB_Cam and the AR Tag on the master turtlebot. Using this
    transform it will alter the velocities in the path to align 
    each turtlebot.
    """
    def __init__(self, marker_num, turtlebot_num, master_turtlebot_num):
        """Setup the names for the transform"""
        self.turtlebot = "/mobile_base" # "/mobile_base_"+turtlebot_num
        self.master_turtlebot = str("/mobile_base_"+master_turtlebot_num) # mobile_base_0
        self.marker_frame = str("ar_marker_"+marker_num) # self.master_turtlebot+"/ar_marker_"+marker_num
        self.camera_frame = "camera_rgb_frame" # self.turtlebot+"/usb_cam_frame"

        """Setup the publisher and tf buffers"""
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(2)

        """Variables for old and new times for relative velocities"""
        self.oldTime = rospy.get_time()
        self.newTime = rospy.get_time()

        try:
            self.oldTransform = self.tfBuffer.lookup_transform(self.camera_frame, self.marker_frame, rospy.Time())
            self.newTransform = self.tfBuffer.lookup_transform(self.camera_frame, self.marker_frame, rospy.Time())
        except Exception as e:
            print(e)
        self.velocities = np.zeros((6, 5))



    def update_velocity(self):
        """Get transform from the camera frame to the marker frame"""
        try:
            self.trans = self.tfBuffer.lookup_transform(self.camera_frame, self.marker_frame, rospy.Time())
        except Exception as e:
            print(e)

        # print(self.trans)
        current = np.array([self.trans.transform.translation.x, self.trans.transform.translation.y])
        desired = np.array([0.5, 0])
        self.velocity = Twist()
        # print("X Position %f" % (current[0]))
        # print("Y Position %f" % (current[1]))

        # self.velocity.linear.x = (current[0] - desired[0])
        # self.velocity.angular.z = (current[1] - desired[1])

    def get_relative_velocity(self):
        self.oldTime = self.newTime
        self.newTime = rospy.get_time()
        self.oldTransform = self.newTransform
        try:
            self.newTransform = self.tfBuffer.lookup_transform(self.camera_frame, self.marker_frame, rospy.Time())
        except Exception as e:
            print(e)
        dx = (self.newTransform.transform.translation.x - self.oldTransform.transform.translation.x)
        dy = (self.newTransform.transform.translation.y - self.oldTransform.transform.translation.y)
        # dz_t = (self.newTransform.transform.angular.x - self.oldTransform.transform.angular.z)
        dt = float(self.newTime - self.oldTime)

        self.velocities[0, 1:] = self.velocities[0, :-1]
        self.velocities[1, 1:] = self.velocities[1, :-1]
        self.velocities[0, 0] = dx/dt
        self.velocities[1, 0] = dy/dt

        self.relative_vel = Twist()
        self.relative_vel.linear.x = np.average(self.velocities[0,:])
        self.relative_vel.linear.y = np.average(self.velocities[1,:])

    def transform_pose(self, pose, starting_frame, ending_frame):
	try:
        	super_trans = self.tfBuffer.lookup_transform(starting_frame, ending_frame, rospy.Time())
	except:
		return None
        trans = vector(super_trans.transform.translation)
        rot = vector(super_trans.transform.rotation)
        # generate homogenous transform
        R = tft.quaternion_matrix(rot)

        pose_q = vector(pose.orientation)
        pose_v = vector(pose.position)
        pose_v.append(1)

        target_q = tft.quaternion_multiply(rot, pose_q)
        target_p = (np.dot(R, np.transpose(pose_v)))[:3] + trans

        target = Pose()
        target.orientation.x = target_q[0]
        target.orientation.y = target_q[1]
        target.orientation.z = target_q[2]
        target.orientation.w = target_q[3]
        target.position.x = target_p[0]
        target.position.y = target_p[1]
        target.position.z = target_p[2]

        return target
