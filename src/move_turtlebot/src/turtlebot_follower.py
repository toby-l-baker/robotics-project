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

class TurtlebotFollower:
    """
    This class will follow another turtlebot using it's localization
    through either AR tags or another method.

    Publishes to cmd_vel_mux
    """
    def __init__(self, marker_name, turtlebot_name, turtlebot_to_follow):
        rospy.init_node('turtlebot_follower', anonymous=True)

        """Setup the names for the transform"""
        self.turtlebot_name = turtlebot_name
        self.turtlebot_to_follow = turtlebot_to_follow
        self.marker_frame = marker_name
        # TODO use ros parameters to remap topics to new namespace
        self.camera_frame = "camera_rgb_frame" # self.turtlebot+"/usb_cam_frame"

        """Setup scaling constants"""
        self.x_scale = 1.0
        self.y_scale = 1.0
        self.x_desired = 0.2
        self.y_desired = 0.0

        """Setup the tf listener"""
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(2)

        """Setup cmd_vel_mux publisher"""
        # TODO: Allow for remapping
        cmd_vel_topic = "cmd_vel_mux/navi"
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

        period = rospy.Duration(0.05)
        self.timer = rospy.Timer(period, self.update_velocity, False)

        rospy.spin()

    def update_velocity(self):
        """Compute cmd_vel messages and publish"""
        try:
            self.trans = self.tfBuffer.lookup_transform(self.turtlebot_name, self.marker_frame, rospy.Time())
            x, y = self.trans.transform.translation.x, self.trans.transform.translation.y
            twist = Twist()
            twist.linear.x = self.x_scale * (x - x_desired)
            twist.angular.z = self.y_scale * (y - y_desired)
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            print(e)
            """Tells robot to stop"""
            self.cmd_vel_pub.publish(Twist())

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
