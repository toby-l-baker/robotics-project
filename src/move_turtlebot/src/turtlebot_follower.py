#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import Empty, Float64
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
    def __init__(self):
        rospy.init_node('turtlebot_follower', anonymous=True)

        """Setup the names for the transform"""
        self.turtlebot_name = rospy.get_param("~name")
        self.turtlebot_frame = self.turtlebot_name + "/base_link"
        self.marker_frame = rospy.get_param("~marker_frame_to_follow")
        self.camera_frame = self.turtlebot_name + "/camera_rgb_frame"

        """Setup scaling constants"""
        self.x_scale = rospy.get_param("~x_gain")
        self.y_scale = rospy.get_param("~y_gain")
        self.x_desired = rospy.get_param("~target_distance")
        self.x_threshold = rospy.get_param("~x_threshold")
        self.x_min_speed = rospy.get_param("~x_min_speed")
        self.target_velocity = rospy.get_param("~target_velocity")
        self.y_desired = 0.0
        self.x_vel_max = 0.5
        self.z_ang_max = 1.0

        self.default_speed = 0.0

        """Setup the tf transformer with 5 second cache time"""
        self.cache_time = rospy.get_param("~cache_time")
        self.transformer = tf.TransformListener()
        rospy.sleep(2)

        """Setup cmd_vel_mux publisher"""
        cmd_vel_topic = rospy.get_param("~follower_motor_cmds")
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

        """Setup error info publisher"""
        error_topic = rospy.get_param("~error_topic")
        self.error_pub = rospy.Publisher(error_topic, Float64, queue_size=1)

        """Setup cmd_vel_mux publisher"""
        reset_topic = "follower/reset"
        self.reset_sub = rospy.Subscriber(reset_topic, Empty, self.reset_params)

        """Speed value topic """
        speed_sub_topic = rospy.get_param("~speed_topic")
        self.speed_sub = rospy.Subscriber(speed_sub_topic, Float64, self.speed_callback)

        """Other Robot twist topic"""
        other_robot_twist_topic = rospy.get_param("~other_robot_twist_topic")
        self.other_robot_twist_sub = rospy.Subscriber(other_robot_twist_topic, Twist, self.other_robot_twist_callback)

        period = rospy.Duration(0.05)
        self.timer = rospy.Timer(period, self.update_velocity, False)

        rospy.spin()

    def reset_params(self, msg):
        self.x_desired = rospy.get_param("~target_distance")
        self.x_scale = rospy.get_param("~x_gain")
        self.y_scale = rospy.get_param("~y_gain")
        self.cache_time = rospy.get_param("~cache_time")
        self.x_threshold = rospy.get_param("~x_threshold")
        self.x_min_speed = rospy.get_param("~x_min_speed")
        self.target_velocity = rospy.get_param("~target_velocity")

    def speed_callback(self, msg):
        self.default_speed = msg.data

    def other_robot_twist_callback(self, msg):
        if msg.linear.x > 0:
            self.target_velocity = msg.linear.x
        else:
            self.target_velocity = 0.0

    def update_velocity(self, event):
        """Compute cmd_vel messages and publish"""
        try:
            latest_time = self.transformer.getLatestCommonTime(self.turtlebot_frame, self.marker_frame)
            current_time = rospy.Time.now()

            if current_time.secs - latest_time.secs > self.cache_time:
                """Publish default speed"""
                command = Twist()
                command.linear.x = self.default_speed
                self.cmd_vel_pub.publish(command)
                print("Out of date transform by {} seconds".format(current_time.secs - latest_time.secs))
                return

            trans, rot = self.transformer.lookupTransform(self.turtlebot_frame,
                                                          self.marker_frame,
                                                          rospy.Time())
            x, y = trans[0], trans[1]
            twist = Twist()
            error = x - self.x_desired
            self.error_pub.publish(Float64(error))
            twist.linear.x = self.target_velocity + self.x_scale * error
            if x - self.x_desired > self.x_threshold:
                twist.linear.x += self.x_min_speed
            if twist.linear.x > self.x_vel_max:
                twist.linear.x = self.x_vel_max
            elif twist.linear.x < -self.x_vel_max:
                twist.linear.x = -self.x_vel_max

            twist.angular.z = self.y_scale * (y - self.y_desired)
            if twist.angular.z > self.z_ang_max:
                twist.angular.z = self.z_ang_max
            elif twist.angular.z < -self.z_ang_max:
                twist.angular.z = -self.z_ang_max

            self.cmd_vel_pub.publish(twist)
        except tf.Exception as e:
            """Tells robot to stop"""
            self.cmd_vel_pub.publish(Twist())
            print("Exception occurred:", e)

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
