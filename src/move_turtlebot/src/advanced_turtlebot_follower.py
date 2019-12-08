#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import Empty, Float64, String
import numpy as np
import tf.transformations as tft
import state_names

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

        """Setup more state"""
        self.i = 0
        self.enabled = False
        self.mode = state_names.FOLLOW_NULL
        self.exchange_start = None

        """Setup scaling constants"""
        self.Kalpha = rospy.get_param("~Kalpha")
        self.Kbeta = rospy.get_param("~Kbeta")
        self.Krho = rospy.get_param("~Krho")
        self.target_distance = rospy.get_param("~target_distance")
        self.duration = rospy.Duration(rospy.get_param("~duration"))
        self.target_velocity = 0.0
        self.y_desired = 0.0
        self.x_vel_max = 0.5
        self.z_ang_max = 1.0

        """Setup the tf transformer with 5 second cache time"""
        self.cache_time = rospy.get_param("~cache_time")
        self.transformer = tf.TransformListener()
        rospy.sleep(2)

        # Publishers
        """Setup cmd_vel_mux publisher"""
        cmd_vel_topic = rospy.get_param("~follower_motor_cmds")
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

        """ack info topic publisher"""
        ack_info_topic = rospy.get_param("~ack_info_topic")
        self.ack_info_pub = rospy.Publisher(ack_info_topic, String, queue_size=1)

        # Subscribers
        """State topic subscriber"""
        state_topic = rospy.get_param("~state_topic")
        self.state_sub = rospy.Subscriber(state_topic, String, self.state_callback)

        """Setup reset subscriber to tune parameters more easily"""
        reset_topic = "follower/reset"
        self.reset_sub = rospy.Subscriber(reset_topic, Empty, self.reset_params)

        """Other Robot twist topic"""
        other_robot_twist_topic = rospy.get_param("~other_robot_twist_topic")
        self.other_robot_twist_sub = rospy.Subscriber(other_robot_twist_topic, Twist, self.other_robot_twist_callback)

        # Timer
        period = rospy.Duration(0.05)
        self.timer = rospy.Timer(period, self.run, False)

        rospy.spin()

    def enable(self):
        """Sets follower to be enabled"""
        if self.enabled:
            return "ACK already enabled"
        self.enabled = True
        self.mode = state_names.FOLLOW_ALIGN
        self.exchange_start = None
        return "ACK enabled"

    def disable(self):
        self.enabled = False
        self.mode = state_names.FOLLOW_NULL
        self.exchange_start = None
        return "ACK disabled"

    def reset_params(self, msg):
        self.duration = rospy.Duration(rospy.get_param("~duration"))
        self.target_distance = rospy.get_param("~target_distance")
        self.cache_time = rospy.get_param("~cache_time")
        self.Kalpha = rospy.get_param("~Kalpha")
        self.Kbeta = rospy.get_param("~Kbeta")
        self.Krho = rospy.get_param("~Krho")

    def state_callback(self, msg):
        if msg.data == "Follow":
            res = self.enable()
            self.ack_info_pub.publish(res)
        else:
            res = self.disable()
            self.ack_info_pub.publish(res)

    def run(self, event):
        if not self.enabled:
            return

        error = self.update_velocity()
        if error < 0:
            """Skip negative returns - indicates error with tf"""
            return
        if error < 0.05 and self.mode == state_names.FOLLOW_ALIGN
            self.mode = state_names.FOLLOW_EXCHANGE
            self.exchange_start = rospy.Time.now()

        if self.mode == state_names.FOLLOW_EXCHANGE:
            """Check time"""
            if rospy.Time.now() - self.exchange_start > self.duration:
                self.ack_info_pub.publish("DONE")
            else:
                self.ack_info_pub.publish("EXCHANGE")

    def update_velocity(self):
        """Compute cmd_vel messages and publish"""
        try:
            latest_time = self.transformer.getLatestCommonTime(self.turtlebot_frame, self.marker_frame)
            current_time = rospy.Time.now()

            if current_time.secs - latest_time.secs > self.cache_time:
                """Publish default speed"""
                command = Twist()
                command.linear.x = 0.0
                self.cmd_vel_pub.publish(command)
                print("Out of date transform by {} seconds".format(current_time.secs - latest_time.secs))
                return -1

            trans, rot = self.transformer.lookupTransform(self.turtlebot_frame,
                                                          self.marker_frame,
                                                          rospy.Time())
            theta = -(tft.euler_from_quaternion(rot)[2] + np.pi / 2.0)
            x, y = trans[0], trans[1]
            x_goal = x - self.target_distance * np.cos(theta)
            y_goal = y - self.target_distance * np.sin(theta)

            if x_goal < 0.0 and x_goal > -0.1:
                # Clip x if past goal
                x_goal = 0.0
            if x_goal < 0.1:
                # Clip y if close to goal
                if y_goal < 0.1 and y_goal > -0.1:
                    y_goal = 0.0

                # Clip theta if close to goal
                if theta < 0.30 and theta > -0.30:
                    theta = 0.0 

            # Using control algorithm from https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/move_to_pose
            rho = np.sqrt(x_goal**2 + y_goal**2)
            alpha = np.arctan2(y_goal, x_goal) - theta
            beta = - theta - alpha

            velocity = self.Krho * rho + self.target_velocity
            omega  = self.Kalpha * alpha + self.Kbeta * beta # TODO consider adding target rotation velocity

            if self.i == 3:
                print("\n\n\nAdvanced Follower\nCommand = ({}, {})".format(velocity, omega))
                print("rho = {}, alpha = {}, beta = {}".format(rho, alpha, beta))
                print("(x_goal, y_goal, theta) = ({}, {}, {})".format(x_goal, y_goal, theta))
                self.i = 0
            else:
                self.i += 1

            twist = Twist()
            twist.linear.x = max(min(velocity, self.x_vel_max), -self.x_vel_max)
            twist.angular.z = max(min(omega, self.z_ang_max), -self.z_ang_max)
            self.cmd_vel_pub.publish(twist)

            return x_goal

        except tf.Exception as e:
            """Tells robot to stop"""
            self.cmd_vel_pub.publish(Twist())
            print("Exception occurred:", e)

            return -1

    def other_robot_twist_callback(self, msg):
        if msg.linear.x > 0:
            self.target_velocity = msg.linear.x
        else:
            self.target_velocity = 0.0

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
