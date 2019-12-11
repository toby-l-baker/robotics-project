#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import Empty, Float64, String
import numpy as np
import tf.transformations as tft
import state_names
from Mechanism import Mechanism

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
        """Setup the names for the transform"""
        self.turtlebot_name = rospy.get_param("~name")
        self.turtlebot_frame = self.turtlebot_name + "/base_link"
        self.marker_frame = rospy.get_param("~marker_frame_to_follow")
        self.camera_frame = self.turtlebot_name + "/camera_rgb_frame"
        """ Initialize the mechanism """
        self.init_mechanism(rospy.get_param("~pin_num"))
        self.tag_upside_down = rospy.get_param("~upside_down")

        """Setup more state"""
        self.i = 0
        self.enabled = False
        self.mode = state_names.FOLLOW_NULL
        self.exchange_start = None
        self.exchange_delay_start = None

        """Setup scaling constants"""
        self.Kalpha = rospy.get_param("~Kalpha")
        self.Kbeta = rospy.get_param("~Kbeta")
        self.Krho = rospy.get_param("~Krho")
        self.target_distance = rospy.get_param("~target_distance")
        self.duration = rospy.Duration(rospy.get_param("~duration"))
        self.target_velocity = 0.0
        self.y_desired = 0.0
        self.x_vel_max = 0.45
        self.z_ang_max = 1.0

        """Setup the tf transformer with 5 second cache time"""
        self.cache_time = rospy.get_param("~cache_time")
        self.transformer = tf.TransformListener()
        rospy.sleep(2)

        # Publishers
        """Setup cmd_vel_mux publisher"""
        cmd_vel_topic = rospy.get_param("~follower_motor_cmds")
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1, latch=True)

        """ack info topic publisher"""
        done_topic = rospy.get_param("~done_topic")
        self.done_pub = rospy.Publisher(done_topic, String, queue_size=1, latch=True)

        # Subscribers
        """start topic subscriber"""
        start_topic = rospy.get_param("~start_topic")
        self.start_sub = rospy.Subscriber(start_topic, String, self.start_callback)

        """Setup reset subscriber to tune parameters more easily"""
        reset_topic = "follower/reset"
        self.reset_sub = rospy.Subscriber(reset_topic, Empty, self.reset_params)

        """Other Robot twist topic"""
        other_robot_twist_topic = rospy.get_param("~other_robot_twist_topic")
        self.other_robot_twist_sub = rospy.Subscriber(other_robot_twist_topic, Twist, self.other_robot_twist_callback)

        # Timer
        self.period = rospy.Duration(0.05)

    def run_to_completion(self):
        self.enable()
        done = False
        while not rospy.is_shutdown() and not done:
            done = self.run()
            rospy.sleep(self.period)

        self.disable()
        return done

    def init_mechanism(self, pin):
        self.mechanism = Mechanism(pin)
        self.mechanism.catch()

    def enable(self):
        """Sets follower to be enabled"""
        if self.enabled:
            return "ACK already enabled"
        self.enabled = True
        self.mode = state_names.FOLLOW_ALIGN
        self.exchange_start = None
        self.mechanism.catch()
        return "ACK enabled"

    def disable(self):
        self.enabled = False
        self.mode = state_names.FOLLOW_NULL
        self.exchange_start = None
        self.mechanism.catch()
        return "ACK disabled"

    def reset_params(self, msg):
        self.duration = rospy.Duration(rospy.get_param("~duration"))
        self.target_distance = rospy.get_param("~target_distance")
        self.cache_time = rospy.get_param("~cache_time")
        self.Kalpha = rospy.get_param("~Kalpha")
        self.Kbeta = rospy.get_param("~Kbeta")
        self.Krho = rospy.get_param("~Krho")

    def start_callback(self, msg):
        print("Follower got message {}".format(msg.data))
        if not self.enabled:
            self.enable()
        else:
            self.disable()

    def run(self):
        error = self.update_velocity(self.enabled)
        if not self.enabled:
            return False

        if error < 0:
            """Skip negative returns - indicates error with tf"""
            return
        if error < 0.05 and self.mode == state_names.FOLLOW_ALIGN:
            print("Follower close - moving to exchange delay")
            self.mode = state_names.FOLLOW_EXCHANGE_DELAY
            self.exchange_delay_start = rospy.Time.now()

        if error < 0.08 and self.mode == state_names.FOLLOW_EXCHANGE_DELAY:
            if rospy.Time.now() - self.exchange_delay_start > rospy.Duration(0.5):
                print("Follower exchanging")
                self.mode = state_names.FOLLOW_EXCHANGE
                self.mechanism.deliver()
                self.exchange_start = rospy.Time.now()
        elif self.mode == state_names.FOLLOW_EXCHANGE_DELAY:
            print("Follower going back to align")
            self.mode = state_names.FOLLOW_ALIGN

        if self.mode == state_names.FOLLOW_EXCHANGE:
            """Check time"""
            print("Follower done")
            if rospy.Time.now() - self.exchange_start > self.duration:
                self.done_pub.publish("{} DONE FOLLOWING".format(self.turtlebot_name))
                return True

        return False

    def update_velocity(self, enabled):
        """Compute cmd_vel messages and publish"""
        try:
            latest_time = self.transformer.getLatestCommonTime(self.turtlebot_frame, self.marker_frame)
            current_time = rospy.Time.now()

            if current_time.secs - latest_time.secs > self.cache_time:
                """Publish default speed"""
                command = Twist()
                command.linear.x = 0.0
                print("Out of date transform by {} seconds".format(current_time.secs - latest_time.secs))
                if enabled:
                    self.cmd_vel_pub.publish(command)
                return -1

            trans, rot = self.transformer.lookupTransform(self.turtlebot_frame,
                                                          self.marker_frame,
                                                          rospy.Time())

            theta = -(tft.euler_from_quaternion(rot)[2] + np.pi / 2.0)
            x, y = trans[0], trans[1]
            if self.tag_upside_down:
                theta = theta + np.pi

            x_goal = x - self.target_distance * np.cos(theta)
            y_goal = y - self.target_distance * np.sin(theta)

            if abs(y_goal) > 0.25 or abs(theta) > np.pi / 4.0:
                self.cmd_vel_pub.publish(Twist())
                return x_goal

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
            if enabled:
                self.cmd_vel_pub.publish(twist)

            return x_goal

        except tf.Exception as e:
            """Tells robot to stop"""
            if enabled:
                self.cmd_vel_pub.publish(Twist())
            print("Exception occurred:", e)

            return -1

    def other_robot_twist_callback(self, msg):
        if msg.linear.x > 0:
            self.target_velocity = msg.linear.x
        else:
            self.target_velocity = 0.0
