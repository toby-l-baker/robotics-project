import rospy
import tf.transformations as tft
import numpy as np
import actionlib

from lab2.msg import XYHV, XYHVPath
from lab2.srv import FollowPath
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped, Vector3, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

from copy import deepcopy

def vector(obj):
    result = []
    attributes = ["x", "y", "z", "w"]
    for attr in attributes:
        if hasattr(obj, attr):
            result.append(getattr(obj, attr))
    return result

def marker(pose_stamped):
    result = Marker()
    result.pose = pose_stamped.pose
    result.header = pose_stamped.header
    result.scale = Vector3(0.05, 0.01, 0.01)
    result.color.r = 1
    result.color.a = 1
    return result


class LinePath():
    """
    Subscribes to the start goal pose, and generates a line of
    waypoints of constant offset and count.
    """

    def __init__(self, offset, count, ahead):
        self.offset = offset
        self.count = count
        self.length = offset * count
        self.ahead = ahead
        self.got_pose = False

        pose_topic = "/amcl_pose"
        self.sub = rospy.Subscriber(pose_topic, PoseWithCovarianceStamped, self.pose_callback)
        rviz_goal_topic = "move_base_simple/goal"
        self.sub = rospy.Subscriber(rviz_goal_topic, PoseStamped, self.callback)
        rviz_marker_topic = "line_path/markers"
        self.marker_pub = rospy.Publisher(rviz_marker_topic, Marker, queue_size=count)

        # Publishes resulting path to the topic for the controller
        self.controller = rospy.ServiceProxy("/controller/follow_path", FollowPath())

        self.current_path = None
        print("Initialized")

    def publish_config_path(self):
        configs = []
        for pose_stamped in self.current_path:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z_ang = tft.euler_from_quaternion(vector(pose_stamped.pose.orientation))[2]
            configs.append([x, y, z_ang])

        h = Header()
        h.stamp = rospy.Time.now()
        desired_speed = 0.2
        ramp_percent = 0.1
        ramp_up = np.linspace(0.0, desired_speed, int(ramp_percent * len(configs)))
        ramp_down = np.linspace(desired_speed, 0.3, int(ramp_percent * len(configs)))
        speeds = np.zeros(len(configs))
        speeds[:] = desired_speed
        speeds[0:len(ramp_up)] = ramp_up
        speeds[-len(ramp_down):] = ramp_down
        path = XYHVPath(h, [XYHV(*[config[0], config[1], config[2], speed]) for config, speed in zip(configs, speeds)])
        success = self.controller(path)


    """
    Callback for PoseStamped start goal messages
    """
    def callback(self, msg):
        self.current_path = []
        if self.ahead: 
       	        pose_stamped = deepcopy(self.pose)
		pose_stamped.header = msg.header
        else:
       	        pose_stamped = msg
        rotation_matrix = tft.quaternion_matrix(vector(pose_stamped.pose.orientation))
        translation = np.dot(rotation_matrix, [1, 0, 0, 0])
        for i in range(self.count):
            cur_pose = deepcopy(pose_stamped)
            m = marker(cur_pose)
            m.id = i
            self.marker_pub.publish(m)

            self.current_path.append(cur_pose)
            pose_stamped.pose.position.x += translation[0] * self.offset
            pose_stamped.pose.position.y += translation[1] * self.offset
            pose_stamped.pose.position.z += translation[2] * self.offset

        self.publish_config_path()

    """
    Callback for PoseWithCovarianceStamped pose messages
    """
    def pose_callback(self, msg):
        if not self.got_pose:
                print("Got pose")
                self.pose = PoseStamped()
                self.pose.pose = msg.pose.pose
                self.got_pose = True
