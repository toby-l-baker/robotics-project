import rospy
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import Marker

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

    def __init__(self, offset, count):
        self.offset = offset
        self.count = count
        self.length = offset * count

        rviz_goal_topic = "move_base_simple/goal"
        self.sub = rospy.Subscriber(rviz_goal_topic, PoseStamped, self.callback)
        rviz_marker_topic = "line_path/markers"
        self.marker_pub = rospy.Publisher(rviz_marker_topic, Marker, queue_size=count)

        self.current_path = None
        print("Initialized")

    """
    Callback for PoseStamped start goal messages
    """
    def callback(self, msg):
        self.current_path = []
        pose_stamped = msg
        rotation_matrix = tft.quaternion_matrix(vector(msg.pose.orientation))
        translation = np.dot(rotation_matrix, [1, 0, 0, 0])
        for i in range(self.count):
            m = marker(deepcopy(pose_stamped))
            m.id = i
            self.marker_pub.publish(m)

            self.current_path.append(deepcopy(pose_stamped))
            pose_stamped.pose.position.x += translation[0] * self.offset
            pose_stamped.pose.position.y += translation[1] * self.offset
            pose_stamped.pose.position.z += translation[2] * self.offset
