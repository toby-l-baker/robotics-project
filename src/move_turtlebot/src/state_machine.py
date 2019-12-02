
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

from states import *

def vector(obj):
    """Returns an np array of the input object"""
    result = []
    attributes = ["x", "y", "z", "w"]
    for attr in attributes:
        if hasattr(obj, attr):
            result.append(getattr(obj, attr))
    return np.array(result)

class StateMachine():
    """
    A node to handle the overarching state machine of transfering
    packages between TurtleBots.
    """
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)

        self.name = rospy.get_param('~name')

        # Get info on which robot has the package
        package_sub_topic = rospy.get_param('~package_sub_topic')
        self.package_sub = rospy.Subscriber(package_sub_topic, String, self.package_callback)
        self.package_owner = None

        # Update info on which robot has the package
        package_pub_topic = package_sub_topic
        self.package_pub = rospy.Publisher(package_pub_topic, String, queue_size=1, latch=True)

        # Outputs motor commands
        motor_pub_topic = rospy.get_param('~motor_pub_topic')
        self.motor_pub = rospy.Publisher(motor_pub_topic, Twist, queue_size=1)

        # Current robot pose information
        my_pose_sub_topic = rospy.get_param('~my_pose_sub_topic')
        self.my_pose_sub = rospy.Subscriber(my_pose_sub_topic, PoseStamped, self.my_pose_callback)
        self.my_pose = PoseStamped()

        # Other robot pose information
        other_pose_sub_topic = rospy.get_param('~other_pose_sub_topic')
        self.other_pose_sub = rospy.Subscriber(other_pose_sub_topic, PoseStamped, self.other_pose_callback)
        self.other_pose = PoseStamped()

        # Motor Inputs - immediately forwarded, so no other variables needed
        move_base_sub_topic = rospy.get_param('~move_base_sub_topic')
        self.move_base_sub = rospy.Subscriber(move_base_sub_topic, Twist, self.move_base_callback)

        follower_sub_topic = rospy.get_param('~follower_sub_topic')
        self.follower_sub = rospy.Subscriber(follower_sub_topic, Twist, self.follower_callback)

        # Meta topic - misc control
        meta_sub_topic = rospy.get_param("~meta_sub_topic")
        self.meta_sub = rospy.Subscriber(meta_sub_topic, String, self.meta_callback)

        self.state = Idle()
        self.next_state = None

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.run()
            rate.sleep()

    def run(self):
        """
        Transitions state if there is a next state
        Otherwise uses exit to check if next state is found
        """
        if self.next_state:
            self.state = self.next_state
            self.next_state = None
            self.state.entry(self)
        else:
            self.next_state = self.state.exit(self)

    def pub_motors(self, twist=Twist()):
        self.motor_pub.pub(twist)

    def get_robot_proximity(self):
        """
        Returns the proximity of this robot and the other robot
        """
        my_point = vector(self.my_pose.pose.position)
        other_point = vector(self.other_pose.pose.position)
        return np.linalg.norm(my_point - other_point)

    def trigger_exchange(self):
        """
        Starts the exchange package process
        """
        # TODO: implement
        pass

    """
    CALLBACKS
    """
    def meta_callback(self, msg):
        if msg.data == "stop":
            self.next_state = Idle()
        if msg.data == "start":
            self.next_state = InitialNavigation()
        if msg.data == "follow":
            self.next_state = FollowerAlign()
        if msg.data == "lead":
            self.next_state = LeaderAlign()

    def move_base_callback(self, msg):
        if self.state.name == "InitialNavigation":
            self.pub_motors(msg)
        elif self.state_name == "FinalNavigation":
            self.pub_motors(msg)

    def follower_callback(self, msg):
        if self.state.name == "FollowerAlign":
            self.pub_motors(msg)

    def my_pose_callback(self, msg):
        self.my_pose = msg

    def other_pose_callback(self, msg):
        self.other_pose = msg

    def package_callback(self, msg):
        self.package_owner = msg.data
