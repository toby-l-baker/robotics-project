
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Empty, Float64
from kobuki_msgs.msg import BumperEvent

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

        self.state = Idle()
        self.next_state = None

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
        self.my_pose_sub = rospy.Subscriber(my_pose_sub_topic, PoseWithCovarianceStamped, self.my_pose_callback)
        self.my_pose = PoseStamped()

        # Other robot pose information
        other_pose_sub_topic = rospy.get_param('~other_pose_sub_topic')
        self.other_pose_sub = rospy.Subscriber(other_pose_sub_topic, PoseWithCovarianceStamped, self.other_pose_callback)
        self.other_pose = PoseStamped()

        # Motor Inputs - immediately forwarded, so no other variables needed
        move_base_sub_topic = rospy.get_param('~move_base_sub_topic')
        self.move_base_sub = rospy.Subscriber(move_base_sub_topic, Twist, self.move_base_callback)

        follower_sub_topic = rospy.get_param('~follower_sub_topic')
        self.follower_sub = rospy.Subscriber(follower_sub_topic, Twist, self.follower_callback)

        exchange_follower_sub_topic = rospy.get_param('~exchange_follower_sub_topic')
        self.exchange_follower_sub = rospy.Subscriber(exchange_follower_sub_topic, Twist, self.exchange_follower_callback)

        follower_error_topic = rospy.get_param('~follower_error')
        self.follower_error_sub = rospy.Subscriber(follower_error_topic, Float64, self.follower_error_callback)

        # Meta topic - misc control
        meta_sub_topic = rospy.get_param("~meta_sub_topic")
        self.meta_sub = rospy.Subscriber(meta_sub_topic, String, self.meta_callback)

        # Bumper topic info
        bumper_sub_topic = rospy.get_param("~bumper_sub_topic")
        self.bumper_sub = rospy.Subscriber(bumper_sub_topic, BumperEvent, self.bumper_callback)

        # Reset parameters
        self.reset_sub = rospy.Subscriber("reset", Empty, self.reset_params)

        speed_pub_topic = rospy.get_param("~speed_topic")
        self.speed_pub = rospy.Publisher(speed_pub_topic, Float64, queue_size=1)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.run()
            rate.sleep()

    def run(self):
        """
        Transitions state if there is a next state
        Otherwise uses exit to check if next state is found
        """
        print("State:", self.state.name)
        if self.next_state:
            self.state = self.next_state
            self.next_state = None
            self.state.entry(self)
        else:
            self.next_state = self.state.exit(self)

    def pub_motors(self, twist=Twist()):
        self.motor_pub.publish(twist)

    def pub_speed(self, speed):
        self.speed_pub.publish(Float64(speed))

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
        self.pub_speed(0.1)

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
        if msg.data == "exchange":
            self.next_state = ExchangePackage()
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
        if self.state.name == "ExchangePackage":
            self.pub_motors(msg)

    def exchange_follower_callback(self, msg):
        if self.state.name == "ExchangePackage":
            self.pub_motors(msg)

    def my_pose_callback(self, msg):
        self.my_pose = msg

    def other_pose_callback(self, msg):
        self.other_pose = msg

    def package_callback(self, msg):
        self.package_owner = msg.data

    def follower_error_callback(self, msg):
        if self.state.name == "FollowerAlign":
            if msg.data < self.state.proximity:
                self.next_state = ExchangePackage()
        elif self.state.name == "ExchangePackage":
            if msg.data > self.state.proximity + self.state.epsilon:
                self.next_state = FollowerAlign()

    def bumper_callback(self, msg):
        if not self.state.name == "FollowerAlign":
            return
        self.next_state = ExchangePackage()

    def reset_params(self, msg):
        self.state.reset_params()
        if self.next_state:
            self.next_state.reset_params()
