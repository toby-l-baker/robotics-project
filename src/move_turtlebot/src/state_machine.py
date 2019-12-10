
import rospy
from std_msgs.msg import String, Bool
import state_names

class StateMachine():
    """
    A node to handle the overarching state machine of transfering
    packages between TurtleBots.
    """
    def __init__(self):
        ## Setup
        rospy.init_node('state_machine', anonymous=True)
        # Initializes node ready checks
        self.path_planner_ready = False
        self.leader_move_ready = False
        self.follower_move_ready = False
        self.follower_follow_ready = False
        self.state = state_names.IDLE
        self.leader_name = rospy.get_param("~leader_name")
        self.follower_name = rospy.get_param("~follower_name")

        # Publishes state on the 'state' topic
        self.state_pub = rospy.Publisher('state', String, queue_size=1);

        # Subscribes to acknowledge messages from each state
        initial_ack_topic = rospy.get_param('~initial_ack')
        follow_ack_topic = rospy.get_param('~follow_ack')
        final_ack_topic = rospy.get_param('~final_ack')
        self.initial_ack = rospy.Subscriber(initial_ack_topic, String, self.initial_callback)
        self.follow_ack = rospy.Subscriber(follow_ack_topic, String, self.follow_callback)
        self.final_ack = rospy.Subscriber(final_ack_topic, String, self.final_callback)

        # Subscribes to ready check from all states
        node_ready_topic = rospy.get_param('~node_ready')
        self.ready_check = rospy.Subscriber(node_ready_topic, String, self.ready_callback)

        # Meta topic - misc control
        meta_sub_topic = rospy.get_param("~meta_sub_topic")
        self.meta_sub = rospy.Subscriber(meta_sub_topic, String, self.meta_callback)
        ##State Machine begin

        # Publish IDLE state
        self.state_pub.publish(self.state)
        # Wait until all nodes are ready
        while(not(self.path_planner_ready and self.leader_move_ready and self.follower_move_ready and self.follower_follow_ready)):
            pass

        # Go to INITIAL state
        self.state = state_names.INITIAL
        self.state_pub.publish(self.state)

        # Keep listening and updating SM
        rospy.spin()

    """
    CALLBACKS
    """
    def meta_callback(self, msg):
        print(msg.data)
        if msg.data == "stop":
            self.state = state_names.IDLE
            self.state_pub.publish(self.state)
        if msg.data == "start":
            self.state = state_names.INITIAL
            self.state_pub.publish(self.state)
        if msg.data == "follow":
            self.state = state_names.FOLLOW
            self.state_pub.publish(self.state + ' ' + self.follower_name)
        if msg.data == "final":
            self.state = state_names.FINAL
            self.state_pub.publish(self.state)

    def ready_callback(self, msg):
        if(msg.data == "LEADER"):
            self.leader_move_ready = True
        elif(msg.data == "FOLLOWER"):
            self.follower_move_ready = True
        elif(msg.data == "FOLLOWER_FOLLOW"):
            self.follower_follow_ready = True
        elif(msg.data == "PATH_PLAN")
            self.path_planner_ready = True

    def initial_callback(self, msg):
        if(self.state != state_names.INITIAL):
            print("Not in initial state! Message was: %s" % msg.data)
        else:
            print('In Initial State, received message: %s' % msg.data)
            if(msg.data == state_names.DONE):
                self.state = state_names.FOLLOW
                self.state_pub.publish(self.state + ' ' + self.follower_name) 
                print("Transitioning to %s" % state_names.FOLLOW)

    def follow_callback(self, msg):
        if(self.state != state_names.FOLLOW):
            print("Not in follow state! Message was: %s" % msg.data)
        else:
            print("In Follow State, received message: %s" % msg.data)
            if(msg.data == state_names.DONE):
                self.state = state_names.FINAL
                self.state_pub.publish(self.state)
                print("Transitioning to %s" % state_names.FINAL)

    def final_callback(self, msg):
        if(self.state != state_names.FINAL):
            print("Not in final state! Message was: %s" % msg.data)
        else:
            print("In Final State, received message: %s" % msg.data)
            if(msg.data == state_names.DONE):
                self.state = state_names.IDLE
                self.state_pub.publish(self.state)
                print("Transitioning to %s" % state_names.IDLE)
