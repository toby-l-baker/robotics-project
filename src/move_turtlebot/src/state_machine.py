
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
        # Publishes state on the 'state' topic
        self.state_pub = rospy.Publisher('state', String, queue_size=1);

        # Subscribes to acknowledge messages from each state
        self.initial_ack = rospy.Subscriber('Initial_ack', String, self.initial_callback)
        self.follow_ack = rospy.Subscriber('Follow_ack', String, self.follow_callback)
        self.final_ack = rospy.Subscriber('Final_ack', String, self.final_callback)
        # Meta topic - misc control
        meta_sub_topic = rospy.get_param("~meta_sub_topic")
        self.meta_sub = rospy.Subscriber(meta_sub_topic, String, self.meta_callback)


        self.leader_name = rospy.get_param("~leader_name")
        self.follower_name = rospy.get_param("~follower_name")

        # Initializes node ready checks
        self.path_planner_ready = False;
        self.leader_move_ready = False;
        self.follower_move_ready = False;
        self.follower_follow_ready = False;

        self.state = state_names.IDLE
        
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

    def initial_callback(self, msg):
        if(self.state != state_names.INITIAL):
            print("Not in initial state! Message was: %s" % msg.data)
            if(msg.data == state_names.READY):
                self.initial_ready = True
        else:
            print('In Initial State, received message: %s' % msg.data)
            if(msg.data == state_names.DONE):
                self.state = state_names.FOLLOW
                self.state_pub.publish(self.state + ' ' + self.follower_name) 
                print("Transitioning to %s" % state_names.FOLLOW)

    def follow_callback(self, msg):
        if(self.state != state_names.FOLLOW):
            print("Not in follow state! Message was: %s" % msg.data)
            if(msg.data == state_names.READY):
                self.follow_ready = True
        else:
            print("In Follow State, received message: %s" % msg.data)
            if(msg.data == state_names.DONE):
                self.state = state_names.FINAL
                self.state_pub.publish(self.state)
                print("Transitioning to %s" % state_names.FINAL)


    def final_callback(self, msg):
        if(self.state != state_names.FINAL):
            print("Not in final state! Message was: %s" % msg.data)
            if(msg.data == state_names.READY):
                self.final_ready = True
        else:
            print("In Final State, received message: %s" % msg.data)
            if(msg.data == state_names.DONE):
                self.state = state_names.IDLE
                self.state_pub.publish(self.state)
                print("Transitioning to %s" % state_names.IDLE)




