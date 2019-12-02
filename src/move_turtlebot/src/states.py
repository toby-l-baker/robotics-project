"""
Classes defining states
"""

import numpy as np
import rospy

class State(object):
    """
    An abstract class for a state machine state value
    """
    def __init__(self, name=None):
        self.name = name

    def entry(self, machine):
        """ Invoked on entering the state - only once
        machine - reference to state machine

        no return
        """
        pass

    def exit(self, machine):
        """
        Invoked to check if time to exit the state - repeated
        machine - reference to state machine

        returns None or a new state
        """
        return None

    def reset_params(self):
        """
        Resets parameters from current parameter server
        """
        pass


class Idle(State):
    """
    State corresponding to robots remaining idle
    """
    def __init__(self):
        super(Idle, self).__init__("Idle")

    def entry(self, machine):
        machine.pub_motors()

    def exit(self, machine):
        return None

    def reset_params(self):
        pass

class InitialNavigation(State):
    """
    State corresponding to robots navigating to each other
    """
    def __init__(self):
        super(InitialNavigation, self).__init__("InitialNavigation")
        self.proximity = rospy.get_param("~initial_proximity")

    def entry(self, machine):
        pass

    def exit(self, machine):
        # Check if poses are close to each other
        if machine.get_robot_proximity() < self.proximity:
            if machine.name == machine.package_owner:
                return FollowerAlign()
            else:
                return LeaderAlign()

        return None

    def reset_params(self):
        self.proximity = rospy.get_param("~initial_proximity")

class FollowerAlign(State):
    """
    State corresponding to robot following to align
    """
    def __init__(self):
        super(FollowerAlign, self).__init__("FollowerAlign")
        # Need to have robots be close to each other before switching state
        self.close_start_time = None
        self.proximity = rospy.get_param("~required_proximity")
        self.duration = rospy.Duration(secs=rospy.get_param("~required_duration"))

    def entry(self, machine):
        pass

    def exit(self, machine):
        return None
        """
        if machine.get_robot_proximity() < self.proximity:
            if not self.close_start_time:
                self.close_start_time = rospy.Time.now()
        else:
            self.close_start_time = None

        if self.close_start_time is not None:
            if (rospy.Time.now() - self.close_start_time) > self.duration:
                return ExchangePackage()

        return None
        """

    def reset_params(self):
        self.proximity = rospy.get_param("~required_proximity")
        self.duration = rospy.Duration(secs=rospy.get_param("~required_duration"))


class LeaderAlign(State):
    """
    State corresponding to robot leading to align
    """
    def __init__(self):
        super(LeaderAlign, self).__init__("LeaderAlign")
        # Need to have robots be close to each other before switching state
        self.close_start_time = None
        self.proximity = rospy.get_param("~required_proximity")
        self.duration = rospy.Duration(secs=rospy.get_param("~required_duration"))

    def entry(self, machine):
        pass

    def exit(self, machine):
        if machine.get_robot_proximity() < self.proximity:
            if not self.close_start_time:
                self.close_start_time = rospy.Time.now()
        else:
            self.close_start_time = None

        if self.close_start_time is not None:
            if (rospy.Time.now() - self.close_start_time) > self.duration:
                return ExchangePackage()

        return None
    
    def reset_params(self):
        self.proximity = rospy.get_param("~required_proximity")
        self.duration = rospy.Duration(secs=rospy.get_param("~required_duration"))

class ExchangePackage(State):
    """
    State corresponding to exchanging package
    """
    def __init__(self):
        super(ExchangePackage, self).__init__("ExchangePackage")
        self.proximity = rospy.get_param("~required_proximity")
        self.epsilon = rospy.get_param("~epsilon")
        self.exchange_start_time = None
        self.duration = rospy.Duration(secs=rospy.get_param("~exchange_duration"))

    def entry(self, machine):
        self.exchange_start_time = rospy.Time.now()
        machine.trigger_exchange()

    def exit(self, machine):
        if (rospy.Time.now() - self.exchange_start_time) > self.duration:
            return FinalNavigation()

        return None

    def reset_params(self):
        self.duration = rospy.Duration(secs=rospy.get_param("~exchange_duration"))
        self.proximity = rospy.get_param("~required_proximity")
        self.epsilon = rospy.get_param("~epsilon")

class FinalNavigation(State):
    """
    State corresponding to robots navigating away from each other
    """
    def __init__(self):
        super(FinalNavigation, self).__init__("FinalNavigation")

    def entry(self, machine):
        machine.pub_speed(0.0)

    def exit(self, machine):
        pass

    def reset_params(self):
        pass
